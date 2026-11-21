"""Waypoint-free self-drive supervisor that drives through Nav2's MPPI.

AutoNav_25-26 drives with ``nav2_mppi_controller::MPPIController`` following
a planned path. This node supplies that path for the goal-free track: each
tick it fits the observed corridor direction, plans a short lane reference
over the live local costmap and sends it to controller_server's FollowPath
action. MPPI alone produces /cmd_vel; this node only publishes zero velocity
when it stops. Run alongside the supervised command gate, which still
requires this node's enable heartbeat. No global plan or ground-truth track
coordinates are read by this process.

Junctions are recognised from the same observed grids (see
autonav_universe.intersection): the robot stops briefly at its stop line,
then goes straight or turns left. The turn request is a standing value from
/self_drive/turn_command (std_msgs/String 'straight' or 'left'), the desktop
control file or SELF_DRIVE_TURN_COMMAND; the latest change wins and the
default is straight.
"""
from __future__ import annotations
import json
import math
import os
from pathlib import Path
import time

import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import FollowPath
from nav_msgs.msg import OccupancyGrid, Odometry, Path as RosPath
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener, TransformException

from autonav_universe.local_drive import LocalGrid
from autonav_universe.lane_reference import LaneReferencePlanner, ReferencePath
from autonav_universe.intersection import (GUIDANCE_MAX_DEVIATION_RAD, IntersectionManager,
                                           trim_to_stop)
from autonav_universe.corridor_tangent import estimate_corridor_tangent
from autonav_universe.guidance_cache import GuidanceCache
from autonav_universe.full_track import ControlLease, read_json
from autonav_universe.referee_startup import RefereeHealth

CONTROLLER_PLUGIN = 'nav2_mppi_controller::MPPIController'
CONTROLLER_ID = 'FollowPath'
GOAL_CHECKER_ID = 'general_goal_checker'
STATUS_NAMES = {GoalStatus.STATUS_SUCCEEDED: 'succeeded', GoalStatus.STATUS_ABORTED: 'aborted',
                GoalStatus.STATUS_CANCELED: 'canceled'}


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def seconds(stamp):
    return stamp.sec + stamp.nanosec*1e-9


def planar_grid(msg):
    """Convert a planar OccupancyGrid; refuse malformed/non-planar origins."""
    q = msg.info.origin.orientation
    norm = math.sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w)
    if (not msg.header.frame_id or not math.isfinite(norm) or abs(norm-1) > .01
            or abs(q.x) > .001 or abs(q.y) > .001):
        return None
    grid = LocalGrid(msg.info.width, msg.info.height, msg.info.resolution,
                     msg.info.origin.position.x, msg.info.origin.position.y,
                     yaw(q), tuple(msg.data))
    return grid if grid.valid() else None


class SelfDriveController(Node):
    def __init__(self):
        super().__init__('self_drive_controller', parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.pending = {}
        self.grid = None
        self.grid_msg = None
        self.grid_received = self.odom_received = -math.inf
        self.divider = None
        self.divider_msg = None
        self.divider_received = -math.inf
        self.divider_available = False
        self.boundary = None
        self.boundary_msg = None
        self.boundary_received = -math.inf
        self.guidance = None
        self.guidance_cache = GuidanceCache(.75)
        self.guidance_available = False
        self.travel_yaw = None
        self.observed_divider_cells = 0
        self.odom_stamp = None
        self.last_pose = None
        self.speed_mps = 0.0
        self.allowed = False
        self.stopped = False
        self.driving = False
        self.last_command = (0.0, 0.0)
        self.previous_path = None
        self.reference = None
        self.goal_sequence = 0
        self.goal_handle = None
        self.goal_handle_sequence = -1
        self.follow_path_state = 'idle'
        self.lease = ControlLease() if os.environ.get('DESKTOP_CONTROL') == 'true' else None
        self.control_path = Path(os.environ['RUN_DIR']) / 'desktop_control.json' if self.lease else None
        self.lane_change_cost_m = float(os.environ.get('SELF_DRIVE_LANE_CHANGE_COST_M', '0.25'))
        self.planner = LaneReferencePlanner(
            self.lane_change_cost_m, float(os.environ.get('SELF_DRIVE_CENTERING_WEIGHT', '1.5')))
        self.junction = IntersectionManager(
            self.planner, float(os.environ.get('SELF_DRIVE_STOP_HOLD_S', '2.0')))
        self.junction.set_request(os.environ.get('SELF_DRIVE_TURN_COMMAND', ''), 'launch')
        self.desktop_turn = None
        self.turn_command_note = None
        self.referee = RefereeHealth()
        self.tf = Buffer()
        # TF gets its own node and thread so planning time cannot make the
        # robot pose stale; the buffer itself is shared and thread-safe.
        self.listener = TransformListener(self.tf, None, spin_thread=True)
        self.plan_ms = None
        self.follow = ActionClient(self, FollowPath, 'follow_path')
        self.create_subscription(OccupancyGrid, '/local_costmap/costmap', self.on_grid, 10)
        self.create_subscription(OccupancyGrid, '/self_drive/divider_costmap', self.on_divider, 10)
        # This is the classified paint-only output of the lidar line layer,
        # not the combined obstacle-bearing local costmap.
        self.create_subscription(OccupancyGrid, '/lidar_line_costmap', self.on_boundary, 10)
        self.create_subscription(Odometry, '/local_ekf/odom', self.on_odom, 10)
        self.create_subscription(Bool, '/self_drive/enable', self.on_enable, 10)
        self.create_subscription(String, '/self_drive/turn_command', self.on_turn_command, 10)
        self.create_subscription(Twist, '/cmd_vel', self.on_command, 10)
        self.create_subscription(String, '/igvc_sim/score',
                                 lambda msg: self.referee.update(msg.data, time.monotonic()), 10)
        self.cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.enable = self.create_publisher(Bool, '/full_track/enable', 10)
        self.path = self.create_publisher(RosPath, '/self_drive/plan', 10)
        self.status = self.create_publisher(String, '/self_drive/status', 10)
        steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(.2, self.tick, clock=steady)
        # The gate drops motion 0.35 s after the last enable. Refresh it
        # independently of planning time while MPPI follows an active path.
        self.create_timer(.1, lambda: self.enable.publish(Bool(data=self.driving)), clock=steady)

    def on_enable(self, msg):
        if self.lease is None:
            self.allowed = msg.data

    def on_turn_command(self, msg):
        self.set_turn(msg.data, 'topic')

    def set_turn(self, value, source):
        if self.junction.set_request(value, source):
            self.turn_command_note = None
            self.get_logger().info(f'turn request: {self.junction.request} ({source})')
        else:
            self.turn_command_note = f'ignored {source} turn command {str(value)[:32]!r}'
            self.get_logger().warn(self.turn_command_note)

    def on_command(self, msg):
        self.last_command = (float(msg.linear.x), float(msg.angular.z))

    # Grid callbacks only keep the newest message. Converting a full grid at
    # 20 Hz per topic starved this single-threaded node of TF callbacks;
    # convert_grids() converts each new message once, at tick rate.
    def on_grid(self, msg):
        self.pending['grid'] = msg
        self.grid_received = time.monotonic()

    def on_odom(self, msg):
        twist = msg.twist.twist
        if (not msg.header.frame_id or msg.child_frame_id != 'base_link'
                or not math.isfinite(twist.linear.x) or not math.isfinite(twist.angular.z)):
            self.odom_stamp = None
            return
        self.odom_stamp = seconds(msg.header.stamp)
        self.speed_mps = float(twist.linear.x)
        self.odom_received = time.monotonic()

    def on_divider(self, msg):
        self.pending['divider'] = msg
        self.divider_received = time.monotonic()

    def on_boundary(self, msg):
        self.pending['boundary'] = msg
        self.boundary_received = time.monotonic()

    def convert_grids(self):
        """Convert only grids that arrived since the last tick."""
        if 'grid' in self.pending:
            msg = self.pending.pop('grid')
            self.grid = planar_grid(msg)
            self.grid_msg = msg if self.grid else None
        if 'divider' in self.pending:
            msg = self.pending.pop('divider')
            grid = planar_grid(msg)
            self.divider, self.divider_msg = (grid, msg) if grid else (None, None)
            self.observed_divider_cells = sum(value > 0 for value in grid.cells) if grid else 0
        if 'boundary' in self.pending:
            msg = self.pending.pop('boundary')
            grid = planar_grid(msg)
            self.boundary, self.boundary_msg = (grid, msg) if grid else (None, None)

    def boundary_in_costmap_frame(self):
        """Transform the observed grid origin, retaining its actual cells."""
        source, msg = self.boundary, self.boundary_msg
        if msg.header.frame_id == self.grid_msg.header.frame_id:
            return source
        transform = self.tf.lookup_transform(self.grid_msg.header.frame_id,
                                             msg.header.frame_id,
                                             Time.from_msg(msg.header.stamp))
        q, t = transform.transform.rotation, transform.transform.translation
        norm = math.sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w)
        if not math.isfinite(norm) or abs(norm-1) > .01 or abs(q.x) > .001 or abs(q.y) > .001:
            raise ValueError('invalid boundary transform')
        angle = yaw(q)
        c, s = math.cos(angle), math.sin(angle)
        return LocalGrid(source.width, source.height, source.resolution,
                         t.x+c*source.origin_x-s*source.origin_y,
                         t.y+s*source.origin_x+c*source.origin_y,
                         source.origin_yaw+angle, source.cells)

    # -- Nav2 FollowPath --------------------------------------------------
    def ros_path(self, reference, frame):
        path = RosPath()
        path.header.frame_id = frame
        path.header.stamp = self.get_clock().now().to_msg()
        for x, y, heading in (reference.poses if reference else ()):
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x, pose.pose.position.y = float(x), float(y)
            pose.pose.orientation.z = math.sin(heading/2)
            pose.pose.orientation.w = math.cos(heading/2)
            path.poses.append(pose)
        return path

    def send_path(self, path):
        """Send a new FollowPath goal; controller_server preempts the last one."""
        goal = FollowPath.Goal()
        goal.path = path
        goal.controller_id = CONTROLLER_ID
        goal.goal_checker_id = GOAL_CHECKER_ID
        self.goal_sequence += 1
        sequence = self.goal_sequence
        future = self.follow.send_goal_async(goal)
        future.add_done_callback(lambda done: self.on_goal_response(sequence, done))

    def on_goal_response(self, sequence, future):
        try:
            handle = future.result()
        except Exception as exc:  # The action client reports transport failures here.
            self.follow_path_state = 'goal_failed: '+str(exc)[:80]
            return
        if not handle.accepted:
            if sequence == self.goal_sequence:
                self.follow_path_state = 'rejected'
            return
        if not self.driving:
            # Stopped while the request was in flight.
            handle.cancel_goal_async()
            return
        if sequence < self.goal_handle_sequence:
            return  # A newer goal already preempted this one.
        self.goal_handle, self.goal_handle_sequence = handle, sequence
        self.follow_path_state = 'following'
        handle.get_result_async().add_done_callback(
            lambda done: self.on_goal_result(sequence, done))

    def on_goal_result(self, sequence, future):
        try:
            status = future.result().status
        except Exception:
            status = GoalStatus.STATUS_UNKNOWN
        # Preempted goals are aborted by controller_server; only the latest
        # goal's outcome describes the controller.
        if sequence == self.goal_sequence:
            self.follow_path_state = STATUS_NAMES.get(status, f'status_{status}')
            if sequence == self.goal_handle_sequence:
                self.goal_handle = None

    def cancel_path(self):
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        self.goal_handle = None
        # Invalidate in-flight requests so their late acceptance is cancelled.
        self.goal_sequence += 1

    # -- Supervisor -------------------------------------------------------
    def stop(self, reason, frame='', reset=True):
        was_driving = self.driving
        self.driving = False
        self.cancel_path()
        if reset:
            self.guidance = None
            self.guidance_cache.clear()
            self.previous_path = None
            self.travel_yaw = None
        # Gate requires both fresh command and enable heartbeat. In the stop
        # case, explicitly publish zero and revoke the enable heartbeat.
        self.enable.publish(Bool(data=False))
        self.cmd.publish(Twist())
        if was_driving:
            self.follow_path_state = 'stopped'
        self.last_command = (0.0, 0.0)
        self.reference = None
        self.path.publish(self.ros_path(None, frame))
        self.publish_status(reason)

    def publish_status(self, reason):
        reference = self.reference
        linear, angular = self.last_command if self.driving else (0.0, 0.0)
        junction = self.junction.status(self.get_clock().now().nanoseconds*1e-9)
        junction['turn_command_note'] = self.turn_command_note
        key = (reason, junction['state'], junction['route'], junction['options'] and junction['options'].get('left'))
        if key != getattr(self, '_logged_key', None):
            # One line per change, so controller.log records the run.
            self._logged_key = key
            pose = self.last_pose
            self.get_logger().info(
                'state=%s junction=%s/%s route=%s(%s) request=%s options=%s stop_line=%s '
                'left_gap=%s guidance=%s pose=%s follow=%s' % (
                    reason, junction['state'], junction['reason'], junction['route'],
                    junction['route_note'], junction['turn_request'], junction['options'],
                    junction['stop_line_m'], junction['left_opening_m'],
                    None if not self.guidance_available else round(self.guidance.yaw_rad, 3),
                    None if pose is None else tuple(round(v, 2) for v in pose),
                    self.follow_path_state))
        self.status.publish(String(data=json.dumps({
            'state': reason, 'controller': CONTROLLER_PLUGIN,
            'junction': junction,
            'follow_path': self.follow_path_state,
            'linear_mps': linear, 'angular_radps': angular,
            'lane_change_cost_m': self.lane_change_cost_m,
            'divider_available': self.divider_available,
            'observed_divider_cells': self.observed_divider_cells,
            'guidance_available': self.guidance_available,
            'guidance_source': self.guidance.source if self.guidance_available else None,
            'guidance_yaw': self.guidance.yaw_rad if self.guidance_available else None,
            'reference_length_m': reference.length_m if reference else 0.0,
            'reference_plan_ms': self.plan_ms,
            'reference_pivot_rad': reference.pivot_rad if reference else 0.0,
            'planned_divider_crossings': reference.divider_crossings if reference else 0})))

    def tick(self):
        wall = time.monotonic()
        self.divider_available = False
        self.guidance_available = False
        if self.lease:
            control = read_json(self.control_path)
            desired = self.lease.update(control, wall)
            turn = control.get('turn_command')
            if turn is not None and turn != self.desktop_turn:
                self.desktop_turn = turn
                self.set_turn(turn, 'desktop')
            self.allowed = desired == 'running'
            if desired == 'stopped':
                self.stopped = True
        if not self.allowed:
            self.stop('stopped' if self.stopped else 'paused_or_not_enabled')
            return
        if not self.referee.permitted(wall):
            self.stop('referee_failed_or_score_stale')
            return
        self.convert_grids()
        sim_time = self.get_clock().now().nanoseconds * 1e-9
        if (not self.grid or not self.grid_msg or self.odom_stamp is None
                or wall-self.grid_received > 1.0 or wall-self.odom_received > 1.0
                or sim_time <= 0 or not -.2 <= sim_time-seconds(self.grid_msg.header.stamp) <= 1.0
                or not -.2 <= sim_time-self.odom_stamp <= 1.0):
            self.stop('stale_or_missing_local_map_or_odom')
            return
        if not self.follow.server_is_ready():
            self.stop('controller_server_unavailable')
            return
        frame = self.grid_msg.header.frame_id
        try:
            tf = self.tf.lookup_transform(frame, 'base_link', Time())
            age = sim_time-seconds(tf.header.stamp)
            q = tf.transform.rotation
            norm = math.sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w)
            if not -.2 <= age <= 1.0 or not math.isfinite(norm) or abs(norm-1) > .01:
                raise ValueError('stale or malformed transform')
            x, y, heading = tf.transform.translation.x, tf.transform.translation.y, yaw(q)
            self.last_pose = (x, y, heading)
            self.divider_available = bool(
                self.divider and self.divider_msg
                and self.divider_msg.header.frame_id == frame
                and wall-self.divider_received <= 1.0
                and -.2 <= sim_time-seconds(self.divider_msg.header.stamp) <= 1.0)
            boundary_available = bool(
                self.boundary and self.boundary_msg
                and wall-self.boundary_received <= 1.0
                and -.2 <= sim_time-seconds(self.boundary_msg.header.stamp) <= 1.0)
            boundary = None
            if boundary_available:
                try:
                    boundary = self.boundary_in_costmap_frame()
                except (TransformException, ValueError, OverflowError):
                    boundary_available = False
            divider = self.divider if self.divider_available else None
            sources = {'divider': self.divider_available, 'boundary': boundary_available}
            fallback = self.travel_yaw if self.travel_yaw is not None else heading
            # At a junction the previous road direction is stale: seed the
            # fit with the committed heading (road or turn-path tangent).
            expected = self.junction.expected_yaw(x, y)
            reference_yaw = (expected if expected is not None
                             else self.guidance_cache.reference(fallback, frame, wall, sources))
            # Reject fits that jump away from the travel direction: paint
            # across the road (a stop line) otherwise pivoted the robot.
            estimate = estimate_corridor_tangent(divider, x, y, reference_yaw, boundary,
                                                 max_deviation_rad=GUIDANCE_MAX_DEVIATION_RAD)
            if estimate is not None:
                self.guidance_cache.update(estimate, frame, wall)
            self.guidance = self.guidance_cache.get(frame, wall, sources)
            self.guidance_available = self.guidance is not None
            directive = self.junction.update(
                sim_time, self.grid, divider, x, y, heading, self.speed_mps,
                self.guidance.yaw_rad if self.guidance_available else None, boundary)
            if directive.new_travel_yaw is not None:
                # Leaving a junction: follow the new road, not cached paint.
                self.travel_yaw = directive.new_travel_yaw
                self.guidance_cache.clear()
                self.previous_path = None
            if directive.mode == 'hold':
                self.stop(directive.reason, frame, reset=False)
                return
            if directive.mode in ('locked', 'stop_at'):
                # The road direction recorded at the stop line, already vetted.
                guidance_yaw = directive.guidance_yaw
            elif directive.mode == 'follow':
                guidance_yaw = None
            elif self.guidance_available:
                guidance_yaw = self.guidance.yaw_rad
                self.travel_yaw = guidance_yaw
            else:
                # Never substitute the instantaneous vehicle heading for an
                # unobserved road direction: doing so allowed a sequence of
                # individually forward arcs to loop around a track corner.
                self.stop('awaiting_lane_observation', frame, reset=False)
                return
            previous = None
            if self.previous_path and self.previous_path[0] == frame and wall-self.previous_path[2] <= 1.0:
                previous = self.previous_path[1]
            started = time.monotonic()
            if directive.mode == 'follow':
                poses = directive.poses
                reference = ReferencePath(poses, 'ok', 0, sum(
                    math.hypot(b[0]-a[0], b[1]-a[1]) for a, b in zip(poses, poses[1:])))
            else:
                reference = self.planner.plan(self.grid, x, y, heading, guidance_yaw,
                                              divider, previous)
                if directive.mode == 'stop_at' and reference:
                    # End the path at the stop pose; MPPI slows to its end.
                    poses = trim_to_stop(reference.poses, directive.stop_point, self.junction.theta)
                    if len(poses) < 2:
                        self.stop('stopping_at_stop_line', frame, reset=False)
                        return
                    reference = ReferencePath(poses, 'ok', reference.divider_crossings, sum(
                        math.hypot(b[0]-a[0], b[1]-a[1]) for a, b in zip(poses, poses[1:])),
                        reference.pivot_rad)
            self.plan_ms = round(1000*(time.monotonic()-started), 1)
        except (TransformException, ValueError, OverflowError) as exc:
            self.stop('transform_unavailable: '+str(exc)[:120])
            return
        if not reference:
            self.stop(reference.reason, frame, reset=False)
            return
        self.reference = reference
        self.previous_path = (frame, reference.poses, wall)
        path = self.ros_path(reference, frame)
        self.driving = True
        self.enable.publish(Bool(data=True))
        self.send_path(path)
        self.path.publish(path)
        self.publish_status('driving' if directive.reason in ('cruise', 'junction_cleared')
                            else directive.reason)


def main():
    rclpy.init()
    node = SelfDriveController()
    try:
        while rclpy.ok() and not node.stopped:
            rclpy.spin_once(node, timeout_sec=.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RCLError:
        if rclpy.ok():
            raise
    finally:
        try:
            if rclpy.ok():
                node.stop('shutdown')
                # Let the cancel request and zero command leave the process.
                deadline = time.monotonic()+.5
                while rclpy.ok() and time.monotonic() < deadline:
                    rclpy.spin_once(node, timeout_sec=.05)
        except RCLError:
            if rclpy.ok():
                raise
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
