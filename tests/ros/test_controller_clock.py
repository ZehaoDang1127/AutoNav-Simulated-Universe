"""Native FollowPath must wait for ROS-clock intervals and cancel while paused."""
from pathlib import Path
import subprocess
import tempfile
import time
import unittest
import yaml

import rclpy
from ament_index_python.packages import get_package_prefix
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rosgraph_msgs.msg import Clock
from tf2_ros import TransformBroadcaster


class ControllerClockTest(unittest.TestCase):
    def test_native_control_cadence_and_paused_cancellation(self):
        rclpy.init()
        node = rclpy.create_node('controller_clock_regression')
        clock_pub = node.create_publisher(Clock, '/clock', 10)
        odom_pub = node.create_publisher(Odometry, '/clock_test_odom', 10)
        broadcaster = TransformBroadcaster(node)
        outputs = []
        sub = node.create_subscription(Twist, '/clock_test_cmd', outputs.append, 10)
        profile = {
            'controller_server': {'ros__parameters': {
                'use_sim_time': True, 'controller_frequency': 10.0,
                'odom_topic': '/clock_test_odom', 'controller_plugins': ['FollowPath'],
                'progress_checker_plugin': 'progress_checker',
                'progress_checker': {'plugin': 'nav2_controller::SimpleProgressChecker',
                    'required_movement_radius': .05, 'movement_time_allowance': 30.0},
                'goal_checker_plugins': ['goal_checker'],
                'goal_checker': {'plugin': 'nav2_controller::SimpleGoalChecker',
                    'xy_goal_tolerance': .1, 'yaw_goal_tolerance': .1},
                'FollowPath': {'plugin': 'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController',
                    'desired_linear_vel': .2, 'use_collision_detection': False}}},
            'local_costmap': {'local_costmap': {'ros__parameters': {
                'use_sim_time': True, 'global_frame': 'odom', 'robot_base_frame': 'base_link',
                'rolling_window': True, 'width': 6, 'height': 6, 'resolution': .1,
                'update_frequency': 20.0, 'publish_frequency': 20.0,
                'track_unknown_space': False, 'plugins': ['obstacle_layer'], 'robot_radius': .3,
                'obstacle_layer': {'plugin': 'nav2_costmap_2d::ObstacleLayer', 'enabled': False}}}}
        }
        sim = 100.0

        def spin(duration):
            stop = time.monotonic() + duration
            while time.monotonic() < stop:
                clock = Clock()
                clock.clock.sec = int(sim)
                clock.clock.nanosec = round((sim-int(sim))*1e9)
                clock_pub.publish(clock)
                t = TransformStamped()
                t.header.stamp = clock.clock
                t.header.frame_id = 'odom'
                t.child_frame_id = 'base_link'
                t.transform.rotation.w = 1.0
                broadcaster.sendTransform(t)
                odom = Odometry()
                odom.header.stamp = clock.clock
                odom.header.frame_id = 'odom'
                odom.child_frame_id = 'base_link'
                odom.pose.pose.orientation.w = 1.0
                odom_pub.publish(odom)
                rclpy.spin_once(node, timeout_sec=.005)

        def complete(future):
            stop = time.monotonic()+5
            while not future.done() and time.monotonic() < stop:
                spin(.02)
            self.assertTrue(future.done(), 'Native controller request timed out')
            return future.result()

        with tempfile.TemporaryDirectory(prefix='controller-clock-') as folder:
            params = Path(folder)/'params.yaml'
            params.write_text(yaml.safe_dump(profile))
            log_path = Path(folder)/'controller.log'
            executable = Path(get_package_prefix('nav2_controller'))/'lib/nav2_controller/controller_server'
            with log_path.open('w') as log:
                process = subprocess.Popen([str(executable), '--ros-args', '--params-file', str(params),
                    '-r', 'cmd_vel:=/clock_test_cmd'], stdout=log, stderr=log)
                try:
                    change = node.create_client(ChangeState, '/controller_server/change_state')
                    stop = time.monotonic()+8
                    while not change.service_is_ready() and time.monotonic() < stop:
                        spin(.05)
                    self.assertTrue(change.service_is_ready(), 'Native controller did not start')
                    for transition in [Transition.TRANSITION_CONFIGURE, Transition.TRANSITION_ACTIVATE]:
                        request = ChangeState.Request()
                        request.transition.id = transition
                        self.assertTrue(complete(change.call_async(request)).success)
                    action = ActionClient(node, FollowPath, '/follow_path')
                    self.assertTrue(action.wait_for_server(timeout_sec=3))
                    goal = FollowPath.Goal()
                    goal.controller_id = 'FollowPath'
                    goal.goal_checker_id = 'goal_checker'
                    goal.path.header.frame_id = 'odom'
                    for x in [0.0, 1.0, 2.0]:
                        p = PoseStamped()
                        p.header.frame_id = 'odom'
                        p.pose.position.x = x
                        p.pose.orientation.w = 1.0
                        goal.path.poses.append(p)
                    handle = complete(action.send_goal_async(goal))
                    self.assertTrue(handle.accepted)
                    spin(.3)
                    self.assertTrue(outputs, 'Native controller did not produce its first command')
                    count = len(outputs)
                    spin(.35)
                    self.assertEqual(len(outputs), count,
                        'Control loop advanced while ROS time was paused')
                    sim += .05
                    spin(.12)
                    self.assertEqual(len(outputs), count,
                        'Native controller fired before its 100 ms ROS deadline')
                    sim += .05
                    spin(.12)
                    self.assertEqual(len(outputs), count+1)
                    for _ in range(4):
                        sim += .1
                        spin(.12)
                    self.assertEqual(len(outputs), count+5,
                        'Native controller did not produce one command per 100 ms ROS-clock interval')
                    self.assertGreater(outputs[-1].linear.x, .1)
                    response = complete(handle.cancel_goal_async())
                    self.assertTrue(response.goals_canceling)
                    result = complete(handle.get_result_async())
                    self.assertEqual(result.status, GoalStatus.STATUS_CANCELED)
                    spin(.1)
                    self.assertEqual(outputs[-1].linear.x, 0.0)
                    self.assertEqual(outputs[-1].angular.z, 0.0)
                except Exception:
                    log.flush()
                    print(log_path.read_text())
                    raise
                finally:
                    process.terminate()
                    try:
                        process.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait()
                    node.destroy_node()
                    rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
