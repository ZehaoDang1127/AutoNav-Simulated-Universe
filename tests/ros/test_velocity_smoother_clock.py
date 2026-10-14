"""The native command ramp must follow ROS time, including frequency changes."""
import subprocess
from pathlib import Path
import time
import unittest

import rclpy
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import Twist
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rosgraph_msgs.msg import Clock


class VelocitySmootherClockTest(unittest.TestCase):
    def test_paused_clock_and_changed_frequency_preserve_ramp_time(self):
        rclpy.init()
        node=rclpy.create_node('velocity_smoother_clock_regression')
        clock_pub=node.create_publisher(Clock,'/clock',10)
        cmd_pub=node.create_publisher(Twist,'/cmd_vel',10)
        outputs=[]
        sub=node.create_subscription(Twist,'/cmd_vel_smoothed',lambda m:outputs.append(m),10)
        executable=Path(get_package_prefix('nav2_velocity_smoother'))/'lib/nav2_velocity_smoother/velocity_smoother'
        process=subprocess.Popen([str(executable),
                                  '--ros-args','--params-file','/host/integrations/robot/full_track_baseline.yaml'],
                                 stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
        sim=100.0
        def spin(duration):
            stop=time.monotonic()+duration
            while time.monotonic()<stop:
                m=Clock();m.clock.sec=int(sim);m.clock.nanosec=round((sim-int(sim))*1e9)
                clock_pub.publish(m)
                rclpy.spin_once(node,timeout_sec=.01)
        def complete(future):
            stop=time.monotonic()+4
            while not future.done() and time.monotonic()<stop:spin(.02)
            self.assertTrue(future.done(),'Native smoother service timed out')
            return future.result()
        try:
            change=node.create_client(ChangeState,'/velocity_smoother/change_state')
            stop=time.monotonic()+6
            while not change.service_is_ready() and time.monotonic()<stop:spin(.05)
            self.assertTrue(change.service_is_ready(),'Native smoother did not start')
            for transition in [Transition.TRANSITION_CONFIGURE,Transition.TRANSITION_ACTIVATE]:
                request=ChangeState.Request();request.transition.id=transition
                self.assertTrue(complete(change.call_async(request)).success)
            command=Twist();command.linear.x=.5;command.angular.z=1.0
            cmd_pub.publish(command);spin(.35)
            self.assertEqual(len(outputs),0,'Command ramp advanced while simulation time was paused')
            for _ in range(4):
                sim+=.05;spin(.10)
            self.assertEqual(len(outputs),4,'20 Hz smoothing did not follow simulation-clock ticks')
            self.assertAlmostEqual(outputs[-1].linear.x,.20,places=5)
            self.assertAlmostEqual(outputs[-1].angular.z,.40,places=5)
            client=node.create_client(SetParameters,'/velocity_smoother/set_parameters')
            request=SetParameters.Request()
            request.parameters=[Parameter(name='smoothing_frequency',
                value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE,double_value=40.0))]
            self.assertTrue(all(x.successful for x in complete(client.call_async(request)).results))
            count=len(outputs);spin(.25)
            self.assertEqual(len(outputs),count,'Changed-frequency timer advanced on wall time')
            command.linear.x=0.;command.angular.z=0.;cmd_pub.publish(command);spin(.05)
            for _ in range(4):
                sim+=.025;spin(.10)
            self.assertEqual(len(outputs),count+4,'40 Hz smoothing did not follow simulation-clock ticks')
            self.assertAlmostEqual(outputs[-1].linear.x,.10,places=5)
            self.assertAlmostEqual(outputs[-1].angular.z,.20,places=5)
        finally:
            process.terminate()
            try:process.wait(timeout=3)
            except subprocess.TimeoutExpired:process.kill();process.wait()
            node.destroy_node();rclpy.shutdown()


if __name__=='__main__':unittest.main()
