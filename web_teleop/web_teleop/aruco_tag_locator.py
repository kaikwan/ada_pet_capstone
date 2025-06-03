#!/usr/bin/env python3

import rclpy
import time
import tf2_ros
from tf2_ros import TransformException
from rclpy.time import Time
from math import pi

from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import TransformStamped
from std_srvs.srv import Trigger  # ✅ NEW

import hello_helpers.hello_misc as hm


class LocateArUcoTag(hm.HelloNode):
    def __init__(self):
        hm.HelloNode.__init__(self)
        hm.HelloNode.main(self, 'aruco_tag_locator', 'aruco_tag_locator', wait_for_first_pointcloud=False)

        self.joint_states_sub = self.create_subscription(JointState, '/stretch/joint_states', self.joint_states_callback, 1)
        self.transform_pub = self.create_publisher(TransformStamped, 'ArUco_transform', 10)

        self.joint_state = None

        self.min_pan_position = -3.8
        self.max_pan_position = 1.50
        self.pan_num_steps = 10
        self.pan_step_size = abs(self.min_pan_position - self.max_pan_position) / self.pan_num_steps

        self.min_tilt_position = -0.2
        self.tilt_num_steps = 3
        self.tilt_step_size = pi / 16

        self.rot_vel = 0.5  # radians per sec

        # ✅ Setup transform listener
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ✅ Add trigger service
        self.service = self.create_service(Trigger, 'start_tag_search', self.handle_find_tag)

    def joint_states_callback(self, msg):
        self.joint_state = msg

    def send_command(self, command):
        if self.joint_state is not None and command is not None:
            joint_name = command['joint']
            trajectory_goal = FollowJointTrajectory.Goal()
            trajectory_goal.trajectory.joint_names = [joint_name]

            point = JointTrajectoryPoint()

            if 'delta' in command:
                joint_index = self.joint_state.name.index(joint_name)
                joint_value = self.joint_state.position[joint_index]
                delta = command['delta']
                new_value = joint_value + delta
                point.positions = [new_value]
            elif 'position' in command:
                point.positions = [command['position']]

            point.velocities = [self.rot_vel]
            trajectory_goal.trajectory.points = [point]
            trajectory_goal.trajectory.header.stamp = self.get_clock().now().to_msg()
            trajectory_goal.trajectory.header.frame_id = 'base_link'

            self.trajectory_client.send_goal(trajectory_goal)

    def handle_find_tag(self, request, response):
        """
        ROS service callback to begin ArUco tag search.
        """
        self.get_logger().info('Received service request to find ArUco tag')

        # Small delay to ensure TF is ready
        time.sleep(1.0)

        result = self.find_tag("cat!")
        if result:
            response.success = True
            response.message = "ArUco tag found and transform published."
        else:
            response.success = False
            response.message = "ArUco tag not found."
        return response

    def find_tag(self, tag_name='docking_station'):
        pan_command = {'joint': 'joint_head_pan', 'position': self.min_pan_position}
        self.send_command(pan_command)
        tilt_command = {'joint': 'joint_head_tilt', 'position': self.min_tilt_position}
        self.send_command(tilt_command)

        for i in range(self.tilt_num_steps):
            for j in range(self.pan_num_steps):
                pan_command = {'joint': 'joint_head_pan', 'delta': self.pan_step_size}
                self.send_command(pan_command)
                time.sleep(0.2)
                try:
                    now = Time()
                    transform = self.tf_buffer.lookup_transform('base_link', tag_name, now)
                    self.get_logger().info(f"Found Requested Tag:\n{transform}")
                    pan_command = {'joint': 'joint_head_pan', 'delta': -self.pan_step_size}
                    self.send_command(pan_command)
                    self.transform_pub.publish(transform)
                    return transform
                except TransformException:
                    continue
            pan_command = {'joint': 'joint_head_pan', 'position': self.min_pan_position}
            self.send_command(pan_command)
            tilt_command = {'joint': 'joint_head_tilt', 'delta': self.tilt_step_size}
            self.send_command(tilt_command)
            time.sleep(0.25)
        self.get_logger().info("The requested tag '%s' was not found", tag_name)
        self.trajectory_client.cancel_all_goals()
        return None

def main():
    try:
        node = LocateArUcoTag()
        # No need to spin manually — HelloNode already starts spinning in a thread
        # Just wait for Ctrl+C
        while rclpy.ok():
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("Keyboard interrupt received, shutting down.")
    except Exception as e:
        print(f"Unhandled exception: {e}")
    finally:
        try:
            node.destroy_node()
        except:
            pass
        rclpy.shutdown()


if __name__ == '__main__':
    main()
