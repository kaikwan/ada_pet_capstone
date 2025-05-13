#!/usr/bin/env python3

# This node uses the tf_listener tutorial from Stretch Tutorials
# Head should always point at ArUco marker even while moving and aligning

import sys
import time
from math import atan2, sqrt

import numpy as np
import rclpy
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Transform, TransformStamped
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf_transformations import (euler_from_quaternion, quaternion_from_euler,
                                quaternion_matrix, quaternion_about_axis,
                                quaternion_multiply)
from trajectory_msgs.msg import JointTrajectoryPoint


# Align x-axis of base_link to x-axis of marker
# Robot should first align x-axes then minimize distance along x-axis and then along y-axis
class AlignToAruco(Node):
    def __init__(self, node, trans_base: TransformStamped, offset=0.75, marker_yaw_offset_deg=0.0):
        self.trans_base = trans_base
        self.offset = offset
        self.node = node

        # Store marker yaw correction in radians
        self.marker_yaw_offset_rad = np.deg2rad(marker_yaw_offset_deg)

        self.trajectory_client = ActionClient(
            self.node,
            FollowJointTrajectory,
            '/stretch_controller/follow_joint_trajectory'
        )

        server_reached = self.trajectory_client.wait_for_server(timeout_sec=60.0)
        if not server_reached:
            self.node.get_logger().error('Unable to connect to trajectory server.')
            sys.exit()


    def compute_difference(self):
        # Extract quaternion from TF
        x = self.trans_base.transform.rotation.x
        y = self.trans_base.transform.rotation.y
        z = self.trans_base.transform.rotation.z
        w = self.trans_base.transform.rotation.w

        self.node.get_logger().info(f"Original quaternion (x, y, z, w): ({x}, {y}, {z}, {w})")

        # Rotation matrix from quaternion
        R = quaternion_matrix((x, y, z, w))
        self.node.get_logger().info(f"Rotation matrix R:\n{R}")

        # Offset vector in base frame
        P_dash = np.array([[0], [-self.offset], [0], [1]])
        self.node.get_logger().info(f"Offset vector P_dash (camera offset):\n{P_dash}")

        # Translation (marker position in base_link frame)
        P = np.array([[self.trans_base.transform.translation.x],
                    [self.trans_base.transform.translation.y],
                    [0],
                    [1]])
        self.node.get_logger().info(f"Original translation vector P (marker wrt base_link):\n{P}")

        # Apply offset using rotation
        X = np.matmul(R, P_dash)
        self.node.get_logger().info(f"Offset vector in world frame (R * P_dash):\n{X}")

        # Final adjusted base position
        P_base = X + P
        P_base[3, 0] = 1  # Ensure homogeneous coordinates
        self.node.get_logger().info(f"Adjusted base position P_base = X + P:\n{P_base}")

        # Extract X, Y for computation
        base_position_x = P_base[0, 0]
        base_position_y = P_base[1, 0]
        self.node.get_logger().info(f"base_position_x: {base_position_x}, base_position_y: {base_position_y}")

        # Compute angle to rotate (phi) and distance to move
        phi = atan2(base_position_y, base_position_x)
        dist = sqrt(pow(base_position_x, 2) + pow(base_position_y, 2))
        self.node.get_logger().info(f"Computed phi (rotation angle to face marker): {phi} rad = {np.degrees(phi)} deg")
        self.node.get_logger().info(f"Computed distance to marker: {dist} meters")

        # Extract existing rotation and compute final orientation adjustment
        x_rot_base, y_rot_base, z_rot_base = euler_from_quaternion([x, y, z, w])
        self.node.get_logger().info(f"Euler angles from marker quaternion: x: {x_rot_base}, y: {y_rot_base}, z: {z_rot_base}")
        
        z_rot_base = -phi + z_rot_base + np.pi/2
        self.node.get_logger().info(f"Final z-axis rotation adjustment (z_rot_base): {z_rot_base} rad = {np.degrees(z_rot_base)} deg")

        return phi, dist, z_rot_base

    
    def align_to_marker(self):
        phi, dist, final_theta = self.compute_difference()

        def send_base_goal(joint_name, inc):
            point = JointTrajectoryPoint()
            point.positions = [inc]
            point.time_from_start = Duration(seconds=5.0).to_msg()

            trajectory_goal = FollowJointTrajectory.Goal()
            trajectory_goal.trajectory.joint_names = [joint_name]
            trajectory_goal.trajectory.points = [point]

            # self.trajectory_client.send_goal_async(trajectory_goal)
            self.node.get_logger().info(f"Sent goal to {joint_name} with increment {inc:.3f}")

        def send_base_goal_blocking(joint_name, inc):
            point = JointTrajectoryPoint()
            point.positions = [inc]
            point.time_from_start = Duration(seconds=5.0).to_msg()

            trajectory_goal = FollowJointTrajectory.Goal()
            trajectory_goal.trajectory.joint_names = [joint_name]
            trajectory_goal.trajectory.points = [point]

            self.node.get_logger().info(f"Sending goal to {joint_name} with increment {inc:.3f}")
            send_goal_future = self.trajectory_client.send_goal_async(trajectory_goal)
            rclpy.spin_until_future_complete(self.node, send_goal_future)
            goal_handle = send_goal_future.result()

            if not goal_handle.accepted:
                self.node.get_logger().error(f"Goal for {joint_name} was rejected!")
                return

            self.node.get_logger().info(f"Goal accepted for {joint_name}. Waiting for result...")
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self.node, result_future)
            result = result_future.result()

            if result.status != 4:
                self.node.get_logger().warn(f"Goal for {joint_name} did not succeed: status {result.status}")
            else:
                self.node.get_logger().info(f"Goal for {joint_name} succeeded!")


        # Step 1: Rotate to face the marker
        send_base_goal_blocking('rotate_mobile_base', phi)

        # Step 2: Move forward to reach the desired offset-aligned position
        send_base_goal_blocking('translate_mobile_base', dist)

        # Step 3: Final rotation to face the marker (marker x-axis)
        send_base_goal_blocking('rotate_mobile_base', final_theta)

def main():
    rclpy.init()
    node = Node("align_to_aruco_node")

    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, node)

    timeout_sec = 10.0
    start_time = time.time()

    required_transforms = [
        ('base_right', 'base_link'),
    ]

    node.get_logger().info('Waiting for required transforms...')
    trans_base = None

    while rclpy.ok():
        missing = []

        try:
            if tf_buffer.can_transform('base_link', 'base_right', Time()):
                trans_base = tf_buffer.lookup_transform('base_link', 'base_right', Time())
            else:
                missing.append('base_link → base_right')
        except TransformException as ex:
            node.get_logger().warn(f"Error during transform lookup: {ex}")
            missing.append("exception")

        if not missing:
            node.get_logger().info('All required transforms are available!')
            node.get_logger().info(
                f'Transform base_link → base_right: '
                f'translation=({trans_base.transform.translation.x:.3f}, '
                f'{trans_base.transform.translation.y:.3f}, '
                f'{trans_base.transform.translation.z:.3f}), '
                f'rotation=({trans_base.transform.rotation.x:.3f}, '
                f'{trans_base.transform.rotation.y:.3f}, '
                f'{trans_base.transform.rotation.z:.3f}, '
                f'{trans_base.transform.rotation.w:.3f}) '
            )
            break

        elapsed = time.time() - start_time
        if elapsed > timeout_sec:
            node.get_logger().error(f"Timeout waiting for transforms: {', '.join(missing)}")
            rclpy.shutdown()
            return

        node.get_logger().info(
            f"Still waiting on: {', '.join(missing)} ({timeout_sec - elapsed:.1f}s remaining)")
        rclpy.spin_once(node, timeout_sec=0.1)

    # Create aligner and run
    align = AlignToAruco(
        node=node,
        trans_base=trans_base,
        offset=0.2,
        marker_yaw_offset_deg=0.0
    )
    align.align_to_marker()

    rclpy.shutdown()



if __name__ == '__main__':
    main()
