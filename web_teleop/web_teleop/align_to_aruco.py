#!/usr/bin/env python3

import sys
import time
from math import atan2, sqrt
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.action import ActionClient
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, TransformListener, TransformException
from tf_transformations import euler_from_quaternion, quaternion_matrix
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from action_msgs.msg import GoalStatus
from std_srvs.srv import Trigger
import time


class AlignToAruco(Node):
    def __init__(self):
        super().__init__("align_to_aruco_node")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.trajectory_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/stretch_controller/follow_joint_trajectory",
        )

        self.srv = self.create_service(Trigger, "/align_to_tag", self.handle_trigger)
        self.get_logger().info("AlignToAruco service node ready on /align_to_tag")

    def compute_difference(self, trans_base: TransformStamped, offset: float = 0.8):
        x, y, z, w = (
            trans_base.transform.rotation.x,
            trans_base.transform.rotation.y,
            trans_base.transform.rotation.z,
            trans_base.transform.rotation.w,
        )
        R = quaternion_matrix((x, y, z, w))

        P_dash = np.array([[0], [-offset], [0], [1]])
        P = np.array(
            [
                [trans_base.transform.translation.x],
                [trans_base.transform.translation.y],
                [0],
                [1],
            ]
        )
        X = np.matmul(R, P_dash)
        P_base = X + P
        base_position_x = P_base[0, 0]
        base_position_y = P_base[1, 0]

        phi = atan2(base_position_y, base_position_x)
        dist = sqrt(base_position_x**2 + base_position_y**2)
        _, _, z_rot_base = euler_from_quaternion([x, y, z, w])
        z_rot_base = -phi + z_rot_base + np.pi

        return phi, dist, z_rot_base

    def send_base_goal_blocking(self, joint_name, inc):
        point = JointTrajectoryPoint()
        point.positions = [inc]
        point.time_from_start = Duration(seconds=5.0).to_msg()

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [joint_name]
        goal.trajectory.points = [point]

        self.get_logger().info(f"[{joint_name}] Sending goal: {inc:.3f}")
        send_goal_future = self.trajectory_client.send_goal_async(goal)

        # Wait for goal to finish
        time.sleep(5.0)
        # rclpy.spin_until_future_complete(self, send_goal_future)
        # goal_handle = send_goal_future.result()

        # if not goal_handle.accepted:
        #     self.get_logger().error(f"Goal for {joint_name} was rejected!")
        #     return False

        # result_future = goal_handle.get_result_async()
        # rclpy.spin_until_future_complete(self, result_future)
        # result = result_future.result()

        # if result.status != GoalStatus.STATUS_SUCCEEDED:
        #     self.get_logger().warn(
        #         f"Goal for {joint_name} did not succeed: status {result.status}"
        #     )
        #     return False

        # self.get_logger().info(f"Goal for {joint_name} succeeded.")
        return True

    def align_to_marker(self, trans_base):
        phi, dist, final_theta = self.compute_difference(trans_base)

        if not self.send_base_goal_blocking("rotate_mobile_base", phi):
            return False
        if not self.send_base_goal_blocking("translate_mobile_base", dist):
            return False
        if not self.send_base_goal_blocking("rotate_mobile_base", final_theta):
            return False
        return True

    def handle_trigger(self, request, response):
        self.get_logger().info("Received alignment trigger")

        if not self.trajectory_client.wait_for_server(timeout_sec=5.0):
            response.success = False
            response.message = "Trajectory server unavailable."
            return response

        try:
            if self.tf_buffer.can_transform("base_link", "cat!", Time()):
                trans_base = self.tf_buffer.lookup_transform(
                    "base_link", "cat!", Time()
                )
            else:
                raise TransformException("Transform base_link → cat! not available")
        except TransformException as ex:
            self.get_logger().error(f"TF error: {ex}")
            response.success = False
            response.message = f"TF error: {ex}"
            return response

        if self.align_to_marker(trans_base):
            response.success = True
            response.message = "Successfully aligned to tag"
        else:
            response.success = False
            response.message = "Failed during movement"

        return response


def main():
    rclpy.init()
    node = AlignToAruco()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
