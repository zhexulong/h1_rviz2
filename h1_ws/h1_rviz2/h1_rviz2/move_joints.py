#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import threading
from sensor_msgs.msg import JointState
from math import pi
import socket


class RotateWheelNode(Node):
    def __init__(self, name):
        super().__init__(name)
        self.get_logger().info(f"node {name} init..")
        self.pub_joint_state_ = self.create_publisher(JointState, "/joint_states", 10)
        self._init_joint_states()
        self.pub_rate = self.create_rate(10)
        self.thread_ = threading.Thread(target=self._thread_pub)
        self.thread_.start()

    def _init_joint_states(self):
        self.joint_states_ = JointState()
        self.joint_states_.name = [
            "left_hip_yaw_joint",
            "left_hip_roll_joint",
            "left_hip_pitch_joint",
            "left_knee_joint",
            "left_ankle_joint",
            "right_hip_yaw_joint",
            "right_hip_roll_joint",
            "right_hip_pitch_joint",
            "right_knee_joint",
            "right_ankle_joint",
            "torso_joint",
            "left_shoulder_pitch_joint",
            "left_shoulder_roll_joint",
            "left_shoulder_yaw_joint",
            "left_elbow_joint",
            "left_hand_joint",
            "L_thumb_proximal_yaw_joint",
            "L_thumb_proximal_pitch_joint",
            "L_thumb_intermediate_joint",
            "L_thumb_distal_joint",
            "L_index_proximal_joint",
            "L_index_intermediate_joint",
            "L_middle_proximal_joint",
            "L_middle_intermediate_joint",
            "L_ring_proximal_joint",
            "L_ring_intermediate_joint",
            "L_pinky_proximal_joint",
            "L_pinky_intermediate_joint",
            "right_shoulder_pitch_joint",
            "right_shoulder_roll_joint",
            "right_shoulder_yaw_joint",
            "right_elbow_joint",
            "right_hand_joint",
            "R_thumb_proximal_yaw_joint",
            "R_thumb_proximal_pitch_joint",
            "R_thumb_intermediate_joint",
            "R_thumb_distal_joint",
            "R_index_proximal_joint",
            "R_index_intermediate_joint",
            "R_middle_proximal_joint",
            "R_middle_intermediate_joint",
            "R_ring_proximal_joint",
            "R_ring_intermediate_joint",
            "R_pinky_proximal_joint",
            "R_pinky_intermediate_joint",
        ]

        self.joint_states_.position = [0.0] * len(self.joint_states_.name)
        self.joint_states_.header.stamp = self.get_clock().now().to_msg()
        self.joint_states_.header.frame_id = ""
        self.joint_states_.velocity = []
        self.joint_states_.effort = []

    def get_angle(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        server_address = ("localhost", 12345)
        sock.bind(server_address)

        data, address = sock.recvfrom(4096)
        angle = float(data.decode())
        self.get_logger().info(
            f"received {data.decode()} from {address}, angle: {angle}"
        )
        return angle

    def _thread_pub(self):
        while rclpy.ok():
            index = self.joint_states_.name.index("left_elbow_joint")
            self.joint_states_.position[index] = self.get_angle() - pi / 2
            # self.joint_states_.position[0] += pi
            self.joint_states_.header.stamp = self.get_clock().now().to_msg()
            self.pub_joint_state_.publish(self.joint_states_)
            self.get_logger().info(f"publishing {self.joint_states_.position[index]}")
            self.pub_rate.sleep()


def main(args=None):
    rclpy.init(args=args)
    node = RotateWheelNode("move_h1_joints")
    rclpy.spin(node)
    rclpy.shutdown()
