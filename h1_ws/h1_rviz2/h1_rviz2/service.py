#!/usr/bin/env python3
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.node import Node
import threading
# from get_joint_name import get_joint_names
from rcl_interfaces.srv import GetParameters,DescribeParameters,GetParameterTypes,ListParameters,SetParameters

class RotateWheelNode(Node):
    def __init__(self,name):
        super().__init__(name)
        self.get_logger().info(f"node {name} init..")
        self.list_params_client = self.create_client(ListParameters, "/joint_state_publisher/list_parameters")
        self.get_param_client = self.create_client(GetParameters, "/joint_state_publisher/get_parameters")
        
        
    def send_list_param_request(self):
        while rclpy.ok() and self.list_params_client.wait_for_service(1)==False:
            self.get_logger().info(f"wait for service...")
        req = ListParameters.Request()
        self.list_params_client.call_async(req).add_done_callback(self.list_params_callback)
        
    def list_params_callback(self, future):
        try:
            response = future.result().result
            self.get_logger().info(f"list_params: {response.names}")
        except Exception as e:
            self.get_logger().error(f"Service call failed {e}")
    
    
    def send_get_param_request(self):
        while rclpy.ok() and self.get_param_client.wait_for_service(1)==False:
            self.get_logger().info(f"wait for service...")
        req = GetParameters.Request()
        req.names = ["source_list"]
        self.get_param_client.call_async(req).add_done_callback(self.get_param_callback)
        
    def get_param_callback(self, future):
        try:
            response = future.result().values
            self.get_logger().info(f"get_param: {response}")
        except Exception as e:
            self.get_logger().error(f"Service call failed {e}")
        
    
    # def _thread_pub(self):
    #     while rclpy.ok():
    #         self.joint_states_.position[0] += 0.1
    #         self.joint_states_.header.stamp = self.get_clock().now().to_msg()
    #         self.pub_joint_state_.publish(self.joint_states_)
    #         self.get_logger().info(f"publishing {self.joint_states_.position[0]}")
    #         self.pub_rate.sleep()

def main(args=None):
    rclpy.init(args=args)
    node = RotateWheelNode("move_h1_joints")
    node.send_list_param_request()
    node.send_get_param_request()
    rclpy.spin(node)
    rclpy.shutdown()
