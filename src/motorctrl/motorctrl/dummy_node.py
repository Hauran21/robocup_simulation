from rclpy.node import Node
import rclpy

from geometry_msgs.msg import Twist
from std_msgs.msg import String, Int8
from rclpy import subscription
from std_srvs.srv import Trigger
from motorctrl_interfaces.srv import TurnRobot, SearchMotors
from motorctrl.diffdrive_config_node import DiffDriveConfigNode

from dynamixel_sdk import GroupSyncRead, GroupSyncWrite, COMM_SUCCESS, DXL_LOBYTE, DXL_HIBYTE, DXL_LOWORD, DXL_HIWORD
from dynamixel_utils.dynamixel import Dynamixel, OperatingMode, ReadAttribute

import sys
import numpy as np
from enum import Enum, auto
import time
from math import pi

class DummyNode(DiffDriveConfigNode):
    _sub_cmd_vel:rclpy.subscription.Subscription

    def __init__(self, node_name):
        super().__init__(node_name)
    
        self.declare_parameter("wheel_speed_resolution", 0.229*(2*pi/60.0))

        self._uart_name = self.get_parameter("uart_name").get_parameter_value().string_value
        self._uart_baudrate = self.get_parameter("uart_baudrate").get_parameter_value().integer_value
        self._motor_ids = self.get_parameter("motor_ids").get_parameter_value().integer_array_value

        self._wheel_radius = self.get_parameter("wheel_radius").get_parameter_value().double_value
        self._wheel_separation = self.get_parameter("wheel_separation").get_parameter_value().double_value

        self._wheel_speed_resolution = self.get_parameter("wheel_speed_resolution").get_parameter_value().double_value
        
        self._sub_cmd_vel = self.create_subscription(Twist, "/cmd_vel", self._sub_cmd_vel_callback, 10)

        self._sub_cmd_drop = self.create_subscription(Int8, "/cmd_drop", self._sub_cmd_drop_cb, 10)

    def _sub_cmd_vel_callback(self, vel:Twist):
        robot_speed = np.array([[vel.linear.x],
                                [vel.angular.z]])
        wheel_speed = np.matmul(self._matrix_speed_conversion, robot_speed) 

        self.get_logger().info(f"Wheel Speed = {wheel_speed[1, 0]:.2f}. {wheel_speed[0, 0]:.2f} rad/s")

    def _sub_cmd_drop_cb(self, msg:Int8):
        if msg.data < 0:
            self.get_logger().warn(f"Drop-Befehl Links: {msg.data}")
        elif msg.data > 0:
            self.get_logger().warn(f"Drop-Befehl Rechts: {msg.data}")
        else:
            self.get_logger().warn(f"Ungültiger Drop-Befehl: {msg.data}")
        

def main_dummy(argv=None):

    if argv is None:
        argv = sys.argv

    node = None
    try:
        rclpy.init(args=argv)
        try:
            node = DummyNode("motorctrl_dummy")
        except Exception as e:
            print(f"Fehler beim Erstellen des Nodes: {e}")
            return

        rclpy.spin(node)

    except KeyboardInterrupt:
        print("Sie haben STRG+C gedrückt!")

    finally:
        if node is not None:
            if rclpy.ok():
                node.get_logger().info(f"Node {node.get_name()} wird beendet!")
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main_dummy()