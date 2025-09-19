import numpy as np
import yaml
from math import pi

import time
from dynamixel_sdk import GroupSyncRead, GroupSyncWrite, COMM_SUCCESS, DXL_LOBYTE, DXL_HIBYTE, DXL_LOWORD, DXL_HIWORD
from enum import Enum, auto

import rclpy
from rclpy.node import Node
import rclpy.subscription
from rclpy.parameter import Parameter

from geometry_msgs.msg import Twist
from std_msgs.msg import String, Int8
from std_srvs.srv import Trigger

from threading import Thread

from dynamixel_utils.dynamixel import Dynamixel, OperatingMode, ReadAttribute
from motorctrl.diffdrive_node import DriveCtrl, DiffDriveNode
from motorctrl.diffdrive_config_node import DiffDriveConfigNode
from motorctrl.drop_node import DropCtrl, Drop

class DiffDriveDropNode(DiffDriveNode):
    _drop:Drop=None

    def __init__(self, node_name):
        super().__init__(node_name)
        self._drop = Drop(self)
        
    def destroy_node(self):
        self.disable_torque()
        self._drop._dropctrl.disable_torque()
        return super().destroy_node()

def test_drive():
    dynconfig = Dynamixel.Config(baudrate=57_600, device_name='/dev/ttyAMA1').instantiate()
    myrobot = DriveCtrl(dynconfig, servo_ids=[1, 2, 3, 4])
    
    myrobot.disable_torque()
    
    myrobot.set_goal_velocity([myrobot.VELOCITY_MAX, myrobot.VELOCITY_MAX, myrobot.VELOCITY_MAX, myrobot.VELOCITY_MAX])
    time.sleep(1)
    myrobot.set_goal_velocity([0, 0, 0, 0])

    myrobot.disable_torque()
    

def test_dropper():
    print("Test Program: Dropper")
    dynconfig = Dynamixel.Config(baudrate=57_600, device_name='/dev/ttyAMA1').instantiate()
    mydropper = DropCtrl(dynconfig, servo_id=5)

    mydropper.drop_left().join()
    mydropper.drop_right().join()
    dynconfig.disconnect()

    print("End of Test-Program Dropper")


def main_diffdrive_drop(args=None):
    rclpy.init(args=args)
    node = DiffDriveDropNode("diffdrive_drop")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Du hast STRG+C gedrückt!")  # STRG+C abfangen
    finally:
        node.get_logger().info(f"Node {node.get_name()} wird beendet!")
        node.disable_torque()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except:
            pass

if __name__ == '__main__':
    test_drive()
    # test_dropper()


    