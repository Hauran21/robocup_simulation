from rclpy.node import Node
import rclpy

from geometry_msgs.msg import Twist
from std_msgs.msg import String, Int8
from rclpy import subscription
from std_srvs.srv import Trigger
from motorctrl_interfaces.srv import TurnRobot, SearchMotors

from dynamixel_sdk import GroupSyncRead, GroupSyncWrite, COMM_SUCCESS, DXL_LOBYTE, DXL_HIBYTE, DXL_LOWORD, DXL_HIWORD
from dynamixel_utils.dynamixel import Dynamixel, OperatingMode, ReadAttribute

import numpy as np
from enum import Enum, auto
import time

from threading import Thread

class DropCtrl:
    _servo_id:int=-1
    dynamixel:Dynamixel=None

    POSITION_HOME:int = -1000
    POSITION_LEFT:int = -2850
    POSITION_RIGHT:int = 850
    DELAY_NORMAL_SEC:float = 0.3
    DELAY_EXTENDED_SEC:float = 0.3

    def __init__(self, dynamixel, servo_id=5, dropper_slot_left = 6, dropper_slot_right = 6):
        self.dynamixel = dynamixel
        self._servo_id = servo_id
        try:
            self.disable_torque()
            self._set_operation_mode()
            self.enable_torque()
            self._go_home()
        # except ConnectionError as err:
        except Exception as err:
            print(f"Dropper-Error: {err}")

        self._dropper_slot_left = dropper_slot_left
        self._dropper_slot_right = dropper_slot_right

    def enable_torque(self):
        for i in range(5):
            try:
                self.dynamixel._enable_torque(self._servo_id)
                break
            except:
                pass

    def disable_torque(self):
        for i in range(5):
            try:
                self.dynamixel._disable_torque(self._servo_id)
                break
            except:
                pass

    def _set_operation_mode(self):
        for i in range(5):
            try:
                self.dynamixel.set_operating_mode(self._servo_id, OperatingMode.EXTENDED_POSITION)
                break
            except:
                pass

    def _go_home(self):
        for i in range(5):
            try:
                self.dynamixel.set_goal_position(self._servo_id, self.POSITION_HOME)
                break
            except:
                pass

    def _drop_right_to_left(self):
        self.dynamixel.set_goal_position(self._servo_id, self.POSITION_RIGHT)
        time.sleep(self.DELAY_EXTENDED_SEC)
        self.dynamixel.set_goal_position(self._servo_id, self.POSITION_RIGHT + 2000)
        time.sleep(self.DELAY_EXTENDED_SEC)
        self.dynamixel.set_goal_position(self._servo_id, self.POSITION_RIGHT)     

    def _drop_left_to_right(self):
        self.dynamixel.set_goal_position(self._servo_id, self.POSITION_LEFT)
        time.sleep(self.DELAY_EXTENDED_SEC)
        self.dynamixel.set_goal_position(self._servo_id, self.POSITION_LEFT - 2000)
        time.sleep(self.DELAY_EXTENDED_SEC)
        self.dynamixel.set_goal_position(self._servo_id, self.POSITION_LEFT)
    
    def drop_left_sync(self, num:int=1) -> bool:
        for i in range(num):
            if self._dropper_slot_left:
                self.dynamixel.set_goal_position(self._servo_id, self.POSITION_LEFT)
                time.sleep(self.DELAY_NORMAL_SEC)
                self._go_home()
                time.sleep(self.DELAY_NORMAL_SEC)
                self._dropper_slot_left -= 1
            else:
                for j in range(5):
                    self.dynamixel.set_goal_position(self._servo_id, self.POSITION_RIGHT)
                    time.sleep(0.05)
                    self.dynamixel.set_goal_position(self._servo_id, self.POSITION_RIGHT + 100)
                    time.sleep(0.05)
                self._drop_right_to_left()
                self._dropper_slot_right -= 1
                return False
        return True

    def drop_left(self, num:int=1) -> Thread:
        thread = Thread(target = self.drop_left_sync, args = (num, ))
        thread.start()
        return thread

        
    def drop_right_sync(self, num:int=1):
        for i in range(num):
            if self._dropper_slot_right:
                self.dynamixel.set_goal_position(self._servo_id, self.POSITION_RIGHT)
                time.sleep(self.DELAY_NORMAL_SEC)
                self._go_home()
                time.sleep(self.DELAY_NORMAL_SEC)
                self._dropper_slot_right -= 1
            else: 
                for j in range(5):
                    self.dynamixel.set_goal_position(self._servo_id, self.POSITION_LEFT)
                    time.sleep(0.05)
                    self.dynamixel.set_goal_position(self._servo_id, self.POSITION_LEFT - 100)
                    time.sleep(0.05)
                self._drop_left_to_right()
                self._dropper_slot_left -= 1
                return False
        return True

    def drop_right(self, num:int=1) -> Thread:
        thread = Thread(target = self.drop_right_sync, args = (num, ))
        thread.start()
        return thread

class Drop():
    _current_rescue_packages:list[int] = [0, 0]
    _dropctrl:DropCtrl=None

    def __init__(self, node:Node):
        self._node = node
        if not self._node.has_parameter("uart_name"):
            self._node.declare_parameter("uart_name", "/dev/ttyAMA1")
        if not self._node.has_parameter("uart_baudrate"):
            self._node.declare_parameter("uart_baudrate", 57_600)
        if not self._node.has_parameter("motor_id"):
            self._node.declare_parameter("motor_id", 5)
        if not self._node.has_parameter("max_number_of_rescue_packages"):
           self._node.declare_parameter("max_number_of_rescue_packages", [6, 6])

        self._uart_name = self._node.get_parameter("uart_name").get_parameter_value().string_value
        self._uart_baudrate = self._node.get_parameter("uart_baudrate").get_parameter_value().integer_value
        self._motor_id = self._node.get_parameter("motor_id").get_parameter_value().integer_value
        self._current_rescue_packages = self._node.get_parameter("max_number_of_rescue_packages").get_parameter_value().integer_array_value
        
        self._sub_cmd_drop = self._node.create_subscription(Int8, "/cmd_drop", self._sub_cmd_drop_cb, 10)

        dynconfig = Dynamixel.Config(baudrate=self._uart_baudrate, device_name=self._uart_name).instantiate()

        self._dropctrl = DropCtrl(dynconfig, servo_id=self._motor_id)

    def _sub_cmd_drop_cb(self, msg:Int8):
        if msg.data < 0:
            self._dropctrl.drop_left()
        elif msg.data > 0:
            self._dropctrl.drop_right() 
        else:
            self._node.get_logger().warn(f"Ungültiger Drop-Befehl: {msg.data}")
    
    # def _sub_cmd_drop_cb(self, msg:Int8): 
    #     if abs(msg.data) > sum(self._current_rescue_packages):
    #         self.get_logger().error(f"Cannot drop {msg.data} packages, only {sum(self._current_rescue_packages)} available.")
        
    #     if msg.data < 0:
    #         self.get_logger().info(f"Drop {abs(msg.data)} packages on left side.")
    #     else:
    #         self.get_logger().info(f"Drop {msg.data} packages on right side.")


class DropNode(Node):
    def __init__(self, node_name: str):
        super().__init__(node_name)
        self._drop = Drop(self)

    def destroy_node(self):
        self._drop._dropctrl.disable_torque()
        return super().destroy_node()
    
def main_drop():
    node = None
    try:
        rclpy.init()
        try:
            node = DropNode("drop")
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
    main_drop()