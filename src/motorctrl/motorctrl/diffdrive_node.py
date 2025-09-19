from rclpy.node import Node
import rclpy

from geometry_msgs.msg import Twist
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

class MotorControlType(Enum):
    PWM = auto()
    POSITION_CONTROL = auto()
    EXTENDED_POSITION_CONTROL = auto()
    VELOCITY_CONTROL = auto()
    DISABLED = auto()
    UNKNOWN = auto()

class DriveCtrl:
    VELOCITY_MAX:int=455
    VELOCITY_STD:int=0
    dynamixel:Dynamixel
    
    def __init__(self, dynamixel:Dynamixel, servo_ids=[1, 2, 3, 4]):
        self.servo_ids = servo_ids
        self.dynamixel = dynamixel

        self.velocity_reader = GroupSyncRead(
            self.dynamixel.portHandler,
            self.dynamixel.packetHandler,
            ReadAttribute.VELOCITY.value,
            4)
        for id in self.servo_ids:
            self.velocity_reader.addParam(id)
        
        self.velocity_writer = GroupSyncWrite(
            self.dynamixel.portHandler,
            self.dynamixel.packetHandler,
            self.dynamixel.ADDR_GOAL_VELOCITY,
            4)
        for id in self.servo_ids:
            self.velocity_writer.addParam(id, [self.VELOCITY_MAX])


        self.hardware_error_reader = GroupSyncRead(
            self.dynamixel.portHandler,
            self.dynamixel.packetHandler,
            ReadAttribute.HARDWARE_ERROR_STATUS.value,
            1)
        for id in self.servo_ids:
            self.hardware_error_reader.addParam(id)

        self._reset()
        time.sleep(0.4) #min. 0.3 sec
        self._disable_torque()
        self.VELOCITY_MAX = self.dynamixel.get_velocity_limit(self.servo_ids[0])

        self.motor_control_state = MotorControlType.DISABLED

    def read_velocity(self):
        self.velocity_reader.txRxPacket()
        velocties = []
        for id in self.servo_ids:
            velocity = self.velocity_reader.getData(id, ReadAttribute.VELOCITY.value, 4)
            if velocity > 2 ** 31:
                velocity -= 2 ** 32
            velocties.append(velocity)
        return velocties

    def set_goal_velocity(self, vels):
        """
        :param action: list or numpy array of target velocities in range [0, 4096]
        """
        if not self.motor_control_state is MotorControlType.VELOCITY_CONTROL:
            self._set_velocity_control()
        for i, motor_id in enumerate(self.servo_ids):
            if abs(vels[i]) > self.VELOCITY_MAX:
                vels[i] = self.VELOCITY_MAX if vels[i] > 0 else -self.VELOCITY_MAX
            data_write = [DXL_LOBYTE(DXL_LOWORD(vels[i])),
                          DXL_HIBYTE(DXL_LOWORD(vels[i])),
                          DXL_LOBYTE(DXL_HIWORD(vels[i])),
                          DXL_HIBYTE(DXL_HIWORD(vels[i]))]
            self.velocity_writer.changeParam(motor_id, data_write)

        self.velocity_writer.txPacket()

    def _reset(self):
        for motor_id in self.servo_ids:
            try:
                dxl_comm_result, dxl_error = self.dynamixel.packetHandler.reboot(self.dynamixel.portHandler, motor_id)
                if dxl_comm_result == COMM_SUCCESS:
                    print("reboot was successfull.")
                else:
                    print(f"erro on reboot: {self.dynamixel.packetHandler.getTxRxResult(dxl_comm_result)}")
            except:
                pass

    def disable_torque(self):
        self.motor_control_state = MotorControlType.DISABLED
        self._disable_torque()

    def enable_torque(self):
        self.motor_control_state = MotorControlType.DISABLED
        self._enable_torque()

    def _disable_torque(self):
        print(f'disabling torque for servos {self.servo_ids}')
        for motor_id in self.servo_ids:
            for i in range(5):
                try:
                    self.dynamixel._disable_torque(motor_id)
                    break
                except:
                    pass

    def _enable_torque(self):
        print(f'enabling torque for servos {self.servo_ids}')
        for motor_id in self.servo_ids:
            for i in range(5):
                try:
                    self.dynamixel._enable_torque(motor_id)
                    break
                except:
                    pass

    def _set_velocity_control(self):
        self._disable_torque()
        for motor_id in self.servo_ids:
            for i in range(5):
                try:
                    self.dynamixel.set_operating_mode(motor_id, OperatingMode.VELOCITY)
                    break
                except:
                    pass
        self._enable_torque()
        self.motor_control_state = MotorControlType.VELOCITY_CONTROL


    def get_motor_status(self):
        self.hardware_error_reader.txRxPacket()
        status = []
        for id in self.servo_ids:
            val = self.hardware_error_reader.getData(id, ReadAttribute.HARDWARE_ERROR_STATUS.value, 1)
            status.append(val)
        return status
  

class DiffDriveNode(DiffDriveConfigNode):
    _drivectrl:DriveCtrl=None
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
        
        dynconfig = Dynamixel.Config(baudrate=self._uart_baudrate, device_name=self._uart_name).instantiate()
        
        self._drivectrl = DriveCtrl(dynconfig, servo_ids=self._motor_ids)
        self._drivectrl.set_goal_velocity([self._drivectrl.VELOCITY_STD, self._drivectrl.VELOCITY_STD, self._drivectrl.VELOCITY_STD, self._drivectrl.VELOCITY_STD])
        self._drivectrl.enable_torque()

        self._sub_cmd_vel = self.create_subscription(Twist, "/cmd_vel", self._sub_cmd_vel_callback, 10)

        # self.create_timer(1.0, self._motor_status_callback)

    def _motor_status_callback(self):
        status = self._drivectrl.get_motor_status()
        if (max(status)):
            self.get_logger().warn(f"HW-Status: {status}")

    def _sub_cmd_vel_callback(self, vel:Twist):
        robot_speed = np.array([[vel.linear.x],
                                [vel.angular.z]])
        wheel_speed = np.matmul(self._matrix_speed_conversion, robot_speed) 

        wheel_speed_left_digval = int(wheel_speed[1, 0] / self._wheel_speed_resolution)
        wheel_speed_right_digval = int(wheel_speed[0, 0] / self._wheel_speed_resolution)
        
        self.get_logger().info(f"Wheel Speed = {wheel_speed[1, 0]:.2f}. {wheel_speed[0, 0]:.2f} rad/s")

        self._drivectrl.set_goal_velocity([wheel_speed_left_digval, wheel_speed_right_digval, wheel_speed_left_digval, wheel_speed_right_digval])

    def disable_torque(self):
        self._drivectrl.set_goal_velocity([0,0,0,0])
        self._drivectrl.disable_torque()
    
    def destroy_node(self):
        self.disable_torque()
        return super().destroy_node()
        

def main_diffdrive(argv=None):

    if argv is None:
        argv = sys.argv

    node = None
    try:
        rclpy.init(args=argv)
        try:
            node = DiffDriveNode("diffdrive")
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
    main_diffdrive()