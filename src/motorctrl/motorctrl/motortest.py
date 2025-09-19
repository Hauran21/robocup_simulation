import numpy as np
from awdbot.dynamixel_utils.dynamixel_utils.dynamixel import Dynamixel, OperatingMode, ReadAttribute
import time
from dynamixel_sdk import GroupSyncRead, GroupSyncWrite, DXL_LOBYTE, DXL_HIBYTE, DXL_LOWORD, DXL_HIWORD
from enum import Enum, auto
from typing import Union

import signal
from threading import Thread

import sys

class MotorControlType(Enum):
    PWM = auto()
    POSITION_CONTROL = auto()
    EXTENDED_POSITION_CONTROL = auto()
    VELOCITY_CONTROL = auto()
    DISABLED = auto()
    UNKNOWN = auto()

class MotorTest:
    _servo_id:int=-1
    dynamixel:Dynamixel=None

    def __init__(self, dynamixel:Dynamixel, servo_id=1):
        self.dynamixel = dynamixel
        self._servo_id = servo_id

        self.disable_torque()
        self._set_operation_mode()
        self.enable_torque()

        print("Velocity Limit: ", self.dynamixel.get_velocity_limit(servo_id))

    def enable_torque(self):
        self.dynamixel._enable_torque(self._servo_id)

    def disable_torque(self):
        self.dynamixel._disable_torque(self._servo_id)

    def _set_operation_mode(self):
        self.dynamixel.set_operating_mode(self._servo_id, OperatingMode.VELOCITY) #EXTENDED_POSITION)

    def _test(self):
        self.dynamixel.set_goal_velocity(self._servo_id, 200)
        time.sleep(1)
        self.dynamixel.set_goal_velocity(self._servo_id, 0)  
        # self.dynamixel.set_goal_velocity(self.servo_id, 200)
        # time.sleep(1)
        # self.dynamixel.set_goal_position(self._servo_id, -500)
        # time.sleep(1)
        # self.dynamixel.set_goal_position(self._servo_id, 0)     

def motor_test(id=1):
    print("Motor-Test")
    dynconfig = Dynamixel.Config(baudrate=57_600, device_name='/dev/ttyAMA1').instantiate()

    #dynconfig = Dynamixel.Config(baudrate=57_600, device_name='/dev/ttyACM1').instantiate()

    mymotor = MotorTest(dynconfig, servo_id=id)
    mymotor._test()
    dynconfig.disconnect()

    print("End of Test-Program")

def main(args=None):
    if len(sys.argv) == 1:
        motor_test()
    else:
        motor_test(int(sys.argv[1]))

if __name__ == '__main__':
    main()


    