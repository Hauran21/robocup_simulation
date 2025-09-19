import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.task import Future
from lifecycle_msgs.msg import State as LcState
from controller_manager_msgs.srv import SetHardwareComponentState

from geometry_msgs.msg import Twist
from std_msgs.msg import String, Int8
from std_srvs.srv import Trigger

from ament_index_python.packages import get_package_share_directory

import numpy as np
import yaml
import os
from pathlib import Path
import re

from dynamixel_sdk import *  # Uses Dynamixel SDK library

from dynamixel_utils.dynamixel import Dynamixel, get_name_of_uarts
from motorctrl_interfaces.srv import TurnRobot, SearchMotors, GetGUIParameters, SetGUIParameters, GetUARTs, SetMotorConfig
from robuboard.rpi.utils import is_raspberry_pi

_STATE_BY_LABEL = {
    'unconfigured': LcState.PRIMARY_STATE_UNCONFIGURED,
    'inactive':     LcState.PRIMARY_STATE_INACTIVE,
    'active':       LcState.PRIMARY_STATE_ACTIVE,
}

def set_hw_state(node:Node, hw_name: str, label: str, timeout: float = 5.0) -> bool:
    """ Setzt den Zustand eines Hardware-Komponenten in ROS2 Control.

    :param node: RCLPy Node, der den Service aufruft.
    :param hw_name: Name der Hardware-Komponente, z.B. "RobotSystem".
    :param label: Zielzustand, z.B. 'unconfigured', 'inactive', 'active'.
    :param timeout: Zeitlimit für den Serviceaufruf in Sekunden.
    :return: True, wenn erfolgreich; False bei Fehlern.
    Beispielaufruf:
    1. Setzt den Zustand der RobotSystem-Hardware-Komponente auf 'unconfigured':
       set_hw_state(node, "RobotSystem", "unconfigured")
    2. Setzt den Zustand der RobotSystem-Hardware-Komponente auf 'active':
       set_hw_state(node, "RobotSystem", "active")
    
    CLI Beispielaufrufe:
    ros2 control set_hardware_component_state RobotSystem unconfigured
    ros2 control set_hardware_component_state RobotSystem active
    """

    cli = node.create_client(SetHardwareComponentState, '/controller_manager/set_hardware_component_state')
    if not cli.wait_for_service(timeout_sec=timeout):
        node.get_logger().error('Service /controller_manager/set_hardware_component_state nicht erreichbar.')
        return False

    req = SetHardwareComponentState.Request()
    req.name = hw_name

    # lifecycle_msgs/State verwenden!
    st = LcState()
    st.label = label  # 'unconfigured' | 'inactive' | 'active'
    # Viele Implementierungen akzeptieren label; id zusätzlich setzen ist robuster:
    st.id = _STATE_BY_LABEL.get(label, 0)
    req.target_state = st

    fut: Future = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout)
    if fut.cancelled():
        node.get_logger().error(f"HW-State '{label}' abgebrochen.")
        return False
    if fut.exception() is not None:
        node.get_logger().error(f"HW-State '{label}' fehlgeschlagen: {fut.exception()}")
        return False

    node.get_logger().info(f"Hardware '{hw_name}' → '{label}' gesetzt.")
    return True


def pkg_name_from_instance(obj) -> str:
    return type(obj).__module__.split('.')[0]

def pkg_share_from_instance(obj) -> str:
    return get_package_share_directory(pkg_name_from_instance(obj))

class DiffDriveConfigNode(Node):
    _uart_name:str
    _uart_baudrate:int
    _motor_ids:int

    def __init__(self, node_name):
        super().__init__(node_name)

        node_name = self.get_name()
        self.get_logger().info(f"Starting {node_name}...")      
        self.declare_parameters("", [("uart_name", "/dev/ttyACM0"), 
                                     ("uart_baudrate", 57_600),
                                     ("motor_ids", [1, 2, 3, 4])])
        self.declare_parameter("wheel_radius", 0.0475)
        self.declare_parameter("wheel_separation", 0.13)

        self._uart_name = self.get_parameter("uart_name").get_parameter_value().string_value
        self._uart_baudrate = self.get_parameter("uart_baudrate").get_parameter_value().integer_value
        self._motor_ids = self.get_parameter("motor_ids").get_parameter_value().integer_array_value

        self._wheel_radius = self.get_parameter("wheel_radius").get_parameter_value().double_value
        self._wheel_separation = self.get_parameter("wheel_separation").get_parameter_value().double_value

        self.get_logger().info(f"Creating Services for {node_name}...")
        self._srv_save_parameters = self.create_service(Trigger, "~/save_parameters", self._srv_save_parameters_cb)
        self._srv_turn_robot = self.create_service(TurnRobot, "~/turn_robot", self._srv_turn_robot_cb)
        self._srv_search_motors = self.create_service(SearchMotors, "~/search_motors", self._srv_search_motors_cb)
        self._srv_get_parameters = self.create_service(GetGUIParameters, "~/get_gui_parameters", self._srv_get_gui_parameters_cb)
        self._srv_set_parameters = self.create_service(SetGUIParameters, "~/set_gui_parameters", self._srv_set_gui_parameters_cb)
        self._srv_get_uarts = self.create_service(GetUARTs, "~/get_uarts", self._srv_get_uarts_cb)
        self._srv_set_motor_config = self.create_service(SetMotorConfig, "~/set_motor_config", self._srv_set_motor_config_cb)

        self._matrix_speed_conversion = np.array([[self._wheel_radius/2, self._wheel_radius/2],
                                           [self._wheel_radius/self._wheel_separation, -self._wheel_radius/self._wheel_separation]])
        self._matrix_speed_conversion = np.linalg.inv(self._matrix_speed_conversion)
        
        self.get_logger().info("The conversion matrix is %s" % self._matrix_speed_conversion)

    def _srv_save_parameters_cb(self, request:Trigger.Request, response:Trigger.Response):
        response.success = True
        response.message = "DiffDriveCtrl configuration saved successfully."

        # Jetzt alle Parameter sammeln
        params_dict = {}
        for param in self._parameters.values():
            if param.name not in ("use_sim_time",):
                params_dict[param.name] = param.value

        # Im YAML-Format speichern
        try:

            # Paket-Share-Verzeichnis ermitteln
            pkg_share = pkg_share_from_instance(self)   
            #pkg_name = __package__.split('.')[0]
            #pkg_share = get_package_share_directory(pkg_name)
            #pkg_share = get_package_share_directory('motorctrl') 
            config_dir = os.path.join(pkg_share, 'config')
            os.makedirs(config_dir, exist_ok=True)

            # Dateiname anhand des Node-Namens generieren
            file_path = os.path.join(config_dir, f"{self.get_name()}.yaml")

            with open(file_path, "w") as f:
                yaml.dump({self.get_name(): {"ros__parameters": params_dict}}, f, default_flow_style=False)
            response.message = f"DiffDriveCtrl configuration dumped to {file_path} successfully."
        except Exception as e:
            response.message = f"Failed to save parameters as YAML: {e}"
            self.get_logger().error(response.message)

        return response
    
    def _srv_turn_robot_cb(self, request:TurnRobot.Request, response:TurnRobot.Response):
        response.status = True
        self.get_logger().info(f"Turning robot by {request.angle} radians")
        return response
    
    def _srv_search_motors_cb(self, request:SearchMotors.Request, response:SearchMotors.Response):
        #ros2 contronl set_hardware_component_state RobotSystem unconfigured
        #ros2 contronl set_hardware_component_state RobotSystem active
        uart_name = request.uart_port
        baudrate = request.baudrate

        if is_raspberry_pi():
            motor_ids = self.dxl_scan(uart_name, baudrate)
        else: #dummy implementation for non-Raspberry Pi systems
            motor_ids = []  # Dummy

        if motor_ids:
            response.motor_ids = motor_ids
            response.status = True
            self.get_logger().info(f"Found Dynamixel motors with IDs: {motor_ids}")
        else:
            response.motor_ids = []
            response.status = False
            self.get_logger().warn("No Dynamixel motors found.")

        return response

    def _srv_get_gui_parameters_cb(self, request:GetGUIParameters.Request, response:GetGUIParameters.Response):

        value = self.get_parameter("motor_ids").get_parameter_value().integer_array_value
        self.get_logger().info(f"motor_ids.integer_array_value: {value}")
        
        response.uart_name = self.get_parameter("uart_name").get_parameter_value().string_value
        response.uart_baudrate = self.get_parameter("uart_baudrate").get_parameter_value().integer_value
        response.wheel_radius = self.get_parameter("wheel_radius").get_parameter_value().double_value
        response.wheel_separation = self.get_parameter("wheel_separation").get_parameter_value().double_value
        response.motor_ids = list(self.get_parameter("motor_ids").get_parameter_value().integer_array_value)
        return response

    def _srv_set_gui_parameters_cb(self, request:SetGUIParameters.Request, response:SetGUIParameters.Response):
        # Set the parameters based on the request
        self.get_logger().info(f"Setting parameters: {request}")
        #uart_str = ''.join(chr(c) for c in request.uart_name)
        self.set_parameters([
            Parameter("uart_name", Parameter.Type.STRING, request.uart_name),
            Parameter("uart_baudrate", Parameter.Type.INTEGER, request.uart_baudrate),
            Parameter("wheel_radius", Parameter.Type.DOUBLE, request.wheel_radius),
            Parameter("wheel_separation", Parameter.Type.DOUBLE, request.wheel_separation),
            Parameter("motor_ids", Parameter.Type.INTEGER_ARRAY, list(request.motor_ids))
        ])

        self.get_logger().info(f"uart_name: {self.get_parameter('uart_name').get_parameter_value().string_value}")
        self.get_logger().info(f"uart_baudrate: {self.get_parameter('uart_baudrate').get_parameter_value().integer_value}")
        self.get_logger().info(f"wheel_radius: {self.get_parameter('wheel_radius').get_parameter_value().double_value}")
        self.get_logger().info(f"wheel_separation: {self.get_parameter('wheel_separation').get_parameter_value().double_value}")
        self.get_logger().info(f"motor_ids: {self.get_parameter('motor_ids').get_parameter_value().integer_array_value}")
        # Update internal state
        self._uart_name = request.uart_name
        self._uart_baudrate = request.uart_baudrate
        self._motor_ids = request.motor_ids

        response.status = True
        return response
    
    def _srv_get_uarts_cb(self, request:GetUARTs.Request, response:GetUARTs.Response):
        uart_names = get_name_of_uarts()
        if uart_names:
            response.uart_names = uart_names
            self.get_logger().info(f"Found UART devices: {uart_names}")
        else:
            response.uart_names = ["DUMMY_UART"]
            self.get_logger().warn("No UART devices found.")

        return response
    
    def _srv_set_motor_config_cb(self, request:SetMotorConfig.Request, response:SetMotorConfig.Response):
        uart_name = request.uart_name
        baudrate = request.uart_baudrate
        motor_id = request.motor_id
        motor_id_new = request.motor_id_new
        baudrate_new = request.uart_baudrate_new

        if not uart_name:
            uart_name = self.get_parameter("uart_name").get_parameter_value().string_value
        if baudrate == 0:
            baudrate = self.get_parameter("uart_baudrate").get_parameter_value().integer_value

        response.success = True
        response.message = f"Motor ID changed from {motor_id} to {motor_id_new} and baudrate set to {baudrate_new}."

        if is_raspberry_pi():
            dxlctrl = Dynamixel(Dynamixel.Config(baudrate=baudrate, device_name=uart_name))

            if (motor_id != motor_id_new):
                if not dxlctrl.set_id(motor_id, motor_id_new):
                    response.success = False
                    response.message = f"Failed to set motor ID from {motor_id} to {motor_id_new}."
            if baudrate != baudrate_new:
                if not dxlctrl.set_baudrate(motor_id_new, baudrate_new):
                    response.success = False
                    response.message += f"Failed to set baudrate for motor ID {motor_id_new} to {baudrate_new}."
            dxlctrl.disconnect()
        self.get_logger().info(response.message)
        return response
        
    def dxl_scan(self, uart_name:str="", baudrate:int=0)-> list[int]:
        if uart_name == "":
            uart_name = self.get_parameter("uart_name").get_parameter_value().string_value
        if baudrate == 0:
            baudrate = self.get_parameter("uart_baudrate").get_parameter_value().integer_value
        
        self.get_logger().info(f"Scanning Dynamixel motors on {uart_name} at {baudrate} baudrate.")
        dxlctrl = Dynamixel(Dynamixel.Config(baudrate=baudrate, device_name=uart_name))
        motor_ids = dxlctrl.scan(uart_name, baudrate)
        if motor_ids:
            self.get_logger().info(f"Found Dynamixel motors with IDs: {motor_ids}")
        else:
            self.get_logger().warn("No Dynamixel motors found.")
        return motor_ids
    
    def destroy_node(self):
        return super().destroy_node()

def main_diffdrive_config(args=None):
    node = None
    try:
        rclpy.init()
        try:
            node = DiffDriveConfigNode("diffdrive_config")
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
