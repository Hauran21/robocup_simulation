#!/usr/bin/env python3
import rclpy
from tf_transformations import quaternion_from_euler

from motorctrl.diffdrive_config_node import DiffDriveConfigNode, set_hw_state

class DiffDriveConfigController(DiffDriveConfigNode):
    def __init__(self, node_name="diffdrive_config_controller"):
        super().__init__(node_name)
        
    def _srv_search_motors_cb(self, request, response):
        set_hw_state(self, "RobotSystem", "unconfigured")
        response = super()._srv_search_motors_cb(request, response)
        set_hw_state(self, "RobotSystem", "configured")
        return response
    
    def _srv_set_motor_config_cb(self, request, response):
        set_hw_state(self, "RobotSystem", "unconfigured")
        response = super()._srv_set_motor_config_cb(request, response)
        set_hw_state(self, "RobotSystem", "configured")
        return response
    
    def save_ros2_control_parameters(self):
        """
        Speichert die ROS2-Control-Parameter in einer YAML-Datei.
        """
        params_dict = {}
    
    def destroy_node(self):
        return super().destroy_node()


def main():
    node = None
    try:
        rclpy.init()
        try:
            node = DiffDriveConfigController("diffdrive_config_controller")
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
    main()
