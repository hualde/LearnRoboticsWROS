#!/usr/bin/env python3
"""
05_gripper_control.py - Controlando la pinza Robotiq 85.
Aprenderás a abrir y cerrar la pinza de forma independiente al brazo.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import GripperCommand

class GripperControl(Node):
    def __init__(self):
        super().__init__('gripper_control_node')
        
        # 1. Crear el Action Client para el gripper
        # El nombre del action suele ser /gripper_position_controller/gripper_cmd 
        # definido en moveit_controllers.yaml
        self._action_client = ActionClient(
            self, 
            GripperCommand, 
            '/gripper_position_controller/gripper_cmd'
        )
        
        self.get_logger().info('⏳ Esperando al controlador de la pinza...')
        self._action_client.wait_for_server()
        self.get_logger().info('✅ Pinza lista!')

    def set_gripper_position(self, position, effort=100.0):
        """
        Envía un comando a la pinza.
        - position: 0.0 (abierto) a 0.79 (cerrado)
        - effort: Fuerza máxima (Newtons aprox)
        """
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = float(position)
        goal_msg.command.max_effort = float(effort)

        self.get_logger().info(f'⚙️  Moviendo pinza a posición: {position}')
        
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        
        handle = send_goal_future.result()
        if not handle.accepted:
            self.get_logger().error('❌ Comando de pinza rechazado')
            return

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        self.get_logger().info('✅ Pinza en posición.')

def main(args=None):
    rclpy.init(args=args)
    gripper = GripperControl()
    
    print("\n📋 CONTROL DE PINZA:")
    print("  1. ABRIR (0.0)")
    print("  2. CERRAR (0.79)")
    print("  0. Salir")
    
    try:
        while rclpy.ok():
            opcion = input("\nElige opción: ")
            if opcion == "1":
                gripper.set_gripper_position(0.0)
            elif opcion == "2":
                gripper.set_gripper_position(0.79)
            elif opcion == "0":
                break
    except KeyboardInterrupt:
        pass
        
    gripper.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
