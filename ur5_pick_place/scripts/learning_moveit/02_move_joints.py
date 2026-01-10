#!/usr/bin/env python3
"""
02_move_joints.py - Controlando los motores uno a uno.
Este es un script FUNCIONAL que puedes ejecutar para mover el robot.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint
import math

class MoveJoints(Node):
    def __init__(self):
        super().__init__('move_joints_node')
        self._action_client = ActionClient(self, MoveGroup, '/move_action')
        self.get_logger().info('⏳ Esperando a MoveIt...')
        self._action_client.wait_for_server()

    def move_to_joints(self, angles_degrees):
        """
        Mueve el robot especificando los 6 ángulos en GRADOS.
        """
        # Convertimos grados a radianes (que es lo que usa ROS)
        angles_rad = [math.radians(a) for a in angles_degrees]
        
        # 1. Crear el Goal
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "ur5_manipulator"
        
        # 2. Definir los nombres de los joints del UR5
        joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint"
        ]
        
        # 3. Crear las "Constraints" (Restricciones)
        # Básicamente le decimos: "Quiero que el estado final cumpla estas condiciones"
        constraints = Constraints()
        for i in range(6):
            jc = JointConstraint()
            jc.joint_name = joint_names[i]
            jc.position = angles_rad[i]
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        
        goal_msg.request.goal_constraints.append(constraints)
        
        # 4. Enviar y esperar
        self.get_logger().info('⚙️ Enviando trayectoria...')
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        
        handle = send_goal_future.result()
        if not handle.accepted:
            self.get_logger().error('❌ Movimiento rechazado')
            return

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        self.get_logger().info('✅ ¡Movimiento finalizado con éxito!')

def main(args=None):
    rclpy.init(args=args)
    node = MoveJoints()
    
    # Ejemplo: Una posición en "L"
    # [Hombro, Elevación, Codo, Muñeca1, Muñeca2, Muñeca3]
    pos_l = [0, -90, 90, -90, -90, 0]
    
    print(f"Moviendo a configuración de joints: {pos_l}")
    node.move_to_joints(pos_l)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
