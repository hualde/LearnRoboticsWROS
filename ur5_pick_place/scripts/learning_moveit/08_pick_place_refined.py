#!/usr/bin/env python3
"""
08_pick_place_refined.py - Versión mejorada y paso a paso del Pick and Place.
En esta primera fase implementamos la estructura básica y el movimiento a HOME.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint
import math
import time

class RefinedPickPlace(Node):
    def __init__(self):
        super().__init__('refined_pick_place_node')
        
        # 1. Cliente de Acción para MoveGroup
        self._arm_client = ActionClient(self, MoveGroup, '/move_action')
        
        self.get_logger().info('⏳ Esperando al servidor de MoveIt...')
        self._arm_client.wait_for_server()
        self.get_logger().info('✅ MoveIt detectado.')

    def move_to_joints(self, angles_deg):
        """Mueve el brazo usando ángulos en grados"""
        angles_rad = [math.radians(a) for a in angles_deg]
        
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "ur5_manipulator"
        goal_msg.request.start_state.is_diff = True
        
        # Usar Pilz Industrial Motion Planner para movimientos simples (PTP)
        goal_msg.request.pipeline_id = "pilz_industrial_motion_planner"
        goal_msg.request.planner_id = "PTP"
        
        # Parámetros para aumentar robustez
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.max_velocity_scaling_factor = 0.1
        goal_msg.request.max_acceleration_scaling_factor = 0.1
        
        joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", 
                       "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
        
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
        
        self.get_logger().info(f'🤖 Moviendo a joints: {angles_deg}')
        send_goal_future = self._arm_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        
        handle = send_goal_future.result()
        if not handle.accepted:
            self.get_logger().error('❌ Petición rechazada')
            return False
        
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()
        
        if result.result.error_code.val == 1:
            self.get_logger().info('✨ Movimiento completado con éxito')
            return True
        else:
            self.get_logger().error(f'❌ Error MoveIt: {result.result.error_code.val}')
            return False

def main(args=None):
    rclpy.init(args=args)
    node = RefinedPickPlace()
    
    print("\n🚀 REFINED PICK AND PLACE - FASE 1: HOME")
    input("Presiona ENTER para mover a HOME...")
    
    # Posición HOME: [0, -90, 0, -90, 0, 0]
    home_angles = [0.0, -90.0, 0.0, -90.0, 0.0, 0.0]
    node.move_to_joints(home_angles)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
