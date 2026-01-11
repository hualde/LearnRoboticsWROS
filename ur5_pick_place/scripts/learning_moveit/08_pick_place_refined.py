#!/usr/bin/env python3
"""
08_pick_place_refined.py - Versión mejorada y paso a paso del Pick and Place.
En esta primera fase implementamos la estructura básica y el movimiento a HOME.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import GripperCommand
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, PositionConstraint, OrientationConstraint, BoundingVolume
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
import math
import time

class RefinedPickPlace(Node):
    def __init__(self):
        super().__init__('refined_pick_place_node')
        
        # 1. Cliente de Acción para MoveGroup
        self._arm_client = ActionClient(self, MoveGroup, '/move_action')
        # 2. Cliente de Acción para Gripper
        self._gripper_client = ActionClient(self, GripperCommand, '/gripper_position_controller/gripper_cmd')
        
        self.get_logger().info('⏳ Esperando al servidor de MoveIt...')
        self._arm_client.wait_for_server()
        self._gripper_client.wait_for_server()
        self.get_logger().info('✅ MoveIt y Gripper detectados.')

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

    def move_to_pose(self, x, y, z, ox, oy, oz, ow):
        """Mueve el brazo a una coordenada XYZ con orientación (Pilz PTP)"""
        
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "ur5_manipulator"
        goal_msg.request.start_state.is_diff = True
        
        # Usar Pilz Industrial Motion Planner (PTP)
        goal_msg.request.pipeline_id = "pilz_industrial_motion_planner"
        goal_msg.request.planner_id = "PTP"
        
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.max_velocity_scaling_factor = 0.1
        goal_msg.request.max_acceleration_scaling_factor = 0.1
        
        constraints = Constraints()
        # Posición
        pc = PositionConstraint()
        pc.header.frame_id = "base_link"
        pc.link_name = "tool0"
        bv = BoundingVolume()
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [0.005] # Tolerancia 5mm
        bv.primitives.append(primitive)
        target_pose = Pose()
        target_pose.position.x = float(x)
        target_pose.position.y = float(y)
        target_pose.position.z = float(z)
        bv.primitive_poses.append(target_pose)
        pc.constraint_region = bv
        pc.weight = 1.0
        constraints.position_constraints.append(pc)
        
        # Orientación
        oc = OrientationConstraint()
        oc.header.frame_id = "base_link"
        oc.link_name = "tool0"
        oc.orientation.x = float(ox)
        oc.orientation.y = float(oy)
        oc.orientation.z = float(oz)
        oc.orientation.w = float(ow)
        oc.absolute_x_axis_tolerance = 0.01
        oc.absolute_y_axis_tolerance = 0.01
        oc.absolute_z_axis_tolerance = 0.01
        oc.weight = 1.0
        constraints.orientation_constraints.append(oc)
        
        goal_msg.request.goal_constraints.append(constraints)
        
        self.get_logger().info(f'📍 Moviendo a Poses: ({x}, {y}, {z})')
        send_goal_future = self._arm_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        
        handle = send_goal_future.result()
        if not handle.accepted:
            self.get_logger().error('❌ Petición rechazada (Pose)')
            return False
        
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()
        
        if result.result.error_code.val == 1:
            self.get_logger().info('✨ Movimiento Pose completado')
            return True
        else:
            self.get_logger().error(f'❌ Error MoveIt Pose: {result.result.error_code.val}')
            return False

    def control_gripper(self, position):
        """Abre o cierra la pinza (0.0 = Abierto, 0.8 = Cerrado)"""
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = float(position)
        goal_msg.command.max_effort = 100.0
        
        self.get_logger().info(f'🖐 Gripper: {position}...')
        send_goal_future = self._gripper_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=5.0)
        
        if not send_goal_future.done():
            self.get_logger().error('❌ Gripper Timeout')
            return False

        handle = send_goal_future.result()
        if not handle.accepted:
            self.get_logger().error('❌ Gripper Rechazado')
            return False
        
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        self.get_logger().info('✨ Gripper OK')
        return True

def main(args=None):
    rclpy.init(args=args)
    node = RefinedPickPlace()
    
    print("\n🚀 REFINED PICK AND PLACE - FASE 1: HOME")
    input("Presiona ENTER para mover a HOME...")
    
    # --- FASE 1 Posición HOME: [0, -90, 0, -90, 0, 0]
    home_angles = [0.0, -90.0, 0.0, -90.0, 0.0, 0.0]
    node.move_to_joints(home_angles)
    
    # --- FASE 1.5: PUNTO INTERMEDIO (READY) ---
    # Evitamos colisión con el muro yendo a una posición elevada y plegada
    print("\n⚠️  Apunto de mover a READY para evitar colisiones...")
    ready_angles = [0.0, -129.0, 80.0, -93.0, -90.0, 0.0]
    node.move_to_joints(ready_angles)
    
    # --- FASE 2: APROXIMACIÓN (APPROACH) ---
    print("\n🚀 FASE 2: APROXIMACIÓN")
    input("Presiona ENTER para mover a la posición de APROXIMACIÓN (Z=0.5)...")
    
    # Coordenadas: [0.5, 0.0, 0.5]
    # Orientación: [0.737, -0.675, 0.020, 0.006]
    node.move_to_pose(0.5, 0.0, 0.5, 0.737, -0.675, 0.020, 0.006)
    
    # --- FASE 3: ABRIR PINZA ---
    print("\n🚀 FASE 3: ABRIR PINZA")
    input("Presiona ENTER para ABRIR la pinza...")
    node.control_gripper(0.0) # 0.0 = Abierto
    
    # --- FASE 4: BAJAR A COGER (PICK) ---
    print("\n🚀 FASE 4: BAJAR (PICK)")
    input("Presiona ENTER para BAJAR a la posición de agarre (Z=0.379)...")
    # Misma X, Y, Orientación, pero bajamos Z a 0.379
    node.move_to_pose(0.5, 0.0, 0.379, 0.737, -0.675, 0.020, 0.006)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
