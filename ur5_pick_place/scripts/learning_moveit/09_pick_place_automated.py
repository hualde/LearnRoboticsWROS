#!/usr/bin/env python3
"""
09_pick_place_automated.py - Versión AUTOMATIZADA del Pick and Place refinado.
Ejecuta toda la secuencia sin intervención del usuario, manteniendo la robustez.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import GripperCommand
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, PositionConstraint, OrientationConstraint, BoundingVolume
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from linkattacher_msgs.srv import AttachLink, DetachLink
import math
import time

class AutomatedPickPlace(Node):
    def __init__(self):
        super().__init__('automated_pick_place_node')
        
        # 1. Cliente de Acción para MoveGroup
        self._arm_client = ActionClient(self, MoveGroup, '/move_action')
        # 2. Cliente de Acción para Gripper
        self._gripper_client = ActionClient(self, GripperCommand, '/gripper_position_controller/gripper_cmd')
        
        # 3. Clientes de Servicio (Simulación Gazebo)
        self.attach_client = self.create_client(AttachLink, '/ATTACHLINK')
        self.detach_client = self.create_client(DetachLink, '/DETACHLINK')
        
        self.get_logger().info('⏳ Esperando al servidor de MoveIt...')
        self._arm_client.wait_for_server()
        self._gripper_client.wait_for_server()
        
        while not self.attach_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('⏳ Esperando servicio /ATTACHLINK...')
            
        self.get_logger().info('✅ MoveIt, Gripper y Servicios detectados.')

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
        primitive.dimensions = [0.005] 
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

    def move_to_pose_lin(self, x, y, z, ox, oy, oz, ow):
        """Mueve el brazo linealmente (Pilz LIN) manteniendo orientación"""
        
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "ur5_manipulator"
        goal_msg.request.start_state.is_diff = True
        
        # Usar Pilz Industrial Motion Planner (LIN)
        goal_msg.request.pipeline_id = "pilz_industrial_motion_planner"
        goal_msg.request.planner_id = "LIN"
        
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
        primitive.dimensions = [0.005] 
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
        
        self.get_logger().info(f'📏 Moviendo LINEAL a: ({x}, {y}, {z})')
        send_goal_future = self._arm_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        
        handle = send_goal_future.result()
        if not handle.accepted:
            self.get_logger().error('❌ Petición LIN rechazada')
            return False
        
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()
        
        if result.result.error_code.val == 1:
            self.get_logger().info('✨ Movimiento LIN completado')
            return True
        else:
            self.get_logger().error(f'❌ Error MoveIt LIN: {result.result.error_code.val}')
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

    def attach_object(self):
        """Llama al servicio de Gazebo para 'pegar' el cubo a la pinza"""
        req = AttachLink.Request()
        req.model1_name = 'cobot'
        req.link1_name = 'wrist_3_link'
        req.model2_name = 'cube_pick'
        req.link2_name = 'link_1'
        
        self.get_logger().info('🔗 Llamando a ATTACHLINK...')
        future = self.attach_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        
        if future.done():
            self.get_logger().info('🔗 ATTACHLINK completado.')
            return future.result()
        else:
            self.get_logger().warning('🔗 ATTACHLINK no respondió a tiempo')
            return None

    def detach_object(self):
        """Llama al servicio de Gazebo para 'soltar' el cubo"""
        req = DetachLink.Request()
        req.model1_name = 'cobot'
        req.link1_name = 'wrist_3_link'
        req.model2_name = 'cube_pick'
        req.link2_name = 'link_1'
        
        self.get_logger().info('🔓 Llamando a DETACHLINK...')
        future = self.detach_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        
        if future.done():
            self.get_logger().info('🔓 DETACHLINK completado.')
            return future.result()
        else:
            self.get_logger().warning('🔓 DETACHLINK no respondió a tiempo')
            return None

def main(args=None):
    rclpy.init(args=args)
    node = AutomatedPickPlace()
    
    # --- FASE 1: HOME ---
    node.get_logger().info('🚀 FASE 1: Moviendo a HOME')
    home_angles = [0.0, -90.0, 0.0, -90.0, 0.0, 0.0]
    node.move_to_joints(home_angles)
    time.sleep(1.0)
    
    # --- FASE 1.5: READY ---
    node.get_logger().info('🚀 FASE 1.5: Moviendo a READY (Safe Waypoint)')
    ready_angles = [0.0, -129.0, 80.0, -93.0, -90.0, 0.0]
    node.move_to_joints(ready_angles)
    time.sleep(1.0)
    
    # --- FASE 2: APPROACH ---
    node.get_logger().info('🚀 FASE 2: Aproximación (Z=0.5)')
    node.move_to_pose(0.5, 0.0, 0.5, 0.737, -0.675, 0.020, 0.006)
    time.sleep(1.0)
    
    # --- FASE 3: ABRIR PINZA ---
    node.get_logger().info('🚀 FASE 3: Abriendo Pinza')
    node.control_gripper(0.0)
    time.sleep(1.0)
    
    # --- FASE 4: PICK DESCENT ---
    node.get_logger().info('🚀 FASE 4: Bajando a posición de Pick (Z=0.379)')
    node.move_to_pose(0.5, 0.0, 0.379, 0.737, -0.675, 0.020, 0.006)
    time.sleep(1.0)
    
    # --- FASE 5: CLOSE & ATTACH ---
    node.get_logger().info('🚀 FASE 5: Cerrando pinza y pegando objeto (Attach)')
    node.control_gripper(0.3)
    node.attach_object()
    time.sleep(1.0)
    
    # --- FASE 6: LIFT ---
    node.get_logger().info('🚀 FASE 6: Subiendo brazo (Z=0.5)')
    node.move_to_pose(0.5, 0.0, 0.5, 0.737, -0.675, 0.020, 0.006)
    time.sleep(1.0)
    
    # --- FASE 7: RETURN TO READY ---
    node.get_logger().info('🚀 FASE 7: Volviendo a READY con el objeto')
    node.move_to_joints(ready_angles)
    time.sleep(1.0)
    
    # --- FASE 8: TRANSLATION (LIN) ---
    node.get_logger().info('🚀 FASE 8: Translación a zona de descarga (Evitando obstáculos)')
    node.get_logger().info('   ... moviéndose a waypoint X=0.0 ...')
    node.move_to_pose_lin(0.0, 0.3, 0.6, 0.665, -0.600, 0.310, -0.320)
    time.sleep(1.0)
    node.get_logger().info('   ... moviéndose a destino X=0.4 ...')
    node.move_to_pose_lin(0.4, 0.545, 0.6, 0.665, -0.600, 0.310, -0.320)
    time.sleep(1.0)
    
    # --- FASE 9: PLACE ---
    node.get_logger().info('🚀 FASE 9: Soltando objeto (Detach + Open)')
    node.control_gripper(0.0)
    node.detach_object()
    time.sleep(1.0)
    
    # --- FASE 10: FINAL RETURN ---
    node.get_logger().info('🚀 FASE 10: Finalizando ciclo (Volviendo a READY)')
    node.move_to_pose_lin(0.0, 0.3, 0.6, 0.665, -0.600, 0.310, -0.320)
    node.move_to_joints(ready_angles)
    
    node.get_logger().info('✨ CICLO COMPLETADO ✨')
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
