#!/usr/bin/env python3
"""
06_sequential_pick_place.py - El "Gran Final".
Combinamos movimientos de joints, cartesianos y gripper en una secuencia lógica.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from control_msgs.action import GripperCommand
from geometry_msgs.msg import Pose, Point
from moveit_msgs.msg import Constraints, JointConstraint, PositionConstraint, OrientationConstraint, BoundingVolume
from shape_msgs.msg import SolidPrimitive
from linkattacher_msgs.srv import AttachLink, DetachLink
import math
import time

class SequentialPickPlace(Node):
    def __init__(self):
        super().__init__('sequential_pick_place_node')
        
        # 1. Clientes de Acción (Movimiento)
        self._arm_client = ActionClient(self, MoveGroup, '/move_action')
        self._gripper_client = ActionClient(self, GripperCommand, '/gripper_position_controller/gripper_cmd')
        
        # 2. Clientes de Servicio (Simulación Gazebo)
        self.attach_client = self.create_client(AttachLink, '/ATTACHLINK')
        self.detach_client = self.create_client(DetachLink, '/DETACHLINK')
        
        self.get_logger().info('⏳ Esperando a los controladores y servicios...')
        self._arm_client.wait_for_server()
        self._gripper_client.wait_for_server()
        
        while not self.attach_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('⏳ Esperando servicio /ATTACHLINK...')
        
        self.get_logger().info('✅ Sistema completo preparado!')

    # --- MÉTODOS DE APOYO (REUTILIZADOS) ---

    def move_to_joints(self, angles_deg):
        """Mueve el brazo usando ángulos (de los scripts anteriores)"""
        angles_rad = [math.radians(a) for a in angles_deg]
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "ur5_manipulator"
        # 💡 MUY IMPORTANTE: Usar el estado ACTUAL del robot como inicio
        goal_msg.request.start_state.is_diff = True
        
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
        
        send_goal_future = self._arm_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        handle = send_goal_future.result()
        if not handle.accepted:
            self.get_logger().error('❌ Petición de joints rechazada')
            return False
        
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()
        
        if result.result.error_code.val == 1:
            return True
        else:
            self.get_logger().error(f'❌ Error en movimiento de joints: {result.result.error_code.val}')
            return False

    def move_to_xyz(self, x, y, z, ox=1.0, oy=0.0, oz=0.0, ow=0.0):
        """Mueve el brazo a una coordenada XYZ con orientación opcional"""
        # 💡 Pausa de seguridad para que la física de Gazebo se estabilice tras el ATTACH
        time.sleep(0.5)

        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "ur5_manipulator"
        goal_msg.request.workspace_parameters.header.frame_id = "base_link"
        goal_msg.request.start_state.is_diff = True
        
        # 💡 Máxima persistencia para el planificador
        goal_msg.request.num_planning_attempts = 50 
        goal_msg.request.allowed_planning_time = 15.0
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
        primitive.dimensions = [0.03] # Subimos a 3cm de tolerancia para mayor robustez
        bv.primitives.append(primitive)
        target_pose = Pose()
        target_pose.position.x = x
        target_pose.position.y = y
        target_pose.position.z = z
        bv.primitive_poses.append(target_pose)
        pc.constraint_region = bv
        pc.weight = 1.0
        constraints.position_constraints.append(pc)
        
        # Orientación (Si ox no es None)
        if ox is not None:
            oc = OrientationConstraint()
            oc.header.frame_id = "base_link"
            oc.link_name = "tool0"
            oc.orientation.x = float(ox)
            oc.orientation.y = float(oy)
            oc.orientation.z = float(oz)
            oc.orientation.w = float(ow)
            # 💡 Tolerancia de rotación generosa (0.4 rad ~ 22º)
            oc.absolute_x_axis_tolerance = 0.4
            oc.absolute_y_axis_tolerance = 0.4
            oc.absolute_z_axis_tolerance = 0.4
            oc.weight = 1.0
            constraints.orientation_constraints.append(oc)
        
        goal_msg.request.goal_constraints.append(constraints)
        
        self.get_logger().info(f'   [IK] Planificando a: ({x:.2f}, {y:.2f}, {z:.2f})...')
        send_goal_future = self._arm_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        handle = send_goal_future.result()
        if not handle.accepted:
            self.get_logger().error(f'   [IK] Petición Cartesian rechazada.')
            return False
        
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()
        
        if result.result.error_code.val == 1:
            return True
        else:
            self.get_logger().error(f'   [IK] Error MoveIt ID: {result.result.error_code.val}')
            return False

    def control_gripper(self, position):
        """Abre o cierra la pinza"""
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = float(position)
        goal_msg.command.max_effort = 100.0
        
        self.get_logger().info(f'   [Gripper] Enviando objetivo: {position}...')
        send_goal_future = self._gripper_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=5.0)
        
        if not send_goal_future.done():
            self.get_logger().error('   [Gripper] Tiempo agotado enviando objetivo')
            return False

        handle = send_goal_future.result()
        if not handle.accepted:
            self.get_logger().error('   [Gripper] Petición rechazada')
            return False
        
        self.get_logger().info('   [Gripper] Objetivo aceptado, esperando resultado...')
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=10.0)
        
        if not result_future.done():
            self.get_logger().warning('   [Gripper] Tiempo agotado esperando resultado (continuando igualmente...)')
            return True # Continuamos para no bloquear la secuencia
            
        self.get_logger().info('   [Gripper] Movimiento finalizado.')
        return True

    # --- MÉTODOS GAZEBO (NUEVOS) ---

    def attach_object(self):
        """Llama al servicio de Gazebo para 'pegar' el cubo a la pinza"""
        req = AttachLink.Request()
        req.model1_name = 'cobot'
        req.link1_name = 'wrist_3_link'
        req.model2_name = 'cube_pick'
        req.link2_name = 'link_1'
        
        self.get_logger().info('🔗 Llamando a ATTACHLINK...')
        future = self.attach_client.call_async(req)
        # Damos un pequeño timeout al servicio por si se cuelga
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        
        if future.done():
            self.get_logger().info('🔗 ATTACHLINK completado.')
            return future.result()
        else:
            self.get_logger().warning('🔗 ATTACHLINK no respondió a tiempo (revisa Gazebo)')
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
        # Damos un pequeño timeout al servicio por si se cuelga
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        
        if future.done():
            self.get_logger().info('🔓 DETACHLINK completado.')
            return future.result()
        else:
            self.get_logger().warning('🔓 DETACHLINK no respondió a tiempo (revisa Gazebo)')
            return None

    # --- LA SECUENCIA ---

    def run_sequence(self):
        self.get_logger().info('🏁 INICIANDO SECUENCIA PICK & PLACE (Coordenadas Reales)')

        # 1. Ir a HOME (Seguridad)
        self.get_logger().info('1️⃣  Yendo a HOME...')
        # Usamos grados: [0, -90, 0, -90, 0, 0] es una posición vertical "Cylindrical"
        # O la posición Home del script 02: [0.0, -129.0, 80.0, -93.0, -90.0, 0.0]
        if not self.move_to_joints([0.0, -90.0, 0.0, -90.0, 0.0, 0.0]):
            self.get_logger().error('🛑 Paso 1 fallido. Abortando.')
            return

        # 2. Abrir Pinza
        self.get_logger().info('2️⃣  Abriendo pinza...')
        self.control_gripper(0.0)
        
        # 3. Posicionarse sobre el objeto (Approach)
        # Coordenadas reales PICK: [0.5, 0.0, 0.379]
        # Orientación: [0.737, -0.675, 0.020, 0.006]
        self.get_logger().info('3️⃣  Aproximación sobre el objeto (Z=0.5)...')
        if not self.move_to_xyz(0.5, 0.0, 0.5, 0.737, -0.675, 0.020, 0.006):
            self.get_logger().error('🛑 Paso 3 fallido. Abortando.')
            return

        # 4. Bajar a coger el objeto (Pick)
        self.get_logger().info('4️⃣  Bajando para agarrar (Z=0.379)...')
        if not self.move_to_xyz(0.5, 0.0, 0.379, 0.737, -0.675, 0.020, 0.006):
            self.get_logger().error('🛑 Paso 4 fallido. Abortando.')
            return

        # 5. Cerrar Pinza y ATTACH
        self.get_logger().info('5️⃣  Cerrando pinza (parcial) + ATTACH')
        # Usamos 0.3 para que no llegue a tocar el cubo y no lo desplace
        self.control_gripper(0.3)
        self.attach_object()
        time.sleep(1.0) 

        # 6. Subir (Lift)
        self.get_logger().info('6️⃣  Levantando objeto...')
        # 💡 Al subir, no nos importa tanto la orientación exacta, así que ponemos ox=None
        if not self.move_to_xyz(0.5, 0.0, 0.5, ox=None):
            self.get_logger().error('🛑 Paso 6 fallido. Abortando.')
            return

        # 7. Mover a zona de descarga (Place area)
        # Coordenadas reales PLACE: [0.4, 0.545, 0.515]
        # Orientación PLACE: [0.665, -0.600, 0.310, -0.320]
        self.get_logger().info('7️⃣  Moviendo a zona de descarga (Z=0.6)...')
        if not self.move_to_xyz(0.4, 0.545, 0.6, 0.665, -0.6, 0.31, -0.32):
            self.get_logger().error('🛑 Paso 7 fallido. Abortando.')
            return

        # 8. Bajar para dejar (Place)
        self.get_logger().info('8️⃣  Bajando para soltar (Z=0.515)...')
        if not self.move_to_xyz(0.4, 0.545, 0.515, 0.665, -0.6, 0.31, -0.32):
            self.get_logger().error('🛑 Paso 8 fallido. Abortando.')
            return

        # 9. Abrir Pinza y DETACH
        self.get_logger().info('9️⃣  Abriendo pinza + DETACH')
        self.detach_object()
        self.control_gripper(0.0)

        # 10. Subir y volver a HOME
        self.get_logger().info('🔟 Finalizando y volviendo a HOME...')
        self.move_to_xyz(0.4, 0.545, 0.6, 0.665, -0.6, 0.31, -0.32)
        self.move_to_joints([0.0, -90.0, 0.0, -90.0, 0.0, 0.0])

        self.get_logger().info('✨ SECUENCIA COMPLETADA CON ÉXITO ✨')

def main(args=None):
    rclpy.init(args=args)
    demo = SequentialPickPlace()
    
    print("\n📦 DEMO: PICK AND PLACE SECUENCIAL")
    print("Este script ejecutará 10 pasos automáticos para simular una tarea real.")
    input("Presiona ENTER para comenzar...")
    
    demo.run_sequence()
    
    demo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
