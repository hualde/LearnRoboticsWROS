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
import math
import time

class SequentialPickPlace(Node):
    def __init__(self):
        super().__init__('sequential_pick_place_node')
        
        # 1. Clientes de Acción
        self._arm_client = ActionClient(self, MoveGroup, '/move_action')
        self._gripper_client = ActionClient(self, GripperCommand, '/gripper_position_controller/gripper_cmd')
        
        self.get_logger().info('⏳ Esperando a los controladores (Brazo + Pinza)...')
        self._arm_client.wait_for_server()
        self._gripper_client.wait_for_server()
        self.get_logger().info('✅ Sistema completo preparado!')

    # --- MÉTODOS DE APOYO (REUTILIZADOS) ---

    def move_to_joints(self, angles_deg):
        """Mueve el brazo usando ángulos (de los scripts anteriores)"""
        angles_rad = [math.radians(a) for a in angles_deg]
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "ur5_manipulator"
        
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
        
        # El código 1 significa SUCCESS en MoveIt
        if result.result.error_code.val == 1:
            return True
        else:
            self.get_logger().error(f'❌ Error en movimiento de joints: {result.result.error_code.val}')
            return False

    def move_to_xyz(self, x, y, z):
        """Mueve el brazo a una coordenada XYZ"""
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "ur5_manipulator"
        goal_msg.request.workspace_parameters.header.frame_id = "base_link"
        
        # Parámetros de planificación
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
        primitive.dimensions = [0.01] # 1cm de tolerancia
        bv.primitives.append(primitive)
        target_pose = Pose()
        target_pose.position.x = x
        target_pose.position.y = y
        target_pose.position.z = z
        bv.primitive_poses.append(target_pose)
        pc.constraint_region = bv
        pc.weight = 1.0
        constraints.position_constraints.append(pc)
        
        # Orientación (mirando hacia abajo)
        oc = OrientationConstraint()
        oc.header.frame_id = "base_link"
        oc.link_name = "tool0"
        oc.orientation.x = 1.0
        oc.orientation.y = 0.0
        oc.orientation.z = 0.0
        oc.orientation.w = 0.0
        oc.absolute_x_axis_tolerance = 0.2
        oc.absolute_y_axis_tolerance = 0.2
        oc.absolute_z_axis_tolerance = 0.2
        oc.weight = 1.0
        constraints.orientation_constraints.append(oc)
        
        goal_msg.request.goal_constraints.append(constraints)
        
        send_goal_future = self._arm_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        handle = send_goal_future.result()
        if not handle.accepted:
            self.get_logger().error('❌ Petición Cartesian rechazada')
            return False
        
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()
        
        if result.result.error_code.val == 1:
            return True
        else:
            self.get_logger().error(f'❌ Error en movimiento Cartesian: {result.result.error_code.val}')
            return False

    def control_gripper(self, position):
        """Abre o cierra la pinza"""
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = float(position)
        goal_msg.command.max_effort = 100.0
        
        send_goal_future = self._gripper_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        handle = send_goal_future.result()
        if not handle.accepted:
            self.get_logger().error('❌ Petición de pinza rechazada')
            return False
        
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return True

    # --- LA SECUENCIA ---

    def run_sequence(self):
        self.get_logger().info('🏁 INICIANDO SECUENCIA PICK & PLACE')

        # 1. Ir a HOME (Seguridad)
        self.get_logger().info('1️⃣  Yendo a HOME...')
        if not self.move_to_joints([0.0, -129.0, 80.0, -93.0, -90.0, 0.0]):
            self.get_logger().error('🛑 Paso 1 fallido. Abortando.')
            return

        # 2. Abrir Pinza
        self.get_logger().info('2️⃣  Abriendo pinza...')
        if not self.control_gripper(0.0):
            self.get_logger().error('🛑 Paso 2 fallido. Abortando.')
            # No abortamos por pinza necesariamente, pero avisamos
        
        # 3. Posicionarse sobre el objeto (Approach)
        self.get_logger().info('3️⃣  Aproximación sobre el objeto...')
        if not self.move_to_xyz(0.4, 0.1, 0.7):
            self.get_logger().error('🛑 Paso 3 fallido. Abortando.')
            return

        # 4. Bajar a coger el objeto (Pick)
        self.get_logger().info('4️⃣  Bajando para agarrar...')
        if not self.move_to_xyz(0.4, 0.1, 0.5):
            self.get_logger().error('🛑 Paso 4 fallido. Abortando.')
            return

        # 5. Cerrar Pinza
        self.get_logger().info('5️⃣  Cerrando pinza (Pick!)')
        self.control_gripper(0.79)
        time.sleep(1.0) 

        # 6. Subir (Lift)
        self.get_logger().info('6️⃣  Levantando objeto...')
        if not self.move_to_xyz(0.4, 0.1, 0.7):
            self.get_logger().error('🛑 Paso 6 fallido. Abortando.')
            return

        # 7. Mover a zona de descarga (Place area)
        self.get_logger().info('7️⃣  Moviendo a zona de descarga...')
        if not self.move_to_xyz(0.4, -0.3, 0.7):
            self.get_logger().error('🛑 Paso 7 fallido. Abortando.')
            return

        # 8. Bajar para dejar (Place)
        self.get_logger().info('8️⃣  Bajando para soltar...')
        if not self.move_to_xyz(0.4, -0.3, 0.55):
            self.get_logger().error('🛑 Paso 8 fallido. Abortando.')
            return

        # 9. Abrir Pinza
        self.get_logger().info('9️⃣  Abriendo pinza (Soltando)')
        self.control_gripper(0.0)

        # 10. Subir y volver a HOME
        self.get_logger().info('🔟 Finalizando y volviendo a HOME...')
        self.move_to_xyz(0.4, -0.3, 0.7)
        self.move_to_joints([0.0, -129.0, 80.0, -93.0, -90.0, 0.0])

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
