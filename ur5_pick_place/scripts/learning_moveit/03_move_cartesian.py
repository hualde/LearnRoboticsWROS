#!/usr/bin/env python3
"""
03_move_cartesian.py - El poder de la cinemática inversa.
Mueve la herramienta a una posición X,Y,Z sin preocuparte de los ángulos de los motores.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from geometry_msgs.msg import PoseStamped

class MoveCartesian(Node):
    def __init__(self):
        super().__init__('move_cartesian_node')
        self._action_client = ActionClient(self, MoveGroup, '/move_action')
        self._action_client.wait_for_server()

    def move_to_xyz(self, x, y, z):
        """
        Lleva la punta del robot (tool0) a una coordenada X, Y, Z.
        """
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "ur5_manipulator"
        
        # 1. Definir el "Pose Target" (el objetivo en el espacio)
        target_pose = PoseStamped()
        target_pose.header.frame_id = "base_link" # Muy importante: ¿Respecto a qué frame?
        
        # Coordenadas
        target_pose.pose.position.x = x
        target_pose.pose.position.y = y
        target_pose.pose.position.z = z
        
        # Orientación (por ahora la dejamos mirando hacia abajo, típica de pick & place)
        # Esto se define en Cuaterniones (x,y,z,w)
        target_pose.pose.orientation.x = 1.0 
        target_pose.pose.orientation.y = 0.0
        target_pose.pose.orientation.z = 0.0
        target_pose.pose.orientation.w = 0.0
        
        # 2. Definir los nombres de los joints (importante para el grupo)
        goal_msg.request.workspace_parameters.header.frame_id = "base_link"
        
        # 3. Crear las Constraints (Restricciones)
        # En ROS 2 MoveIt no hay "pose_targets" directo. Se usan restricciones de posición y orientación.
        from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint
        from moveit_msgs.msg import BoundingVolume
        from shape_msgs.msg import SolidPrimitive
        from geometry_msgs.msg import Point, Pose
        
        constraints = Constraints()
        
        # --- A. Restricción de POSICIÓN ---
        pc = PositionConstraint()
        pc.header.frame_id = "base_link"
        pc.link_name = "tool0" # El link que queremos mover (punta del robot)
        
        # Definimos una "región" (esfera de tolerancia) donde queremos que esté la punta
        bv = BoundingVolume()
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [0.005] # 5mm de tolerancia
        bv.primitives.append(primitive)
        
        # Pose del centro de esa esfera (nuestro objetivo X, Y, Z)
        target_pose_pos = Pose()
        target_pose_pos.position.x = x
        target_pose_pos.position.y = y
        target_pose_pos.position.z = z
        bv.primitive_poses.append(target_pose_pos)
        
        pc.constraint_region = bv
        pc.weight = 1.0
        constraints.position_constraints.append(pc)
        
        # --- B. Restricción de ORIENTACIÓN ---
        oc = OrientationConstraint()
        oc.header.frame_id = "base_link"
        oc.link_name = "tool0"
        oc.orientation.x = 1.0 # Mirando hacia abajo
        oc.orientation.y = 0.0
        oc.orientation.z = 0.0
        oc.orientation.w = 0.0
        oc.absolute_x_axis_tolerance = 0.1
        oc.absolute_y_axis_tolerance = 0.1
        oc.absolute_z_axis_tolerance = 0.1
        oc.weight = 1.0
        constraints.orientation_constraints.append(oc)
        
        # Añadimos ambas al Goal
        goal_msg.request.goal_constraints.append(constraints)
        
        # 4. Enviar y esperar
        self.get_logger().info(f'📍 Enviando petición Cartesian (IK) para: X={x}, Y={y}, Z={z}')
        
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        
        handle = send_goal_future.result()
        if not handle.accepted:
            self.get_logger().error('❌ Movimiento Cartesian rechazado (¿Posición fuera de alcance?)')
            return

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        self.get_logger().info('✅ ¡Movimiento Cartesian completado!')

def main(args=None):
    rclpy.init(args=args)
    node = MoveCartesian()
    
    # Coordenada típica sobre la mesa en este proyecto
    node.move_to_xyz(0.4, 0.1, 0.5)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
