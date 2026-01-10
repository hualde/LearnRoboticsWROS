#!/usr/bin/env python3
"""
08_planning_scene.py - Gestión de la Escena de Planificación en Python.
Este script es el equivalente al archivo C++ 'test_ik_collision_obj.cpp'.
Enseña cómo añadir y quitar obstáculos para que MoveIt los tenga en cuenta.
"""

import rclpy
from rclpy.node import Node
from moveit_msgs.msg import PlanningScene, CollisionObject
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
import time

class PlanningSceneManager(Node):
    def __init__(self):
        super().__init__('planning_scene_manager_node')
        
        # 1. Crear el cliente para el servicio oficial de MoveIt
        self.client = self.create_client(ApplyPlanningScene, '/apply_planning_scene')
        
        self.get_logger().info('⏳ Esperando al servicio /apply_planning_scene...')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('... todavía esperando ...')
            
        self.get_logger().info('✅ MoveIt detectado. Todo listo.')

    def add_objects(self):
        """Añade los mismos objetos que el archivo C++"""
        self.get_logger().info('📦 Añadiendo objetos a la escena...')
        
        # Creamos una lista de objetos
        objs = []

        # --- 1. MESA (Table) ---
        table = CollisionObject()
        table.id = 'table1'
        table.header.frame_id = 'world'
        table.operation = CollisionObject.ADD
        
        # Forma: Caja [X, Y, Z]
        sp = SolidPrimitive()
        sp.type = SolidPrimitive.BOX
        sp.dimensions = [0.608, 2.0, 1.0]
        table.primitives.append(sp)
        
        # Posición
        p = Pose()
        p.position.x = 0.576
        p.position.y = 0.0
        p.position.z = 0.5
        table.primitive_poses.append(p)
        objs.append(table)

        # --- 2. MURO 1 (Wall 1) ---
        wall1 = CollisionObject()
        wall1.id = 'wall1'
        wall1.header.frame_id = 'world'
        wall1.operation = CollisionObject.ADD
        sp_w1 = SolidPrimitive()
        sp_w1.type = SolidPrimitive.BOX
        sp_w1.dimensions = [0.5, 0.05, 0.5]
        wall1.primitives.append(sp_w1)
        p_w1 = Pose()
        p_w1.position.x = 0.5
        p_w1.position.y = 0.2
        p_w1.position.z = 1.25
        wall1.primitive_poses.append(p_w1)
        objs.append(wall1)

        # --- 3. BASEMENT (Cilindro) ---
        base = CollisionObject()
        base.id = 'basement'
        base.header.frame_id = 'world'
        base.operation = CollisionObject.ADD
        sp_b = SolidPrimitive()
        sp_b.type = SolidPrimitive.CYLINDER
        sp_b.dimensions = [0.8, 0.2] # [Altura, Radio]
        base.primitives.append(sp_b)
        p_b = Pose()
        p_b.position.x = 0.0
        p_b.position.y = 0.0
        p_b.position.z = 0.4
        base.primitive_poses.append(p_b)
        objs.append(base)

        # --- ENVIAR A MOVEIT ---
        self._apply_objects(objs)

    def remove_objects(self):
        """Quita los objetos por su ID"""
        self.get_logger().info('🧹 Quitando objetos de la escena...')
        ids = ['table1', 'wall1', 'basement']
        objs = []
        
        for name in ids:
            obj = CollisionObject()
            obj.id = name
            obj.operation = CollisionObject.REMOVE
            objs.append(obj)
            
        self._apply_objects(objs)

    def _apply_objects(self, collision_objects):
        """Función helper para enviar la actualización a MoveIt"""
        # Creamos una escena de planificación "diferencial" (is_diff=True)
        # Esto significa que solo enviamos los CAMBIOS, no toda la escena completa.
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = collision_objects
        
        req = ApplyPlanningScene.Request()
        req.scene = scene
        
        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result().success:
            self.get_logger().info('✨ Escena actualizada correctamente.')
        else:
            self.get_logger().error('❌ Fallo al actualizar la escena.')

def main():
    rclpy.init()
    node = PlanningSceneManager()
    
    print("\n💡 GESTOR DE ESCENA MOVEIT (PYTHON)")
    print("1. Añadir objetos (como en C++)")
    print("2. Quitar objetos")
    print("3. Salir")
    
    try:
        while rclpy.ok():
            choice = input("\nElige una opción: ")
            if choice == '1':
                node.add_objects()
            elif choice == '2':
                node.remove_objects()
            elif choice == '3':
                break
            else:
                print("Opción no válida.")
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
