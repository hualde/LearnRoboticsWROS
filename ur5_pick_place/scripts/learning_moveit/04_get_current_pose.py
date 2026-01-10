#!/usr/bin/env python3
"""
04_get_current_pose.py - ¿Dónde está mi robot ahora mismo?
Este script usa TF2 para leer la posición exacta de la punta del robot (tool0)
respecto a la base (base_link).
"""

import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
import time

class PoseMonitor(Node):
    def __init__(self):
        super().__init__('pose_monitor_node')
        
        # El buffer guarda las transformaciones
        self.tf_buffer = Buffer()
        # El listener escucha los topics de /tf y las guarda en el buffer
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.get_logger().info('🔍 Monitor de Pose iniciado (esperando 2s para llenar buffer)...')
        time.sleep(2.0)

    def print_current_pose(self):
        try:
            # Buscamos la transformación entre la base y la punta
            # 'base_link' -> 'tool0'
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                'base_link', 
                'tool0', 
                now,
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            
            p = trans.transform.translation
            o = trans.transform.rotation
            
            print("\n📌 POSE ACTUAL DEL ROBOT (tool0 respecto a base_link):")
            print(f"   X: {p.x:.4f} m")
            print(f"   Y: {p.y:.4f} m")
            print(f"   Z: {p.z:.4f} m")
            print(f"   Orientación (Quaternion): [x:{o.x:.2f}, y:{o.y:.2f}, z:{o.z:.2f}, w:{o.w:.2f}]")
            
        except Exception as e:
            self.get_logger().error(f'❌ No se pudo obtener la pose: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = PoseMonitor()
    
    try:
        while rclpy.ok():
            node.print_current_pose()
            print("\n(Pulsa Ctrl+C para salir)")
            time.sleep(2.0)
            # Necesitamos spin_once para que el transform_listener se actualice
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
