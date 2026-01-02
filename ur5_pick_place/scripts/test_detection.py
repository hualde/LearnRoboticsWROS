#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ColorDetector(Node):
    def __init__(self):
        super().__init__('color_detector')
        self.bridge = CvBridge()
        
        self.sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        
        self.get_logger().info('🔍 Detector iniciado - Buscando objetos verdes')
        self.get_logger().info('Presiona Ctrl+C para salir')
    
    def image_callback(self, msg):
        try:
            # Convertir a OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # Convertir a HSV
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            
            # Rango para verde
            lower_green = np.array([40, 40, 40])
            upper_green = np.array([80, 255, 255])
            
            # Crear máscara
            mask = cv2.inRange(hsv, lower_green, upper_green)
            
            # Limpiar ruido
            kernel = np.ones((5,5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            
            # Encontrar contornos
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, 
                                           cv2.CHAIN_APPROX_SIMPLE)
            
            # Dibujar contornos
            num_objects = 0
            for contour in contours:
                area = cv2.contourArea(contour)
                if area > 100:  # Filtrar ruido pequeño
                    num_objects += 1
                    M = cv2.moments(contour)
                    if M['m00'] != 0:
                        cx = int(M['m10'] / M['m00'])
                        cy = int(M['m01'] / M['m00'])
                        
                        # Dibujar centro
                        cv2.circle(cv_image, (cx, cy), 5, (0, 255, 255), -1)
                        
                        # Dibujar contorno
                        cv2.drawContours(cv_image, [contour], -1, (0, 255, 0), 2)
                        
                        # Texto con info
                        text = f'Verde #{num_objects} ({cx}, {cy})'
                        cv2.putText(cv_image, text, 
                                  (cx-80, cy-20), 
                                  cv2.FONT_HERSHEY_SIMPLEX, 
                                  0.5, (0, 255, 255), 2)
                        
                        self.get_logger().info(
                            f'✅ Objeto verde detectado en píxel ({cx}, {cy}) - Área: {int(area)}'
                        )
            
            # Mostrar contador
            cv2.putText(cv_image, f'Objetos verdes: {num_objects}', 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 
                       1, (0, 255, 0), 2)
            
            # Mostrar imágenes
            cv2.imshow('Deteccion de Objetos Verdes', cv_image)
            cv2.imshow('Mascara (solo verdes)', mask)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f'Error procesando imagen: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = ColorDetector()
    
    print("=" * 60)
    print("🎯 DETECTOR DE OBJETOS VERDES")
    print("=" * 60)
    print("✓ Esperando imágenes de /camera/image_raw")
    print("✓ Se abrirán 2 ventanas:")
    print("  1. Detección (con marcas amarillas)")
    print("  2. Máscara (solo objetos verdes)")
    print("")
    print("Presiona Ctrl+C en la terminal para salir")
    print("=" * 60)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n🛑 Deteniendo detector...")
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()
        print("✅ Detector cerrado correctamente")

if __name__ == '__main__':
    main()
