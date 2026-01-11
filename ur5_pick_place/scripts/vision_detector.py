#!/usr/bin/env python3
"""
vision_detector.py - CON TRANSFORMACIONES TF (SOLO VERDES)
Detecta objetos VERDES y transforma coordenadas al frame del robot
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, PoseArray, TransformStamped, PointStamped
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge
import cv2
import numpy as np
import json
from tf2_ros import TransformListener, Buffer
from tf2_geometry_msgs import do_transform_point
import tf2_ros

class VisionDetector(Node):
    def __init__(self):
        super().__init__('vision_detector')
        
        self.bridge = CvBridge()
        
        # Datos de sensores
        self.latest_image = None
        self.latest_depth = None
        self.camera_info = None
        
        # TF2 para transformaciones
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Frame de referencia (cambiar si tu robot usa otro)
        self.robot_frame = 'base_link'  # Frame del robot
        self.camera_frame = 'camera_link'  # Frame de la cámara
        
        # Suscriptores
        self.rgb_sub = self.create_subscription(
            Image, '/camera/image_raw', self.rgb_callback, 10)
        
        self.depth_sub = self.create_subscription(
            Image, '/camera/depth/image_raw', self.depth_callback, 10)
        
        self.info_sub = self.create_subscription(
            CameraInfo, '/camera/camera_info', self.camera_info_callback, 10)
        
        # Publicadores
        self.objects_pub = self.create_publisher(String, '/detected_objects', 10)
        self.poses_pub = self.create_publisher(PoseArray, '/object_poses', 10)
        self.markers_pub = self.create_publisher(MarkerArray, '/object_markers', 10)
        
        # Timer
        self.process_rate = 2.0
        self.timer = self.create_timer(1.0/self.process_rate, self.process_and_detect)
        
        self.detection_count = 0
        
        self.get_logger().info('🔍 Vision Detector con TF iniciado - SOLO OBJETOS VERDES 🟢')
        self.get_logger().info(f'🤖 Frame del robot: {self.robot_frame}')
        self.get_logger().info(f'📷 Frame de cámara: {self.camera_frame}')
    
    def rgb_callback(self, msg):
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
    
    def depth_callback(self, msg):
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, 'passthrough')
        except Exception as e:
            self.get_logger().error(f'Error en depth_callback: {e}')
    
    def camera_info_callback(self, msg):
        self.camera_info = msg
        # Actualizar el frame de la cámara desde camera_info
        if msg.header.frame_id:
            self.camera_frame = msg.header.frame_id
    
    def detect_green_objects(self, image):
        """Detecta SOLO objetos verdes"""
        objects = []
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # SOLO rango verde
        green_ranges = [(np.array([40, 100, 100]), np.array([80, 255, 255]))]
        
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        
        for lower, upper in green_ranges:
            mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lower, upper))
        
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for contour in contours:
            area = cv2.contourArea(contour)
            
            if area > 300:
                M = cv2.moments(contour)
                if M['m00'] != 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])
                    
                    objects.append({
                        'color': 'green',
                        'pixel_x': cx,
                        'pixel_y': cy,
                        'area': area
                    })
        
        return objects
    
    def pixel_to_3d_point(self, pixel_x, pixel_y):
        """Convierte píxel a 3D en el frame de la cámara"""
        if self.latest_depth is None or self.camera_info is None:
            return None
        
        try:
            height, width = self.latest_depth.shape
            
            if not (0 <= pixel_x < width and 0 <= pixel_y < height):
                return None
            
            # Obtener profundidad
            depth = float(self.latest_depth[pixel_y, pixel_x])
            
            # Promediar si no es válido
            if not np.isfinite(depth) or depth <= 0.01 or depth > 10.0:
                depths = []
                for dy in range(-2, 3):
                    for dx in range(-2, 3):
                        py = pixel_y + dy
                        px = pixel_x + dx
                        if 0 <= px < width and 0 <= py < height:
                            d = float(self.latest_depth[py, px])
                            if np.isfinite(d) and 0.01 < d < 10.0:
                                depths.append(d)
                
                if not depths:
                    return None
                
                depth = np.mean(depths)
            
            # Parámetros intrínsecos
            fx = self.camera_info.k[0]
            fy = self.camera_info.k[4]
            cx = self.camera_info.k[2]
            cy = self.camera_info.k[5]
            
            # Convertir a 3D (en frame de cámara)
            z = depth
            x = (pixel_x - cx) * z / fx
            y = (pixel_y - cy) * z / fy
            
            return (x, y, z)
            
        except Exception as e:
            self.get_logger().error(f'Error en pixel_to_3d: {e}')
            return None
    
    def transform_point_to_robot_frame(self, x, y, z):
        """
        Transforma punto del frame de cámara al frame del robot
        """
        try:
            # Crear punto en frame de cámara
            point_camera = PointStamped()
            point_camera.header.frame_id = self.camera_frame
            point_camera.header.stamp = self.get_clock().now().to_msg()
            point_camera.point.x = x
            point_camera.point.y = y
            point_camera.point.z = z
            
            # Obtener transformación
            transform = self.tf_buffer.lookup_transform(
                self.robot_frame,
                self.camera_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            
            # Transformar punto
            point_robot = do_transform_point(point_camera, transform)
            
            return (
                point_robot.point.x,
                point_robot.point.y,
                point_robot.point.z
            )
            
        except (tf2_ros.LookupException, 
                tf2_ros.ConnectivityException, 
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f'No se pudo transformar: {e}', throttle_duration_sec=2.0)
            return None
    
    def create_marker(self, obj_id, x, y, z, frame_id):
        """Crea marcador verde"""
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "detected_objects"
        marker.id = obj_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        marker.pose.position.x = float(x)
        marker.pose.position.y = float(y)
        marker.pose.position.z = float(z)
        marker.pose.orientation.w = 1.0
        
        # Tamaño del marcador
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        
        # Color verde
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.9
        
        marker.lifetime.sec = 2
        
        return marker
    
    def create_text_marker(self, obj_id, x, y, z, frame_id):
        """Crea marcador de texto"""
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "object_labels"
        marker.id = obj_id + 1000
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        
        marker.pose.position.x = float(x)
        marker.pose.position.y = float(y)
        marker.pose.position.z = float(z) + 0.1  # Un poco arriba
        marker.pose.orientation.w = 1.0
        
        marker.text = f"GREEN\n({x:.2f},{y:.2f},{z:.2f})"
        marker.scale.z = 0.04
        
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        
        marker.lifetime.sec = 2
        
        return marker
    
    def process_and_detect(self):
        """Pipeline principal con transformaciones - SOLO VERDES"""
        # Verificar datos
        if self.latest_image is None:
            self.get_logger().warn('Esperando imagen RGB...', throttle_duration_sec=3.0)
            return
        
        if self.latest_depth is None:
            self.get_logger().warn('Esperando depth...', throttle_duration_sec=3.0)
            return
        
        if self.camera_info is None:
            self.get_logger().warn('Esperando camera_info...', throttle_duration_sec=3.0)
            return
        
        # Detectar SOLO objetos verdes
        objects_2d = self.detect_green_objects(self.latest_image.copy())
        
        if not objects_2d:
            return
        
        # Procesar cada objeto verde
        objects_3d = []
        poses = []
        markers = MarkerArray()
        
        for obj in objects_2d:
            # 1. Obtener coordenadas en frame de cámara
            point_camera = self.pixel_to_3d_point(obj['pixel_x'], obj['pixel_y'])
            
            if point_camera is None:
                continue
            
            x_cam, y_cam, z_cam = point_camera
            
            # 2. Transformar a frame del robot
            point_robot = self.transform_point_to_robot_frame(x_cam, y_cam, z_cam)
            
            if point_robot is None:
                continue
            
            x_robot, y_robot, z_robot = point_robot
            
            # 3. Guardar datos
            obj_data = {
                'id': self.detection_count,
                'color': 'green',
                'pixel_x': obj['pixel_x'],
                'pixel_y': obj['pixel_y'],
                'area': float(obj['area']),
                'position_camera_frame': {
                    'x': float(x_cam),
                    'y': float(y_cam),
                    'z': float(z_cam),
                    'frame': self.camera_frame
                },
                'position_robot_frame': {
                    'x': float(x_robot),
                    'y': float(y_robot),
                    'z': float(z_robot),
                    'frame': self.robot_frame
                }
            }
            objects_3d.append(obj_data)
            
            # 4. Crear pose en frame del robot
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = self.robot_frame
            pose.pose.position.x = float(x_robot)
            pose.pose.position.y = float(y_robot)
            pose.pose.position.z = float(z_robot)
            pose.pose.orientation.w = 1.0
            poses.append(pose.pose)
            
            # 5. Crear marcadores en frame del robot
            marker = self.create_marker(
                self.detection_count, 
                x_robot, y_robot, z_robot, 
                self.robot_frame
            )
            text_marker = self.create_text_marker(
                self.detection_count, 
                x_robot, y_robot, z_robot, 
                self.robot_frame
            )
            markers.markers.append(marker)
            markers.markers.append(text_marker)
            
            # 6. Log
            self.get_logger().info(
                f"✅ 🟢 VERDE: "
                f"Cámara({x_cam:.3f}, {y_cam:.3f}, {z_cam:.3f}) → "
                f"Robot({x_robot:.3f}, {y_robot:.3f}, {z_robot:.3f})m"
            )
            
            self.detection_count += 1
        
        # Publicar
        if objects_3d:
            # JSON
            json_msg = String()
            json_msg.data = json.dumps(objects_3d, indent=2)
            self.objects_pub.publish(json_msg)
            
            # Poses
            pose_array = PoseArray()
            pose_array.header.stamp = self.get_clock().now().to_msg()
            pose_array.header.frame_id = self.robot_frame
            pose_array.poses = poses
            self.poses_pub.publish(pose_array)
            
            # Marcadores
            self.markers_pub.publish(markers)
            
            self.get_logger().info(f'📡 Publicados {len(objects_3d)} objetos VERDES en frame {self.robot_frame}')

def main(args=None):
    rclpy.init(args=args)
    
    print("=" * 70)
    print("🎯 VISION DETECTOR - SOLO OBJETOS VERDES 🟢")
    print("=" * 70)
    print("✅ Detecta ÚNICAMENTE objetos verdes")
    print("✅ Transforma coordenadas de cámara → robot")
    print("✅ Marcadores verdes en frame correcto (base_link)")
    print("=" * 70)
    
    node = VisionDetector()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n🛑 Deteniendo...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()