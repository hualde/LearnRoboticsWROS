#!/usr/bin/env python3
"""
robot_mover_action.py - Usando Move Action directamente (como RViz)
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MotionPlanRequest
from sensor_msgs.msg import JointState
import time


class RobotMoverAction(Node):
    
    def __init__(self):
        super().__init__('robot_mover_action')
        
        self.get_logger().info('🤖 Inicializando con Move Action...')
        
        # Action client (el mismo que usa RViz)
        self._action_client = ActionClient(self, MoveGroup, '/move_action')
        
        # Esperar a que el servidor esté disponible
        self.get_logger().info('⏳ Esperando action server...')
        self._action_client.wait_for_server()
        
        # Suscriptor a joint states
        self.current_joints = None
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10)
        
        time.sleep(1.0)
        self.get_logger().info('✅ Listo!')
    
    def joint_callback(self, msg):
        """Guarda el estado actual de los joints"""
        # Los primeros 6 son los del UR5
        self.current_joints = list(msg.position[:6])
    
    def move_to_joint_state(self, joint_positions):
        """Mueve a posiciones específicas de joints"""
        self.get_logger().info(f'🔧 Moviendo a: {[f"{j:.3f}" for j in joint_positions]}')
        
        if self.current_joints is None:
            self.get_logger().error('❌ No hay joint states disponibles')
            return False
        
        # Crear goal
        goal_msg = MoveGroup.Goal()
        
        # Configurar request
        goal_msg.request.group_name = "ur5_manipulator"
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.max_velocity_scaling_factor = 0.1
        goal_msg.request.max_acceleration_scaling_factor = 0.1
        
        # Crear constraints para cada joint
        joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint"
        ]
        
        constraints = Constraints()
        for i, (name, position) in enumerate(zip(joint_names, joint_positions)):
            constraint = JointConstraint()
            constraint.joint_name = name
            constraint.position = float(position)
            constraint.tolerance_above = 0.01
            constraint.tolerance_below = 0.01
            constraint.weight = 1.0
            constraints.joint_constraints.append(constraint)
        
        goal_msg.request.goal_constraints.append(constraints)
        
        # Configurar planning options
        goal_msg.planning_options.plan_only = False  # Plan Y ejecuta
        goal_msg.planning_options.replan = True
        goal_msg.planning_options.replan_attempts = 5
        
        # Enviar goal
        self.get_logger().info('⚙️  Enviando goal...')
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        
        # Esperar a que se acepte
        rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=2.0)
        
        goal_handle = send_goal_future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('❌ Goal rechazado')
            return False
        
        self.get_logger().info('✅ Goal aceptado, ejecutando...')
        
        # Esperar resultado
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=30.0)
        
        result = result_future.result()
        
        if result.result.error_code.val == 1:  # SUCCESS
            self.get_logger().info('✅ Movimiento completado!')
            return True
        else:
            self.get_logger().error(f'❌ Error: {result.result.error_code.val}')
            return False


def main():
    rclpy.init()
    
    print("=" * 70)
    print("🤖 ROBOT MOVER - Move Action (como RViz)")
    print("=" * 70)
    
    robot = RobotMoverAction()
    
    print("\n⏳ Esperando joint states...")
    for i in range(10):
        rclpy.spin_once(robot, timeout_sec=0.5)
        if robot.current_joints is not None:
            break
    
    if robot.current_joints is None:
        print("❌ No se recibieron joint states")
        return
    
    print(f"✅ Joints actuales: {[f'{j:.3f}' for j in robot.current_joints]}")
    
    # Poses predefinidas
    poses = {
        "home": [0.0, -2.2564, 1.4059, -1.6315, -1.57, 0.0],
        "zero": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        "up": [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]
    }
    
    print("\n📋 MENÚ:")
    print("  1. Mover a HOME")
    print("  2. Mover a ZERO")
    print("  3. Mover a UP")
    print("  0. Salir")
    print()
    
    try:
        while rclpy.ok():
            opcion = input("Opción: ")
            
            if opcion == "1":
                print("\n🏠 Moviendo a HOME...")
                robot.move_to_joint_state(poses["home"])
                
            elif opcion == "2":
                print("\n0️⃣ Moviendo a ZERO...")
                robot.move_to_joint_state(poses["zero"])
                
            elif opcion == "3":
                print("\n⬆️  Moviendo a UP...")
                robot.move_to_joint_state(poses["up"])
                
            elif opcion == "0":
                break
            
            print()
            
    except KeyboardInterrupt:
        print("\n🛑 Saliendo...")
    finally:
        robot.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()