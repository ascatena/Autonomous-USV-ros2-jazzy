import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from math import sin, cos, pi

class SyntheticOdomPublisher(Node):
    def __init__(self):
        super().__init__('synthetic_odom_publisher')
        
        # --- CONFIGURACIÓN ---
        # Inercia
        self.inertia_factor = 0.1  
        
        # TIEMPO DE ESPERA (WATCHDOG)
        # Si no recibimos cmd_vel en X segundos, frenamos.
        self.cmd_timeout = 1.0 
        
        # Publicador y Suscriptor
        self.pub = self.create_publisher(Odometry, '/wheel/odometry', 10)
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        
        # Timer a 30 Hz
        self.timer = self.create_timer(0.033, self.timer_callback)

        # Variables de estado
        self.target_vx = 0.0
        self.target_wz = 0.0
        self.current_vx = 0.0
        self.current_wz = 0.0
        
        # Posición acumulada (Odometría pura)
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        
        self.last_time = self.get_clock().now()
        
        # Inicializamos el tiempo del último comando
        self.last_cmd_time = self.get_clock().now()

    def cmd_callback(self, msg: Twist):
        # Guardamos la velocidad deseada
        self.target_vx = msg.linear.x
        self.target_wz = msg.angular.z
        
        # ACTUALIZAMOS EL WATCHDOG: Recibimos datos frescos
        self.last_cmd_time = self.get_clock().now()

    def get_quaternion_from_euler(self, roll, pitch, yaw):
        """Convierte ángulos de Euler a Cuaternión manualmente"""
        qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
        qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
        qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
        qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
        return [qx, qy, qz, qw]

    def timer_callback(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # --- LÓGICA DE WATCHDOG (Seguridad) ---
        # Calculamos tiempo sin recibir comandos
        time_since_cmd = (current_time - self.last_cmd_time).nanoseconds / 1e9

        if time_since_cmd > self.cmd_timeout:
            # Si pasó mucho tiempo, asumimos que Nav2 está callado.
            # Forzamos los objetivos a 0 para que el barco frene suavemente.
            self.target_vx = 0.0
            self.target_wz = 0.0

        # 1. SIMULACIÓN DE INERCIA
        # Acercamos la velocidad actual a la objetivo suavemente (Filtro)
        # Nota: Si el watchdog se activó arriba, target_vx será 0 y esto frenará el barco gradualmente
        self.current_vx += (self.target_vx - self.current_vx) * self.inertia_factor
        self.current_wz += (self.target_wz - self.current_wz) * self.inertia_factor

        # "Snap to zero": Si es muy bajo, lo forzamos a 0 para asegurar el anclaje
        if abs(self.current_vx) < 0.01: self.current_vx = 0.0
        if abs(self.current_wz) < 0.01: self.current_wz = 0.0

        # 2. INTEGRACIÓN (Calcular posición teórica)
        delta_x = (self.current_vx * cos(self.th)) * dt
        delta_y = (self.current_vx * sin(self.th)) * dt
        delta_th = self.current_wz * dt

        self.x += delta_x
        self.y += delta_y
        self.th += delta_th

        # 3. CONSTRUCCIÓN DEL MENSAJE
        msg = Odometry()
        msg.header.stamp = current_time.to_msg()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"

        # Pose (Posición + Orientación)
        q = self.get_quaternion_from_euler(0, 0, self.th)
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.x = q[0]
        msg.pose.pose.orientation.y = q[1]
        msg.pose.pose.orientation.z = q[2]
        msg.pose.pose.orientation.w = q[3]

        # Twist (Velocidad lineal y angular)
        msg.twist.twist.linear.x = self.current_vx
        msg.twist.twist.angular.z = self.current_wz

        # 4. COVARIANZA DINÁMICA 
        is_stopped = (abs(self.target_vx) < 0.01 and abs(self.target_wz) < 0.01)

        if is_stopped:
            # ANCLA: Confianza casi infinita (1e-9)
            c_val = 1e-9
        else:
            # MOVIMIENTO: Confianza baja (0.5)
            c_val = 0.5

        # Diagonal: X, Y, Z, Roll, Pitch, Yaw
        msg.twist.covariance = [
            c_val, 0.0,   0.0,   0.0,   0.0,   0.0,    # Varianza en X (Velocidad)
            0.0,   c_val, 0.0,   0.0,   0.0,   0.0,    # Varianza en Y (Lateral)
            0.0,   0.0,   c_val, 0.0,   0.0,   0.0,    # Varianza en Z
            0.0,   0.0,   0.0,   c_val, 0.0,   0.0,    # Varianza Roll
            0.0,   0.0,   0.0,   0.0,   c_val, 0.0,    # Varianza Pitch
            0.0,   0.0,   0.0,   0.0,   0.0,   c_val   # Varianza Yaw (Giro)
        ]

        # La covarianza de pose la dejamos fija y alta (no la usamos para localizar)
        msg.pose.covariance = [0.1] * 36

        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SyntheticOdomPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()