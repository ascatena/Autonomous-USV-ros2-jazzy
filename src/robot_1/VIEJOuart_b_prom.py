#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from std_msgs.msg import String, Float32
from sensor_msgs.msg import Imu, MagneticField, NavSatFix, NavSatStatus
from geometry_msgs.msg import Quaternion, Twist
from math import sqrt, atan2, cos, sin
import struct
import serial
from tf_transformations import euler_from_quaternion, quaternion_from_euler

class UARTNode(Node):

    def __init__(self):
        super().__init__('uart_brigde')

        QoSProf = QoSProfile(depth=10)

        # Parámetros configurables
        self.declare_parameter('port', '/dev/ttyAMA0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('frame_id', 'imu_frame')

        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value
        self.frame_id = self.get_parameter('frame_id').value

        self.get_logger().info(f"Abrir UART: {port} @ {baud}")

        self.ser = serial.Serial(port=port, baudrate=baud, timeout=1)

        # Publicadores
        self.pub_imu_raw = self.create_publisher(Imu, '/imu', QoSProf)
        self.pub_mag     = self.create_publisher(MagneticField, '/mpu9250/mag', QoSProf)
        self.pub_gps     = self.create_publisher(NavSatFix, '/gps/fix', QoSProf)
        self.pub_heading = self.create_publisher(Float32, '/heading', QoSProf)
        
        # Suscriptor a turtlesim
        self.linear_x = 0.0  
        self.angular_z = 0.0 
        self.subscription = self.create_subscription(Twist,'/turtle1/cmd_vel',self.cmd_vel_callback,10)

        # Timer a 30 Hz
        self.timer = self.create_timer(0.033, self.read_uart)
        # Timer de la señal StillAlive
        self.still_alive_timer = self.create_timer(2.0, self.send_still_alive)
        
        # --- VARIABLES DE CALIBRACIÓN ---
        self.is_calibrated = False
        self.calibration_start_time = None
        self.calibration_duration = 20.0 # Segundos
        
        # Almacenamiento temporal de datos para el promedio
        self.calib_buffer = {
            'ax': [], 'ay': [], 'az': [],
            'gx': [], 'gy': [], 'gz': []
        }
        
        # Offsets calculados (Bias)
        self.imu_offsets = {
            'ax': 0.0, 'ay': 0.0, 'az': 0.0,
            'gx': 0.0, 'gy': 0.0, 'gz': 0.0
        }
        self.get_logger().info("--- INICIANDO CALIBRACION DE IMU (20s) - MANTEN EL ROBOT QUIETO ---")

    def send_still_alive(self):
        try:
            msg = b"W\1\1"
            self.ser.write(msg)
        except Exception as e:
            self.get_logger().error(f"Error enviando stillAlive: {e}")

    def read_uart(self):
        if self.ser.in_waiting >= 55:
            try:
                buf = self.ser.read(55)
                header = buf[0]
                if header == 0x49:   # 'I'
                    self.publish_imu(buf)
                elif header == 0x47:  # 'G'
                    self.publish_gps(buf)
                else:
                    self.get_logger().warn(f"Header desconocido: {hex(header)}")
            except Exception as e:
                self.get_logger().error(f"UART read error: {e}")

    # =============================================================
    #   PUBLICACIÓN IMU (MODIFICADA CON CALIBRACIÓN)
    # =============================================================
    def publish_imu(self, buf):

        ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw, mx, my, mz, qx, qy, qz, qw = \
            self.DataParseIMU(buf)

        # 1. Conversión a Unidades Físicas (m/s^2 y rad/s)
        # Respetamos tu lógica de inversión de signos y escalas
        phys_ax = -float(ay_raw)/ 16384 * 9.80665
        phys_ay = -float(ax_raw)/ 16384 * 9.80665
        phys_az = float(az_raw)/ 16384 * 9.80665

        phys_gx = float(gy_raw)/32.8 * 3.14159265/180
        phys_gy = float(gx_raw)/32.8 * 3.14159265/180
        phys_gz = -float(gz_raw)/32.8 * 3.14159265/180

        # ---------------------------------------------------------
        #  RUTINA DE CALIBRACIÓN
        # ---------------------------------------------------------
        if not self.is_calibrated:
            current_time = self.get_clock().now()
            
            # Inicializar tiempo de inicio en el primer paquete
            if self.calibration_start_time is None:
                self.calibration_start_time = current_time

            elapsed = (current_time - self.calibration_start_time).nanoseconds / 1e9

            if elapsed < self.calibration_duration:
                # Acumular datos
                self.calib_buffer['ax'].append(phys_ax)
                self.calib_buffer['ay'].append(phys_ay)
                self.calib_buffer['az'].append(phys_az)
                self.calib_buffer['gx'].append(phys_gx)
                self.calib_buffer['gy'].append(phys_gy)
                self.calib_buffer['gz'].append(phys_gz)
                
                # Feedback cada 1 segundo (opcional)
                if len(self.calib_buffer['ax']) % 30 == 0: 
                    self.get_logger().info(f"Calibrando... {elapsed:.1f}/{self.calibration_duration}s")
                return # IMPORTANTE: No publicamos durante la calibración para no ensuciar el EKF
            
            else:
                # Tiempo cumplido: Calcular promedios
                count = len(self.calib_buffer['ax'])
                if count > 0:
                    self.imu_offsets['ax'] = sum(self.calib_buffer['ax']) / count
                    self.imu_offsets['ay'] = sum(self.calib_buffer['ay']) / count
                    # Para Z, el offset es la diferencia respecto a la gravedad esperada (9.81)
                    # Si el promedio es 9.9, el offset es 0.1. (9.9 - 0.1 = 9.8)
                    self.imu_offsets['az'] = (sum(self.calib_buffer['az']) / count) - 9.80665
                    
                    self.imu_offsets['gx'] = sum(self.calib_buffer['gx']) / count
                    self.imu_offsets['gy'] = sum(self.calib_buffer['gy']) / count
                    self.imu_offsets['gz'] = sum(self.calib_buffer['gz']) / count

                    self.get_logger().info(f"--- CALIBRACION FINALIZADA ---")
                    self.get_logger().info(f"Offsets Accel: {self.imu_offsets['ax']:.3f}, {self.imu_offsets['ay']:.3f}, {self.imu_offsets['az']:.3f}")
                    self.get_logger().info(f"Offsets Gyro:  {self.imu_offsets['gx']:.3f}, {self.imu_offsets['gy']:.3f}, {self.imu_offsets['gz']:.3f}")
                
                self.is_calibrated = True
                # Limpiar buffer para liberar memoria
                self.calib_buffer.clear()

        # ---------------------------------------------------------
        #  APLICACIÓN DE OFFSETS (Desafectar bias)
        # ---------------------------------------------------------
        # Restamos el promedio calculado a la lectura actual
        phys_ax -= self.imu_offsets['ax']
        phys_ay -= self.imu_offsets['ay']
        phys_az -= self.imu_offsets['az']

        phys_gx -= self.imu_offsets['gx']
        phys_gy -= self.imu_offsets['gy']
        phys_gz -= self.imu_offsets['gz']

        # ---------------------------------------------------------
        #  LLENADO DEL MENSAJE
        # ---------------------------------------------------------
        imu_msg = Imu()
        mag_msg = MagneticField()

        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = self.frame_id

        imu_msg.linear_acceleration.x = phys_ax
        imu_msg.linear_acceleration.y = phys_ay
        imu_msg.linear_acceleration.z = phys_az
        
        imu_msg.angular_velocity.x = phys_gx
        imu_msg.angular_velocity.y = phys_gy
        imu_msg.angular_velocity.z = phys_gz

        # Covarianzas (Mantenemos tus valores)
        diagonal_val_acc = 1.0 # Modifiqué esto que estaba en 1.0 arriba en tu código original pero 0.01 abajo?
        imu_msg.linear_acceleration_covariance = [diagonal_val_acc, 0.0, 0.0, 0.0, diagonal_val_acc, 0.0, 0.0, 0.0, diagonal_val_acc]
        
        diagonal_val_gyr = 0.01
        imu_msg.angular_velocity_covariance = [diagonal_val_gyr, 0.0, 0.0, 0.0, diagonal_val_gyr, 0.0, 0.0, 0.0, diagonal_val_gyr]

        # --- Lógica de Orientación / Magnetómetro ---
        # (Aquí va tu lógica corregida de Tilt Compensation si la usas,
        #  o la original que tenías. Dejo la original por ahora para no romper nada extra)
        mxf = float(-(float(mx)-120.2))
        myf = float(float(my)-118.3)
        heading = atan2(myf,mxf) - 1.570796327    

        # Normalizar cuaternión entrante
        norm = sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
        qx /= norm; qy /= norm; qz /= norm; qw /= norm
        
        quaternion = [qx, qy, qz, qw]
        roll, pitch, yaw = euler_from_quaternion(quaternion, 'sxyz')
        
        # Sobreescribir Yaw con Heading (según tu código)
        yaw = heading
        qx, qy, qz, qw = quaternion_from_euler(roll, pitch, yaw, 'sxyz')

        imu_msg.orientation.x = float(qx)
        imu_msg.orientation.y = float(qy)
        imu_msg.orientation.z = float(qz)
        imu_msg.orientation.w = float(qw)

        imu_msg.orientation_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]

        # Publicar Topics
        heading_msg = Float32()
        heading_msg.data = heading

        self.pub_imu_raw.publish(imu_msg)
        self.pub_heading.publish(heading_msg)

        # Publicar Magnetómetro
        mag_msg.header.stamp = imu_msg.header.stamp
        mag_msg.header.frame_id = self.frame_id
        mag_msg.magnetic_field.x = -(float(mx)-120.2)
        mag_msg.magnetic_field.y = float(my)-118.3
        mag_msg.magnetic_field.z = float(mz)+67.1

        self.pub_mag.publish(mag_msg)

    # ... (El resto de tus métodos DataParseGPS, DataParseIMU, etc. siguen igual)
    # Copia aquí abajo tus funciones DataParseIMU, DataParseGPS, cmd_vel_callback, main, etc.
    # No olvides incluirlas para que el script funcione completo.

    # =============================================================
    #   PUBLICACIÓN GPS
    # =============================================================
    def publish_gps(self, buf):

        gps_msg = NavSatFix()

        lat, lon, alt, sats, status_char = self.DataParseGPS(buf)

        # Completar NavSatFix
        gps_msg.header.stamp = self.get_clock().now().to_msg()
        gps_msg.header.frame_id = "gps_frame"

        gps_msg.latitude  = float(lat)
        gps_msg.longitude = float(lon)
        gps_msg.altitude  = float(alt) / 3.2808399 #Lo convierto a metros osea ft -> m

        # -----------------------------
        # GPS status
        # -----------------------------
        gps_msg.status.service = NavSatStatus.SERVICE_GPS

        # A o D → FIX válido
        if status_char in (b'A', b'D'):
            gps_msg.status.status = NavSatStatus.STATUS_FIX
        else:
            gps_msg.status.status = NavSatStatus.STATUS_NO_FIX

        # Covarianzas 
        #gps_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        #gps_msg.position_covariance = [float('0')] * 9
        gps_msg.position_covariance = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0
        ]
        gps_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED

        # Publicar
        self.pub_gps.publish(gps_msg)

        # Log "DEBUG MODE" si el GPS manda 'D'
        if status_char == b'D':
            self.get_logger().info("DEBUG MODE activo: GPS")
            self.get_logger().info(
            f"GPS → Lat:{lat:.6f} Lon:{lon:.6f} Alt:{alt:.2f} "
            f"Sats:{sats} Status:{status_char.decode('ascii')}"
            )
        else :
            self.get_logger().info(
            f"GPS → Lat:{lat:.6f} Lon:{lon:.6f} Alt:{alt:.2f} "
            f"Sats:{sats} Status:{status_char.decode('ascii')}"
            )        

    # =============================================================
    #   PARSER IMU
    # =============================================================
    def DataParseIMU(self, buf):

        if buf[0] != 0x49:
            raise ValueError("Header IMU incorrecto")

        payload = buf[1:]
        expected_size = 9*4 + 4*4  # 52 bytes

        return struct.unpack('<9i4f', payload[:expected_size])


    # =============================================================
    #   PARSER GPS
    # =============================================================
    def DataParseGPS(self, buf):

        if buf[0] != 0x47:
            raise ValueError("Header GPS incorrecto")

        payload = buf[1:]

        lat, lon, alt = struct.unpack('<fff', payload[0:12])
        sats  = payload[12]
        status = payload[13:14]

        return lat, lon, alt, sats, status
    # =============================================================
    #   Leer Cmd_vel de turtlesim y mandarlo por uart 
    # =============================================================
    def cmd_vel_callback(self, msg: Twist):
        # tomo los valores del mensaje, float > int > uint8_t (unsigned char)
        self.aux_x = max(0, min(255, int(msg.linear.x*255)))
        self.aux_z = max(0, min(255, int(msg.angular.z*255 + 128)))

        if self.aux_x == self.linear_x and self.aux_z == self.angular_z:
            return

        self.linear_x = self.aux_x
        self.angular_z = self.aux_z
        self.get_logger().info(
            f'Linear X = {self.linear_x:.2f}, Angular Z = {self.angular_z:.2f}'
        )
    #def publish_cmd_vel(self):
        try:
            velocidades = struct.pack('<BB', self.linear_x, self.angular_z)
            # Mando como unsigned char (uint8_t)
            mensaje_vel = b'V' + velocidades
            self.ser.write(mensaje_vel)

            #self.get_logger().info(
            #    f"[UART] Enviado → v={self.linear_x:.2f}, w={self.angular_z:.2f}"
            #)
        except Exception as e:
            self.get_logger().error(f"error enviando cmd_vel al UART: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = UARTNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()