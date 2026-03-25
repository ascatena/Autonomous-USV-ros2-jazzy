#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.duration import Duration

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
        self.declare_parameter('teleop_timeout', 1.0) 

        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value
        self.frame_id = self.get_parameter('frame_id').value
        self.teleop_timeout = self.get_parameter('teleop_timeout').value

        self.get_logger().info(f"Abrir UART: {port} @ {baud}")

        self.ser = serial.Serial(port=port, baudrate=baud, timeout=1)

        # Publicadores
        self.pub_imu_raw = self.create_publisher(Imu, '/imu', QoSProf)
        self.pub_mag     = self.create_publisher(MagneticField, '/mpu9250/mag', QoSProf)
        self.pub_gps     = self.create_publisher(NavSatFix, '/gps/fix', QoSProf)
        self.pub_heading = self.create_publisher(Float32, '/heading', QoSProf)
        
        # --- LÓGICA DE PRIORIDAD DE VELOCIDAD ---
        self.linear_x = 0.0  
        self.angular_z = 0.0 
        
        # Guardamos el tiempo del último mensaje de teleop. 
        self.last_teleop_time = self.get_clock().now() - Duration(seconds=10)

        # 1. Suscriptor PRIORITARIO (Manual / Teleop)
        self.sub_teleop = self.create_subscription(
            Twist,
            '/turtle1/cmd_vel',
            self.teleop_callback,
            10
        )

        # 2. Suscriptor SECUNDARIO (Nav2 / Autónomo)
        self.sub_nav2 = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.nav2_callback,
            10
        )

        # Timer a 30 Hz para lectura de sensores
        self.timer = self.create_timer(0.033, self.read_uart)
        # Timer de la señal StillAlive
        self.still_alive_timer = self.create_timer(2.0, self.send_still_alive)
        
        # --- VARIABLES DE CALIBRACIÓN ---
        self.is_calibrated = False
        self.calibration_start_time = None
        self.calibration_duration = 20.0 
        
        self.calib_buffer = {
            'ax': [], 'ay': [], 'az': [],
            'gx': [], 'gy': [], 'gz': []
        }
        
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

    def publish_imu(self, buf):
       
        
        # -- Inicio Resumen del código previo --
        ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw, mx, my, mz, qx, qy, qz, qw = self.DataParseIMU(buf)
        phys_ax = -float(ay_raw)/ 16384 * 9.80665
        phys_ay = -float(ax_raw)/ 16384 * 9.80665
        phys_az = float(az_raw)/ 16384 * 9.80665
        phys_gx = float(gy_raw)/32.8 * 3.14159265/180
        phys_gy = float(gx_raw)/32.8 * 3.14159265/180
        phys_gz = -float(gz_raw)/32.8 * 3.14159265/180

        if not self.is_calibrated:
            current_time = self.get_clock().now()
            if self.calibration_start_time is None: self.calibration_start_time = current_time
            elapsed = (current_time - self.calibration_start_time).nanoseconds / 1e9
            if elapsed < self.calibration_duration:
                self.calib_buffer['ax'].append(phys_ax); self.calib_buffer['ay'].append(phys_ay); self.calib_buffer['az'].append(phys_az)
                self.calib_buffer['gx'].append(phys_gx); self.calib_buffer['gy'].append(phys_gy); self.calib_buffer['gz'].append(phys_gz)
                if len(self.calib_buffer['ax']) % 30 == 0: self.get_logger().info(f"Calibrando... {elapsed:.1f}/{self.calibration_duration}s")
                return
            else:
                count = len(self.calib_buffer['ax'])
                if count > 0:
                    self.imu_offsets['ax'] = sum(self.calib_buffer['ax']) / count
                    self.imu_offsets['ay'] = sum(self.calib_buffer['ay']) / count
                    self.imu_offsets['az'] = (sum(self.calib_buffer['az']) / count) - 9.80665
                    self.imu_offsets['gx'] = sum(self.calib_buffer['gx']) / count
                    self.imu_offsets['gy'] = sum(self.calib_buffer['gy']) / count
                    self.imu_offsets['gz'] = sum(self.calib_buffer['gz']) / count
                    self.get_logger().info(f"--- CALIBRACION FINALIZADA ---")
                self.is_calibrated = True
                self.calib_buffer.clear()

        phys_ax -= self.imu_offsets['ax']; phys_ay -= self.imu_offsets['ay']; phys_az -= self.imu_offsets['az']
        phys_gx -= self.imu_offsets['gx']; phys_gy -= self.imu_offsets['gy']; phys_gz -= self.imu_offsets['gz']

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
        diagonal_val_acc = 1.0
        imu_msg.linear_acceleration_covariance = [diagonal_val_acc, 0.0, 0.0, 0.0, diagonal_val_acc, 0.0, 0.0, 0.0, diagonal_val_acc]
        diagonal_val_gyr = 0.01
        imu_msg.angular_velocity_covariance = [diagonal_val_gyr, 0.0, 0.0, 0.0, diagonal_val_gyr, 0.0, 0.0, 0.0, diagonal_val_gyr]

        mxf = float(-(float(mx)-120.2))
        myf = float(float(my)-118.3)
        heading = atan2(myf,mxf) - 1.570796327
        norm = sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
        qx /= norm; qy /= norm; qz /= norm; qw /= norm
        quaternion = [qx, qy, qz, qw]
        roll, pitch, yaw = euler_from_quaternion(quaternion, 'sxyz')
        yaw = heading
        qx, qy, qz, qw = quaternion_from_euler(roll, pitch, yaw, 'sxyz')
        imu_msg.orientation.x = float(qx); imu_msg.orientation.y = float(qy)
        imu_msg.orientation.z = float(qz); imu_msg.orientation.w = float(qw)
        imu_msg.orientation_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
        
        heading_msg = Float32()
        heading_msg.data = heading
        self.pub_imu_raw.publish(imu_msg)
        self.pub_heading.publish(heading_msg)

        mag_msg.header.stamp = imu_msg.header.stamp
        mag_msg.header.frame_id = self.frame_id
        mag_msg.magnetic_field.x = -(float(mx)-120.2)
        mag_msg.magnetic_field.y = float(my)-118.3
        mag_msg.magnetic_field.z = float(mz)+67.1
        self.pub_mag.publish(mag_msg)
        # -- Fin Resumen --

    def publish_gps(self, buf):
        # Mismo código que tenías
        gps_msg = NavSatFix()
        lat, lon, alt, sats, status_char = self.DataParseGPS(buf)
        gps_msg.header.stamp = self.get_clock().now().to_msg()
        gps_msg.header.frame_id = "gps_frame"
        gps_msg.latitude  = float(lat)
        gps_msg.longitude = float(lon)
        gps_msg.altitude  = float(alt) / 3.2808399 
        gps_msg.status.service = NavSatStatus.SERVICE_GPS
        if status_char in (b'A', b'D'): gps_msg.status.status = NavSatStatus.STATUS_FIX
        else: gps_msg.status.status = NavSatStatus.STATUS_NO_FIX
        gps_msg.position_covariance = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        gps_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        self.pub_gps.publish(gps_msg)
        if status_char == b'D': self.get_logger().info(f"DEBUG MODE GPS Lat:{lat:.6f} Lon:{lon:.6f}")
        else: self.get_logger().info(f"GPS Lat:{lat:.6f} Lon:{lon:.6f} Sats:{sats}")

    def DataParseIMU(self, buf):
        if buf[0] != 0x49: raise ValueError("Header IMU incorrecto")
        payload = buf[1:]
        expected_size = 9*4 + 4*4 
        return struct.unpack('<9i4f', payload[:expected_size])

    def DataParseGPS(self, buf):
        if buf[0] != 0x47: raise ValueError("Header GPS incorrecto")
        payload = buf[1:]
        lat, lon, alt = struct.unpack('<fff', payload[0:12])
        sats  = payload[12]
        status = payload[13:14]
        return lat, lon, alt, sats, status

    # =============================================================
    #   CALLBACKS DE VELOCIDAD (MUX LOGIC)
    # =============================================================

    # 1. CALLBACK MANUAL (PRIORITARIO)
 
    def teleop_callback(self, msg: Twist):
        # Actualizamos el timestamp de la última orden manual
        self.last_teleop_time = self.get_clock().now()
        
        # --- LOGICA DE "IMPULSO AL GIRAR" ---
        # Detectamos si el usuario quiere girar (z != 0) pero no está acelerando (x == 0).
        # Esto sucede típimanete en teleop al presionar solo Izquierda/Derecha.
        
        target_x = msg.linear.x
        target_z = msg.angular.z

        # Usamos 0.01 como umbral para compensar errores de punto flotante
        if abs(target_x) < 0.01 and abs(target_z) > 0.01:
            # ¡Detectado intento de giro estático!
            # Forzamos la marcha adelante para que el timón funcione
            target_x = 1.0 
            # target_z se queda como estaba (1.0 o -1.0)
            
            # (Opcional) Log para depurar que la lógica funciona
            # self.get_logger().info(f"Giro estático detectado: Forzando X=1.0, Z={target_z}")

        # Creamos un mensaje modificado para enviarlo a la función de procesamiento
        mod_msg = Twist()
        mod_msg.linear.x = float(target_x)
        mod_msg.angular.z = float(target_z)
        
        # Procesamos y enviamos inmediatamente
        self.process_and_send_vel(mod_msg)
    # 2. CALLBACK NAV2 (SECUNDARIO)
    def nav2_callback(self, msg: Twist):
        # Calculamos tiempo transcurrido desde el último comando manual
        current_time = self.get_clock().now()
        time_diff = current_time - self.last_teleop_time
        seconds_passed = time_diff.nanoseconds / 1e9

        # Si pasaron más de X segundos (ej 1.0), el teleop está inactivo -> Usamos Nav2
        if seconds_passed > self.teleop_timeout:
            # self.get_logger().info("NAV2 CONTROL ACTIVE")
            self.process_and_send_vel(msg)
        else:
            # Si hace poco recibimos comandos manuales, IGNORAMOS a Nav2
            pass

    # =============================================================
    #   HELPER PARA ENVIAR VELOCIDAD
    # =============================================================
    def process_and_send_vel(self, msg: Twist):
        # Lógica común de conversión y envío por UART
        aux_x = max(0, min(255, int(msg.linear.x * 255)))
        aux_z = max(0, min(255, int(msg.angular.z * 127 + 128)))

        # Evitar enviar datos repetidos si no cambiaron
        if aux_x == self.linear_x and aux_z == self.angular_z:
            return

        self.linear_x = aux_x
        self.angular_z = aux_z
        
        # self.get_logger().info(f'TX Vel -> X: {self.linear_x}, Z: {self.angular_z}')

        try:
            velocidades = struct.pack('<BB', self.linear_x, self.angular_z)
            mensaje_vel = b'V' + velocidades
            self.ser.write(mensaje_vel)
        except Exception as e:
            self.get_logger().error(f"error enviando cmd_vel al UART: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = UARTNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()