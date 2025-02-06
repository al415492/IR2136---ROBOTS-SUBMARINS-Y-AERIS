import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Point
from pymavlink import mavutil
import time

class BatteryGPSNode(Node):
    def __init__(self):
        super().__init__('battery_gps_node')
        
        # Crear el objeto de conexión MAVLink
        self.connection = mavutil.mavlink_connection('udp:127.0.0.1:14550')
        self.connection.wait_heartbeat()  # Esperar al primer "heartbeat" para confirmar la conexión
        self.get_logger().info('Conectado al SITL (Software In The Loop)')

        # Publicar el estado de la batería
        self.battery_publisher = self.create_publisher(Float32, 'battery_status', 10)
        
        # Suscribirse a las coordenadas GPS del objetivo
        self.target_gps_subscriber = self.create_subscription(
            Point, 
            'target_gps', 
            self.target_gps_callback, 
            10
        )
        
        # Llamar a la función de publicación de batería periódicamente
        self.timer = self.create_timer(1.0, self.publish_battery_status)

    def publish_battery_status(self):
        # Leer el estado de la batería usando MAVLink
        msg = self.connection.recv_match(type='BATTERY_STATUS', blocking=False)
        if msg:
            battery_percentage = msg.battery_remaining  # Porcentaje de batería restante
            battery_msg = Float32()
            battery_msg.data = battery_percentage
            self.battery_publisher.publish(battery_msg)
            self.get_logger().info(f'Batería: {battery_percentage}%')
        else:
            self.get_logger().warn('No se pudo leer el estado de la batería.')

    def target_gps_callback(self, msg):
        # Recibir la coordenada GPS y enviar al dron
        lat = msg.x  # Latitud
        lon = msg.y  # Longitud
        alt = msg.z  # Altitud

        # Enviar al dron el comando para moverse a la nueva posición
        self.send_gps_to_drone(lat, lon, alt)

    def send_gps_to_drone(self, lat, lon, alt):
        # Comando para mover el dron a la posición recibida (usando MAVLink)
        self.connection.mav.set_position_target_global_int_send(
            0,  # Timestamp
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,  # Marco de referencia de altitud relativa
            0b110111111000,  # Usar velocidad y aceleración
            int(lat * 1e7),  # Latitud como entero
            int(lon * 1e7),  # Longitud como entero
            alt,  # Altitud en metros
            0, 0, 0,
            0, 0, 0,  
            0, 0
        )
        self.get_logger().info(f'Moviendo el dron a Lat:{lat}, Lon:{lon}, Alt:{alt}m.')

def main(args=None):
    rclpy.init(args=args)

    node = BatteryGPSNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
