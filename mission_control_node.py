import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Point
from pymavlink import mavutil
import argparse
import time

class MissionControlNode(Node):
    def __init__(self, target_lat, target_lon, target_alt):
        super().__init__('mission_control_node')

        # Crear el objeto de conexión MAVLink
        self.connection = mavutil.mavlink_connection('udp:127.0.0.1:14550')
        self.connection.wait_heartbeat()  # Esperar al primer "heartbeat" para confirmar la conexión
        self.get_logger().info('Conectado al SITL (Software In The Loop)')

        # Publicar la posición GPS objetivo
        self.target_gps_publisher = self.create_publisher(Point, 'target_gps', 10)

        # Publicar el comando de aterrizaje
        self.landing_publisher = self.create_publisher(String, 'drone_landing', 10)

        # Llamar a la función de monitoreo de batería periódicamente
        self.timer = self.create_timer(5.0, self.monitor_battery_status)

        # Publicar la posición GPS objetivo inmediatamente
        self.publish_target_gps(target_lat, target_lon, target_alt)

    def publish_target_gps(self, lat, lon, alt):
        # Crear el mensaje GPS
        gps_msg = Point()
        gps_msg.x = lat
        gps_msg.y = lon
        gps_msg.z = alt

        # Publicar la posición GPS
        self.target_gps_publisher.publish(gps_msg)
        self.get_logger().info(f'Publicando objetivo GPS: Lat:{lat}, Lon:{lon}, Alt:{alt}')

    def monitor_battery_status(self):
        # Leer el estado de la batería usando MAVLink
        msg = self.connection.recv_match(type='BATTERY_STATUS', blocking=False)
        if msg:
            battery_percentage = msg.battery_remaining  # Porcentaje de batería restante
            self.get_logger().info(f'Batería: {battery_percentage}%')

            # Si la batería cae por debajo del 20%, enviar el comando de aterrizaje
            if battery_percentage <= 20:
                self.get_logger().warn('Batería baja (<20%). Iniciando aterrizaje de emergencia.')
                self.initiate_landing()

    def initiate_landing(self):
        # Enviar comando de aterrizaje al dron
        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0,  # Confirmación
            0, 0, 0, 0, 0, 0, 0  # No se necesitan parámetros adicionales
        )
        self.landing_publisher.publish(String(data="Landing initiated"))
        self.get_logger().info("Comando de aterrizaje enviado al dron.")

def main(args=None):
    rclpy.init(args=args)

    # Parsear los argumentos de la línea de comandos para recibir las coordenadas GPS
    parser = argparse.ArgumentParser(description='Mission Control Node')
    parser.add_argument('latitude', type=float, help='Latitud del objetivo GPS')
    parser.add_argument('longitude', type=float, help='Longitud del objetivo GPS')
    parser.add_argument('altitude', type=float, help='Altitud del objetivo GPS en metros')
    args = parser.parse_args()

    # Crear el nodo con las coordenadas recibidas
    node = MissionControlNode(args.latitude, args.longitude, args.altitude)

    # Mantener el nodo en ejecución
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
