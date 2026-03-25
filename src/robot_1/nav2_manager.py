#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav2_msgs.action import FollowWaypoints
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient

class WaypointSender(Node):
    def __init__(self):
        super().__init__('waypoint_sender')
        self._action_client = ActionClient(
            self, 
            FollowWaypoints, 
            'follow_waypoints'
        )
        
    def send_waypoints(self, waypoints_gps):
        """
        waypoints_gps: lista de tuplas [(lat1, lon1), (lat2, lon2), ...]
        """
        goal_msg = FollowWaypoints.Goal()
        
        for lat, lon in waypoints_gps:
            pose = PoseStamped()
            pose.header.frame_id = 'map'  # o 'utm' si usas UTM
            pose.header.stamp = self.get_clock().now().to_msg()
            
            # Convierte GPS a coordenadas map/utm
            # Esto depende de tu configuración de robot_localization
            # Si usas navsat_transform con broadcast_utm_transform: true
            # Las coordenadas ya estarán en UTM
            pose.pose.position.x = lon  # ajustar conversión
            pose.pose.position.y = lat  # ajustar conversión
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            
            goal_msg.poses.append(pose)
        
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)
    
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Waypoint actual: {feedback.current_waypoint}')
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rechazado')
            return
        
        self.get_logger().info('Goal aceptado')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Resultado: {result}')

def main():
    rclpy.init()
    node = WaypointSender()
    
    # Ejemplo: waypoints GPS
    waypoints = [
        (-34.9011, -57.9547),  # lat, lon
        (-34.9015, -57.9550),
        (-34.9020, -57.9555)
    ]
    
    node.send_waypoints(waypoints)
    rclpy.spin(node)

if __name__ == '__main__':
    main()