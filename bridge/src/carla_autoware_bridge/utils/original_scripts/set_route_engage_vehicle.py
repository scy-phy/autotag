import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from autoware_adapi_v1_msgs.srv import SetRoutePoints, ChangeOperationMode, ClearRoute

class AutowareAPIController(Node):
    def __init__(self):
        super().__init__('autoware_api_controller')
        
        # Initialize AD API Service Clients
        self.route_client = self.create_client(SetRoutePoints, '/api/routing/set_route_points')
        self.engage_client = self.create_client(ChangeOperationMode, '/api/operation_mode/change_to_autonomous')
        self.clear_route_client = self.create_client(ClearRoute, '/api/routing/clear_route')

    def send_route(self, goal_coords, waypoint_coords_list=None):
        if waypoint_coords_list is None:
            waypoint_coords_list = []

        self.get_logger().info('Waiting for routing service...')
        self.route_client.wait_for_service()

        req = SetRoutePoints.Request()
        req.header.frame_id = 'map'
        req.header.stamp = self.get_clock().now().to_msg()
        
        # 1. Set the Final Goal
        req.goal.position.x = float(goal_coords[0])
        req.goal.position.y = float(goal_coords[1])
        req.goal.position.z = float(goal_coords[2])
        req.goal.orientation.w = 1.0  # Default facing forward

        # 2. Append Optional Waypoints (Checkpoints)
        for wp in waypoint_coords_list:
            pose = Pose()
            pose.position.x = float(wp[0])
            pose.position.y = float(wp[1])
            pose.position.z = float(wp[2])
            pose.orientation.w = 1.0
            req.waypoints.append(pose)

        self.get_logger().info('Sending route request to Autoware...')
        future = self.route_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def engage_vehicle(self):
        self.get_logger().info('Waiting for operation mode service...')
        self.engage_client.wait_for_service()

        req = ChangeOperationMode.Request()
        
        self.get_logger().info('Engaging Autonomous Mode...')
        future = self.engage_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()
    
    def clear_route(self):
        self.get_logger().info('Waiting for clear route service')
        self.clear_route_client.wait_for_service()

        req = ClearRoute.Request()

        self.get_logger().info('Clearing Route If Any...')
        future = self.clear_route_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

def main(args=None):
    rclpy.init(args=args)
    node = AutowareAPIController()
    
    # Define your coordinates here (based on the CARLA map)
    # Format: [x, y, z]

    """
    Coordinates:

    Lower right corner:
        position:
            x: 79.78695678710938
            y: -134.97999572753906
            z: 0.0
    Top right corner:
        position:
            x: 103.70005798339844
            y: 37.24365997314453
            z: 0.0
    Top left corner:
        position:
            x: -86.4452896118164
            y: 61.70002746582031
            z: 0.0
    Lower left corner:
        position:
            x: -97.85589599609375
            y: -119.13723754882812
            z: 0.0
    Goal: (rough start position)
        position:
            x: -51.107513427734375
            y: -137.6417236328125
            z: 0.0
    """

    """target_goal = [61.3809814453125, -138.8399658203125, 0.0] 
    target_waypoints = [
        [-3.9495954513549805, -138.27919006347656, 0.0]
    ]"""

    #target_goal = [-51.107513427734375, -137.6417236328125, 0.0] 
    target_goal = [61.3809814453125, -138.8399658203125, 0.0] 
    target_waypoints = [
        [79.78695678710938, -134.97999572753906, 0.0],
        #[106.36577606201172, 20.710391998291016, 0.0],
        #[-86.4452896118164, 61.70002746582031, 0.0],
        #[-97.85589599609375, -119.13723754882812, 0.0]
    ]
    
    # Execute the sequence
    clear_route_response = node.clear_route()
    node.get_logger().info(f'Clear Route Status: Code {clear_route_response.status.code}')

    route_response = node.send_route(target_goal, target_waypoints)
    node.get_logger().info(f'Set Route Status: Code {route_response.status.code}')
    
    if route_response.status.success:
        engage_response = node.engage_vehicle()
        node.get_logger().info(f'Engage Status: Code {engage_response.status.code}')
    else:
        print(route_response)
        node.get_logger().error('Failed to set route. Aborting engage command.')
        
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
