import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from autoware_adapi_v1_msgs.srv import SetRoutePoints, ChangeOperationMode, ClearRoute
import argparse
import yaml
import sys

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
        req.goal.position.x = float(goal_coords[0][0])
        req.goal.position.y = float(goal_coords[0][1])
        req.goal.position.z = float(goal_coords[0][2])
        req.goal.orientation.x = float(goal_coords[1][0])
        req.goal.orientation.y = float(goal_coords[1][1])
        req.goal.orientation.z = float(goal_coords[1][2])
        req.goal.orientation.w = float(goal_coords[1][3])

        # 2. Append Optional Waypoints (Checkpoints)
        for wp in waypoint_coords_list:
            pose = Pose()
            pose.position.x = float(wp[0][0])
            pose.position.y = float(wp[0][1])
            pose.position.z = float(wp[0][2])
            pose.orientation.x = float(wp[1][0])
            pose.orientation.y = float(wp[1][1])
            pose.orientation.z = float(wp[1][2])
            pose.orientation.w = float(wp[1][3])
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
    # Parse the custom configuration argument, but use parse_known_args so we don't block standard ROS 2 terminal arguments
    argparser = argparse.ArgumentParser(description='Autoware Route Setter')
    argparser.add_argument('--config', default='default_config.yaml', help='Path to config.yaml')
    parsed_args, ros_args = argparser.parse_known_args(sys.argv)

    rclpy.init(args=ros_args)
    
    # Load YAML Configuration
    with open(parsed_args.config, 'r') as f:
        config = yaml.safe_load(f)

    scenario = config.get('scenario', {})
    route_config = scenario.get('route', {})

    custom_route = route_config.get('customRoute', [])
    route_id = route_config.get('id', 1)

    target_goal = []
    target_waypoints = []

    # Parse Route Data
    if custom_route and len(custom_route) > 0:
        print("Using customRoute array from config.yaml...")
        target_goal = custom_route[-1]       # The last coordinate block in the array
        target_waypoints = custom_route[:-1] # Everything except the last coordinate block
    
    elif route_id == 1: #Full outer circle
        print("Using Route 1")
        #Each point: position (x,y,z), orientation (x, y, z, w)
        target_goal =  [(-80.41688537597656, -133.84925842285156, 0.0), (0.0, 0.0, -0.2, 1.0)] 
        target_waypoints = [
            [(61.3809814453125, -138.8399658203125, 0.0), (0.0, 0.0, 0.0, 1.0)],
            [(106.18518829345703, 22.528865814208984, 0.0), (0.0, 0.0, 0.7, 0.7)],
            [(-73.5740966796875, 65.62899780273438, 0.0), (0.0, 0.0, 1.0, 0.0)],
            [(-111.03147888183594, -84.86442565917969, 0.0), (0.0, 0.0, -0.7, 0.7)]
        ]

    elif route_id == 2: #Longer route covering almost all streets (including lane change)
        print ("Using Route 2")
        target_goal = [(-111.03147888183594, -84.86442565917969, 0.0), (0.0, 0.0, -0.7, 0.7)] 
        target_waypoints = [
            [(61.3809814453125, -138.8399658203125, 0.0), (0.0, 0.0, 0.0, 1.0)],
            [(63.55352020263672, -13.056787490844727, 0.0), (0.0, 0.0, 1.0, 0.0)],
            [(-41.76060485839844, 37.60853576660156, 0.0), (0.0, 0.0, 0.7, 0.7)],
            [(59.625579833984375, 61.30542755126953, 0.0), (0.0, 0.0, 0.0, 1.0)],
            [(-2.1737747192382812, -131.2568359375, 0.0), (0.0, 0.0, 1.0, 0.0)],
            [(-41.852386474609375, -84.33612060546875, 0.0), (0.0, 0.0, 0.7, 0.7)],
            [(-45.400474548339844, -46.20709228515625, 0.0), (0.0, 0.0, 0.7, 0.7)],
            [(-111.03147888183594, -84.86442565917969, 0.0), (0.0, 0.0, -0.7, 0.7)]
        ]

    elif route_id == 3: # Straight line 
        print ("Using Route 3")
        target_goal = [(61.3809814453125, -138.8399658203125, 0.0), (0.0, 0.0, 0.0, 1.0)]
        target_waypoints = []

    else:
        print("Invalid route configuration. Please define 'customRoute' or use a valid 'id'. Exiting.")
        sys.exit(1)

    node = AutowareAPIController()

    # Execute the sequence
    clear_route_response = node.clear_route()
    node.get_logger().info(f'Clear Route Status: Code {clear_route_response.status.code}')

    route_response = node.send_route(target_goal, target_waypoints)
    node.get_logger().info(f'Set Route Status: Code {route_response.status.code}')

    if route_response.status.success:
        engage_response = node.engage_vehicle()
        print(engage_response)
        node.get_logger().info(f'Engage Status: Code {engage_response.status.code}')

    else:
        print(route_response)
        node.get_logger().error('Failed to set route. Aborting engage command.')
        
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
