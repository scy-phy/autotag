import rclpy
import json
from rclpy.node import Node
from lidar_attack_msgs.msg import AttackConfig


class AttackPublisher(Node):

    def __init__(self):
        super().__init__('attack_publisher')

        # Use simple depth QoS (compatible with CARLA bridge)
        self.pub = self.create_publisher(
            AttackConfig,
            '/lidar_attack/config',
            10
        )

        self.current_msg = None

        # Republish at specific rate 
        self.timer = self.create_timer(0.5, self._timer_callback)

    # Set the config
    def set_config(self, attack_type, attack_class, parameters, enabled=True):
        msg = AttackConfig()
        msg.enabled = enabled
        msg.attack_type = attack_type
        msg.attack_class = attack_class
        msg.parameter_json = json.dumps(parameters)

        self.current_msg = msg

        self.get_logger().info(
            f"Updated attack config | {attack_type} | enabled={enabled}"
        )

    # Publish continously - TODO: this is actually probably not needed 
    def _timer_callback(self):
        if self.current_msg is not None:
            self.pub.publish(self.current_msg)

