import rclpy
from rclpy.node import Node
from lidar_attack_msgs.msg import AttackConfig, Patch


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
    def set_config(self, patches, enabled=True):
        msg = AttackConfig()
        msg.enabled = enabled

        for p in patches:
            patch_msg = Patch()
            patch_msg.x_offset = p["x_offset"]
            patch_msg.y_offset = p["y_offset"]
            patch_msg.z_offset = p["z_offset"]
            patch_msg.patch_size = p["patch_size"]
            patch_msg.side = p["side"]
            patch_msg.stabilize_anchor = p.get("stabilize_anchor", True)
            msg.patches.append(patch_msg)

        self.current_msg = msg

        self.get_logger().info(
            f"Updated attack config | enabled={enabled} | patches={len(patches)}"
        )

    # Publish continously - TODO: this is actually probably not needed 
    def _timer_callback(self):
        if self.current_msg is not None:
            self.pub.publish(self.current_msg)


def main():
    rclpy.init()
    node = AttackPublisher()

    """
    example_patches = [
        {
            "x_offset": 0.7,
            "y_offset": 0.4,
            "z_offset": 1.0,
            "patch_size": 0.4,
            "side": "right",
            "stabilize_anchor": False
        },
        {
            "x_offset": -0.9,
            "y_offset": 0.5,
            "z_offset": 1.5,
            "patch_size": 0.4,
            "side": "left",
            "stabilize_anchor": False
        }
    ]"""

    """example_patches = [
        {
            "x_offset": 0.83,
            "y_offset": 0.49,
            "z_offset": 0.87,
            "patch_size": 0.4,
            "side": "right",
            "stabilize_anchor": False
        },
        {
            "x_offset": -0.94,
            "y_offset": 0.88,
            "z_offset": 1.84,
            "patch_size": 0.4,
            "side": "left",
            "stabilize_anchor": False
        },
        {
            "x_offset": 0.92,
            "y_offset": 0.21,
            "z_offset": 0.93,
            "patch_size": 0.4,
            "side": "left",
            "stabilize_anchor": False
        },
    {
            "x_offset": -0.36,
            "y_offset": -0.25,
            "z_offset": 0.27,
            "patch_size": 0.4,
            "side": "right",
            "stabilize_anchor": False
        },
        {
            "x_offset": 0.81,
            "y_offset": -0.23,
            "z_offset": 2.25,
            "patch_size": 0.4,
            "side": "left",
            "stabilize_anchor": False
        }
    ]"""

    example_patches = [
        {
            "x_offset": 0.43,
            "y_offset": 0.91,
            "z_offset": 1.58,
            "patch_size": 0.4,
            "side": "left",
            "stabilize_anchor": False
        },
        {
            "x_offset": 1.45,
            "y_offset": -0.51,
            "z_offset": 1.73,
            "patch_size": 0.4,
            "side": "right",
            "stabilize_anchor": False
        },
        {
            "x_offset": -0.31,
            "y_offset": 0.64,
            "z_offset": 0.26,
            "patch_size": 0.4,
            "side": "right",
            "stabilize_anchor": False
        },
    {
            "x_offset": -0.23,
            "y_offset": -0.11,
            "z_offset": 0.74,
            "patch_size": 0.4,
            "side": "left",
            "stabilize_anchor": False
        },
        {
            "x_offset": 2.4,
            "y_offset": 0.49,
            "z_offset": 2.2,
            "patch_size": 0.4,
            "side": "left",
            "stabilize_anchor": False
        }
    ]

    node.set_config(example_patches, enabled=True)

    # Keep node alive so config keeps being published
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

#Uncomment if this is run standalone
if __name__ == '__main__':
    main()

