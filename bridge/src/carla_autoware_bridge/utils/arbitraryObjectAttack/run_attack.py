import rclpy
import json
from rclpy.node import Node
from lidar_attack_msgs.msg import AttackConfig
from patch_config_publisher import AttackPublisher


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

    example_patches = {
        "patches": [
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
    }

    node.set_config(attack_type="lidar_object_patch", attack_class="Hiding Attack", parameters=example_patches, enabled=True)

    # Keep node alive so config keeps being published
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

#Uncomment if this is run standalone
if __name__ == '__main__':
    main()

