import rclpy
import json
from rclpy.node import Node
from lidar_attack_msgs.msg import AttackConfig
from patch_config_publisher import AttackPublisher
import yaml
import argparse

# To disable attack after running: 
#python run_attack.py --disable

#Specify different config file:
#python run_attack.py --config attack_config.yaml

def main():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--config",
        default="attack_config.yaml",
        help="Path to attack YAML config"
    )

    parser.add_argument(
        "--disable",
        action="store_true",
        help="Disable attack publishing"
    )

    args = parser.parse_args()

    with open(args.config, "r") as f:
        config = yaml.safe_load(f)

    attack_cfg = config["attack_run_info"]

    enabled = (
        attack_cfg.get("enabled", True)
        and not args.disable
    )

    #TODO: right now only lidar_object_patch attack supported - for other attacks need different stuff here and error handling
    parameters = {
        "attack_name": config["attack_metadata"]["attack_name"],
        "patches": config["patches"],
        "front_filter": config["front_filter"]
    }

    if attack_cfg["attack_type"] != "lidar_object_patch":
        print("Attack type not supported, no attack publishing!")
        return


    rclpy.init()
    node = AttackPublisher()

    #node.set_config(attack_type="lidar_object_patch", attack_class="Hiding Attack", parameters=example_patches, enabled=True)
    node.set_config(
        attack_type=attack_cfg["attack_type"],
        attack_class=attack_cfg["attack_class"],
        parameters=parameters,
        enabled=enabled
    )

    # Keep node alive so config keeps being published
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

#Uncomment if this is run standalone
if __name__ == '__main__':
    main()

