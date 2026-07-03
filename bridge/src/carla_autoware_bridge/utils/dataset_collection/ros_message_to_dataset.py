import os
from tokenize import String
import cv2
import yaml
import json
import numpy as np
import time
from datetime import datetime, timezone
import os
import shutil

import rclpy

from rclpy.node import Node
from rclpy.qos import QoSProfile
from rosidl_runtime_py.utilities import get_message
from rosidl_runtime_py.convert import message_to_ordereddict
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from sensor_msgs.msg import Image, PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import String



# Helpers

def ensure_dir(path):
    os.makedirs(path, exist_ok=True)


def timestamp_ns(msg):

    if hasattr(msg, "header") and hasattr(msg.header, "stamp"):

        return (
            msg.header.stamp.sec * 1_000_000_000
            + msg.header.stamp.nanosec
        )
    
    elif hasattr(msg, "stamp"):
        return (
            msg.stamp.sec * 1_000_000_000
            + msg.stamp.nanosec
        )
    
    else:
        return None



# Main Recorder

class GenericDatasetRecorder(Node):

    def __init__(self, config_path):

        super().__init__("generic_dataset_recorder")

        # time.sleep(1)  # Allow time for ROS graph to initialize

        
        # Load YAML config --> specifying topics to record and output directory
        with open(config_path, "r") as f:
            self.full_config = yaml.safe_load(f)
        self.config = self.full_config["dataset"]
        self.scenario = self.full_config["scenario"]

        self.output_dir = self.config["output_dir"]

        self.topic_type_map = {}

        ensure_dir(self.output_dir)

        self.json_dir = os.path.join(
            self.output_dir,
            "json"
        )

        self.image_dir = os.path.join(
            self.output_dir,
            "images"
        )

        self.lidar_dir = os.path.join(
            self.output_dir,
            "lidar"
        )

        ensure_dir(self.json_dir)
        ensure_dir(self.image_dir)
        ensure_dir(self.lidar_dir)

        # Discover ROS topic types
        req_topics = {entry["topic"] for cat in ["json_topics", "image_topics", "lidar_topics"] 
                      for entry in self.config.get(cat, {}).values()}
        
        self.get_logger().info(f"Waiting for {len(req_topics)} requested topics to appear...")

        # Poll until topics found
        start_time = time.time()
        while time.time() - start_time < 15.0:
            if req_topics.issubset(dict(self.get_topic_names_and_types()).keys()):
                self.get_logger().info("All topics discovered!")
                break

            # Block until any new topic joins the DDS network - function does not exist
            #graph_event = self.get_node_graph_interface().get_graph_event()
            #self.get_node_graph_interface().wait_for_graph_change(graph_event, timeout_ns=1_000_000_000)
            rclpy.spin_once(self, timeout_sec=1.0)

        for topic_name, topic_types in (
            self.get_topic_names_and_types()
        ):

            if len(topic_types) > 0:
                self.topic_type_map[topic_name] = (
                    topic_types[0]
                )

        self.get_logger().info(
            f"Discovered {len(self.topic_type_map)} topics"
        )

        #Write metadata
        self.write_dataset_metadata()

        # Open JSONL files
        self.json_files = {}

        # QoS for bag playback / recording life data (needs to be compatible with Autoware)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE
        )

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            durability=DurabilityPolicy.VOLATILE
        )

        
        # Structured JSON topics (all except images/lidar)
        json_topics = self.config.get(
            "json_topics",
            {}
        )

        for dataset_name, entry in json_topics.items():

            topic_name = entry["topic"]

            if topic_name not in self.topic_type_map:

                self.get_logger().warn(
                    f"Topic not found: {topic_name}"
                )

                continue

            type_str = self.topic_type_map[topic_name]

            try:

                msg_type = get_message(type_str)

                self.create_subscription(
                    msg_type,
                    topic_name,
                    lambda msg,
                    t=topic_name,
                    n=dataset_name:
                    self.record_json(msg, t, n),
                    qos_profile
                )

                filepath = os.path.join(
                    self.json_dir,
                    f"{dataset_name}.jsonl"
                )

                self.json_files[dataset_name] = open(
                    filepath,
                    "a"
                )

                self.get_logger().info(
                    f"JSON topic: {topic_name}"
                )

            except Exception as e:

                self.get_logger().error(
                    f"Failed JSON topic {topic_name}: {e}"
                )

        
        # Image topics --> stored as PNG
        image_topics = self.config.get(
            "image_topics",
            {}
        )

        for dataset_name, entry in image_topics.items():

            topic_name = entry["topic"]

            self.create_subscription(
                Image,
                topic_name,
                lambda msg,
                n=dataset_name:
                self.record_image(msg, n),
                sensor_qos
            )

            ensure_dir(
                os.path.join(
                    self.image_dir,
                    dataset_name
                )
            )

            self.get_logger().info(
                f"Image topic: {topic_name}"
            )

        
        # LiDAR topics
        lidar_topics = self.config.get(
            "lidar_topics",
            {}
        )

        for dataset_name, entry in lidar_topics.items():

            topic_name = entry["topic"]

            self.create_subscription(
                PointCloud2,
                topic_name,
                lambda msg,
                n=dataset_name:
                self.record_lidar(msg, n),
                sensor_qos
            )

            ensure_dir(
                os.path.join(
                    self.lidar_dir,
                    dataset_name
                )
            )

            self.get_logger().info(
                f"LiDAR topic: {topic_name}"
            )


        #Custom message for attack labeling
        #self.attack_events = {}
        self.current_attack = (False, "None")

        try:
            self.create_subscription(
                String,
                "/attack/status",
                self.attack_callback,
                sensor_qos
            )

        except Exception as e:
            self.get_logger().error(
                f"Failed to create attack subscription: {e}"
            )
    
    def check_attack_status(self, timestamp_ns):

        #Here attack labeling based on attack message
        #Could implement something different here

        #This is set based on message 
        return self.current_attack
    

    # JSON recording
    def record_json(
        self,
        msg,
        topic_name,
        dataset_name
    ):

        try:

            data = message_to_ordereddict(msg)
            msg_timestamp = timestamp_ns(msg)
            
            frame_is_attacked = self.check_attack_status(msg_timestamp)

            entry = {
                "_topic": topic_name,
                "_type": type(msg).__name__,
                "_data": data,
                "_is_attacked": frame_is_attacked[0] if frame_is_attacked else False,
                "_attack_name": frame_is_attacked[1] if frame_is_attacked else "None",
            }

            if msg_timestamp and msg_timestamp > 0:
                entry["_timestamp_ns"] = msg_timestamp

            json.dump(
                entry,
                self.json_files[dataset_name]
            )

            self.json_files[
                dataset_name
            ].write("\n")

        except Exception as e:

            self.get_logger().error(
                f"JSON record failed: {e}"
            )


    def attack_callback(self, msg):

        data = json.loads(msg.data)

        self.current_attack = (
            data["attack_applied"],
            data["attack_name"]
        )

        #To do logic based on timestamp - discarded
        #ts = round(data["timestamp_sec"] * 1_000_000_000 + data["timestamp_nanosec"], -8) # Round to nearest 100ms 
        #self.attack_events[ts] = (data["attack_applied"], data["attack_name"])


    
    # Image recording
    def record_image(
        self,
        msg,
        dataset_name
    ):

        try:

            timestamp = timestamp_ns(msg.header)

            img = np.frombuffer(
                msg.data,
                dtype=np.uint8
            )

            channels = int(
                len(msg.data)
                / (msg.height * msg.width)
            )

            img = img.reshape(
                msg.height,
                msg.width,
                channels
            )

            output_dir = os.path.join(
                self.image_dir,
                dataset_name
            )

            filename = os.path.join(
                output_dir,
                f"{timestamp}.png"
            )

            cv2.imwrite(filename, img)

        except Exception as e:

            self.get_logger().error(
                f"Image save failed: {e}"
            )

    
    # LiDAR recording
    def record_lidar(
        self,
        msg,
        dataset_name
    ):

        try:

            timestamp = timestamp_ns(msg.header)

            # Structured numpy array
            structured = pc2.read_points(
                msg,
                field_names=("x", "y", "z"),
                skip_nans=True
            )

            # Convert -> Nx3 float32 array
            points = np.stack(
                [
                    structured["x"],
                    structured["y"],
                    structured["z"]
                ],
                axis=-1
            ).astype(np.float32)

            output_dir = os.path.join(
                self.lidar_dir,
                dataset_name
            )

            filename = os.path.join(
                output_dir,
                f"{timestamp}.npy"
            )

            np.save(filename, points)

        except Exception as e:

            self.get_logger().error(
                f"Lidar save failed: {e}"
            )

    #Store metadata information about dataset / scenario #TODO: custom stuff needs to be added!
    def write_dataset_metadata(self):

        metadata = {
            "dataset_name": self.output_dir,
            "creation_time": datetime.now(
                timezone.utc
            ).isoformat(),
            "scenario": {},
            "sensors": {},
            "json_topics": {}
        }

        #Attack information
        attack_cfg = self.full_config.get("attack")

        if attack_cfg:

            #TODO: sometimes specific target car is set, ideally this placement would also be specified here???

            config_file = attack_cfg.get("attack_path")

            if not config_file or not os.path.isfile(config_file):
                raise FileNotFoundError(f"Attack config file not found: {config_file}; either disable attack or specify valid config file")
            
            copied_name = os.path.basename(config_file)
            shutil.copy(config_file, os.path.join(self.output_dir, copied_name))

            metadata["attack"] = {
                "config_file": copied_name,

                #"name": attack_cfg.get("name"),
                #"description": attack_cfg.get("description")
            }

            if "name" in attack_cfg:
                metadata["attack"]["name"] = attack_cfg["name"]

            if "description" in attack_cfg:
                metadata["attack"]["description"] = attack_cfg["description"]

        # Scenario information
        metadata["scenario"] = {
            "route_id": (
                self.scenario
                .get("route", {})
                .get("id")
            ),

            "custom_route_waypoints": (
                self.scenario
                .get("route", {})
                .get("customRoute", [])
            ),

            "traffic_level": (
                self.scenario
                .get("traffic", {})
                .get("level")
            ),

            "custom_traffic_overrides": (
                self.scenario
                .get("traffic", {})
                .get("customTraffic", {})
            ),

            "traffic_seed": (
                self.scenario
                .get("traffic", {})
                .get("seed")
            ),

            "weather_environment": (
                self.scenario
                .get("weather", {})
                .get("environment")
            ),
        }

        #Extra scenario information if specified
        if "target_vehicle" in self.scenario:
            metadata["scenario"]["target_vehicle"] = self.scenario["target_vehicle"]

        # Image sensors
        for sensor_name, entry in (
            self.config.get(
                "image_topics",
                {}
            ).items()
        ):

            topic = entry["topic"]

            metadata["sensors"][
                sensor_name
            ] = {
                "topic": topic,
                "type": self.topic_type_map.get(
                    topic,
                    "unknown"
                )
            }


        # LiDAR sensors
        for sensor_name, entry in (
            self.config.get(
                "lidar_topics",
                {}
            ).items()
        ):

            topic = entry["topic"]

            metadata["sensors"][
                sensor_name
            ] = {
                "topic": topic,
                "type": self.topic_type_map.get(
                    topic,
                    "unknown"
                )
            }

        # JSON topics
        for name, entry in (
            self.config.get(
                "json_topics",
                {}
            ).items()
        ):

            topic = entry["topic"]

            metadata["json_topics"][
                name
            ] = {
                "topic": topic,
                "type": self.topic_type_map.get(
                    topic,
                    "unknown"
                )
            }

        with open(
            os.path.join(
                self.output_dir,
                "metadata.json"
            ),
            "w"
        ) as f:

            json.dump(
                metadata,
                f,
                indent=2
            )

    
    # Cleanup
    def close(self):

        for f in self.json_files.values():
            f.close()





# Main
def main(args=None):

    rclpy.init(args=args)

    config_path = "default_config.yaml"

    node = GenericDatasetRecorder(
        config_path
    )

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        node.get_logger().info(
            "Stopping recorder..."
        )

    finally:

        node.close()

        node.destroy_node()

        rclpy.shutdown()


if __name__ == "__main__":

    main()