import rclpy
from rclpy.node import Node
from autoware_auto_perception_msgs.msg import DetectedObjects, ObjectClassification, DetectedObject
from autoware_localization_msgs.msg import KinematicState
from nav_msgs.msg import Odometry
from copy import deepcopy
import yaml
import argparse
from std_msgs.msg import String
import json
import random
import threading


# Run with: python3 boundingBoxModifier.py --config attack_config.yaml

# Right now only messages of type "DetectedObjects" are supported!

# FILTERS
class ObjectFilter:
    def match(self, obj) -> bool:
        return True

#Filter objects based on their label
class LabelFilter(ObjectFilter):
    def __init__(self, allowed_labels):
        self.allowed_labels = allowed_labels

    def match(self, obj):
        if not obj.classification:
            return False
        return obj.classification[0].label in self.allowed_labels

#Filter objects in a specific range of x, y, z coordinates
class PositionFilter(ObjectFilter):
    def __init__(self, x_pos_min=0.0, x_pos_max=100.0, y_pos_min=-10.0, y_pos_max=10.0, z_pos_min=0.6, z_pos_max=2.0):
        self.x_pos_min = x_pos_min
        self.x_pos_max = x_pos_max
        self.y_pos_min = y_pos_min
        self.y_pos_max = y_pos_max
        self.z_pos_min = z_pos_min
        self.z_pos_max = z_pos_max

    def match(self, obj):
        pos = obj.kinematics.pose_with_covariance.pose.position
        return (self.x_pos_min <= pos.x <= self.x_pos_max and
                self.y_pos_min <= pos.y <= self.y_pos_max and
                self.z_pos_min <= pos.z <= self.z_pos_max)
    
#TODO: right now the same changes are applied to all objects that match filter 

# MODIFIERS / CREATORS (ATTACKS)
class ObjectModifier:
    def apply(self, obj):
        return obj
    
class ObjectCreator:
    def apply(self, objects, ego_pose=None):
        return objects, False

#Shift the bounding box
class ShiftPosition(ObjectModifier):
    def __init__(self, dx=1.0, dy=0.0, dz=0.0, gradual=False, gradual_x=0.0, gradual_y=0.0, gradual_z=0.0):
        self.dx = dx
        self.dy = dy
        self.dz = dz

        #TODO: gradual shift is global - if multiple objects fall in filter they affect each other!
        self.gradual = gradual
        self.gradual_x = gradual_x
        self.gradual_y = gradual_y
        self.gradual_z = gradual_z

    def apply(self, obj):
        pose = obj.kinematics.pose_with_covariance.pose
        pose.position.x += self.dx
        pose.position.y += self.dy
        pose.position.z += self.dz

        if self.gradual:
            # Update the shift for the next frame
            self.dx += self.gradual_x
            self.dy += self.gradual_y
            self.dz += self.gradual_z
        return obj

#Set a specific position for the bounding box
class SetPosition(ObjectModifier):
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

    def apply(self, obj):
        pose = obj.kinematics.pose_with_covariance.pose
        pose.position.x = self.x
        pose.position.y = self.y
        pose.position.z = self.z
        return obj

#Change the size of the bounding box by scaling the dimensions
class ScaleBoundingBox(ObjectModifier):
    def __init__(self, sx=1.0, sy=1.0, sz=1.0):
        self.sx = sx
        self.sy = sy
        self.sz = sz

    def apply(self, obj):
        shape = obj.shape
        shape.dimensions.x *= self.sx
        shape.dimensions.y *= self.sy
        shape.dimensions.z *= self.sz
        return obj
    
#Setting specific dimensions for the bounding box
class SetBoundingBoxDimensions(ObjectModifier):
    def __init__(self, x=1.0, y=1.0, z=1.0):
        self.sx = x
        self.sy = y
        self.sz = z

    def apply(self, obj):
        shape = obj.shape
        shape.dimensions.x = self.sx
        shape.dimensions.y = self.sy
        shape.dimensions.z = self.sz
        return obj

#Change label of object 
class Misclassify(ObjectModifier):
    def __init__(self, new_label):
        self.new_label = new_label

    def apply(self, obj):
        if obj.classification:
            obj.classification[0].label = self.new_label
        return obj

#Remove object entirely 
class DropObject(ObjectModifier):
    #Return None to remove object
    def apply(self, obj):
        return None
    
#TODO: add capability to add new fake objects - this is a bit more complex as need to fill in all the details of the object message (position, shape, classification etc.)
class SpawnObject(ObjectCreator):

    def __init__(
        self,
        existence_prob,
        label,
        init_pos_x,
        init_pos_y,
        init_pos_z,
        orient_x,
        orient_y,
        orient_z,
        orient_w,
        dim_x,
        dim_y,
        dim_z,
        gradual=False,
        gradual_x=0.0,
        gradual_y=0.0,
        gradual_z=0.0,
    ):
        self.existence_prob = existence_prob
        self.label = label

        self.x = init_pos_x
        self.y = init_pos_y
        self.z = init_pos_z

        self.ox = orient_x
        self.oy = orient_y
        self.oz = orient_z
        self.ow = orient_w

        self.dx = dim_x
        self.dy = dim_y
        self.dz = dim_z

        self.world_initialized = False

        self.world_x = None
        self.world_y = None
        self.world_z = None

        self.gradual = gradual
        self.gradual_x = gradual_x
        self.gradual_y = gradual_y
        self.gradual_z = gradual_z

        self.enabled = False

    def apply(self, objects, ego_pose=None):
        if not self.enabled:
            return objects, False
        
        if ego_pose is None:
            return objects, False
        
        if not self.world_initialized:
            self.world_x = ego_pose.position.x + self.x
            self.world_y = ego_pose.position.y + self.y
            self.world_z = ego_pose.position.z + self.z
            self.world_initialized = True

        obj = DetectedObject()

        obj.existence_probability = self.existence_prob

        cls = ObjectClassification()
        cls.label = self.label
        cls.probability = 1.0
        obj.classification.append(cls)

        pose = obj.kinematics.pose_with_covariance.pose

        x, y, z = self.world_to_ego(
            self.world_x,
            self.world_y,
            self.world_z,
            ego_pose
        )

        pose.position.x = x
        pose.position.y = y
        pose.position.z = z

        pose.orientation.x = self.ox
        pose.orientation.y = self.oy
        pose.orientation.z = self.oz
        pose.orientation.w = self.ow

        obj.shape.dimensions.x = self.dx
        obj.shape.dimensions.y = self.dy
        obj.shape.dimensions.z = self.dz

        objects.append(obj)

        #TODO: needs fix: should be applied to world coordinates
        """if self.gradual:
            self.x += self.gradual_x
            self.y += self.gradual_y
            self.z += self.gradual_z"""

        return objects, True
    
    #Helpers

    #Transform world coordinates into ego coordinates
    def world_to_ego(self, wx, wy, wz, ego_pose):
        ex = ego_pose.position.x
        ey = ego_pose.position.y
        ez = ego_pose.position.z

        return (
            wx - ex,
            wy - ey,
            wz - ez
        )

    


# ATTACK PIPELINE 
class AttackPipeline:
    def __init__(self, filters=None, modifiers=None, creators=None, activation_probability=1.0):
        self.filters = filters or []
        self.modifiers = modifiers or []
        self.creators = creators or []
        self.activation_probability = activation_probability

    def _matches(self, obj):
        return all(f.match(obj) for f in self.filters)

    def _apply_modifiers(self, obj):
        for mod in self.modifiers:
            obj = mod.apply(obj)
            if obj is None:
                return None
        return obj

    def process(self, obj):
        if not self._matches(obj):
            return obj, False
        return self._apply_modifiers(obj), True

    def process_batch(self, objects, ego_pose=None):
        result = []
        attack_applied = False

        #Only activate the attack in a frame with a certain prob. 
        if random.random() > self.activation_probability:
            return objects, False

        for obj in objects:
            new_obj, attacked = self.process(obj)
            if new_obj is not None:
                result.append(new_obj)

            #For attack status message - label if at least one object modified
            if attacked:
                attack_applied = True
        
        for creator in self.creators:
            result, applied = creator.apply(result, ego_pose)
            attack_applied = attack_applied or applied

        return result, attack_applied



# ROS2 NODE
class BoundingBoxModifier(Node):

    def __init__(self, pipeline, input_topic, output_topic):
        super().__init__('bounding_box_modifier')

        self.pipeline = pipeline

        self.sub = self.create_subscription(
            DetectedObjects,
            input_topic,
            self.callback,
            10
        )

        self.pub = self.create_publisher(
            DetectedObjects,
            output_topic,
            10
        )

        self.get_logger().info(f"Subscribed to: {input_topic}")
        self.get_logger().info(f"Publishing to: {output_topic}")

        #For publishing message when attack is active
        self.attack_status_pub = self.create_publisher(
            String,
            "/attack/status",
            10
        )

        self.ego_state = None

        self.create_subscription(
            Odometry,
            "/localization/kinematic_state",
            self.ego_state_callback,
            10
        )

    def callback(self, msg: DetectedObjects):
        out_msg = DetectedObjects()
        out_msg.header = msg.header

        objects = [deepcopy(obj) for obj in msg.objects]
        ego_pose = self.get_ego_pose()

        out_msg.objects, attack_applied = self.pipeline.process_batch(objects, ego_pose)

        self.pub.publish(out_msg)

        #Publish attack status message
        status_msg = String()
        payload = {
            "attack_name": "BoundingBoxModifier", #TODO: make this more specific based on which modifiers are applied
            "timestamp_sec": msg.header.stamp.sec,
            "timestamp_nanosec": msg.header.stamp.nanosec,
            "attack_applied": attack_applied
        }
        status_msg.data = json.dumps(payload)
        self.attack_status_pub.publish(status_msg)

    def ego_state_callback(self, msg):
        self.ego_state = msg

    def get_ego_pose(self):
        if self.ego_state is None:
            return None

        return self.ego_state.pose.pose


def build_pipeline_from_yaml(config_path, available_filters, available_modifiers, available_creators):
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)

    print("\nLoaded YAML Config:")
    print(yaml.dump(config, sort_keys=False))

    pipeline_cfg = config.get("pipeline", {})

    filters = []
    for f_cfg in pipeline_cfg.get("filters", []):
        name = f_cfg["type"]
        if name not in available_filters:
            raise ValueError(f"Unknown filter: {name}")
        cls = available_filters[name]
        params = f_cfg.get("params", {})
        filters.append(cls(**params))

    activation_cfg = pipeline_cfg.get("activation", {})
    activation_probability = activation_cfg.get("probability", 1.0)

    modifiers = []
    for m_cfg in pipeline_cfg.get("modifiers", []):
        name = m_cfg["type"]
        if name not in available_modifiers:
            raise ValueError(f"Unknown modifier: {name}")
        cls = available_modifiers[name]
        params = m_cfg.get("params", {})
        modifiers.append(cls(**params))

    creators = []
    for c_cfg in pipeline_cfg.get("creators", []):
        name = c_cfg["type"]
        if name not in available_creators:
            raise ValueError(f"Unknown creator: {name}")

        cls = available_creators[name]
        params = c_cfg.get("params", {})
        creators.append(cls(**params))

    return (
        AttackPipeline(filters=filters, modifiers=modifiers, creators=creators, activation_probability=activation_probability),
        config["topics"]["input"],
        config["topics"]["output"]
    )

#Helper function for appearing attack
def keyboard_listener(spawn_creator):
    print("Press 's' + Enter to toggle SpawnObject.")

    while True:
        if input().strip().lower() == "s":
            spawn_creator.enabled = not spawn_creator.enabled
            state = "ENABLED" if spawn_creator.enabled else "DISABLED"
            print(f"SpawnObject {state}")


# MAIN
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--config",
        required=True,
        help="Path to YAML config file"
    )
    args = parser.parse_args()

    # Register available classes - when specifying new filters / modifiers add here
    available_filters = {
        "LabelFilter": LabelFilter,
        "PositionFilter": PositionFilter,
    }

    available_modifiers = {
        "ShiftPosition": ShiftPosition,
        "SetPosition": SetPosition,
        "Misclassify": Misclassify,
        "DropObject": DropObject,
        "SetBoundingBoxDimensions": SetBoundingBoxDimensions,
        "ScaleBoundingBox": ScaleBoundingBox,
    }

    available_creators = {
        "SpawnObject": SpawnObject,
    }

    pipeline, input_topic, output_topic = build_pipeline_from_yaml(
        args.config,
        available_filters,
        available_modifiers,
        available_creators
    )

    rclpy.init()

    node = BoundingBoxModifier(
        pipeline=pipeline,
        input_topic=input_topic,
        output_topic=output_topic
    )

    node.get_logger().info(
        f"Loaded pipeline: {len(pipeline.filters)} filters, "
        f"{len(pipeline.modifiers)} modifiers, "
        f"{len(pipeline.creators)} creators"
    )

    spawn_creator = next((c for c in pipeline.creators if isinstance(c, SpawnObject)), None) #check whether spawncreator used

    if spawn_creator is not None:
        threading.Thread(
            target=keyboard_listener,
            args=(spawn_creator,),
            daemon=True
        ).start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down (Ctrl+C)")

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()