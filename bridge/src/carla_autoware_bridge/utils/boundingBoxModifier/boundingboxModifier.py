import rclpy
from rclpy.node import Node
from autoware_auto_perception_msgs.msg import DetectedObjects
from copy import deepcopy
import yaml
import argparse


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
    
#TODO: right now every the same changes are applied to all objects that match filter 

# MODIFIERS (ATTACKS)
class ObjectModifier:
    def apply(self, obj):
        return obj

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
    


# ATTACK PIPELINE 
class AttackPipeline:
    def __init__(self, filters=None, modifiers=None):
        self.filters = filters or []
        self.modifiers = modifiers or []

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
            return obj
        return self._apply_modifiers(obj)

    def process_batch(self, objects):
        result = []
        for obj in objects:
            new_obj = self.process(obj)
            if new_obj is not None:
                result.append(new_obj)
        return result



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

    def callback(self, msg: DetectedObjects):
        out_msg = DetectedObjects()
        out_msg.header = msg.header

        objects = [deepcopy(obj) for obj in msg.objects]
        out_msg.objects = self.pipeline.process_batch(objects)

        self.pub.publish(out_msg)


def build_pipeline_from_yaml(config_path, available_filters, available_modifiers):
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

    modifiers = []
    for m_cfg in pipeline_cfg.get("modifiers", []):
        name = m_cfg["type"]
        if name not in available_modifiers:
            raise ValueError(f"Unknown modifier: {name}")
        cls = available_modifiers[name]
        params = m_cfg.get("params", {})
        modifiers.append(cls(**params))

    return (
        AttackPipeline(filters=filters, modifiers=modifiers),
        config["topics"]["input"],
        config["topics"]["output"]
    )


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

    pipeline, input_topic, output_topic = build_pipeline_from_yaml(
        args.config,
        available_filters,
        available_modifiers
    )

    rclpy.init()

    node = BoundingBoxModifier(
        pipeline=pipeline,
        input_topic=input_topic,
        output_topic=output_topic
    )

    node.get_logger().info(
        f"Loaded pipeline: {len(pipeline.filters)} filters, "
        f"{len(pipeline.modifiers)} modifiers"
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down (Ctrl+C)")

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()