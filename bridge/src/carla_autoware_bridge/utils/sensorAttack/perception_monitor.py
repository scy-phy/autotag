import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from autoware_auto_perception_msgs.msg import DetectedObjects, PredictedObjects
import math
import time
from collections import deque, Counter

TOPICS = {
    "centerpoint": ("/perception/object_recognition/detection/centerpoint/objects", DetectedObjects),
    "detection": ("/perception/object_recognition/detection/objects", DetectedObjects),
    "final": ("/perception/object_recognition/objects", PredictedObjects),
    "fusion": ("/perception/object_recognition/detection/clustering/camera_lidar_fusion/objects", DetectedObjects)
}

FRONT_FILTER_X_MIN = 3.0
FRONT_FILTER_X_MAX = 50.0
FRONT_FILTER_Y_MAX = 5.0
N_FRAMES = 100

class MultiPerceptionMonitor(Node):
    def __init__(self):
        super().__init__('multi_perception_monitor')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE
        )

        self.buffers = {name: deque(maxlen=N_FRAMES) for name in TOPICS.keys()}
        self.frame_counters = {name: 0 for name in TOPICS.keys()}
        self.last_print_time = time.time()
        self.last_summary = None

        # Subscribers - subscribe to the different topics 
        self.subscribers = {}
        for name, (topic, msg_type) in TOPICS.items():
            self.subscribers[name] = self.create_subscription(
                msg_type,
                topic,
                lambda msg, n=name: self.update_state(msg, n),
                qos_profile
            )
            self.get_logger().info(f"Subscribed to {topic} ({msg_type.__name__})")

    def reset_buffers(self):
        for key in self.buffers.keys():
            self.buffers[key].clear()

    def update_state(self, msg, topic_name):
        """Collect a new frame into the buffer"""
        frame_total = len(msg.objects)
        frame_front_labels = []
        frame_front_class_probs = []
        frame_front_exist_probs = []

        front_detected = 0
        front_car = 0

        for obj in msg.objects:
            # Detect pose
            pose = None
            if hasattr(obj.kinematics, "pose_with_covariance"): #This is for DetectedObjects
                pose = obj.kinematics.pose_with_covariance.pose
            elif hasattr(obj.kinematics, "initial_pose_with_covariance"): #This is for PredictedObjects
                pose = obj.kinematics.initial_pose_with_covariance.pose

            if pose:
                x = pose.position.x
                y = pose.position.y
                distance = math.sqrt(x ** 2 + y ** 2)

                # Front filter for DetectedObjects
                if isinstance(msg, DetectedObjects) and x > 0 and FRONT_FILTER_X_MIN < distance < FRONT_FILTER_X_MAX and abs(y) < FRONT_FILTER_Y_MAX:
                    label = obj.classification[0].label if obj.classification else -1
                    class_prob = obj.classification[0].probability if obj.classification else 0.0
                    exist_prob = getattr(obj, "existence_probability", 0.0)

                    frame_front_labels.append(label)
                    frame_front_class_probs.append(class_prob)
                    frame_front_exist_probs.append(exist_prob)

                    front_detected += 1
                    if label == 1:  # label 1 = car
                        front_car += 1

                #TODO: add filtering here as well 
                # For PredictedObjects, just count labels - filtering is not that easy here as predicted objects are in map coordinates not relative to sensor
                if isinstance(msg, PredictedObjects):
                    label = obj.classification[0].label if obj.classification else -1
                    frame_front_labels.append(label)

        # Add frame to buffer
        self.buffers[topic_name].append({
            "total": frame_total,
            "front_labels": frame_front_labels,
            "front_class_probs": frame_front_class_probs, # classification prob. (front obj.)
            "front_exist_probs": frame_front_exist_probs, # existence prob. (front obj.)
            "front_detected": front_detected, # count any object detected in front
            "front_car": front_car # count how many classified as cars 
        })
        self.frame_counters[topic_name] += 1

        # Print summary every N_FRAMES for this topic
        """if self.frame_counters[topic_name] >= N_FRAMES:
            self.print_summary()
            self.frame_counters[topic_name] = 0"""


    def compute_front_detection(self, topic_name):
        buffer = self.buffers.get(topic_name, [])
        if not buffer:
            return (0, 0.0, 0.0, 0.0, 0.0)

        total_frames = len(buffer)

        detected_frames = sum(1 for f in buffer if f["front_detected"] > 0)
        correct_car_frames = sum(1 for f in buffer if f["front_car"] > 0)

        detection_rate = detected_frames / total_frames
        classification_rate = correct_car_frames / total_frames

        class_probs = []
        exist_probs = []

        for f in buffer:
            class_probs.extend(f["front_class_probs"])
            exist_probs.extend(f["front_exist_probs"])

        avg_class_prob = sum(class_probs)/len(class_probs) if class_probs else 0.0
        avg_exist_prob = sum(exist_probs)/len(exist_probs) if exist_probs else 0.0

        #TODO: better turn this into a dictionary
        return (
            total_frames,
            detection_rate,
            classification_rate,
            avg_class_prob,
            avg_exist_prob
        )
    
    def compute_max_confidence(self, topic_name):
        buffer = self.buffers.get(topic_name, [])
        if not buffer:
            return (0, 0.0, 0.0, 0.0, 0.0)

        total_frames = len(buffer)

        detected_frames = sum(1 for f in buffer if f["front_detected"] > 0)
        correct_car_frames = sum(1 for f in buffer if f["front_car"] > 0)

        detection_rate = detected_frames / total_frames
        classification_rate = correct_car_frames / total_frames

        class_probs = []
        exist_probs = []

        for f in buffer:
            class_probs.extend(f["front_class_probs"])
            exist_probs.extend(f["front_exist_probs"])

        max_class_prob = max(class_probs) if class_probs else 0.0
        max_exist_prob = max(exist_probs) if exist_probs else 0.0

        #TODO: better turn this into a dictionary
        return (
            total_frames,
            detection_rate,
            classification_rate,
            max_class_prob,
            max_exist_prob
        )
    
    def print_front_detection_metrics(self):
        """
        Print detection and classification rates for all relevant topics
        (excluding prediction).
        """

        print("\n" + "=" * 60)
        print("Front Object Detection Metrics")

        for topic_name in self.buffers.keys():

            if topic_name == "final":
                continue  # skip predicted objects for now

            total_frames, detection_rate, classification_rate, avg_class_prob, avg_exist_prob = \
                self.compute_front_detection(topic_name)

            print(f"\nTopic: {topic_name}")
            print(f"  Total frames:        {total_frames}")
            print(f"  Detection rate:      {detection_rate:.3f}")
            print(f"  Correct car rate:    {classification_rate:.3f}")
            print(f"  Avg class prob:      {avg_class_prob:.3f}")
            print(f"  Avg existence prob:  {avg_exist_prob:.3f}")

        print("=" * 60)

    #TODO: this is not ideally
    def print_summary(self):
        """Compute summary over last N_FRAMES and print"""
        output = []

        for topic, buffer in self.buffers.items():
            if not buffer:
                continue

            # Average total objects
            total_avg = sum(f["total"] for f in buffer) / len(buffer)

            # Front label occurrences
            label_occurrences = Counter()
            for f in buffer:
                label_occurrences.update(f["front_labels"])

            # Compute number of frames each label appeared in
            label_frame_count = {label: count for label, count in label_occurrences.items()}

            # Only for detected objects: front objects
            front_count = len(label_frame_count) if topic != "final" else None

            output.append((topic, total_avg, front_count, label_frame_count))

        print("\n" + "="*60)
        print(f"Perception summary @ {time.strftime('%H:%M:%S')}")
        for topic, total_avg, front_count, label_frame_count in output:
            print(f"\nTopic: {topic}")
            print(f"  Avg total objects: {total_avg:.2f}")
            if front_count is not None:
                print(f"  Front filtered objects: {front_count}")
            print(f"  Label occurrences in last {N_FRAMES} frames: {label_frame_count}")
        print("="*60)


def main():
    rclpy.init()
    node = MultiPerceptionMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


"""if __name__ == "__main__":
    main()"""



