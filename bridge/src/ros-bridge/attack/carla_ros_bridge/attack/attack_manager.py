from .arbitrary_object_patch_attack import ObjectPatchAttack
from .base_lidar_attack import BaseLidarAttack
import json

class AttackManager:

    ATTACKS = {
        "lidar_object_patch": ObjectPatchAttack,
    }

    def __init__(self):
        self.enabled = False
        self.attack_instance = None

    def update_config(self, msg):

        self.enabled = msg.enabled

        attack_type = msg.attack_type
        attack_cls = self.ATTACKS.get(attack_type)

        parameters = json.loads(msg.parameter_json)

        if attack_cls:
            self.attack_instance = attack_cls(parameters)
        else:
            print(f"Unknown lidar attack type: {msg.attack_type}")
            self.attack_instance = None

    def apply(self, lidar_data):

        if not self.enabled:
            return lidar_data, False, None

        if self.attack_instance is None:
            return lidar_data, False, None
        
        modified_lidar_data, attack_applied = self.attack_instance.apply(lidar_data)

        return modified_lidar_data, attack_applied, self.attack_instance.NAME 
    