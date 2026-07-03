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
        self.attack_type = None
        self.parameters = None

    def update_config(self, msg):

        new_enabled = msg.enabled
        new_attack_type = msg.attack_type
        new_parameters = json.loads(msg.parameter_json)

        # Nothing changed
        if (new_enabled == self.enabled and new_attack_type == self.attack_type and new_parameters == self.parameters):
            return

        self.enabled = new_enabled

        attack_cls = self.ATTACKS.get(new_attack_type)

        if not attack_cls:
            print(f"Unknown lidar attack type: {new_attack_type}")
            self.attack_instance = None
            self.attack_type = None
            self.parameters = None
            return

        # New attack type -> create fresh instance
        if (self.attack_instance is None or self.attack_type != new_attack_type):
            print(f"Creating attack: {new_attack_type}")

            self.attack_instance = attack_cls(new_parameters)

        # Same attack type, parameters changed
        elif new_parameters != self.parameters:
            print(f"Updating attack parameters: {new_attack_type}")

            self.attack_instance.update_parameters(new_parameters)

        elif new_enabled != self.enabled:
            print(f"Disable attack")
            self.attack_instance.disable_attack()

        self.attack_type = new_attack_type
        self.parameters = new_parameters

    def apply(self, lidar_data):

        if not self.enabled:
            return lidar_data, False, None

        if self.attack_instance is None:
            return lidar_data, False, None
        
        modified_lidar_data, attack_applied = self.attack_instance.apply(lidar_data)

        return modified_lidar_data, attack_applied, self.attack_instance.NAME 
    