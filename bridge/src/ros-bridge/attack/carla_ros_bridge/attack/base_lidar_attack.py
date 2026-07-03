
# Base attack class - no logic here
class BaseLidarAttack():

    NAME = "Base: No attack"
    
    def apply(self, lidar_data):
        pass
    
    def update_parameters(self, parameters):
        pass

    def disable_attack(self):
        pass