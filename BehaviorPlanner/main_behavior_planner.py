
class MainBehaviorPlanner():
    def __init__(self):
        self.AutonomousDrivingModeList = ["CC", "ACC", "STOP", "EMERGENCY"]
        self.ModeAD = None

    def decide_ACC_target_vehicle(self, VehicleObjects):
        # for surroundings in VehicleObjects:
            
        target_vehicle_ID = 0