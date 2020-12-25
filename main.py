import os 
import math
import numpy as np
import matplotlib.pyplot as plt

from SimulatorUtils import simulator_settings as sets
from Animation import main_animation_module as animation
from VehicleModel import main_vehicle_model as vhclmodel
from Sensing import main_sensing as sensing
# from BehaviorPlanner import main_behavior_planner as behaviorplanner
from MotionPlanner import main_motion_planner as motionplanner
from LowlevelController import main_lowlevel_controller as lwlvcontroller

class SimulatorMain():
    def __init__(self):
        # Initialize
        ''' === Create instance === '''
        self.drawer = animation.MainAnimation()
        self.perception = sensing.MainSensing()
        self.controller = lwlvcontroller.MainController()
        self.trajectory_planner = motionplanner.MainMotionPlanner()
        print("Initialized")

    def draw_image(self):
        # Generate simulator images
        tmp = 0

    def simulation(self):
        for n in range(sets.Num_episodes):
            ''' === Setup Simulator Environment === '''
            # Set each vehicle's parameters
            Car0 = vhclmodel.MainVehicleModel(0, 0, 18, 0, 0, 0, 0, 0, 4.8, 1.7, sets.Ts)
            Car1 = vhclmodel.MainVehicleModel(10, 3.5, 18, 0, 0, 0, 0, 0, 4.75, 1.75, sets.Ts)
            Car2 = vhclmodel.MainVehicleModel(-5, 3.5, 16, 0, 0, 0, 0, 0, 4.75, 1.75, sets.Ts)
            Car3 = vhclmodel.MainVehicleModel(30, 0, 19, 0, 0, 0, 0, 0, 4.75, 1.75, sets.Ts)
            Car4 = vhclmodel.MainVehicleModel(-10, 0, 18, 0, 0, 0, 0, 0, 4.75, 1.75, sets.Ts)
            self.trajectory_planner.path_generation()
            target_idx, vhcl_idx, L_lat = self.trajectory_planner.search_target_index(Car0)

            ''' === Main Process === '''
            for m in range(sets.MaxLoopTimes):
                ''' 描画設定 '''
                self.drawer.plot_lane(self.trajectory_planner.path, target_idx, vhcl_idx)
                # yaw角度変化を表示したい時
                self.drawer.plot_rectangle(Car0, Car1, Car2, Car3, Car4)

                ''' Autonomous Driving System '''
                # [Step1] Sensing

                # [Step2] Behavior Planner 
                
                # [Step3] Motion Planner
                # 注視点更新
                target_idx, vhcl_idx, L_lat = self.trajectory_planner.search_target_index(Car0)

                # [Step4] Lowlevel Controller
                # 制御演算
                delta, vhcl_id = self.controller.pure_pursuit(Car0, self.trajectory_planner, target_idx)
                # delta, _ = self.controller.kanayama_method(Car0, self.trajectory_planner, vhcl_idx)

                # [Step5] Update simulation environment
                # 車両状態更新
                Car0.state_update(delta, 0.0)
                Car1.state_update(delta, 0.0)
                Car2.state_update(delta, 0.0)
                Car3.state_update(delta, 0.0)
                Car4.state_update(delta, 0.0)

            ''' === Finalize Main Process === '''
            self.drawer.close_figure()
            print("")
            

if __name__ == "__main__":
    simulation = SimulatorMain()
    simulation.simulation()
