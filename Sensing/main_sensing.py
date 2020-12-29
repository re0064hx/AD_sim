import sys
sys.path.append('../')
import numpy as np
import pandas as pd

# Objectクラスの定義
class Object():
    def __init__(self, ID, Type):
        self.ObjectID = ID
        self.ObjectType = Type
        self.x_relative = 0.0
        self.y_relative = 0.0
        self.vx_relative = 0.0
        self.vy_relative = 0.0
        self.theta_relative = 0.0

# ObjectInfomationクラスの定義
class ObjectsInfo():
    def __init__(self):
        # self.MaxDetectableObjects = 4
        self.VehicleObjects = [Object(0, "Car"), Object(1, "Car"), Object(2, "Car"), Object(3, "Car")]

# Sensingクラスの定義
class MainSensing():
    def __init__(self):
        self.path = np.zeros([])
        self.cx = np.zeros([])
        self.cy = np.zeros([])
        self.theta_r = np.zeros([])
        self.rho_r = np.zeros([])
        self.Objects = ObjectsInfo()

        # for instance in self.Objects.VehicleObjects:
        #     print(instance.ObjectID)

        self.old_nearest_point_index = None
        print("Sensing module sucsessfully initialized.")

    def path_generation(self):
        # csv_input = pd.read_csv('reference_path.csv')
        csv_input = pd.read_csv('SimulatorUtils/oval_course.csv')
        self.path = csv_input.values
        self.cx = self.path[:, 0]
        self.cy = self.path[:, 1]
        self.theta_r = np.arctan2(self.cy, self.cx)

    def calc_relative_position(self, EgoVehicle, SurroundingVehicle):
        x_relative = EgoVehicle.X - SurroundingVehicle.X
        y_relative = EgoVehicle.Y - SurroundingVehicle.Y
        return x_relative, y_relative

    def calc_relative_velocity(self, EgoVehicle, SurroundingVehicle):
        vx_relative = EgoVehicle.Vx - SurroundingVehicle.Vx
        vy_relative = EgoVehicle.Vy - SurroundingVehicle.Vy
        return vx_relative, vy_relative

    def calc_relative_angle(self, EgoVehicle, SurroundingVehicle):
        return EgoVehicle.theta - SurroundingVehicle.theta
    
    def generate_object_info(self, EgoVehicle, Car1, Car2, Car3, Car4):
        # # Relative Position
        # self.Objects.Object1.x_relative, self.Objects.Object1.y_relative = self.calc_relative_position(Car1, EgoVehicle)
        # self.Objects.Object2.x_relative, self.Objects.Object2.y_relative = self.calc_relative_position(Car2, EgoVehicle)
        # self.Objects.Object3.x_relative, self.Objects.Object3.y_relative = self.calc_relative_position(Car3, EgoVehicle)
        # self.Objects.Object4.x_relative, self.Objects.Object4.y_relative = self.calc_relative_position(Car4, EgoVehicle)
        # # Relative Velocity
        # self.Objects.Object1.vx_relative, self.Objects.Object1.vy_relative = self.calc_relative_velocity(Car1, EgoVehicle)
        # self.Objects.Object2.vx_relative, self.Objects.Object2.vy_relative = self.calc_relative_velocity(Car2, EgoVehicle)
        # self.Objects.Object3.vx_relative, self.Objects.Object3.vy_relative = self.calc_relative_velocity(Car3, EgoVehicle)
        # self.Objects.Object4.vx_relative, self.Objects.Object4.vy_relative = self.calc_relative_velocity(Car4, EgoVehicle)
        # # Relative Angle
        # self.Objects.Object1.theta_relative = self.calc_relative_angle(Car1, EgoVehicle)
        # self.Objects.Object2.theta_relative = self.calc_relative_angle(Car2, EgoVehicle)
        # self.Objects.Object3.theta_relative = self.calc_relative_angle(Car3, EgoVehicle)
        # self.Objects.Object4.theta_relative = self.calc_relative_angle(Car4, EgoVehicle)

        # Relative Position
        self.Objects.VehicleObjects[0].x_relative, self.Objects.VehicleObjects[0].y_relative = self.calc_relative_position(Car1, EgoVehicle)
        self.Objects.VehicleObjects[1].x_relative, self.Objects.VehicleObjects[1].y_relative = self.calc_relative_position(Car2, EgoVehicle)
        self.Objects.VehicleObjects[2].x_relative, self.Objects.VehicleObjects[2].y_relative = self.calc_relative_position(Car3, EgoVehicle)
        self.Objects.VehicleObjects[3].x_relative, self.Objects.VehicleObjects[3].y_relative = self.calc_relative_position(Car4, EgoVehicle)
        # Relative Velocity
        self.Objects.VehicleObjects[0].vx_relative, self.Objects.VehicleObjects[0].vy_relative = self.calc_relative_velocity(Car1, EgoVehicle)
        self.Objects.VehicleObjects[1].vx_relative, self.Objects.VehicleObjects[1].vy_relative = self.calc_relative_velocity(Car2, EgoVehicle)
        self.Objects.VehicleObjects[2].vx_relative, self.Objects.VehicleObjects[2].vy_relative = self.calc_relative_velocity(Car3, EgoVehicle)
        self.Objects.VehicleObjects[3].vx_relative, self.Objects.VehicleObjects[3].vy_relative = self.calc_relative_velocity(Car4, EgoVehicle)
        # Relative Angle
        self.Objects.VehicleObjects[0].theta_relative = self.calc_relative_angle(Car1, EgoVehicle)
        self.Objects.VehicleObjects[1].theta_relative = self.calc_relative_angle(Car2, EgoVehicle)
        self.Objects.VehicleObjects[2].theta_relative = self.calc_relative_angle(Car3, EgoVehicle)
        self.Objects.VehicleObjects[3].theta_relative = self.calc_relative_angle(Car4, EgoVehicle)
        # print("x_car1:", self.Objects.VehicleObjects[0].x_relative, " y_car1:", self.Objects.VehicleObjects[0].y_relative,
        #     "vx_car1:", self.Objects.VehicleObjects[0].vx_relative, " vy_car1:", self.Objects.VehicleObjects[0].vy_relative)
        


