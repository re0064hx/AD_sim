import sys
sys.path.append('../')
import numpy as np
import pandas as pd

# Sensingクラスの定義
class MainSensing():
    def __init__(self):
        self.path = np.zeros([])
        self.cx = np.zeros([])
        self.cy = np.zeros([])
        self.theta_r = np.zeros([])
        self.rho_r = np.zeros([])

        self.old_nearest_point_index = None
        print("Sensing module sucsessfully initialized.")

    def path_generation(self):
        # csv_input = pd.read_csv('reference_path.csv')
        csv_input = pd.read_csv('SimulatorUtils/oval_course.csv')
        self.path = csv_input.values
        self.cx = self.path[:, 0]
        self.cy = self.path[:, 1]
        self.theta_r = np.arctan2(self.cy, self.cx)
