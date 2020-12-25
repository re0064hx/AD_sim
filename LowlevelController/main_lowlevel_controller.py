import sys
sys.path.append('../')
import math
import numpy as np

from SimulatorUtils import simulator_settings as sets


class MainController():
    def __init__(self):
        self.e1 = 0.0
        self.e2 = 0.0
        self.e3 = 0.0
        print("Controller sucsessfully initialized.")

    def calc_error(self, car, ref):
        self.e1 = car.X - ref[0]
        self.e2 = car.Y - ref[1]
        self.e3 = car.theta - ref[2]
        # print("e2:", self.e2, " e3:", self.e3)

    def kanayama_method(self, car, path, idx):
        '''
        This function is calculate lateral control input
        based on Kanayama's method.
        '''
        K2 = 0.05
        K3 = 0.5
        rho = 0
        yaw_ref = np.arctan2(path.cy[idx]-path.cy[idx-1], path.cx[idx]-path.cx[idx-1])
        print("yaw_ref:", yaw_ref, " theta:", car.theta)
        ref = np.array([path.cx[idx], path.cy[idx], yaw_ref])
        self.calc_error(car, ref)

        term1 = (car.Kf*car.lf - car.Kr*car.lr)/(car.Kf*car.V) * car.YR
        term2 = (car.Kf + car.Kr)/(car.Kf) * car.beta
        term3 = (car.m * car.V)/(2*car.Kf) * (rho*(car.V*np.cos(self.e3))/(1-self.e2*rho) - K2*self.e2*car.V - K3*np.sin(self.e3))

        self.delta = term1 + term2 + term3
        return self.delta, idx

    def pure_pursuit(self, car, path, pidx):
        idx, idx_vhcl, Lf = path.search_target_index(car)
        yaw_ref = np.arctan2(path.cy[idx]-path.cy[idx-1], path.cx[idx]-path.cx[idx-1])
        ref = np.array([path.cx[idx], path.cy[idx], yaw_ref])
        self.calc_error(car, ref)

        if pidx >= idx:
            idx = pidx

        if idx < len(path.cx):
            tx = path.cx[idx]
            ty = path.cy[idx]
        else:  # toward goal
            tx = path.cx[-1]
            ty = path.cy[-1]
            idx = len(path.cx) - 1

        alpha = math.atan2(ty - car.Y, tx - car.X) - car.theta
        delta = math.atan2(2.0 * (car.length) * math.sin(alpha) / Lf, 1.0)

        return delta, idx

    def sliding_mode_control(self, car, path, idx):
        return delta, idx
