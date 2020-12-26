import sys
sys.path.append('../')
import math
import numpy as np

from SimulatorUtils import simulator_settings as sets

class PID():
    def __init__(self, Kp_recv, Ki_recv, Kd_recv):
        self.Kp = Kp_recv    # Proportial term gain
        self.Ki = Ki_recv    # Integral term gain
        self.Kd = Kd_recv    # Differencial term gain
        self.error_P = 0.0
        self.error_P_pre = 0.0
        self.error_I = 0.0
        self.error_D = 0.0
    
    def PID_control(self, ref_value, act_value, dt):
        self.error_P = ref_value - act_value
        self.error_I += self.error_P * dt
        self.error_D = (self.error_P - self.error_P_pre) / dt
        self.error_P_pre = self.error_P

        term_proportial = self.Kp * self.error_P
        term_integral = self.Ki * self.error_I
        term_differential = self.Kd * self.error_D

        return (term_proportial + term_integral + term_differential)

class MainController():
    def __init__(self):
        self.e1 = 0.0
        self.e2 = 0.0
        self.e3 = 0.0
        self.pid_velocity_controller = PID(0.5, 0.01, 0.2)
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

    def velocity_control(self, v_ref, v_act, dt):
        print("v_ref:", v_ref, " v_act:", v_act)
        accel_cmd = self.pid_velocity_controller.PID_control(v_ref, v_act, dt)
        return accel_cmd
