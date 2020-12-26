import sys
sys.path.append('../')
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.animation as animation
# from scipy.stats._continuous_distns import t_gen
import numpy as np

from SimulatorUtils import simulator_settings as sets

class MainAnimation():
    def __init__(self):
        ## plot 初期化
        # グラフ仕様設定
        # self.fig = plt.figure(figsize=(2,6))
        # self.fig, self.ax0 = plt.subplots(figsize=(2,6))
        self.fig, self.ax0 = plt.subplots(figsize=(6, 6))
        self.fig.canvas.draw()
        self.fig.show()
        # 軸
        # 最大値と最小値⇒軸の範囲設定
        self.max_x = 20
        self.min_x = -20
        self.max_y = 20
        self.min_y = -20
        print("Animation drawer sucsessfully initialized.")

    def plot_lane(self, path, idx, vhcl_idx):
        # 直線路の時
        # lane_base_x = np.arange(-100, 500).T
        # lane_base_y = np.zeros_like(lane_base_x)
        # # print("lane:", lane_base_x, lane_base_y)
        # self.ax0.plot(lane_base_y+1.75, lane_base_x, color="k", linestyle="dashed")
        # self.ax0.plot(lane_base_y-1.75, lane_base_x, color="k", linestyle="dashed")
        # self.ax0.plot(lane_base_y+5.25, lane_base_x, color="k", linestyle="dashed")
        # self.ax0.plot(lane_base_y-5.25, lane_base_x, color="k", linestyle="dashed")

        # 参照経路プロット
        self.ax0.plot(path[:, 1], path[:, 0], "-k")
        # 注視点プロット
        self.ax0.plot(path[idx, 1], path[idx, 0], "xr")
        # 参照経路における車両位置プロット
        self.ax0.plot(path[vhcl_idx, 1], path[vhcl_idx, 0], "ob")

    def plot_rectangle(self, Car0, Car1, Car2, Car3, Car4):
        # 自車中心プロット
        self.ax0.plot(Car0.Y, Car0.X, "or")

        # Limitation of x-axis and y-axis
        # 描画時，x-y軸は逆転させる
        self.ax0.set_ylim(self.min_x + Car0.X, self.max_x + Car0.X)
        self.ax0.set_xlim(self.min_y + Car0.Y, self.max_y + Car0.Y)
        # self.ax0.set_xlim(self.min_x, self.max_x)
        # self.ax0.set_ylim(self.min_y, self.max_y)

        # x軸（自車中心座標y軸方向）の正負を逆転
        self.ax0.invert_xaxis()

        # # 軸の縦横比, 正方形，単位あたりの長さを等しくする
        self.ax0.set_aspect('equal')
        # self.change_aspect_ratio(ax, 1/5) # 横を1/5倍長く（縦を5倍長く）設定

        # 軸の名前設定
        self.ax0.set_xlabel('Y [m]')
        self.ax0.set_ylabel('X [m]')

        # その他グラフ仕様
        self.ax0.grid(True)  # グリッド
        # 凡例
        # self.ax0.legend()

        # Generate rectangle
        # self.rect_0 = patches.Rectangle((Car0.Y-Car0.width/2, Car0.X-Car0.length/2),Car0.width,Car0.length,angle=-Car0.theta*180/np.pi, ec='r', fill=False)
        car0_y, car0_x = self.calc_rectangle_point(Car0)
        self.rect_0 = patches.Rectangle(
            (Car0.Y-car0_y, Car0.X+car0_x), Car0.width, Car0.length, angle=-Car0.theta*180/np.pi, ec='r', fill=False)
        self.rect_1 = patches.Rectangle((Car1.Y-Car1.width/2, Car1.X-Car1.length/2),
                                        Car1.width, Car1.length, angle=-Car1.theta*180/np.pi, ec='b', fill=False)
        self.rect_2 = patches.Rectangle((Car2.Y-Car2.width/2, Car2.X-Car2.length/2),
                                        Car2.width, Car2.length, angle=-Car2.theta*180/np.pi, ec='b', fill=False)
        self.rect_3 = patches.Rectangle((Car3.Y-Car3.width/2, Car3.X-Car3.length/2),
                                        Car3.width, Car3.length, angle=-Car3.theta*180/np.pi, ec='b', fill=False)
        self.rect_4 = patches.Rectangle((Car4.Y-Car4.width/2, Car4.X-Car4.length/2),
                                        Car4.width, Car4.length, angle=-Car4.theta*180/np.pi, ec='b', fill=False)

        self.plot0 = self.ax0.add_patch(self.rect_0)
        self.plot1 = self.ax0.add_patch(self.rect_1)
        self.plot2 = self.ax0.add_patch(self.rect_2)
        self.plot3 = self.ax0.add_patch(self.rect_3)
        self.plot4 = self.ax0.add_patch(self.rect_4)
        # plt.show()
        plt.pause(sets.Ts)

        # 角度変化も描画したい場合
        self.ax0.clear()

    def plot_loop(self, Car0, Car1, Car2, Car3, Car4):
        # 描画時，x-y軸は逆転させる
        self.ax0.set_ylim(self.min_x + Car0.X, self.max_x + Car0.X)
        self.ax0.set_xlim(self.min_y + Car0.Y, self.max_y + Car0.Y)
        # x軸（自車中心座標y軸方向）の正負を逆転
        self.ax0.invert_xaxis()

        self.plot0.set_xy([Car0.Y-Car0.width/2, Car0.X-Car0.length/2])
        self.plot1.set_xy([Car1.Y-Car1.width/2, Car1.X-Car1.length/2])
        self.plot2.set_xy([Car2.Y-Car2.width/2, Car2.X-Car2.length/2])
        self.plot3.set_xy([Car3.Y-Car3.width/2, Car3.X-Car3.length/2])
        self.plot4.set_xy([Car4.Y-Car4.width/2, Car4.X-Car4.length/2])
        plt.pause(sets.Ts)

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def change_aspect_ratio(self, ax, ratio):
        '''
        This function change aspect ratio of figure.
        Parameters:
            ax: ax (matplotlit.pyplot.subplots())
                Axes object
            ratio: float or int
                relative x axis width compared to y axis width.
        '''
        aspect = (1/ratio) * (ax0.get_xlim()
                              [1] - ax0.get_xlim()[0]) / (ax0.get_ylim()[1] - ax0.get_ylim()[0])
        ax0.set_aspect(aspect)

    def close_figure(self):
        # plt.cla()
        # plt.clf()
        plt.close(self.fig)

    def calc_rectangle_point(self, car):
        # print(car.theta)
        R = np.sqrt(np.power(car.length, 2) + np.power(car.width, 2))/2.0
        ang = -np.pi/2.0 + car.theta + \
            np.arctan2(car.width/2.0, car.length/2.0)
        x = R*np.cos(ang)
        y = R*np.sin(ang)
        return x, y
