import sys
import csv
import numpy as np
import matplotlib.pyplot as plt

def plot_course(X,Y):
    f0 = plt.figure()
    ax1 = f0.add_subplot(111)
    ax1.plot(Y, X,'.-r')

    plt.show()

def csv_output(X,Y):
    k_num = len(X)
    fname_result = "oval_course.csv"
    with open(fname_result, 'w') as f:
        writer = csv.writer(f, lineterminator='\n')
        
        for i in range(k_num):
        # データをリストに保持
            csvlist = []
            csvlist.append(X[i])
            csvlist.append(Y[i])
            # 出力
            writer.writerow(csvlist)

def main():
    R = 100 # 曲線距離
    L = 400 # 直線距離
    len_wp = 1.0
    ds = len_wp/R
    theta1 = np.arange(-np.pi/2, np.pi/2, ds)
    theta2 = np.arange(np.pi/2, 3*np.pi/2, ds)
    # plt.plot(theta1, '.-r')
    # plt.plot(theta2, '.-r')
    # k_num = len(theta)

    X1 = R*np.cos(theta1) + L/2
    Y1 = R*np.sin(theta1) + R

    X2 = R*np.cos(theta2) - L/2
    Y2 = R*np.sin(theta2) + R

    X3 = np.arange(0, L/2, len_wp)
    Y3 = np.zeros([len(X3)])

    X4 = -np.arange(-L/2, L/2, len_wp)
    Y4 = 2*R*np.ones([len(X4)])

    X5 = np.arange(-L/2, 0, len_wp)
    Y5 = np.zeros([len(X3)])

    X = np.hstack([X3,X1,X2,X4,X5])
    Y = np.hstack([Y3,Y1,Y2,Y4,Y5])

    # plot_course(X,Y)
    csv_output(X,Y)

if __name__ == "__main__":
    main()
