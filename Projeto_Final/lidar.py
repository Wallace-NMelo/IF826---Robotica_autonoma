import math

import matplotlib.pyplot as plt
import numpy as np
from sklearn import linear_model


def arrangeData(measuredPPosition):
    x = np.zeros(int(len(measuredPPosition) / 2))
    y = np.zeros(int(len(measuredPPosition) / 2))
    for i in range(int(len(measuredPPosition) / 2)):
        x[i] = measuredPPosition[i * 2]
        y[i] = measuredPPosition[i * 2 + 1]
    return x.reshape(-1, 1), y


def linRegression(x, y):
    fitted_y = np.zeros(len(y))
    reg = linear_model.LinearRegression()
    ang_coef = 0
    lin_coef = 0
    if x.size != 0 and y.size != 0:
        reg.fit(x, y)
        ang_coef = reg.coef_
        lin_coef = reg.intercept_

    return ang_coef, lin_coef


def find_farthest_point(x, y, ang_coef, lin_coef):
    # fitted_y = ang_coef * x + lin_coef
    max_distance = 0
    max_y_index = -1
    for i in range(len(y)):
        distance = line_to_point_dist(x[i], y[i], ang_coef, lin_coef)
        if distance > max_distance:
            max_distance = distance
            max_y_index = i
    return max_distance, max_y_index


def plot_SAM(x, y):
    ang_coef, lin_coef = linRegression(x, y)
    farthest_distance, farthest_point_index = find_farthest_point(x, y, ang_coef, lin_coef)
    fitted_y = ang_coef * x + lin_coef
    if farthest_point_index != -1:
        plt.plot(x, fitted_y, color='green', linewidth=3)
        plt.scatter(x, y, color='black')
        plt.scatter(x[int(farthest_point_index)], y[int(farthest_point_index)], color='r')
        plt.xticks(())
        plt.yticks(())
        plt.show()


def line_to_point_dist(xp, yp, a, c):
    return np.abs(yp - xp * a - c) / math.sqrt(a ** 2 + 1 ** 2)


def split(x, y):
    threshold = 0.02
    ang_coef, lin_coef = linRegression(x, y)
    farthest_distance, farthest_point_index = find_farthest_point(x, y, ang_coef, lin_coef)
    xout = []
    yout = []
    if farthest_distance > threshold and farthest_point_index == 0 and len(x) > 0:
        splitted_x1 = np.split(x, [farthest_point_index + 1])
        splitted_y1 = np.split(y, [farthest_point_index + 1])
        # split_x2, split_y2 = split(splitted_x1[0], splitted_y1[0])

        split_x3, split_y3 = split(splitted_x1[1], splitted_y1[1])

        # for i in range(len(split_x2)):
        #     xout.append(split_x2[i])
        for i in range(len(split_x3)):
            xout.append(split_x3[i])

        # for i in range(len(split_y2)):
        #     yout.append(split_y2[i])
        for i in range(len(split_y3)):
            yout.append(split_y3[i])

    if farthest_distance > threshold and farthest_point_index > 0 and len(x) > 0:
        plot_SAM(x, y)

        splitted_x1 = np.split(x, [farthest_point_index])
        splitted_y1 = np.split(y, [farthest_point_index])
        split_x2, split_y2 = split(splitted_x1[0], splitted_y1[0])

        split_x3, split_y3 = split(splitted_x1[1], splitted_y1[1])

        for i in range(len(split_x2)):
            xout.append(split_x2[i])

        for i in range(len(split_x3)):
            xout.append(split_x3[i])

        for i in range(len(split_y2)):
            yout.append(split_y2[i])
        for i in range(len(split_y3)):
            yout.append(split_y3[i])

    # x = list(x)
    if farthest_distance <= threshold or len(x) == 1:
        if len(x) == 1:
            xout.append(x[0])
            yout.append(y[0])
        else:
            xout.append(x)
            yout.append(y)
    return xout, yout


def merge(xout, yout):
    print("line x: ", xout)
    # print("line y: ", yout)


def split_and_merge(x, y):
    plot_SAM(x, y)
    x, y = split(x, y)
    print(x)
    for i in range(len(x)):
        if len(x[i]) > 1:
            plt.scatter(x[i], y[i])
            plt.show()
