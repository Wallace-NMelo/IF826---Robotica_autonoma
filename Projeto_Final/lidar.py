import math

import matplotlib.pyplot as plt
import numpy as np
from sklearn import linear_model


def arrangeData(measuredPPosition):
    x = np.zeros(int(len(measuredPPosition) / 3))
    y = np.zeros(int(len(measuredPPosition) / 3))

    for i in range(int(len(measuredPPosition) / 3)):
        x[i] = measuredPPosition[i * 3]
        y[i] = measuredPPosition[i * 3 + 1]
    return x.reshape(-1, 1), y


def calculare_R(x, y, ang_coef, lin_coef):
    R = np.zeros((2, 2))

    F = np.zeros((2, 2))

    C = np.diag([0.001, 0.001])

    sum1 = 0
    sum2 = 0
    sum3 = 0
    sum4 = 0
    sum6 = 0
    sum7 = 0
    sum8 = 0
    sum9 = 0

    sum10 = 0
    sum11 = 0
    sum12 = 0
    sum13 = 0
    r, theta = toPolar(x, y)
    n = len(r)
    # print(r.shape)
    for i in range(n):
        sum1 = sum1 + r[i] * np.sin(2 * theta[i])
        sum2 = sum2 + r[i] * np.cos(2 * theta[i])

        sum6 = sum6 + (r[i] ** 2) * np.sin(2 * theta[i])
        sum7 = sum7 + (r[i] ** 2) * np.cos(2 * theta[i])

        sum10 = sum10 + (r[i] ** 2) * np.cos(2 * theta[i])
        sum11 = sum11 - 2 * (r[i] ** 2) * np.sin(2 * theta[i])
        for j in range(n):
            sum3 = sum3 + r[j] * np.cos(theta[i]) * np.sin(theta[j])
            sum4 = sum4 + r[j] * np.cos(theta[i] + theta[j])
            sum8 = sum8 + r[i] * r[j] * np.cos(theta[i]) * np.sin(theta[j])
            sum9 = sum9 + r[i] * r[j] * np.cos(theta[i] + theta[j])

            sum12 = sum12 + r[j] * r[i] * np.sin(theta[i]) * np.sin(theta[j])
            sum13 = sum13 + r[j] * r[i] * np.sin(theta[i] + theta[j])
    alpha = ang_coef

    dN = (2 * sum1 - 2 * sum3 / n)
    dD = (2 * sum2 - sum4 / n)
    N = sum6 - 2 * sum8 / n
    D = sum7 - sum9 / n

    F[0][0] = (D * dN - N * dD / (D ** 2)) / ((1 + np.tan(2 * alpha) ** 2) * (D ** 2))
    F[1][0] = np.sum(np.cos(theta - np.ones(n) * alpha))

    dN = (2 * sum10 + 2 * sum12 / n)
    dD = (2 * sum11 + sum13 / n)
    N = sum6 - 2 * sum8 / n
    D = sum7 - sum9 / n

    F[0][1] = (D * dN - N * dD / (D ** 2)) / ((1 + np.tan(2 * alpha) ** 2) * (D ** 2))
    F[1][1] = np.sum(-np.sin(theta - np.ones(n) * alpha))

    R = F * C * F.T

    return R


def linRegression(x, y):
    fitted_y = np.zeros(len(y))
    reg = linear_model.LinearRegression()
    ang_coef = [0]
    lin_coef = [0]
    if len(x) != 0 and len(y) != 0:
        reg.fit(x.reshape(-1, 1), y.reshape(-1, 1))
        ang_coef = np.arctan(reg.coef_)
        lin_coef = reg.intercept_

    return ang_coef, lin_coef


def find_farthest_point(x, y, ang_coef, lin_coef):
    fitted_y = ang_coef * x + lin_coef
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
        plt.plot(x, fitted_y)
        plt.scatter(x, y)
        plt.scatter(x[int(farthest_point_index)], y[int(farthest_point_index)], color='r')
        plt.show()


def line_to_point_dist(xp, yp, a, c):
    return np.abs(yp - float(xp) * float(a) - float(c)) / math.sqrt(float(a) ** 2 + 1 ** 2)


def split(x, y):
    threshold = 0.07
    ang_coef, lin_coef = linRegression(x, y)
    farthest_distance, farthest_point_index = find_farthest_point(x, y, ang_coef, lin_coef)
    xout = []
    yout = []

    if farthest_distance > threshold and farthest_point_index == 0 and len(x) > 0:
        splitted_x1 = np.split(x, [farthest_point_index + 1])
        splitted_y1 = np.split(y, [farthest_point_index + 1])
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

    if farthest_distance > threshold and farthest_point_index > 0 and len(x) > 0:
        # plot_SAM(x, y)
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

    if farthest_distance <= threshold or len(x) == 1:
        if len(x) == 1:
            xout.append(x[0])
            yout.append(y[0])
        else:
            xout.append(x)
            yout.append(y)
    return xout, yout


def point_dist(x1, y1, x2, y2):
    return np.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)


def is_point(x):
    size = len(x)
    assert size >= 1, "array size error"
    if size == 1:
        return 1

    return 0


def merge(x, y):
    threshold = 0.2
    output_list_x = []
    output_list_y = []
    output_lines = np.array([], dtype=np.int64).reshape(0, 2)
    new_element_x = np.array([])
    new_element_y = np.array([])
    R = []
    for i in range(len(x) - 1):
        if is_point(x[i]) == 1 and is_point(x[i + 1]) == 1:

            x1 = float(x[i])
            x2 = float(x[i + 1])
            y1 = float(y[i])
            y2 = float(y[i + 1])
            if point_dist(float(x[i]), float(y[i]), float(x[i + 1]), float(y[i + 1])) < threshold:
                new_element_x = np.append(new_element_x, np.array([x1, x2]))
                new_element_y = np.append(new_element_y, np.array([y1, y2]))
        elif is_point(x[i]) == 1 and is_point(x[i + 1]) != 1:
            output_list_x.append(new_element_x)
            output_list_y.append(new_element_y)
            new_element_x = []
            new_element_y = []
        elif is_point(x[i]) != 1:
            output_list_x.append(x[i])
            output_list_y.append(y[i])
    if len(x[len(x) - 1]) > 1:
        output_list_x.append(x[len(x) - 1])
        output_list_y.append(y[len(x) - 1])
    for i in range(len(output_list_x)):
        ang_coef, lin_coef = linRegression(output_list_x[i], output_list_y[i])

        # R.append(calculare_R(output_list_x[i],output_list_y[i],ang_coef,lin_coef))
        output_lines = np.hstack(
            (output_lines, np.array([ang_coef, lin_coef], dtype=np.float32))) if output_lines.size else np.array(
            [ang_coef, lin_coef], dtype=np.float32)
    return output_lines, len(output_list_x)


def split_and_merge(x, y):
    # plot_SAM(x, y)
    x, y = split(x, y)
    return merge(x, y)


def toPolar(x, y):
    r = np.zeros(len(x))
    alpha = np.zeros(len(x))
    for i in range(len(x)):
        r[i] = np.sqrt(x[i] ** 2 + y[i] ** 2)
        alpha[i] = np.arctan(y[i] / x[i])
    return r, alpha
