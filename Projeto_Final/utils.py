import math

import numpy as np
import matplotlib.pyplot as plt
import sim


def readObservations(clientID):
    returnCode, sensorData = sim.simxGetStringSignal(clientID, "points", sim.simx_opmode_streaming)
    measureData = sim.simxUnpackFloats(sensorData)

    return measureData


def readOdometry(clientID, leftMotor, rightMotor, l_rot_prev, r_rot_prev):
    ret, l_rot_cur = sim.simxGetJointPosition(clientID, leftMotor, sim.simx_opmode_oneshot_wait)
    ret, r_rot_cur = sim.simxGetJointPosition(clientID, rightMotor, sim.simx_opmode_oneshot_wait)

    dPhiL = getSmallestAngle(l_rot_cur, l_rot_prev)
    dPhiR = getSmallestAngle(r_rot_cur, r_rot_prev)

    l_rot_prev = l_rot_cur
    r_rot_prev = r_rot_cur

    return dPhiL, dPhiR, l_rot_prev, r_rot_prev


def getSmallestAngle(a, b):
    diff = toPositiveAngle(a) - toPositiveAngle(b)
    if diff > np.pi:
        diff -= 2 * np.pi
    elif diff < -np.pi:
        diff += 2 * np.pi

    return diff


def toPositiveAngle(angle):
    angle = math.fmod(angle, 2 * np.pi)

    while angle < 0:
        angle += 2 * np.pi

    return angle


def getSimTimeMs(clientID):
    return sim.simxGetLastCmdTime(clientID)


def getPosition(clientID, object_handle, add_noise=False):
    ret, pos = sim.simxGetObjectPosition(clientID, object_handle, -1, sim.simx_opmode_oneshot_wait)
    if ret > 0:
        print("Error reading object position\n")
        return

    ret, angles = sim.simxGetObjectOrientation(clientID, object_handle, -1, sim.simx_opmode_oneshot_wait)
    if ret > 0:
        print("Error reading object orientation\n")
        return

    # I don't need the z-axis
    theta = angles[2]
    pos[2] = theta

    if add_noise:
        noise = np.random.normal(0, 0.1, len(pos))
        pos = pos + noise
    return pos


def toPolar(x, y):
    return np.sqrt(np.square(x) + np.square(y)), np.arctan(y / x)


def plot_animation(estPosition, truePosition, show_animation, hxEst, hxTrue=None):

    if show_animation:
        plt.cla()
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect('key_release_event',
                                     lambda event: [exit(0) if event.key == 'escape' else None])

        # plt.plot(hxTrue[0, :].flatten(),
        #          hxTrue[1, :].flatten(), "-b")
        plt.plot(hxEst[0, :].flatten(),
                 hxEst[1, :].flatten(), "-r")
        plt.axis("equal")
        plt.grid(True)
        plt.pause(0.001)
