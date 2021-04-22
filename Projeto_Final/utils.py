import numpy as np

import sim


def readOdometry(clientID, leftMotor, rightMotor, l_rot_prev, r_rot_prev):
    ret, l_rot_cur = sim.simxGetJointPosition(clientID, leftMotor, sim.simx_opmode_oneshot_wait)
    ret, r_rot_cur = sim.simxGetJointPosition(clientID, rightMotor, sim.simx_opmode_oneshot_wait)

    dPhiL = getSmallestAngle(l_rot_cur, l_rot_prev)
    dPhiR = getSmallestAngle(r_rot_cur, r_rot_prev)

    l_rot_prev = l_rot_cur
    r_rot_prev = r_rot_cur

    return dPhiL, dPhiR, l_rot_prev, r_rot_prev


def getSmallestAngle(a, b):
    diff = a - b
    if diff > 180:
        diff -= 360
    elif diff < -180:
        diff += 360

    return abs(diff)


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
