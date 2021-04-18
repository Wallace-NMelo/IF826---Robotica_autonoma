import sys

import matplotlib.pyplot as plt
import numpy as np

import sim


def getPosition(clientID, object_handle):
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
    return pos


def main():
    clientID = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)  # Connect to CoppeliaSim
    if clientID != -1:
        print('Connected to remote API server')
        res, (motor_left, motor_right) = get_motor_var(clientID)
        position = get_position(clientID)
        sim.simxSetJointTargetVelocity(clientID, motor_left, 0, sim.simx_opmode_streaming)
        sim.simxSetJointTargetVelocity(clientID, motor_right, 0, sim.simx_opmode_streaming)
        # The first call
        _, rangeValues = sim.simxGetStringSignal(clientID, "scanRanges", sim.simx_opmode_streaming)
        print(f'position = {position}')

    else:
        print('Failed connecting to remote API server')
        sys.exit('Could not connect')
    fig = plt.figure()
    canvas = np.zeros((480, 640))
    while sim.simxGetConnectionId(clientID) != -1:
        _, rangeValues = sim.simxGetStringSignal(clientID, "scanRanges", sim.simx_opmode_buffer)
        sim.simxSetJointTargetVelocity(clientID, motor_left, -10, sim.simx_opmode_streaming)
        sim.simxSetJointTargetVelocity(clientID, motor_right, 10, sim.simx_opmode_streaming)
        floatValues = sim.simxUnpackFloats(rangeValues)
        print(floatValues)
        x = range(len(floatValues))
        y = floatValues
        #plt.scatter(x, y)
        #plt.show()


if __name__ == "__main__":
    main()

# # Unpacks a string into an array of floats.
# floatValues = sim.simxUnpackFloats(rangeValues)
# print(floatValues)
# x = range(len(floatValues))
# y = floatValues
# plt.scatter(x, y)
# plt.show()
# print('Program ended')