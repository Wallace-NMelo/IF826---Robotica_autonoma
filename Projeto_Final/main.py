import sim
import sys
import time
import matplotlib.pyplot as plt
import numpy as np


def to_deg(radians):
    return radians * (180 / np.pi)


def get_position(clientID, object_handle):
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
        # Get handles for robot parts:
        res, motorRight = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor', sim.simx_opmode_oneshot_wait)
        res, motorLeft = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor', sim.simx_opmode_oneshot_wait)
        res, robot = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx', sim.simx_opmode_oneshot_wait)
    else:
        print('Failed connecting to remote API server')
        sys.exit('Could not connect')

    while sim.simxGetConnectionId(clientID) != -1:
        pos = get_position(clientID, robot)  # [x,y,theta] in [cm cm rad]
        print(f'Robot position: [{pos[0]:.2f} {pos[1]:.2f} {to_deg(pos[2]):.2f}]')

        # Number of times we will read the sensor
        n_samples = 500
        _, _ = sim.simxGetStringSignal(clientID, 'scanRanges', sim.simx_opmode_streaming)
        error_code, scan_values = sim.simxGetStringSignal(clientID, 'scanRanges', sim.simx_opmode_buffer)
        distances = np.array([np.array(sim.simxUnpackFloats(scan_values)) for i in range(n_samples)])
        # add some noise
        for j in range(distances.shape[1]):
            distances[:, j] = distances[:, j] + 1 * np.random.randn()
        plt.plot(distances[0])
        plt.show()


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
