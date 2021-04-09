import sim
import sys
import time
import matplotlib.pyplot as plt

def main():
    clientID = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)  # Connect to CoppeliaSim
    if clientID != -1:
        print('Connected to remote API server')
        # Get handles for robot parts, actuators and sensors:
        res, motorRight = sim.simxGetObjectHandle(clientID, "Pioneer_p3dx_rightMotor", sim.simx_opmode_oneshot_wait)
        res, motorLeft = sim.simxGetObjectHandle(clientID, "Pioneer_p3dx_leftMotor", sim.simx_opmode_oneshot_wait)
    else:
        print('Failed connecting to remote API server')
        sys.exit('Could not connect')

    # while sim.simxGetConnectionId(clientID) != -1:

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
