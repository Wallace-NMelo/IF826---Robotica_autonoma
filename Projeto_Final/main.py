from pathlib import Path

import numpy as np
import pandas as pd

import lidar
from kalman import Kalman
from robot import Robot
from utils import *

# Path of current directory
current_path = Path.cwd()
mapInputs = pd.read_csv(current_path.joinpath('map_lines.csv'))


def main():
    print('Program Started')
    sim.simxFinish(-1)
    clientID = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)
    if clientID != -1:
        print('Connected to remote API server')
    else:
        print('Failed connecting to remote API server')

    returnCode, leftMotor = sim.simxGetObjectHandle(clientID,
                                                    "Pioneer_p3dx_leftMotor", sim.simx_opmode_oneshot_wait)
    returnCode, rightMotor = sim.simxGetObjectHandle(clientID,
                                                     "Pioneer_p3dx_rightMotor", sim.simx_opmode_oneshot_wait)
    # robot = Robot(clientID, 2)
    returnCode, robotHandle = sim.simxGetObjectHandle(clientID, "Pioneer_p3dx", sim.simx_opmode_oneshot_wait)
    returnCode, sensorData = sim.simxGetStringSignal(clientID, "scanRanges", sim.simx_opmode_streaming)
    l_rot_prev, r_rot_prev = 0, 0
    # Initial state and initial covariance matrix
    prevPosition = np.matrix(getPosition(clientID, robotHandle)).T
    prevErrorPosition = np.zeros((3, 3))
    a = 0
    g = 0.5
    hxEst, hxTrue = np.zeros(shape=(3, 1)), prevPosition
    count = 0
    while sim.simxGetConnectionId(clientID) != -1:
        v = np.zeros((2, 1))
        # speedMotors = robot.breit_controller(clientID)
        # sim.simxSetJointTargetVelocity(clientID, leftMotor, speedMotors[0], sim.simx_opmode_streaming)
        # sim.simxSetJointTargetVelocity(clientID, rightMotor, speedMotors[1], sim.simx_opmode_streaming)
        if count == 100:
            # Dead reckoning
            dPhiL, dPhiR, l_rot_prev, r_rot_prev = readOdometry(clientID, leftMotor, rightMotor, l_rot_prev, r_rot_prev)
            # Initialize Kalman
            kalman_filter = Kalman(dPhiL, dPhiR)

            predPosition, predError = kalman_filter.prediction(prevPosition, prevErrorPosition)
            truePos = getPosition(clientID, robotHandle)

            # Observations
            observedFeatures = readObservations(clientID)
            x, y = lidar.arrangeData(observedFeatures)
            lidarInputs = lidar.split_and_merge(x, y)
            distances = []
            d_ant = 10000000000
            S = np.zeros((2, 2))
            H = np.zeros((2, 3))
            for i in range(len(lidarInputs)):
                for j in range(len(mapInputs)):
                    y, S, H = kalman_filter.getV(predPosition, predError, mapInputs[j, :],
                                                 lidarInputs[i, :])
                    d = y.T @ np.linalg.pinv(S) @ y
                    if d < g ** 2 and d < d_ant:
                        v = y
                        d_ant = d

            estPosition, estError, y, S = kalman_filter.update(predPosition, predError, v, S, H)

            prevPosition = estPosition
            prevErrorPosition = estError
            print(f'True Position: {truePos} \n Estimated position: {estPosition}')
            count = 0
            hxEst = np.hstack((hxEst, estPosition))
            # hxTrue = np.hstack((hxTrue, truePos))
            plot_animation(estPosition, truePos, show_animation=True, hxEst=hxEst)
        count += 1


            #     x, y = lidar.arrangeData(measuredPPosition)
        #     plt.plot(x, y, 'o')
        #     plt.show()
        #     lidar.split_and_merge(x, y)
        #     a = a + 1
        # if a == 2:
        #     break


if __name__ == "__main__":
    main()
