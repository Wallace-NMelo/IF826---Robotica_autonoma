from pathlib import Path

import pandas as pd
import lidar
from kalman import Kalman
from utils import *
from robot import Robot

# Path of current directory
current_path = Path.cwd()
mapInputs = pd.read_csv(current_path.joinpath('map_lines.csv'))
mapInputs = mapInputs.to_numpy()
show_animation = False


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

    robot = Robot(clientID, 2)
    returnCode, robotHandle = sim.simxGetObjectHandle(clientID, "Pioneer_p3dx", sim.simx_opmode_oneshot_wait)
    returnCode, sensorData = sim.simxGetStringSignal(clientID, "scanRanges", sim.simx_opmode_streaming)

    l_rot_prev, r_rot_prev = 0, 0

    # Initial state and initial covariance matrix
    prevPosition = np.matrix(getPosition(clientID, robotHandle)).T
    odPrevPos = np.matrix(getPosition(clientID, robotHandle)).T
    prevErrorPosition = np.zeros((3, 3))
    odPrevError = np.zeros((3, 3))
    a = 0
    g = 0.3

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
            odometry = Kalman(dPhiL, dPhiR)

            predPosition, predError = kalman_filter.prediction(prevPosition, prevErrorPosition)
            truePos = np.matrix(getPosition(clientID, robotHandle)).T
            odPosition, odError = odometry.prediction(odPrevPos, odPrevError)

            odPrevPos = odPosition
            odPrevError = odError
            # Observations
            observedFeatures = readObservations(clientID)
            x, y = lidar.arrangeData(observedFeatures)

            lidarInputs, nLidarInputs = lidar.split_and_merge(x, y)

            d_ant = 10000000000
            S = np.zeros((2, 2))
            H = np.zeros((2, 3))
            predPosition[2] = truePos[2]
            for i in range(nLidarInputs):
                for j in range(len(mapInputs)):
                    y, S, H = kalman_filter.getInnovation(predPosition, predError, mapInputs[j, :], lidarInputs[:, i])
                    d = y.T @ np.linalg.pinv(S) @ y
                    if d < g ** 2 and d < d_ant:
                        #print("=============== MATCH ================\n")
                        v = y
                        d_ant = d

            estPosition, estError, y, S = kalman_filter.update(predPosition, predError, v, S, H)

            prevPosition = estPosition
            prevErrorPosition = estError
            print(f'Error (True Position - Estimated Position = \n{truePos - estPosition}) \n')
            count = 0

        count += 1


if __name__ == "__main__":
    main()
