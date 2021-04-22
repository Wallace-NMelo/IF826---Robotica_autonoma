import sim
from kalman import Kalman
from robot import Robot
from utils import *


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
    l_rot_prev, r_rot_prev = 0, 0
    prevPos = getPosition(clientID, robotHandle)
    while sim.simxGetConnectionId(clientID) != -1:
        time = getSimTimeMs(clientID)
        speedMotors = robot.breit_controller(clientID)
        sim.simxSetJointTargetVelocity(clientID, leftMotor, speedMotors[0], sim.simx_opmode_streaming)
        sim.simxSetJointTargetVelocity(clientID, rightMotor, speedMotors[1], sim.simx_opmode_streaming)
        if (time % 2000) == 0:
            dPhiL, dPhiR, l_rot_prev, r_rot_prev = readOdometry(clientID, leftMotor, rightMotor, l_rot_prev, r_rot_prev)
            kalman_filter = Kalman(dPhiL, dPhiR, prevPos)
            updatedPos, matrix = kalman_filter.prediction()
            truePos = getPosition(clientID, robotHandle)
            prevPos = updatedPos
            print(matrix)


if __name__ == "__main__":
    main()

