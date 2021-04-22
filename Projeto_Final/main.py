import sim
from kalman import Kalman
from robot import Robot
from utils import getPosition, getSimTimeMs

L = 330


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
    prev_x = 0
    prev_y = 0
    prev_theta = 0
    pos = getPosition(clientID, robotHandle)
    while sim.simxGetConnectionId(clientID) != -1:
        # print('previous position: ', pos)
        time = getSimTimeMs(clientID)
        speedMotors = robot.breit_controller(clientID)
        sim.simxSetJointTargetVelocity(clientID, leftMotor, speedMotors[0], sim.simx_opmode_streaming)
        sim.simxSetJointTargetVelocity(clientID, rightMotor, speedMotors[1], sim.simx_opmode_streaming)
        if (time % 1000) == 0:
            kalmanF = Kalman(next_pos, pos)
            next_pos = getPosition(clientID, robotHandle)
            updatedPos, m1 = kalmanF.prediction()
            print(next_pos)
            print(updatedPos)


if __name__ == "__main__":
    main()
