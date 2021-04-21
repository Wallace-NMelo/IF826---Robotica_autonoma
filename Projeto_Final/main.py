import sim
from robot import Robot


def main():
    print('Program Started')
    sim.simxFinish(-1)
    clientID = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)
    if clientID != -1:
        print('Connected to remote API server')
    else:
        print('Failed connecting to remote API server')

    # # The first call
    # _, rangeValues = sim.simxGetStringSignal(clientID, "scanRanges", sim.simx_opmode_streaming)
    # print(f'position = {position}')
    # res, (motor_left, motor_right) = get_motor_var(clientID)
    # position = get_position(clientID)
    # sim.simxSetJointTargetVelocity(clientID, motor_left, 0, sim.simx_opmode_streaming)
    # sim.simxSetJointTargetVelocity(clientID, motor_right, 0, sim.simx_opmode_streaming)
    returnCode, leftMotor = sim.simxGetObjectHandle(clientID,
                                                    "Pioneer_p3dx_leftMotor", sim.simx_opmode_oneshot_wait)
    returnCode, rightMotor = sim.simxGetObjectHandle(clientID,
                                                     "Pioneer_p3dx_rightMotor", sim.simx_opmode_oneshot_wait)
    robot = Robot(clientID, 2)
    while sim.simxGetConnectionId(clientID) != -1:
        # _, rangeValues = sim.simxGetStringSignal(clientID, "scanRanges", sim.simx_opmode_buffer)
        speedMotors = robot.breit_controller(clientID)
        sim.simxSetJointTargetVelocity(clientID, leftMotor, speedMotors[0], sim.simx_opmode_streaming)
        sim.simxSetJointTargetVelocity(clientID, rightMotor, speedMotors[1], sim.simx_opmode_streaming)
        # floatValues = sim.simxUnpackFloats(rangeValues)
        # print(floatValues)
        # x = range(len(floatValues))
        # y = floatValues
        # plt.scatter(x, y)
        # plt.show()


if __name__ == "__main__":
    main()