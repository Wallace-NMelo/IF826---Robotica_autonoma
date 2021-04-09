import sim


def get_object_handle(Client_ID, handle_object):
    # Get Object Handle
    return sim.simxGetObjectHandle(Client_ID, handle_object, sim.simx_opmode_oneshot_wait)


def get_motor_var(Client_ID):
    # Get handles for robot parts, actuators and sensors:
    res_R, motor_right = get_object_handle(Client_ID, 'Pioneer_p3dx_rightMotor')
    res_L, motor_left = get_object_handle(Client_ID, 'Pioneer_p3dx_leftMotor')
    return (res_L, res_R), (motor_left, motor_right)


def get_position(Client_ID):

    # res, position = sim.simxGetObjectPosition(Client_ID, get_object_handle(Client_ID, 'Pioneer_p3dx'),
    #                                           -1, sim.simx_opmode_oneshot_wait)
    # if res > 0:
    #     print('Error getting position')
    #     return False
    # return position
    return None

def get_rotation(Client_ID):
    pass
