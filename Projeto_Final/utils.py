import numpy as np

import sim


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
        noise = np.random.normal(0, 0.1, pos.shape)
        pos = pos + noise
    return pos
