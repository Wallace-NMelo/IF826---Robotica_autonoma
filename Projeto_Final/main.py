import sim
import sys
import time
import matplotlib.pyplot as plt

sim.simxFinish(-1)  # just in case, close all opened connections
clientID = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)  # Connect to CoppeliaSim
if clientID != -1:
    print('Connected to remote API server')
else:
    print('Failed connecting to remote API server')
    sys.exit('Could not connect')

res, motorRight = sim.simxGetObjectHandle(clientID, "Pioneer_p3dx_rightMotor", sim.simx_opmode_oneshot_wait)
res, motorLeft = sim.simxGetObjectHandle(clientID, "Pioneer_p3dx_leftMotor", sim.simx_opmode_oneshot_wait)
sim.simxSetJointTargetVelocity(clientID, motorLeft, 0, sim.simx_opmode_streaming)
sim.simxSetJointTargetVelocity(clientID, motorRight, 0, sim.simx_opmode_streaming)

# The first call
_, rangeValues = sim.simxGetStringSignal(clientID, "scanRanges", sim.simx_opmode_streaming)
time.sleep(0.1)

# The following calls
_, rangeValues = sim.simxGetStringSignal(clientID, "scanRanges", sim.simx_opmode_buffer)

# Unpacks a string into an array of floats.
floatValues = sim.simxUnpackFloats(rangeValues)
print(floatValues)
x = range(len(floatValues))
y = floatValues
plt.scatter(x, y)
plt.show()
print('Program ended')



