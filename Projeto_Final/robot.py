import numpy as np

import sim


class Robot:
    def __init__(self, clientID, v0):
        motorRight = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor', sim.simx_opmode_oneshot_wait)[1]
        motorLeft = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor', sim.simx_opmode_oneshot_wait)[1]
        self.sensorHandles = np.zeros(16)
        self.v0 = v0
        self.detect = np.zeros(16)

        for i in range(16):
            returnCode, sensorHandle = sim.simxGetObjectHandle(clientID,
                                                                        'Pioneer_p3dx_ultrasonicSensor' + str(i+1),
                                                                        sim.simx_opmode_oneshot_wait)
            self.sensorHandles[i] = sensorHandle

            returnCode, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = sim.simxReadProximitySensor(
                clientID, sensorHandle, sim.simx_opmode_streaming)

        # weighted matrix that converts the sensor inputs into motor speeds
        self.matrix = np.array([[-0.2, -0.4, -0.6, -0.8, -1, -1.2, -1.4, -1.6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                                [-1.6, -1.4, -1.2, -1, -0.8, -0.6, -0.4, -0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]])
        self.speedMotors = np.zeros(2)
        self.maxDetectionRadius = 0.5
        self.minDistance = 0.2

    def breit_controller(self, clientID):
        for i in range(16):
            returnCode, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = sim.simxReadProximitySensor(
                clientID, int(self.sensorHandles[i]), sim.simx_opmode_buffer)
            distToObject = np.sqrt(detectedPoint[0] ** 2 + detectedPoint[1] ** 2 + detectedPoint[2] ** 2)

            if (detectionState == True) and (distToObject < self.maxDetectionRadius):
                if distToObject < self.minDistance:
                    distToObject = self.minDistance
                normalizedDist = (distToObject - self.minDistance) / (self.maxDetectionRadius - self.minDistance)
                self.detect[i] = 1 - normalizedDist
            else:
                self.detect[i] = 0

        self.speedMotors[0] = self.v0
        self.speedMotors[1] = self.v0

        for i in range(2):
            for j in range(16):
                self.speedMotors[i] = self.speedMotors[i] + self.matrix[i, j] * self.detect[j]

        return self.speedMotors
