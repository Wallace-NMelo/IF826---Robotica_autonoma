import numpy as np

L = 0.33
k_r = 2
k_l = 2
RADIUS = 0.0975


class Kalman:
    def __init__(self, dPhiL, dPhiR, prev_pos):
        # deltaX = (next_pos[0] - prev_pos[0])
        # deltaY = (next_pos[1] - prev_pos[1])
        # deltaT = (next_pos[2] - prev_pos[2])
        # deltaS = deltaX / np.cos(next_pos[2] + deltaT / 2)
        # deltaSl = (2 * deltaS - L * deltaT) / 2
        # deltaSr = (2 * deltaS + L * deltaT) / 2

        self.dPhiL = dPhiL
        self.dPhiR = dPhiR
        self.deltaTheta = (dPhiL*RADIUS - dPhiR*RADIUS) / L
        self.deltaS = (dPhiL*RADIUS + dPhiR*RADIUS) / 2

        self.prev_pos = np.array(prev_pos)

    def prediction(self):
        updatedArray = np.array([self.deltaS * np.cos(self.prev_pos[2] + self.deltaTheta / 2),
                                 self.deltaS * np.sin(self.prev_pos[2] + self.deltaTheta / 2),
                                 self.deltaTheta
                                 ])
        updatePos = self.prev_pos + updatedArray
        matrix1 = np.array([[k_r * abs(self.dPhiL*RADIUS), 0], [0, k_l * abs(self.dPhiR*RADIUS)]])
        return updatePos, matrix1

    def correction(self):#falta fazer essa parte
        pass
