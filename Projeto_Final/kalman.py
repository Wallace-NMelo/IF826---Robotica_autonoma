import numpy as np

L = 0.33
k_r = 2
k_l = 2


class Kalman:
    def __init__(self, next_pos, prev_pos):
        deltaX = (next_pos[0] - prev_pos[0])
        deltaY = (next_pos[1] - prev_pos[1])
        deltaT = (next_pos[2] - prev_pos[2])
        deltaS = deltaX / np.cos(next_pos[2] + deltaT / 2)
        deltaSl = (2 * deltaS - L * deltaT) / 2
        deltaSr = (2 * deltaS + L * deltaT) / 2

        self.prev_pos = np.array(prev_pos)
        self.next_pos = next_pos
        self.deltaS = deltaS
        self.deltaT = deltaT
        self.deltaSl = deltaSl
        self.deltaSr = deltaSr

    def prediction(self):
        updatedArray = np.array([self.deltaS * np.cos(self.prev_pos[2] + self.deltaT / 2),
                                 self.deltaS * np.sin(self.prev_pos[2] + self.deltaT / 2),
                                 self.deltaT
                                 ])
        updatePos = self.prev_pos + updatedArray
        matrix1 = np.array([[k_r * abs(self.deltaSr)], [k_l * abs(self.deltaSl)]])
        return updatePos, matrix1

    def correction(self):
        pass
