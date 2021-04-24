import numpy as np

L = 0.33
k_r = 2
k_l = 2
RADIUS = 0.0975


class Kalman:
    def __init__(self, dPhiL, dPhiR):
        self.dPhiL = dPhiL
        self.dPhiR = dPhiR
        self.deltaTheta = (dPhiL * RADIUS - dPhiR * RADIUS) / L
        self.deltaS = (dPhiL * RADIUS + dPhiR * RADIUS) / 2

    def prediction(self, prev_pos, prev_cov):
        updatedArray = np.array([self.deltaS * np.cos(prev_pos[2] + self.deltaTheta / 2),
                                 self.deltaS * np.sin(prev_pos[2] + self.deltaTheta / 2),
                                 self.deltaTheta
                                 ])
        updatePos = prev_pos + updatedArray
        Q = np.array([[k_r * abs(self.dPhiL * RADIUS), 0], [0, k_l * abs(self.dPhiR * RADIUS)]])
        Fp, Frl = self.jacobians(prev_pos)
        est_cov = Fp.dot(prev_cov).dot(Fp.T) + Frl.dot(Q).dot(Frl.T)

        return updatePos, est_cov

    def jacobians(self, prev_pos):
        Fp = np.array([[1, 0, -self.deltaS * np.sin(prev_pos[2] + self.deltaTheta / 2)],
                       [0, 1, self.deltaS * np.cos(prev_pos[2] + self.deltaTheta / 2)],
                       [0, 0, 1]])

        a00 = 0.5 * np.cos(prev_pos[2] + self.deltaTheta / 2) - (self.deltaS / 2 * L) * np.sin(
            prev_pos[2] + self.deltaTheta / 2)

        a01 = 0.5 * np.cos(prev_pos[2] + self.deltaTheta / 2) + (self.deltaS / 2 * L) * np.sin(
            prev_pos[2] + self.deltaTheta / 2)

        a10 = 0.5 * np.sin(prev_pos[2] + self.deltaTheta / 2) + (self.deltaS / 2 * L) * np.cos(
            prev_pos[2] + self.deltaTheta / 2)

        a11 = 0.5 * np.sin(prev_pos[2] + self.deltaTheta / 2) - (self.deltaS / 2 * L) * np.cos(
            prev_pos[2] + self.deltaTheta / 2)

        a20 = 1 / L

        a21 = -1 / L

        Frl = np.array([[a00, a01], [a10, a11], [a20, a21]])

        return Fp, Frl

    def correction(self):
        pass
