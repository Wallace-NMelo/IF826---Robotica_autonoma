import numpy as np

from lidar import line_to_point_dist

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

    def prediction(self, prevPos, prevCov):
        updatedArray = np.array([self.deltaS * np.cos(np.float(prevPos[2]) + self.deltaTheta / 2),
                                 self.deltaS * np.sin(np.float(prevPos[2]) + self.deltaTheta / 2),
                                 self.deltaTheta
                                 ])
        arr_ = prevPos
        updatePos = np.add(arr_, np.asmatrix(updatedArray).T)
        Q = np.array([[k_r * abs(self.dPhiL * RADIUS), 0], [0, k_l * abs(self.dPhiR * RADIUS)]])
        Fp, Frl = self.jacobians(prevPos)
        estCov = Fp.dot(prevCov).dot(Fp.T) + Frl.dot(Q).dot(Frl.T)

        return updatePos, estCov

    def jacobians(self, prev_pos):
        Fp = np.array([[1, 0, -self.deltaS * np.sin(np.float(prev_pos[2]) + self.deltaTheta / 2)],
                       [0, 1, self.deltaS * np.cos(np.float(prev_pos[2]) + self.deltaTheta / 2)],
                       [0, 0, 1]])

        a00 = 0.5 * np.cos(float(prev_pos[2]) + self.deltaTheta / 2) - (self.deltaS / 2 * L) * np.sin(
            float(prev_pos[2]) + self.deltaTheta / 2)

        a01 = 0.5 * np.cos(float(prev_pos[2]) + self.deltaTheta / 2) + (self.deltaS / 2 * L) * np.sin(
            float(prev_pos[2]) + self.deltaTheta / 2)

        a10 = 0.5 * np.sin(float(prev_pos[2]) + self.deltaTheta / 2) + (self.deltaS / 2 * L) * np.cos(
            float(prev_pos[2]) + self.deltaTheta / 2)

        a11 = 0.5 * np.sin(float(prev_pos[2]) + self.deltaTheta / 2) - (self.deltaS / 2 * L) * np.cos(
            float(prev_pos[2]) + self.deltaTheta / 2)

        a20 = 1 / L

        a21 = -1 / L

        Fu = np.array([[a00, a01], [a10, a11], [a20, a21]])

        return Fp, Fu

    def update(self, predPosition, predError, y, S, H):
        # supondo que soh tem 1 elemento no mapa e a posicao estimada
        # R = np.diag([0.1, 0.1]) ** 2
        # zPrior, H = self.measurementPrediction(mapInputs, predPosition)
        # y = observedFeatures - zPrior
        # S = H @ predError @ H.T + R
        K = predError @ H.T @ np.linalg.pinv(S)

        estPosition = predPosition.reshape(-1, 1) + K @ y
        estError = (np.eye(len(estPosition)) - K @ H) @ predError
        return estPosition, estError, y, S

    def getV(self, predPosition, predError, mapInput, observedFeature):
        # supondo que soh tem 1 elemento no mapa e a posicao estimada
        predX, predY, predTheta = predPosition
        R = np.diag([0.1, 0.1]) ** 2
        zPrior, H = self.measurementPrediction(mapInput, predPosition)

        a, b = observedFeature
        zp = np.array([a - np.pi / 2, line_to_point_dist(predX, predY, a, b)])

        v = zp - zPrior

        S = H @ predError @ H.T + R
        return v, S, H

    def to_rad(self, a):
        return a * np.pi / 180

    def measurementPrediction(self, M, predictedPosition):
        # x, y, theta = M
        # r, alpha = toPolar(x, y)

        a, b = M

        a = self.to_rad(a)

        # calculando a posicao da reta em relacao ao mundo
        r = line_to_point_dist(0, 0, a, b)

        alpha = a - np.pi / 2
        predX, predY, predTheta = predictedPosition
        H = np.zeros(2, 3)
        zPrior = np.zeros(M.shape)

        zPrior = np.array([alpha - predTheta], [r - (predX * np.cos(alpha) + predY * np.sin(alpha))])
        H = np.array([0, 0, -1], [-np.cos(alpha), -np.sin(alpha), 0])

        return zPrior, H

    def matching(self, storedMap, observedFeatures, predictedPosition, errorPosition, g):
        nMeasurements = len(observedFeatures)
        nMapInputs = len(storedMap)
        d = np.zeros(nMeasurements, nMapInputs)
        H = np.zeros(nMeasurements * nMapInputs, 2, 3)
        v = np.zeros(nMeasurements * nMapInputs, 2)
        # R = np.cov(observedFeatures, rowvar=False)  # obsFeat -> (n, 2)
        for i in range(nMeasurements):
            for j in range(nMapInputs):
                zPrior, H[j + i * nMapInputs, :, :] = self.measurementPrediction(storedMap[j, :], predictedPosition)
                v[j + i * nMapInputs, :] = observedFeatures[i, :] - zPrior
                W = H[j + i * nMapInputs, :, :] * errorPosition * H[j + i * nMapInputs, :, :].T  # + R[:, :]
                d[i, j] = v[j + i * nMapInputs, :].T * np.linalg.inv(W) * v[j + i * nMapInputs, :]

        lowestDistance, lowestDistanceIndex = np.argmin(d, axis=1), np.amin(d, axis=1)
        validatedIndex = np.where(lowestDistance <= g ** 2)
        lowestDistanceIndex = lowestDistance[validatedIndex]
        # minima, min_index = np.argmin(d, axis=1), np.min(d, axis=1)
        # measurement_index = np.where(minima < g ** 2)
        # min_index = min_index[measurement_index]
        #
        v = v[lowestDistance + validatedIndex * nMapInputs, :]
        H = H[lowestDistance + validatedIndex * nMapInputs, :, :]
        # R = R[:, :, measurement_index]
        return v, H
