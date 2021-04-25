import matplotlib.pyplot as plt
import numpy as np
from sklearn import datasets, linear_model
from sklearn.metrics import mean_squared_error, r2_score
import pandas as pd


def arrangeData(measuredPPosition):
    x = np.zeros(int(len(measuredPPosition)/3))
    y = np.zeros(int(len(measuredPPosition)/3))

    for i in range(int(len(measuredPPosition)/3)):
        x[i] = measuredPPosition[i*3]
        y[i] = measuredPPosition[i*3 + 1]
    return x,y

df = pd.read_csv("teste.csv")