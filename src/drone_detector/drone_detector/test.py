import numpy as np

thresholds = {"brown": (np.array([0, 0, 0]), np.array([100, 100, 100])),
                   "beige": (np.array([0, 0, 0]), np.array([100, 100, 100])),
                   "golden": (np.array([0, 0, 0]), np.array([100, 100, 100]))}

for col in thresholds:
    thres = thresholds[col]
    print(type(col))