import numpy as np

import sys
import os
def UR3_Slist():
    sys.path.append('../modern_robotics')
    from core import VecToso3
    #计算UR3各空间转轴
    omg1 = np.array([[0], [0], [1]])
    omg2 = np.array([[-1],[0], [0]])
    omg3 = np.array([[-1], [0], [0]])
    omg4 = np.array([[-1], [0], [0]])
    omg5 = np.array([[0], [0], [1]])
    omg6 = np.array([[-1], [0], [0]])

    q1 = np.array([[0.00012], [0.000086], [0.1475]])
    q2 = np.array([[-0.1115], [0.000054], [0.1519]])
    q3 = np.array([[-0.1115], [0.00013], [0.3955]])
    q4 = np.array([[-0.1115], [0.000085], [0.6088]])
    q5 = np.array([[-0.1122], [0.000085], [0.6930]])
    q6 = np.array([[-0.1115], [0.000085], [0.6941]])

    v1 = -VecToso3(omg1.reshape(-1)).dot(q1);
    S1 = np.vstack([omg1, v1])
    v2 = -VecToso3(omg2.reshape(-1)).dot(q2);
    S2 = np.vstack([omg2, v2])
    v3 = -VecToso3(omg3.reshape(-1)).dot(q3);
    S3 = np.vstack([omg3, v3])
    v4 = -VecToso3(omg4.reshape(-1)).dot(q4);
    S4 = np.vstack([omg4, v4])
    v5 = -VecToso3(omg5.reshape(-1)).dot(q5);
    S5 = np.vstack([omg5, v5])
    v6 = -VecToso3(omg6.reshape(-1)).dot(q6);
    S6 = np.vstack([omg6, v6])

    Slist = np.hstack([S1, S2, S3, S4, S5, S6])
    return Slist
