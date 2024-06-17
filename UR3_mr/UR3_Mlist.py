import numpy as np
from numpy import pi
import sys
import os
def UR3_Mlist():
    sys.path.append('../modern_robotics')
    from core import TransInv


    #UR3_link2
    Shape_1 = np.array([[1, 0, 0, -0.0036],[0, 1, 0, -0.00007],[0, 0, 1, 0.1470],[0, 0, 0, 1]])
    XYZ_1 = np.array([-3.125e-03,+7.451e-05,-1.374e-03])
    ABG_1 = np.array([+1.59e-02,-5.67e+00,+9.00e+01]) * pi / 180
    #UR3_link3
    Shape_2 = np.array([[1, 0, 0, -0.1154],[0, 1, 0, 0.00021],[0, 0, 1, 0.2697],[0, 0, 0, 1]])
    XYZ_2 = np.array([-2.925e-03,-1.149e-04,-1.784e-02])
    ABG_2 = np.array([+2.38e-02,-3.02e+00,+8.98e+01]) *pi / 180
    #UR3_link4
    Shape_3 = np.array([[1, 0, 0, -0.0281],[0, 1, 0, -0.000032],[0, 0, 1, 0.4995],[0, 0, 0, 1]])
    XYZ_3 = np.array([-3.855e-03,+1.352e-04,+1.422e-02])
    ABG_3 = np.array([-2.80e-03,+3.10e+00,+9.06e+01]) *pi / 180;
    #UR3_link5
    Shape_4 = np.array([[1, 0, 0, -0.1110],[0, 1, 0, 0.00014],[0, 0, 1, 0.6100],[0, 0, 0, 1]])
    XYZ_4 = np.array([+1.975e-03,-1.176e-04,+1.445e-03])
    ABG_4 = np.array([+3.71e-01,-9.01e-01,+9.04e+01]) *pi / 180;
    #UR3_link6
    Shape_5 = np.array([[1, 0, 0, -0.1141],[0, 1, 0, 0.0005],[0, 0, 1, 0.6929],[0, 0, 0, 1]])
    XYZ_5 = np.array([-1.462e-03,+1.011e-04,-1.811e-03])
    ABG_5 = np.array([-7.35e-01,+1.45e+00,+8.98e+01]) *pi / 180;
    #UR3_link7
    Shape_6 = np.array([[0, 0, 1, -0.1779],[1, 0, 0, 0.00018],[0, 1, 0, 0.6943],[0, 0, 0, 1]])
    XYZ_6 = np.array([-1.169e-09,-1.182e-09,-5.091e-20])
    ABG_6 = np.array([+8.49e-07,+4.29e-07,+8.34e-09]) *pi / 180;
    sys.path.append('../UR3_mr')
    from UR3_InertiaFrame import UR3_InertiaFrame
    M1 = UR3_InertiaFrame(Shape_1, XYZ_1, ABG_1);#4x4

    M2 = UR3_InertiaFrame(Shape_2, XYZ_2, ABG_2);
    M3 = UR3_InertiaFrame(Shape_3, XYZ_3, ABG_3);
    M4 = UR3_InertiaFrame(Shape_4, XYZ_4, ABG_4);
    M5 = UR3_InertiaFrame(Shape_5, XYZ_5, ABG_5);
    M6 = UR3_InertiaFrame(Shape_6, XYZ_6, ABG_6);

    M01 = M1;
    M12 = TransInv(M1).dot(M2)#4x4
    M23 = TransInv(M2).dot(M3);
    M34 = TransInv(M3).dot(M4);
    M45 = TransInv(M4).dot(M5);
    M56 = TransInv(M5).dot(M6);
    M67 = np.array([[1, 0, 0, 0],[0, 1, 0, 0],[0, 0, 1, 0],[0, 0, 0, 1]])

    Mlist = np.stack([M01, M12, M23, M34, M45, M56, M67],axis=2)
    return Mlist

