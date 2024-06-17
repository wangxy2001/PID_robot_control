import numpy as np
#import sys
#import os
#from UR3_InertiaFrame import UR3_InertiaFrame
from core import TransInv,VecToso3, MatrixExp3, RpToTrans

from numpy import pi
import math
from matlab_func import wrapToPi
def ReduceError(x, i):

    if x > -1/(10**i) and x < 1/(10**i):
        x = 0
    return x


def UR3_CTC_workcell_init(sim):
    robot_name = 'UR3'

    ##arm joints
    armJoints = []#-np.ones((6,));


    for i in range(6):  # get handles of joints
        # print("str",str(i),robot_name+'_joint'+str(i))
        #print(robot_name + '_joint' + str(i + 1))
        armJoints.append(int(sim.getObject('/' + robot_name + '/' + robot_name + '_joint' + str(i + 1)))) #

    # print(armJoints)[18. 21. 24. 27. 30. 33.] numpy.ndarray


    ##streaming joint positions and velocities
    for i in range(6):
        sim.getJointPosition(armJoints[i]) #joint position
        sim.getObjectFloatParam(armJoints[i], 2012); #joint velocity
        #vrchk(sim, res, True);
    #Make sure that all streaming data has reached the client at least once
    #sim.simxGetPingTime(clientID);
    return armJoints


def UR3_InertiaFrame(T, p, theta):
    #sys.path.append('../modern_robotics')

    #T is shape frame
    omghat_x = np.array([1, 0, 0]);
    omghat_y = np.array([0, 1, 0]);
    omghat_z = np.array([0, 0, 1]);
    #print("VecToso3(omghat_x * theta[0])",VecToso3(omghat_x * theta[0]))#√
    #print("MatrixExp3((VecToso3(omghat_x * theta[0]))", MatrixExp3((VecToso3(omghat_y * theta[1])))) # √

    R = MatrixExp3((VecToso3(omghat_x * theta[0]))).dot(MatrixExp3(VecToso3(omghat_y * theta[1]))).dot(MatrixExp3(VecToso3(omghat_z * theta[2])))
    #print("R",R)#xxxxxxx
    #MatrixExp3 3x3  R 3x3  p 3  RpToTrans 4x4   T4x4  M 4x4
    M = T.dot(RpToTrans(R, p))
    return M

def UR3_Slist():
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
    #print("Slist",Slist)
    return Slist

def UR3_Mlist():
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

    Mlist = np.stack([M01, M12, M23, M34, M45, M56, M67],axis=0)
    return Mlist

def UR3_Glist():

    #UR3_link2
    m_1 = 1.000e+00;
    Ix_1 = 1.410e-02 * m_1;
    Iy_1 = 1.336e-02 * m_1;
    Iz_1 = 9.901e-03 * m_1;
    #UR3_link3
    m_2 = 2.500e+00;
    Ix_2 = 8.740e-02 * m_2;
    Iy_2 = 8.478e-02 * m_2;
    Iz_2 = 9.851e-03 * m_2;
    #UR3_link4
    m_3 = 2.500e+00;
    Ix_3 = 6.091e-02 * m_3;
    Iy_3 = 5.970e-02 * m_3;
    Iz_3 = 5.315e-03 * m_3;
    #UR3_link5
    m_4 = 1.000e+00;
    Ix_4 = 7.624e-03 * m_4;
    Iy_4 = 7.461e-03 * m_4;
    Iz_4 = 4.633e-03 * m_4;
    #UR3_link6
    m_5 = 1.000e+00;
    Ix_5 = 7.785e-03 * m_5;
    Iy_5 = 4.669e-03 * m_5;
    Iz_5 = 7.605e-03 * m_5;
    #UR3_link7
    m_6 = 1.000e+00;
    Ix_6 = 2.621e-03 * m_6;
    Iy_6 = 2.621e-03 * m_6;
    Iz_6 = 3.870e-03 * m_6;

    G1 = np.diag(np.array([Ix_1, Iy_1, Iz_1, m_1, m_1, m_1]));
    G2 = np.diag(np.array([Ix_2, Iy_2, Iz_2, m_2, m_2, m_2]));
    G3 = np.diag(np.array([Ix_3, Iy_3, Iz_3, m_3, m_3, m_3]));
    G4 = np.diag(np.array([Ix_4, Iy_4, Iz_4, m_4, m_4, m_4]));
    G5 = np.diag(np.array([Ix_5, Iy_5, Iz_5, m_5, m_5, m_5]));
    G6 = np.diag(np.array([Ix_6, Iy_6, Iz_6, m_6, m_6, m_6]));

    #Glist = cat(3, G1, G2, G3, G4, G5, G6);
    Glist = np.stack([G1, G2, G3, G4, G5, G6], axis=0)
    return Glist

def UR_syms_IK(Tsb, W1, W2, L1, L2, H1, H2, q0):
    q1 = np.ones(2) * 99;
    judge_q5 = np.zeros(2);
    q5 = np.ones(4) * 99;
    sin_q5 = np.zeros(4)
    q6 = np.ones(4) * 99;
    q234 = np.zeros(4)
    M = np.zeros(4)
    N = np.zeros(4)
    judge_q2 = np.zeros(4)
    q2 = np.ones(8) * 99;
    q23 = np.zeros(8)
    q3 = np.ones(8) * 99;
    q4 = np.ones(8) * 99;
    Norm = np.ones(8) * 99;
    Q = np.ones((6, 8)) * 99;

    nx = Tsb[0, 0];
    ny = Tsb[1, 0];
    nz = Tsb[2, 0];

    ox = Tsb[0, 1];
    oy = Tsb[1, 1];
    oz = Tsb[2, 1];

    ax = Tsb[0, 2];
    ay = Tsb[1, 2];
    az = Tsb[2, 2];

    px = Tsb[0, 3];
    py = Tsb[1, 3];
    pz = Tsb[2, 3];

    A = W2 * ax - px;
    B = py - W2 * ay;
    judge_q1 = A ** 2 + B ** 2 - W1 ** 2;
    judge_q1 = ReduceError(judge_q1, 11);
    q1[0] = np.arctan2(W1, np.sqrt(judge_q1)) - np.arctan2(B, A);
    q1[1] = np.arctan2(W1, -np.sqrt(judge_q1)) - np.arctan2(B, A);

    judge_q5[0] = (ny * math.cos(q1[0]) - nx * math.sin(q1[0])) ** 2 + (
                ox * math.sin(q1[0]) - oy * math.cos(q1[0])) ** 2
    judge_q5[1] = (ny * math.cos(q1[1]) - nx * math.sin(q1[1])) ** 2 + (
                ox * math.sin(q1[1]) - oy * math.cos(q1[1])) ** 2
    #print("judge_q501",judge_q5[0],judge_q5[1],q1[0],q1[1])#√

    if judge_q5[0] < 0:
        pass
    else:
        q5[0] = math.atan2(math.sqrt(judge_q5[0]), ay * math.cos(q1[0]) - ax * math.sin(q1[0]))
        q5[1] = math.atan2(-math.sqrt(judge_q5[0]), ay * math.cos(q1[0]) - ax * math.sin(q1[0]))
        sin_q5[0] = ReduceError(math.sin(q5[0]), 10)
        sin_q5[1] = ReduceError(math.sin(q5[1]), 10)
        if sin_q5[0] == 0:
            q5[0] = ReduceError(q5[0],10)
            q6[0] = q0.flatten()[5]
            if q5[0] == 0:
                q234[0] = math.atan2(nz * math.cos(q6[0]) - oz * math.sin(q6[0]),
                                     nz * math.sin(q6[0]) + oz * math.cos(q6[0]))
            else:
                q234[0] = math.atan2(oz * math.sin(q6[0]) - nz * math.cos(q6[0]),
                                     nz * math.sin(q6[0]) + oz * math.cos(q6[0]))
        else:
            q6[0] = math.atan2((ox * math.sin(q1[0]) - oy * math.cos(q1[0])) / sin_q5[0],
                               (ny * math.cos(q1[0]) - nx * math.sin(q1[0])) / sin_q5[0])
            q234[0] = math.atan2(-az / sin_q5[0], (ax * math.cos(q1[0]) + ay * math.sin(q1[0])) / sin_q5[0])
        if sin_q5[1] == 0:
            q5[1] = ReduceError(q5[1],10)
            q6[1] = q0.flatten()[5]
            if q5[1] == 0:
                q234[1] = math.atan2(nz * math.cos(q6[1]) - oz * math.sin(q6[1]),
                                     nz * math.sin(q6[1]) + oz * math.cos(q6[1]))
            else:
                q234[1] = math.atan2(oz * math.sin(q6[1]) - nz * math.cos(q6[1]),
                                     nz * math.sin(q6[1]) + oz * math.cos(q6[1]))
        else:
            q6[1] = math.atan2((ox * math.sin(q1[0]) - oy * math.cos(q1[0])) / sin_q5[1],
                               (ny * math.cos(q1[0]) - nx * math.sin(q1[0])) / sin_q5[1])
            q234[1] = math.atan2(-az / sin_q5[1], (ax * math.cos(q1[0]) + ay * math.sin(q1[0])) / sin_q5[1])
        #print("q5q6",q5,q6)#√

        M[0] = px * math.cos(q1[0]) + py * math.sin(q1[0]) + H2 * math.sin(q234[0]) - W2 * math.cos(q234[0]) * sin_q5[0]
        M[1] = px * math.cos(q1[0]) + py * math.sin(q1[0]) + H2 * math.sin(q234[1]) - W2 * math.cos(q234[1]) * sin_q5[1]
        N[0] = H1 - pz - H2 * math.cos(q234[0]) - W2 * math.sin(q234[0]) * sin_q5[0]
        N[1] = H1 - pz - H2 * math.cos(q234[1]) - W2 * math.sin(q234[1]) * sin_q5[1]
        judge_q2[0] = 4 * L1 ** 2 * (M[0] ** 2 + N[0] ** 2) - (M[0] ** 2 + N[0] ** 2 + L1 ** 2 - L2 ** 2) ** 2
        judge_q2[1] = 4 * L1 ** 2 * (M[1] ** 2 + N[1] ** 2) - (M[1] ** 2 + N[1] ** 2 + L1 ** 2 - L2 ** 2) ** 2
        judge_q2[0]=ReduceError(judge_q2[0],3)
        judge_q2[1] = ReduceError(judge_q2[1], 3)
        #print("MN", M, N,judge_q2)
        if judge_q2[0] < 0:
            pass
        else:
            q2[0] = math.atan2(M[0] ** 2 + N[0] ** 2 + L1 ** 2 - L2 ** 2, math.sqrt(judge_q2[0])) - math.atan2(M[0],
                                                                                                               N[0])
            q2[1] = math.atan2(M[0] ** 2 + N[0] ** 2 + L1 ** 2 - L2 ** 2, -math.sqrt(judge_q2[0])) - math.atan2(M[0],
                                                                                                                N[0])
            q23[0] = math.atan2(N[0] - L1 * math.sin(q2[0]), M[0] - L1 * math.cos(q2[0]))
            q23[1] = math.atan2(N[0] - L1 * math.sin(q2[1]), M[0] - L1 * math.cos(q2[1]))
            q3[0] = q23[0] - q2[0]
            q3[1] = q23[1] - q2[1]
            q4[0] = q234[0] - q23[0]
            q4[1] = q234[0] - q23[1]
            Q[:, 0] = [q1[0], q2[0], q3[0], q4[0], q5[0], q6[0]]
            Q[:, 1] = [q1[0], q2[1], q3[1], q4[1], q5[0], q6[0]]
            Norm[0] = np.linalg.norm(wrapToPi((q0 - Q[:, 0]).flatten()))
            Norm[1] = np.linalg.norm(wrapToPi((q0 - Q[:, 1]).flatten()))
        #print("q0",(q0-Q[:, 0]).flatten(),wrapToPi((q0 - Q[:, 0]).flatten()))
        #print("QNorm",Q,Norm)#q√norm√
        if judge_q2[1] < 0:
            pass
        else:
            q2[2] = math.atan2(M[1] ** 2 + N[1] ** 2 + L1 ** 2 - L2 ** 2, math.sqrt(judge_q2[1])) - math.atan2(M[1],
                                                                                                               N[1])
            q2[3] = math.atan2(M[1] ** 2 + N[1] ** 2 + L1 ** 2 - L2 ** 2, -math.sqrt(judge_q2[1])) - math.atan2(M[1],
                                                                                                                N[1])
            q23[2] = math.atan2(N[1] - L1 * math.sin(q2[2]), M[1] - L1 * math.cos(q2[2]))
            q23[3] = math.atan2(N[1] - L1 * math.sin(q2[3]), M[1] - L1 * math.cos(q2[3]))
            q3[2] = q23[2] - q2[2]
            q3[3] = q23[3] - q2[3]
            q4[2] = q234[1] - q23[2]
            q4[3] = q234[1] - q23[3]
            Q[:, 2] = np.array([q1[0], q2[2], q3[2], q4[2], q5[1], q6[1]])
            Q[:, 3] = np.array([q1[0], q2[3], q3[3], q4[3], q5[1], q6[1]])
            Norm[2] = np.linalg.norm(wrapToPi((q0 - Q[:, 2]).flatten()))
            Norm[3] = np.linalg.norm(wrapToPi((q0 - Q[:, 3]).flatten()))
    #print("Q[:, 2],Q[:, 3] Norm[2],Norm[3]",Q[:, 2],Q[:, 3],Norm[2],Norm[3])
    if judge_q5[1] < 0:
        pass
    else:
        q5[2] = math.atan2(math.sqrt(judge_q5[1]), ay * math.cos(q1[1]) - ax * math.sin(q1[1]))
        q5[3] = math.atan2(-math.sqrt(judge_q5[1]), ay * math.cos(q1[1]) - ax * math.sin(q1[1]))
        sin_q5[2] = ReduceError(math.sin(q5[2]),10)
        sin_q5[3] = ReduceError(math.sin(q5[3]),10)
        if sin_q5[2] == 0:
            q5[2] = ReduceError(q5[2],10)
            q6[2] = q0.flatten()[5]
            if q5[2] == 0:
                q234[2] = math.atan2(nz * math.cos(q6[2]) - oz * math.sin(q6[2]),
                                     nz * math.sin(q6[2]) + oz * math.cos(q6[2]))
            else:
                q234[2] = math.atan2(oz * math.sin(q6[2]) - nz * math.cos(q6[2]),
                                     nz * math.sin(q6[2]) + oz * math.cos(q6[2]))
        else:
            q6[2] = math.atan2((ox * math.sin(q1[1]) - oy * math.cos(q1[1])) / sin_q5[2],
                               (ny * math.cos(q1[1]) - nx * math.sin(q1[1])) / sin_q5[2])
            q234[2] = math.atan2(-az / sin_q5[2], (ax * math.cos(q1[1]) + ay * math.sin(q1[1])) / sin_q5[2])
        if sin_q5[3] == 0:
            q5[3] = ReduceError(q5[3],10)
            q6[3] = q0.flatten()[5]
            if q5[3] == 0:
                q234[3] = math.atan2(nz * math.cos(q6[3]) - oz * math.sin(q6[3]),
                                     nz * math.sin(q6[3]) + oz * math.cos(q6[3]))
            else:
                q234[3] = math.atan2(oz * math.sin(q6[3]) - nz * math.cos(q6[3]),
                                     nz * math.sin(q6[3]) + oz * math.cos(q6[3]))
        else:
            q6[3] = math.atan2((ox * math.sin(q1[1]) - oy * math.cos(q1[1])) / sin_q5[3],
                               (ny * math.cos(q1[1]) - nx * math.sin(q1[1])) / sin_q5[3])
            q234[3] = math.atan2(-az / sin_q5[3], (ax * math.cos(q1[1]) + ay * math.sin(q1[1])) / sin_q5[3])
        M[2] = px * math.cos(q1[1]) + py * math.sin(q1[1]) + H2 * math.sin(q234[2]) - W2 * math.cos(q234[2]) * sin_q5[2]
        M[3] = px * math.cos(q1[1]) + py * math.sin(q1[1]) + H2 * math.sin(q234[3]) - W2 * math.cos(q234[3]) * sin_q5[3]
        N[2] = H1 - pz - H2 * math.cos(q234[2]) - W2 * math.sin(q234[2]) * sin_q5[2]
        N[3] = H1 - pz - H2 * math.cos(q234[3]) - W2 * math.sin(q234[3]) * sin_q5[3]
        judge_q2[2] = 4 * L1 ** 2 * (M[2] ** 2 + N[2] ** 2) - (M[2] ** 2 + N[2] ** 2 + L1 ** 2 - L2 ** 2) ** 2
        judge_q2[3] = 4 * L1 ** 2 * (M[3] ** 2 + N[3] ** 2) - (M[3] ** 2 + N[3] ** 2 + L1 ** 2 - L2 ** 2) ** 2
        judge_q2[2] = ReduceError(judge_q2[2], 3)
        judge_q2[3] = ReduceError(judge_q2[3], 3)
        if judge_q2[2] < 0:
            pass
        else:
            q2[4] = math.atan2(M[2] ** 2 + N[2] ** 2 + L1 ** 2 - L2 ** 2, math.sqrt(judge_q2[2])) - math.atan2(M[2],
                                                                                                               N[2])
            q2[5] = math.atan2(M[2] ** 2 + N[2] ** 2 + L1 ** 2 - L2 ** 2, -math.sqrt(judge_q2[2])) - math.atan2(M[2],
                                                                                                                N[2])
            q23[4] = math.atan2(N[2] - L1 * math.sin(q2[4]), M[2] - L1 * math.cos(q2[4]))
            q23[5] = math.atan2(N[2] - L1 * math.sin(q2[5]), M[2] - L1 * math.cos(q2[5]))
            q3[4] = q23[4] - q2[4]
            q3[5] = q23[5] - q2[5]
            q4[4] = q234[2] - q23[4]
            q4[5] = q234[2] - q23[5]
            Q[:, 4] = np.array([q1[1], q2[4], q3[4], q4[4], q5[2], q6[2]])
            Q[:, 5] = np.array([q1[1], q2[5], q3[5], q4[5], q5[2], q6[2]])
            Norm[4] = np.linalg.norm(wrapToPi((q0 - Q[:, 4]).flatten()))
            Norm[5] = np.linalg.norm(wrapToPi((q0 - Q[:, 5]).flatten()))
        if judge_q2[3] < 0:
            pass
        else:
            q2[6] = math.atan2(M[3] ** 2 + N[3] ** 2 + L1 ** 2 - L2 ** 2, math.sqrt(judge_q2[3])) - math.atan2(M[3],
                                                                                                               N[3])
            q2[7] = math.atan2(M[3] ** 2 + N[3] ** 2 + L1 ** 2 - L2 ** 2, -math.sqrt(judge_q2[3])) - math.atan2(M[3],
                                                                                                                N[3])
            q23[6] = math.atan2(N[3] - L1 * math.sin(q2[6]), M[3] - L1 * math.cos(q2[6]))
            q23[7] = math.atan2(N[3] - L1 * math.sin(q2[7]), M[3] - L1 * math.cos(q2[7]))
            q3[6] = q23[6] - q2[6]
            q3[7] = q23[7] - q2[7]
            q4[6] = q234[3] - q23[6]
            q4[7] = q234[3] - q23[7]
            Q[:, 6] = np.array([q1[1], q2[6], q3[6], q4[6], q5[3], q6[3]])
            Q[:, 7] = np.array([q1[1], q2[7], q3[7], q4[7], q5[3], q6[3]])
            Norm[6] = np.linalg.norm(wrapToPi((q0 - Q[:, 6]).flatten()))
            Norm[7] = np.linalg.norm(wrapToPi((q0 - Q[:, 7]).flatten()))
    #print("Q[:, 2],Q[:, 3] Norm[2],Norm[3]", Q[:, 2], Q[:, 3], Norm[2], Norm[3])
    pos = np.argmin(Norm)
    delta = np.min(Norm)
    #print("NormQ",Q,Norm)
    #print("q23 q234", q23, q234)
    #print("MN", M, N)
    if delta > 10:
        raise ValueError('Out of workspace!')
    else:
        q = Q[:, pos]
    return q

def UR_syms_p_pdot(q, qdot, W1, W2, L1, L2, H1, H2):
    q234 = q[1] + q[2] + q[3]
    qdot234 = qdot[1] + qdot[2] + qdot[3]
    q23 = q[1] + q[2]
    qdot23 = qdot[1] + qdot[2]

    px = -W1 * np.sin(q[0]) \
        - H2 * np.sin(q234) * np.cos(q[0]) \
        + np.cos(q[0]) * (L1 * np.cos(q[1]) + L2 * np.cos(q23)) \
        + W2 * (-np.cos(q[4]) * np.sin(q[0]) + np.cos(q234) * np.sin(q[4]) * np.cos(q[0]))

    py = W1 * np.cos(q[0]) \
        - H2 * np.sin(q234) * np.sin(q[0]) \
        + np.sin(q[0]) * (L1 * np.cos(q[1]) + L2 * np.cos(q23)) \
        + W2 * (np.cos(q[4]) * np.cos(q[0]) + np.cos(q234) * np.sin(q[4]) * np.sin(q[0]))

    pz = H1 - L1 * np.sin(q[1]) - L2 * np.sin(q23) \
        - H2 * np.cos(q234) - W2 * np.sin(q234) * np.sin(q[4])

    p = np.array([px, py, pz])

    pxdot = -W1 * np.cos(q[0]) * qdot[0] \
        - H2 * (np.cos(q234) * qdot234 * np.cos(q[0]) - np.sin(q234) * np.sin(q[0]) * qdot[0]) \
        - np.sin(q[0]) * qdot[0] * (L1 * np.cos(q[1]) + L2 * np.cos(q23)) \
        + np.cos(q[0]) * (-L1 * np.sin(q[1]) * qdot[1] - L2 * np.sin(q23) * qdot23) \
        + W2 * (np.sin(q[4]) * qdot[4] * np.sin(q[0]) - np.cos(q[4]) * np.cos(q[0]) * qdot[0] \
        - np.sin(q234) * qdot234 * np.sin(q[4]) * np.cos(q[0]) + np.cos(q234) * (np.cos(q[4]) * qdot[4] * np.cos(q[0]) - np.sin(q[4]) * np.sin(q[0]) * qdot[0]))

    pydot = -W1 * np.sin(q[0]) * qdot[0] \
        - H2 * (np.cos(q234) * qdot234 * np.sin(q[0]) + np.sin(q234) * np.cos(q[0]) * qdot[0]) \
        + np.cos(q[0]) * qdot[0] * (L1 * np.cos(q[1]) + L2 * np.cos(q23)) \
        + np.sin(q[0]) * (-L1 * np.sin(q[1]) * qdot[1] - L2 * np.sin(q23) * qdot23) \
        + W2 * (-np.sin(q[4]) * qdot[4] * np.cos(q[0]) - np.cos(q[4]) * np.sin(q[0]) * qdot[0] \
        - np.sin(q234) * qdot234 * np.sin(q[4]) * np.sin(q[0]) + np.cos(q234) * (np.cos(q[4]) * qdot[4] * np.sin(q[0]) + np.sin(q[4]) * np.cos(q[0]) * qdot[0]))

    pzdot = -L1 * np.cos(q[1]) * qdot[1] - L2 * np.cos(q23) * qdot23 \
        + H2 * np.sin(q234) * qdot234 - W2 * (np.cos(q234) * qdot234 * np.sin(q[4]) + np.sin(q234) * np.cos(q[4]) * qdot[4])

    pdot = np.array([pxdot, pydot, pzdot])

    return p, pdot

def UR_A_B(q, qdot, W1, W2, L1, L2, H2):
    A = np.zeros((3, 6))
    B = np.zeros((3, 1))

    q234 = q[1] + q[2] + q[3]
    qdot234 = qdot[1] + qdot[2] + qdot[3]

    q23 = q[1] + q[2]
    qdot23 = qdot[1] + qdot[2]

    A[0, 0] = -W1 * np.cos(q[0]) + H2 * np.sin(q234) * np.sin(q[0]) - np.sin(q[0]) * (L1 * np.cos(q[1]) + L2 * np.cos(q23)) - W2 * (np.cos(q[4]) * np.cos(q[0]) + np.cos(q234) * np.sin(q[4]) * np.sin(q[0]))
    A[0, 1] = -H2 * np.cos(q234) * np.cos(q[0]) - np.cos(q[0]) * (L1 * np.sin(q[1]) + L2 * np.sin(q23)) - W2 * np.sin(q234) * np.sin(q[4]) * np.cos(q[0])
    A[0, 2] = -H2 * np.cos(q234) * np.cos(q[0]) - np.cos(q[0]) * L2 * np.sin(q23) - W2 * np.sin(q234) * np.sin(q[4]) * np.cos(q[0])
    A[0, 3] = -H2 * np.cos(q234) * np.cos(q[0]) - W2 * np.sin(q234) * np.sin(q[4]) * np.cos(q[0])
    A[0, 4] = W2 * (np.sin(q[4]) * np.sin(q[0]) + np.cos(q234) * np.cos(q[4]) * np.cos(q[0]))
    B[0, 0] = W1 * np.sin(q[0]) * qdot[0] ** 2 \
              + H2 * (np.sin(q234) * np.cos(q[0]) * qdot[0] ** 2 + 2 * np.cos(q234) * np.sin(q[0]) * qdot[
        0] * qdot234 + np.sin(q234) * np.cos(q[0]) * qdot234 ** 2) \
              - np.cos(q[0]) * (L1 * np.cos(q[1]) + L2 * np.cos(q23)) * qdot[0] ** 2 \
              + 2 * np.sin(q[0]) * qdot[0] * (L1 * np.sin(q[2]) * qdot[2] + L2 * np.sin(q23) * qdot23) \
              - np.cos(q[0]) * (L1 * np.cos(q[1]) * qdot[1] ** 2 + L2 * np.cos(q23) * qdot23 ** 2) \
              + W2 * (np.cos(q[4]) * np.sin(q[0]) * qdot[0] ** 2 + 2 * np.sin(q[4]) * np.cos(q[0]) * qdot[0] * qdot[
        4] + np.cos(q[4]) * np.sin(q[0]) * qdot[4] ** 2 \
                      - np.cos(q234) * np.sin(q[4]) * np.cos(q[0]) * qdot234 ** 2 \
                      - 2 * np.sin(q234) * qdot234 * (
                                  np.cos(q[4]) * qdot[4] * np.cos(q[0]) - np.sin(q[4]) * np.sin(q[0]) * qdot[0]) \
                      - np.cos(q234) * (
                                  np.sin(q[4]) * np.cos(q[0]) * qdot[0] ** 2 + 2 * np.cos(q[4]) * np.sin(q[0]) * qdot[
                              0] * qdot[4] + np.sin(q[4]) * np.cos(q[0]) * qdot[4] ** 2))

    A[1, 0] = -W1 * np.sin(q[0]) - H2 * np.sin(q234) * np.cos(q[0]) + np.cos(q[0]) * (L1 * np.cos(q[1]) + L2 * np.cos(q23)) + W2 * (-np.cos(q[4]) * np.sin(q[0]) + np.cos(q234) * np.sin(q[4]) * np.cos(q[0]))
    A[1, 1] = -H2 * np.cos(q234) * np.sin(q[0]) - np.sin(q[0]) * (L1 * np.sin(q[1]) + L2 * np.sin(q23)) - W2 * np.sin(q234) * np.sin(q[4]) * np.sin(q[0])
    A[1, 2] = -H2 * np.cos(q234) * np.sin(q[0]) - np.sin(q[0]) * L2 * np.sin(q23) - W2 * np.sin(q234) * np.sin(q[4]) * np.sin(q[0])
    A[1, 3] = -H2 * np.cos(q234) * np.sin(q[0]) - W2 * np.sin(q234) * np.sin(q[4]) * np.sin(q[0])
    A[1, 4] = W2 * (-np.sin(q[4]) * np.cos(q[0]) + np.cos(q234) * np.cos(q[4]) * np.sin(q[0]))
    B[1, 0] = -W1 * np.cos(q[0]) * qdot[0] ** 2 \
              + H2 * (np.sin(q234) * np.sin(q[0]) * qdot[0] ** 2 - 2 * np.cos(q234) * np.cos(q[0]) * qdot[
        0] * qdot234 + np.sin(q234) * np.sin(q[0]) * qdot234 ** 2) \
              - np.sin(q[0]) * (L1 * np.cos(q[1]) + L2 * np.cos(q23)) * qdot[0] ** 2 \
              - 2 * np.cos(q[0]) * qdot[0] * (L1 * np.sin(q[1]) * qdot[1] + L2 * np.sin(q23) * qdot23) \
              - np.sin(q[0]) * (L1 * np.cos(q[1]) * qdot[1] ** 2 + L2 * np.cos(q23) * qdot23 ** 2) \
              - W2 * (np.cos(q[4]) * np.cos(q[0]) * qdot[0] ** 2 - 2 * np.sin(q[4]) * np.sin(q[0]) * qdot[0] * qdot[
        4] + np.cos(q[4]) * np.cos(q[0]) * qdot[4] ** 2 \
                      + np.cos(q234) * np.sin(q[4]) * np.sin(q[0]) * qdot234 ** 2 \
                      + 2 * np.sin(q234) * qdot234 * (
                                  np.cos(q[4]) * qdot[4] * np.sin(q[0]) + np.sin(q[4]) * np.cos(q[0]) * qdot[0]) \
                      + np.cos(q234) * (
                                  np.sin(q[4]) * np.sin(q[0]) * qdot[0] ** 2 - 2 * np.cos(q[4]) * np.cos(q[0]) * qdot[
                              0] * qdot[4] + np.sin(q[4]) * np.sin(q[0]) * qdot[4] ** 2))
    A[2, 1] = -L1 * np.cos(q[1]) - L2 * np.cos(q23) + H2 * np.sin(q234) - W2 * np.cos(q234) * np.sin(q[4])
    A[2, 2] = -L2 * np.cos(q23) + H2 * np.sin(q234) - W2 * np.cos(q234) * np.sin(q[4])
    A[2, 3] = H2 * np.sin(q234) - W2 * np.cos(q234) * np.sin(q[4])
    A[2, 4] = -W2 * np.sin(q234) * np.cos(q[4])
    B[2, 0] = L1 * np.sin(q[1]) * qdot[1] ** 2 + L2 * np.sin(q23) * qdot23 ** 2 + H2 * np.cos(q234) * qdot234 ** 2 + W2 * (np.sin(q234) * np.sin(q[4]) * qdot[4] ** 2 - 2 * np.cos(q234) * np.cos(q[4]) * qdot[4] * qdot234 + np.sin(q234) * np.sin(q[4]) * qdot234 ** 2)

    return A, B