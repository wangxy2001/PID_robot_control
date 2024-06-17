# Make sure to have the add-on "ZMQ remote API" running in
# CoppeliaSim. Do not launch simulation, but run this script
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time
import sys
import numpy as np
import os
sys.path.append('modern_robotics')
sys.path.append('remApi')
sys.path.append('UR3_mr')
from matlab_func import wrapToPi, quadprog
from modern_robotics.core import JointTrajectory, VecToso3, MatrixExp3, TransToRp,RpToTrans,CartesianTrajectory,MassMatrix,InverseDynamics
from UR3 import UR3_Slist, UR3_Mlist, UR3_Glist, UR3_CTC_workcell_init,UR_syms_IK,UR_syms_p_pdot,UR_A_B
import cvxopt
#q_arr=np.loadtxt('D:\post-graduate\\robot-arm-control\project\\test\\test3\\'+'q.txt')
#qdot_arr=np.loadtxt('D:\post-graduate\\robot-arm-control\project\\test\\test3\\'+'qdot.txt')

print('Program started')

client = RemoteAPIClient()
sim = client.require('sim')

CBF_on = 0;#关闭避障
sim.loadScene(os.getcwd()+'\\UR3_CT_with_obstacle.ttt')#加载无障碍物的scene
#CBF_on = 1;#开启避障
#sim.loadScene(os.getcwd()+'\\UR3_CT_no_obstacle.ttt')#加载有障碍物的scene

sim.setStepping(True)
#start the simulation
sim.startSimulation()

dt = 8e-3
#sim.simxSetFloatingParameter(clientID, sim.sim_floatparam_simulation_time_step, dt, sim.simx_opmode_blocking);#x
#sim.simxSynchronous(clientID, True)#x

#a=sim.simxStartSimulation(clientID, sim.simx_opmode_blocking)
#print("sim",a)
##initialization
W1 = 112
W2 = 66
L1 = 244
L2 = 213
H1 = 152
H2 = 85
omghat_x = np.array([1,0,0],dtype=np.double)
omghat_y = np.array([0,1,0],dtype=np.double)
omghat_z = np.array([0,0,1],dtype=np.double)

Slist = UR3_Slist()

Mlist = UR3_Mlist()
#print("Mlist",Mlist[:,:,1])
Glist = UR3_Glist()
#print("Glist",Glist[:,:,0])
joint_handles = np.zeros(6)

joint_handles=UR3_CTC_workcell_init(sim)#= np.zeros(6, dtype=int)
#for i in range(6):
    #joint_handles[i] = int(handles[i])
g = np.array([0,0,-9.81],dtype=np.double)
M = np.array([[1, 0, 0, 0],[0, 0, 1, 178],[0, -1, 0, 694],[0, 0, 0, 1]],dtype=np.double)
#sim.simxSynchronousTrigger(clientID);#x
#simGetReferencedHandles()

q = np.zeros(6,dtype=np.double)
qdot = np.zeros(6,dtype=np.double)

##obstacle
r = 30 + 31.5 + 15;
x0 = 0;
y0 = 178;
z0 = 494;
#print("action0",action0)

##precomputed trajectory
action0_qstart = np.array([0,-np.pi / 2,0,-np.pi / 2,0,0],dtype=np.double);

action0_Tsb = M;
action0_qend = UR_syms_IK(action0_Tsb, W1, W2, L1, L2, H1, H2,action0_qstart);
#print("action0_qend",action0_qend) #xxxxx[ 0.         -1.57079633  0.          4.71238898  0.          0.        ]

q0 = action0_qend;

action0_qend = wrapToPi(action0_qend - action0_qstart);
#print("action0_qend",action0_qend)

action0_Tf = 0.2;
action0_n = action0_Tf / dt + 1;#26
action0_method = 5;
action0_qDesired = JointTrajectory(np.zeros(6), action0_qend, action0_Tf, action0_n, action0_method);
qDesired = action0_qDesired;
#print("qDesired",qDesired)
N = int(action0_n)#26
#stay still
Ts = 0.6;
ns = int(Ts / dt + 1);#76
#print("111",ns,N)
qDesired = np.vstack((qDesired, np.ones((ns, 1)) * qDesired[N-1,:]))
#print("111")
#print("qDesired222", qDesired)
N = N + ns;

#action1
action1_Xstart = action0_Tsb;
action1_Rstart, action1_pstart = TransToRp(action1_Xstart);
#print("omghat_y",omghat_y)
action1_Rend = action1_Rstart.dot(MatrixExp3(VecToso3(omghat_y * (90 * np.pi / 180))))
action1_pend = action1_pstart + np.array([-200, 0, -200],dtype=np.double)
action1_Xend = RpToTrans(action1_Rend, action1_pend);
action1_Tf = 5;
action1_n = int(action1_Tf / dt + 1)
action1_method = 5;
action1_traj = CartesianTrajectory(action1_Xstart, action1_Xend, action1_Tf, action1_n, action1_method);
action1_qDesired = np.zeros((6, action1_n),dtype=np.double)
#print("action1_traj",action1_traj)
for i in range(action1_n):
    action1_qDesired[:, i] = UR_syms_IK(action1_traj[i], W1, W2, L1, L2, H1, H2, q0);
    q0 = action1_qDesired[:, i];
#print("action1_qDesired", action1_qDesired)
qDesired = np.vstack((qDesired, action1_qDesired.T))
N = N + action1_n;
# stay still
qDesired = np.vstack((qDesired, np.ones((ns, 1)) * qDesired[N - 1, :]))
N = N + ns;
# action2
action2_Xstart = action1_Xend;
action2_Rstart, action2_pstart = TransToRp(action2_Xstart);
# print("action2_Rpstart",action2_Xstart, action2_pstart, action2_Rstart)
action2_Rend = action2_Rstart.dot(MatrixExp3(VecToso3(omghat_z * (0 * np.pi / 180))))
# √
# print("action2_Rstart", action2_Rstart)
# print("action2_Rend",MatrixExp3(VecToso3(omghat_z * (0 * np.pi / 180))),action2_Rend)
action2_pend = action2_pstart + np.array([500, 0, 0],dtype=np.double)  # √
action2_Xend = RpToTrans(action2_Rend, action2_pend);
#
# print("action2_Xend",action2_Rend, action2_pend,action2_Xend)
action2_Tf = 10;
action2_n = int(action2_Tf / dt + 1)

action2_method = 5;
action2_traj = CartesianTrajectory(action2_Xstart, action2_Xend, action2_Tf, action2_n, action2_method);

action2_qDesired = np.zeros((6, action2_n),dtype=np.double)
for i in range(action2_n):
    action2_qDesired[:, i] = UR_syms_IK(action2_traj[i], W1, W2, L1, L2, H1, H2, q0);
    q0 = action2_qDesired[:, i];
qDesired = np.vstack((qDesired, action2_qDesired.T))
#print("action2_traj", action2_traj)
#print("action2_qDesired",action2_qDesired)
N = N + action2_n;
# stay still
qDesired = np.vstack((qDesired, np.ones((ns, 1)) * qDesired[N - 1, :]))
N = N + ns;
# action3
action3_Xstart = action2_Xend;
action3_Xend = action1_Xstart;
action3_Tf = 5;
action3_n = int(action3_Tf / dt + 1);
action3_method = 5;
# print("action3Xstart",action3_Xstart,action3_Xend,action3_n)
action3_traj = CartesianTrajectory(action3_Xstart, action3_Xend, action3_Tf, action3_n, action3_method);
action3_qDesired = np.zeros((6, action3_n),dtype=np.double)
# print("action3_traj",action3_traj)
for i in range(action3_n):
    action3_qDesired[:, i] = UR_syms_IK(action3_traj[i], W1, W2, L1, L2, H1, H2, q0);
    q0 = action3_qDesired[:, i];
#print("action3_traj", action3_traj)
#print("action3_qDesired", action3_qDesired)
qDesired = np.vstack((qDesired, action3_qDesired.T))
N = N + action3_n;
# stay still
qDesired = np.vstack((qDesired, np.ones((ns, 1)) * qDesired[N - 1, :]))
N = N + ns;

h1 = np.zeros(N)
h2 = np.zeros(N)
h3 = np.zeros(N)
phi1 = np.zeros(N)
phi2 = np.zeros(N)
phi3 = np.zeros(N)
exitflag = np.zeros(N)
qDesired = wrapToPi(qDesired - np.vstack(
    (np.zeros((int(action0_n + ns), 1)), np.ones((int(N - action0_n - ns), 1)))) * action0_qstart.T)
# qDesired = np.vstack((qDesired, np.ones((ns, 1)) * qDesired[N - 1, :]))


qdotDesired = np.zeros((N, 6),dtype=np.double)
qddotDesired = np.zeros((N, 6),dtype=np.double)
q_sim = np.zeros((N, 6),dtype=np.double)
qdot_sim = np.zeros((N, 6),dtype=np.double)
qddot_sim = np.zeros((N, 6),dtype=np.double)
tau_sim = np.zeros((N, 6),dtype=np.double)

for i in range(N - 1):
    qdotDesired[i + 1, :] = wrapToPi(qDesired[i + 1, :] - qDesired[i, :]) / dt
    qddotDesired[i + 1, :] = (qdotDesired[i + 1, :] - qdotDesired[i, :]) / dt

## CBF parameters

alpha1 = 3.5;
alpha2 = 5;
ub = np.array([[1], [1], [1], [1], [1], [1]]) * 80;
lb = -ub;

##PD controller parameters
weights = np.diag([1, 1, 1, 1, 1, 1]);
Kp = 20 * weights;
Kd = 10 * weights;

##computed torque control
i = 0;
tau_arr=np.zeros((10,6),dtype=np.double)
vel_arr=np.zeros((10,6),dtype=np.double)
#q_arr=np.zeros((10,6))
#qdot_arr=np.zeros((10,6))
startTime = sim.getSimulationTime()
while 1:#(t :=sim.getSimulationTime()) - startTime < 15:#True:# and i<10
    if i > N - 1:
        break

    # get states feedback
    for j in range(6):
        #print("joint_handles[j]", joint_handles[j],type(joint_handles[j]))
        # joint_handles[j, 0]=int(joint_handles[j, 0])
        # print(joint_handles[j, 0])

        q[j] = sim.getJointPosition(joint_handles[j]);  # sim.getJointPosition
        qdot[j] = sim.getObjectFloatParam(joint_handles[j], 2012);
        #q[j]=q_arr[i,j]
        #qdot[j]=qdot_arr[i, j]
    #print("qandqdot",q,qdot)
    #print("q and qtrue",q==q_arr[i,:],qdot==qdot_arr[i,:])
    #print(q,q_arr[i,:])
    #q = 1.0e-03 * np.array([0.096, 0.1077, -0.0008, 0.4994, 0.4971, -0.2250]);
    #qdot = 1.0e-03 * np.array([-0.0031, 0.0220, -0.0401, 0.1197, 0.0735, 0.2074])

    e = wrapToPi(qDesired[i, :] - q);
    edot = qdotDesired[i, :] - qdot;

    if CBF_on:
        qddotNorm = qddotDesired[i, :] + Kp.dot(e) + Kd.dot(edot)  # 列->行

        # quadprog
        q1 = q + action0_qstart;

        q_sim[i, :] = q1;
        qdot_sim[i, :] = qdot;

        H = np.diag([2, 2, 2, 2, 2, 2]);
        f = -2 * qddotNorm.reshape(-1, 1);

        [p1, pdot1] = UR_syms_p_pdot(q1, qdot, W1, W2, L1, L2, H1, H2);  # q1 qdot改为1维，函数要改
        [p2, pdot2] = UR_syms_p_pdot(q1, qdot, W1, W2 - 48, L1, L2, H1, H2);
        [p3, pdot3] = UR_syms_p_pdot(q1, qdot, W1, W2 - 96, L1, L2, H1, H2);
        # print("(p1(1) - x0)",p1)[518.73862815 187.481881    29.88822137]
        h1[i] = (p1[0] - x0) ** 2 + (p1[1] - y0) ** 2 + (p1[2] - z0) ** 2 - r ** 2;
        h2[i] = (p2[0] - x0) ** 2 + (p2[1] - y0) ** 2 + (p2[2] - z0) ** 2 - r ** 2;
        h3[i] = (p3[0] - x0) ** 2 + (p3[1] - y0) ** 2 + (p3[2] - z0) ** 2 - r ** 2;

        phi1[i] = 2 * (p1[0] - x0) * pdot1[0] + 2 * (p1[1] - y0) * pdot1[1] + 2 * (p1[2] - z0) * pdot1[2] + alpha1 * h1[
            i];
        phi2[i] = 2 * (p2[0] - x0) * pdot2[0] + 2 * (p2[1] - y0) * pdot2[1] + 2 * (p2[2] - z0) * pdot2[2] + alpha1 * h2[
            i];
        phi3[i] = 2 * (p3[0] - x0) * pdot3[0] + 2 * (p3[1] - y0) * pdot3[1] + 2 * (p3[2] - z0) * pdot3[2] + alpha1 * h3[
            i];

        A1, B1 = UR_A_B(q1, qdot, W1, W2, L1, L2, H2);  # q1改为1维，函数要改
        A2, B2 = UR_A_B(q1, qdot, W1, W2 - 48, L1, L2, H2);
        A3, B3 = UR_A_B(q1, qdot, W1, W2 - 96, L1, L2, H2);

        C1 = np.array([p1[0] - x0, p1[1] - y0, p1[2] - z0]).reshape(1, -1)
        C2 = np.array([p2[0] - x0, p2[1] - y0, p2[2] - z0]).reshape(1, -1)
        C3 = np.array([p3[0] - x0, p3[1] - y0, p3[2] - z0]).reshape(1, -1)

        D1 = pdot1[1] ** 2 + pdot1[2] ** 2 + pdot1[0] ** 2 + (alpha1 + alpha2) / 2 * phi1[i] - alpha1 ** 2 / 2 * h1[i];
        D2 = pdot2[1] ** 2 + pdot2[2] ** 2 + pdot2[0] ** 2 + (alpha1 + alpha2) / 2 * phi2[i] - alpha1 ** 2 / 2 * h2[i];
        D3 = pdot3[1] ** 2 + pdot3[2] ** 2 + pdot3[0] ** 2 + (alpha1 + alpha2) / 2 * phi3[i] - alpha1 ** 2 / 2 * h3[i];
        # print("C1 * A1",C1, A1)#(3,),3x6 C1 * A1 1x6
        # a = np.array([[-C1.dot(A1)],[-C2.dot(A2)],[-C3.dot(A3)]])#3x6
        # b = np.array([[C1.dot(B1) + D1],[C2.dot(B2) + D2],[C3.dot(B3) + D3]])
        a = np.vstack((-C1.dot(A1), -C2.dot(A2), -C3.dot(A3)))
        b = np.vstack((C1.dot(B1) + D1, C2.dot(B2) + D2, C3.dot(B3) + D3))
        # print("qwww1", q,qdot,e)#√
        #print("C1",C1,A1,C2,A2,C3,A3)#√
        #print("H, f, a, b, lb, ub",i,H, f, a, b, lb, ub)#a从i=1开始错 其他对 -> A1A2A3错
        qddotSafe = quadprog(H, f, a, b, None, None, lb, ub);  # , fval, exitflag[i]
        # print("q, Mlist, Glist, Slist", q, Mlist, Glist, Slist)
        # print("InverseDynamics21",InverseDynamics(q, qdot, np.zeros(6), g, np.zeros(6), Mlist, Glist, Slist),q, qdot, g)
        tau = MassMatrix(q, Mlist, Glist, Slist).dot(qddotSafe) + InverseDynamics(q, qdot, np.zeros(6), g, np.zeros(6),
                                                                                  Mlist, Glist, Slist).reshape(-1, 1);
        #print("qddotSafe MassMatrix tau", qddotSafe, tau)
        tau_sim[i, :] = tau.T

    else:

        q1 = q + action0_qstart;
        q_sim[i, :] = q1;  # q1改为1维
        qdot_sim[i, :] = qdot;

        tau = MassMatrix(q, Mlist, Glist, Slist).dot((Kp.dot(e) + Kd.dot(edot)).reshape(-1,1)) + InverseDynamics(q, qdot, qddotDesired[i, :],
                                                                                            g, np.zeros(6), Mlist, Glist, Slist).reshape(-1, 1);
        tau_sim[i, :] = tau.T;

    # Send torque to vrep
    for j in range(6):

        if tau[j] < 0:
            set_vel = -99999;
            set_tau = -tau[j,0];
        else:
            set_vel = 99999;
            set_tau = tau[j,0];
        #print("set_vel",set_vel,type(set_vel))#99999 <class 'int'>
        sim.setJointTargetVelocity(joint_handles[j], set_vel);
        #print("set_tau",set_tau,float(set_tau))#set_tau [0.0003838]
        sim.setJointMaxForce(joint_handles[j], float(set_tau));
        #tau_arr[i,j]=set_tau
        #vel_arr[i,j]=set_vel
        #tau[j]='a'

    #sim.simxSynchronousTrigger(clientID);
    #sim.simxGetPingTime(clientID);
    i = i + 1;

    #s = f'Simulation time: {t:.2f} [s] (simulation running synchronously ' 'to client, i.e. stepping)'
    #print(s)




    #sim.addLog(sim.verbosity_scriptinfos, s)
    sim.step()  # triggers next simulation step

np.savetxt('D:\post-graduate\\robot-arm-control\project\\test\\test3\\'+'vel.txt', np.c_[vel_arr],fmt='%f',delimiter='\t')
np.savetxt('D:\post-graduate\\robot-arm-control\project\\test\\test3\\'+'tau.txt', np.c_[tau_arr],fmt='%f',delimiter='\t')

sim.stopSimulation()

N=2833;
for i in range(N - 1):
    qddot_sim[i + 1, :] = (qdot_sim[i + 1, :] - qdot_sim[i, :]) / dt;

print('Program ended')
