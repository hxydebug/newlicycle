import numpy as np
import time
from scipy.optimize import minimize_scalar
import math
# import matplotlib.pyplot as plt
from scipy.optimize import minimize
# from scipy.spatial.transform import Rotation as Rot
from numpy.linalg import inv


global R, Lt, L, lt, mb, hG, Jt, e, dt, b0, b1, b2, a0, a1, sim_step, PI

R = 0.21
PI = math.pi
mb = 20
hG = 0.4
Jb = 0.8
Jt = Jb+mb*hG*hG
L = 0.87
lb = 0.42
g = 9.8
e = 17 * PI/180
lt = R * math.tan(e)
b0 = 1
b1 = 2
b2 = 3
a0 = 10
a1 = 2
dt = 0.02
sim_step = 3000


# global L0, L1, L2, width, W, Tau
# L0 = 0.0755
# L1 = 0.2115
# L2 = 0.20727
# width = 0.1
# W = np.diag([1, 1, 1, 2, 2, 2])


# def Arm_Kinematics(x):
#     q0 = x[0]
#     q1 = x[1]
#     q2 = x[2]
#     C0 = math.cos(q0)
#     C1 = math.cos(q1)
#     C2 = math.cos(q2)
#     S0 = math.sin(q0)
#     S1 = math.sin(q1)
#     S2 = math.sin(q2)

#     tau = np.array([[x[3]],
#                     [x[4]],
#                     [x[5]]])

#     Jacob =[[0,  L2 * (C1 * C2 - S1 * S2) + L1 * C1,    L2 * (C1 * C2 - S1 * S2)],
#             [L2 * (C0 * C1 * C2 - C0 * S1 * S2) - L0 * S0 + L1 * C0 * C1, - L2 * (C1 * S0 * S2 + C2 * S0 * S1) - L1 * S0 * S1, - L2 * (C1 * S0 * S2 + C2 * S0 * S1)],
#             [L0 * C0 - L2 * (S0 * S1 * S2 - C1 * C2 * S0) + L1 * C1 * S0, L2 * (C0 * C1 * S2 + C0 * C2 * S1) + L1 * C0 * S1, L2 * (C0 * C1 * S2 + C0 * C2 * S1)]
#             ]
#     Jacob = np.array(Jacob)

#     rb1 = L2 * (C1 * S2 + C2 * S1) + L1 * S1
#     rb2 = L0 * C0 - L2 * (S0 * S1 * S2 - C1 * C2 * S0) + L1 * C1 * S0 + width / 2
#     rb3 = L0 * S0 - L2 * (C0 * C1 * C2 - C0 * S1 * S2) - L1 * C0 * C1
#     r_b = np.array([rb1, rb2, rb3])

#     r = Rot.from_euler('zyx', [q0*180/PI, q1*180/PI, q2*180/PI], degrees=True)
#     R = r.as_matrix()
#     r_H = R @  np.transpose(r_b)
#     Jacob1 = Jacob@R
#     Jacob_I = np.transpose(Jacob1) @ inv(Jacob1@np.transpose(Jacob1)+0.025*np.identity(3))
#     force = Jacob_I @ tau
#     # force = pinv(np.transpose(Jacob)@R)@tau

#     torque = np.cross(np.transpose(r_H), np.transpose(force))

#     return torque[0], r_b


# def Leg_Obj(x, Tau, h):
#     q = x[0:2]
#     tau = np.array([x[3], x[4], x[5]])
#     M, r_b = Arm_Kinematics(x)
#     c_1 = 1e3*(M[0] - Tau) ** 2
#     c_2 = 1e6*(r_b[2] + h) ** 2

#     f = x@W@np.transpose(x) + c_1 + c_2 + 10*(r_b[0]-0.2)**2
#     return f


def Ref_Traj(t):
    ve = 1.5
    cr = 5
    w = ve/cr
    xe = cr * math.cos(w * t - PI / 2)
    ye = cr + cr * math.sin(w * t - PI / 2)
    dxe = -vr * math.sin(w * t - PI / 2)
    dye = vr * math.cos(w * t - PI / 2)
    ddxe = -vr * math.cos(w * t - PI / 2) * w
    ddye = -vr * math.sin(w * t - PI / 2) * w
    return xe, ye, dxe, dye, ddxe, ddye


def newtown(vr, dpsi, u_psi):
    i = 0
    res = 0
    second = (vr + g * lt * lb * math.cos(e) / (hG * vr)) * dpsi + lb * u_psi
    while 1:
        i = i + 1
        init = res
        fun = hG * dpsi * dpsi * math.sin(init) + g * math.tan(init) + second
        tmp = math.cos(init)
        dfun = hG * dpsi * dpsi * tmp + g / (tmp * tmp)
        res = init - fun / dfun
        if (np.abs(res - init) < 1e-5) or (i > 3000):
            break
    return res


def MPC_Obj(dx0, T0, dT, X0):
    Dt=0.2
    cost = 0
    x = X0[0]
    y = X0[1]
    dx = X0[2]
    dy = X0[3]
    ddx = X0[4]
    ddy = X0[5]
    varphi = X0[6]
    psi = X0[7]
    dpsi = X0[8]
    dvarphi = dx0

    vr = 1.5
    for i in range(dT):
        t = i * dt + T0
        xe, ye, dxe, dye = Ref_Traj(t)[0:4]
        ep = [xe-x, ye-y]
        dep = [dxe-dx, dye-dy]
        ##
        u_w_ext = [ddx+b1*dep[0]+b1*ep[0], ddy+b1*dep[1]+b1*ep[0]]
        u_psi = np.sign(u_w_ext[1])/np.sign(dx) * math.sqrt(u_w_ext[0]**2+u_w_ext[1]**2)/vr  # psi dot

        varphie = -vr*u_psi/g
        eb = [varphie - varphi, - dvarphi]
        v_psi_int = a0*eb[0] + a1*eb[1]

        f_varphi = mb * hG * g * math.sin(varphi)
        g_varphi = mb * hG * math.cos(e) * vr*vr / L
        phi = math.atan2((Jt * v_psi_int-f_varphi), g_varphi)

        dpsi = vr * math.cos(e) * math.tan(phi) / L
        psi = psi + dpsi * dt

        ddvarphi = v_psi_int
        dvarphi = dvarphi + ddvarphi * dt
        varphi = varphi + dvarphi * dt

        new_dx = vr * math.cos(psi)
        new_dy = vr * math.sin(psi)
        x = x + new_dx * dt
        y = y + new_dy * dt
        ddx = (new_dx - dx) / dt
        ddy = (new_dy - dy) / dt
        dx = new_dx
        dy = new_dy
        cost = cost + (ep[0]**2 + ep[1]**2 + eb[0]**2 + 0.1*eb[1]**2 + 10*phi*phi) * dt

    return cost


# initial state
varphi = 0.1
dvarphi = 0.1
ddvarphi = 0
dpsi = 0
ddpsi = 0
dvr = 0
dphi = 0
vr = 1.5
State = []
Time = []
T0 = 0
x, y, dx, dy, ddx, ddy = Ref_Traj(T0)
psi = math.atan(dy/dx)
x = x + 0.05
y = y - 0.05
X0 = [x, y, dx, dy, ddx, ddy, varphi, psi, dpsi]

#  control bounds
b_min = -0.5
b_max = 0.5
bnds = (b_min, b_max)
dT = 30
dx0 = dvarphi
t1 = time.time()
res = minimize_scalar(MPC_Obj, dx0, args=(T0, dT, X0), method='Bounded', bounds=bnds)
t2 = time.time()
dX0 = res.x
dvarphi = dX0
delta_T = 3*dt
Tau = Jt*(dX0-dx0)/delta_T


# ## leg optimization
# q_min = -150*PI/180
# q_max = 150*PI/180
# tau_min = -25
# tau_max = 25
# bnds = ((q_min, q_max), (q_min, q_max), (q_min, q_max), (tau_min, tau_max), (tau_min, tau_max), (tau_min, tau_max))

# q0 = [0, 0, 0, 1.3, 1.1, 1.2]
# h = 0.35 # height of the arm center from ground, when express the position in H frame (origin at C1-C2 center ),
# # it should be zero

# res = minimize(Leg_Obj, q0, args=(Tau, h), method='SLSQP', tol=1e-5, bounds=bnds)
# Q = res.x

# M, r_b = Arm_Kinematics(Q)
# print('Time duration = ', delta_T)
print('Impulsive Torque Command =', Tau)
# print('Joint angle and torque =', Q)
# print('Impulsive Torque=', M)
# print('Leg-end=', r_b)
# print(dvarphi)
print(t2-t1)




# # simulation
# for i in range(sim_step):
#     t = i*dt + T0
#     xe, ye, dxe, dye, ddxe, ddye = Ref_Traj(t)
#     ep = np.array([xe, ye])-np.array([x, y])
#     dep = np.array([dxe, dye])-np.array([dx, dy])
#     ddep = np.array([ddxe, ddye])-np.array([ddx, ddy])

#     rd3 = np.array([0, 0])
#     rd4 = rd3
#     rd5 = rd3
#     ##
#     u_w_ext = rd3 + b2 * ddep + b1 * dep + b0 * ep
#     ur = 0
#     u_psi = -2 * dvr * dpsi / vr - math.sin(psi) / vr * u_w_ext[0] + math.cos(psi) / vr * u_w_ext[1]
#     varphie = newtown(vr, dpsi, u_psi)

#     ##
#     du_w_ext = rd4 + b2 * (rd3 - u_w_ext) + b1 * ddep + b0 * dep
#     ddu_w_ext = rd5 + b2 * (rd4 - du_w_ext) + b1 * (rd3 - u_w_ext) + b0 * ddep

#     du_psi = -2 * ((ur * vr - dvr * dvr) / vr / vr * dpsi + dvr / vr * u_psi) - (math.cos(psi) * dpsi * vr - math.sin(psi) * dvr) / vr / vr * u_w_ext[0] \
#              - math.sin(psi) / vr * du_w_ext[0] + (-math.sin(psi) * dpsi * vr - math.cos(psi) * dvr) / vr / vr * \
#              u_w_ext[1] + math.cos(psi) / vr * du_w_ext[1]

#     ddu_psi = (math.sin(psi) * dpsi * dpsi - math.cos(psi) * u_psi) / vr * u_w_ext[0] - 2 * math.cos(psi) * dpsi / vr * du_w_ext[0]\
#               - math.sin(psi) / vr * ddu_w_ext[0] - (math.cos(psi) * dpsi * dpsi + math.sin(psi) * u_psi) / vr * u_w_ext[1] \
#               - 2 * math.sin(psi) * dpsi / vr * du_w_ext[1] + math.cos(psi) / vr * ddu_w_ext[1]

#     M1 = hG * math.cos(varphie) * dpsi * dpsi + g / math.cos(varphie) / math.cos(varphie)
#     M2 = (u_psi * vr + dpsi * dvr + 2 * hG * dpsi * u_psi * math.sin(varphie) + g * lb * lt * math.cos(e) / hG * (u_psi * vr - dpsi * dvr) / vr / vr + lb * du_psi)
#     dvarphie = -1/M1 * M2

#     dM1 = 2 * hG * dpsi * u_psi * math.cos(varphie) - hG * dpsi * dpsi * math.sin(varphie) * dvarphie + 2 * g / math.cos(varphie) / math.cos(varphie) * math.tan(varphie) * dvarphie
#     dM2 = du_psi * vr + 2 * u_psi * dvr + dpsi * ur + 2 * hG * (u_psi * u_psi * math.sin(varphie) + dpsi * du_psi * math.sin(varphie) + dpsi * u_psi * math.cos(varphie) * dvarphie)\
#           + g * lb * lt * math.cos(e) / hG * (du_psi * vr - 2 * u_psi * dvr - dpsi * ur + 2 * dpsi * dvr * dvr / vr) / vr / vr + lb * ddu_psi;

#     ddvarphie = np.power(M1, -2) * dM1 * M2 - 1/M1 * dM2

#     # ddvarphie = 0
#     # dvarphie = 0
#     v_psi_int = ddvarphie + a1 * (dvarphie - dvarphi) + a0 * (varphie - varphi)

#     f_varphi = mb * hG * math.cos(varphi) * vr * dpsi + mb * hG * hG * math.cos(varphi) * math.sin(varphi) * dpsi * dpsi+mb * hG * g * math.sin(varphi) + mb * g * lt * lb * dpsi * math.cos(varphi) / vr * math.cos(e)
#     g_psi_varphi = mb * hG * lb * math.cos(varphi)
#     u_psi_int = (-f_varphi + Jt * v_psi_int) / g_psi_varphi

#     ##
#     new_phi = (dpsi + u_psi_int * dt) * L * math.cos(varphi) / vr / math.cos(e)
#     phi = new_phi

#     ddpsi = u_psi_int
#     dpsi = dpsi + ddpsi * dt
#     psi = psi + dpsi * dt
#     ddvarphi = v_psi_int

#     dvarphi = dvarphi + ddvarphi * dt
#     varphi = varphi + dvarphi * dt

#     new_dx = vr * math.cos(psi)
#     new_dy = vr * math.sin(psi)
#     x = x + new_dx * dt
#     y = y + new_dy * dt
#     ddx = (new_dx - dx) / dt
#     ddy = (new_dy - dy) / dt
#     dx = new_dx
#     dy = new_dy

#     X = np.array([x, y, varphi, dx, dy, dvarphi, xe, ye, varphie, dxe, dye, dvarphie, psi, phi, dpsi])
#     State.append(X.tolist())
#     Time.append(t)


# State = np.transpose(State)
# State = np.reshape(State, (15, sim_step))

# ##  plot the control and state
# fig1, (ax1, ax2, ax3) = plt.subplots(3, 1, sharex=True)
# ax1.plot(Time, State[0])
# ax1.plot(Time, State[1])
# ax1.plot(Time, State[6])
# ax1.plot(Time, State[7])
# ax1.legend(('x1', 'x2', 'xr_1', 'xr_2'))
# ax1.set_ylabel('State')

# ax2.plot(Time, State[0]-State[6])
# ax2.plot(Time, State[1]-State[7])
# ax2.plot(Time, State[2]-State[8])
# ax2.legend(('ex', 'ey','e_b'))
# ax2.set_ylabel('Tracking Error')

# ax3.plot(Time, State[2])
# ax3.plot(Time, State[13])
# ax3.legend(('varphi', 'phi'))
# ax3.set_xlabel('Time, s')
# ax3.set_ylabel('Angle')
# ax3.set_xlim([Time[0], Time[-1]])

# fig = plt.figure()
# plt.plot(State[0], State[1])
# plt.plot(State[6], State[7])
# plt.legend(('Actual','Reference'))
# plt.show()