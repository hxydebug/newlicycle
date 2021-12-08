import numpy as np
import time
from scipy.optimize import minimize_scalar
import math
from numpy.linalg import inv

## 需要经常修改的值
global velocity,stance_period
velocity = 1.2
stance_period = 0.05

global R, Lt, L, lt, mb, hG, Jt, e, dt, b0, b1, b2, a0, a1, PI

R = 0.21
PI = math.pi
mb = 25
hG = 0.365
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


def MPC_Obj(dx0, dT, X0):

    cost = 0
    varphi = X0
    dvarphi = dx0

    for i in range(dT):
        varphie = 0.0
        # 这个值是平衡位置，完美跟踪直线 varphie = 0deg, 但是这个值选择不好会造成很大的力矩，可以把这个值作为一个argument传递过来
        # 他的选择可以根据自行车碰撞之前的实际角速度或者实际EIC算出的期望角速度，然后在这个优化里保持不变
        # 对于圆形轨迹，varphie 有解析的公式

        eb = [varphie - varphi, - dvarphi]
        v_int = a0*eb[0] + a1*eb[1]

        f_varphi = mb * hG * g * varphi
        tau_s = Jt * v_int-f_varphi

        ddvarphi = v_int
        dvarphi = dvarphi + ddvarphi * dt
        varphi = varphi + dvarphi * dt

        cost = cost + (eb[0]**2 + eb[1]**2 + 10*tau_s**2) * dt

    return cost




def calculate_Tau(req_varphi,req_dvarphi):
    # get state from system
    varphi = req_varphi
    dvarphi = req_dvarphi

    #  control bounds
    b_min = -0.5
    b_max = 0.5
    bnds = (b_min, b_max)

    X0 = varphi

    dT = 45
    dx0 = 0.0
    res = minimize_scalar(MPC_Obj, dx0, args=(dT, X0), method='Bounded', bounds=bnds)
    dX0 = res.x
    delta_T = stance_period
    Tau = -Jt*(dX0-dvarphi)/delta_T + mb*hG*g*math.sin(varphi)
    # print('Impulsive Torque Command =', Tau)
    return Tau

if __name__=="__main__":
    # initial state
    varphi = 0.1
    dvarphi = 0.1
    
    X0 = varphi

    #  control bounds
    b_min = -0.25
    b_max = 0.25
    bnds = (b_min, b_max)
    dT = 50
    dx0 = 0.0
    t1 = time.time()
    res = minimize_scalar(MPC_Obj, dx0, args=(dT, X0), method='Bounded', bounds=bnds)
    t2 = time.time()
    dX0 = res.x
    dvarphi = dX0
    delta_T = 3*dt
    Tau = Jt*(dX0-dx0)/delta_T
    print('Impulsive Torque Command =', Tau)
    print(t2-t1)
    calculate_Tau(0.1,0,0.1,0,1,0,0,0)