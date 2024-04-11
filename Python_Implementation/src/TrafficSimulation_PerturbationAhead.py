import numpy as np
import matplotlib.pyplot as plt
from math import cos, sin, pi, acos
# =========================================================================
#          Traffic Simulation when One Perturbation Happens Ahead
# Scenario:
#       There are m HDVs ahead of the CAV and n HDVs behind the CAV
#       There also exists a head vehicle at the very beginning
#       One slight disturbance happens at the head vehicle
#
# See Section V.A of the following paper for details
#   Title : Leading Cruise Control in Mixed Traffic Flow:
#                      System Modeling,Controllability,and String Stability
#   Author:Jiawei Wang, Yang Zheng, Chaoyi Chen, Qing Xu and Keqiang Li
# =========================================================================

# ------------------------------------------------------------------------
# Parameter setup
# ------------------------------------------------------------------------
m = 2                  # number of preceding vehicles
n = 2                  # number of following vehicles
PerturbedID = 0        # perturbation on vehicle
                       # 0. Head vehicle
                       # 1 - m. Preceding vehicles
                       # m+2 - n+m+1. Following vehicles
PerturbedType = 1      # perturbation type
                       # 1:Sine-wave Perturbation;  2: Braking
mix = 1                # Mix traffic or all HDVs

# ------------------------------------------------------------------------
# Connectivity pattern
# ------------------------------------------------------------------------
connectivityType = 1   # Different connectivity patterns
K = np.array([1, -1, 1, -1, 0, 0, -1, -1, -1, -1])  # Feedback gain

if connectivityType == 1:
    K[2:] = 0
elif connectivityType == 2:
    K[4:] = 0
elif connectivityType == 3:
    K[8:] = 0

# ------------------------------------------------------------------------
# Parameters in the car-following model
# ------------------------------------------------------------------------
alpha = 0.6
beta = 0.9
s_st = 5
s_go = 35

# Traffic equilibrium
v_star = 15         # Equilibrium velocity
acel_max = 2
dcel_max = -5
v_max = 30
s_star = np.arccos(1 - v_star / v_max * 2) / np.pi * (s_go - s_st) + s_st  # Equilibrium spacing

# linearized model
alpha1 = alpha * v_max / 2 * np.pi / (s_go - s_st) * np.sin(pi * (s_star - s_st) / (s_go - s_st))
alpha2 = alpha + beta
alpha3 = beta

# Simulation length
TotalTime = 100
Tstep = 0.01
NumStep = int(TotalTime / Tstep)

# ------------------------------------------------------------------------
# Some output information
# ------------------------------------------------------------------------
print("============================================================")
print("    Traffic Simulation when One Perturbation Happens Ahead ")
print("                 By Meihui Liu, Yue Yin ")
print()  
print("                  Converted from Matlab Version ")
print("          By Jiawei Wang, Yang Zheng ")
print("============================================================")

print(f"Number of HDV vehicles behind: {n}")
print(f"Number of HDV vehicles ahead : {m}")
print(f"Perturbation vehicle Id      : {PerturbedID}")
print("---------------------------")
print("HDV car-following model: optimal velocity model (OVM) ")
print("Parameter setup in HDV car-following model: ")
print(f"    alpha  beta  s_st  s_go  v_max \n    {alpha:4.2f}  {beta:4.2f}  {s_st:4.2f}  {s_go:4.2f}  {v_max:4.2f}")
print("Coefficients in linearized HDV car-following model: ")
print(f"    alpha1  alpha2  alpha3 \n    {alpha1:4.2f}    {alpha2:4.2f}    {alpha3:4.2f}")
print("---------------------------")
print("Feedback gain of the controller:")
print("mu_{-2}  k_{-2}  mu_{-1}  k_{-1}  mu_{1}  k_{1}  mu_{2}  k_{2}")
print(f"{K[0]:4.2f}    {K[1]:4.2f}    {K[2]:4.2f}    {K[3]:4.2f}    {K[5]:4.2f}    {K[6]:4.2f}    {K[7]:4.2f}    {K[8]:4.2f}")
print("---------------------------")
print(f"Simulation length (time step): {TotalTime}  ({Tstep:4.2f})")
print("-----------------------------------------------------------")
print("   Simulation begins ...")

# ------------------------------------------------------------------------
# Traffic simulation
# ------------------------------------------------------------------------
if mix == 1:
    # When will the controller work. 0:Controller Work; Large: won't work
    ActuationTime = 0
else:
    ActuationTime = 99999
# -----------------------------------------------
# Define state variables
# -----------------------------------------------
# Initial State for each vehicle
S = np.zeros((NumStep, m + n + 2, 3))
dev_s = 0
dev_v = 0
co_v = 1.0
v_ini = co_v * v_star
#from -dev to dev
S[0, :, 0] = (np.linspace(0, -(m + n + 1) * s_star, m + n + 2) + (np.random.rand(m + n + 2) * 2 * dev_s - dev_s)).flatten()
#The vehicles are uniformly distributed on the ring road with a random deviation
S[0, :, 1] = (v_ini * np.ones(m + n + 2) + (np.random.rand(m + n + 2) * 2 * dev_v - dev_v)).flatten()

# meaning of parameters
# 1:head vehicle
# 2~(m+1): preceding vehicles
# m+2:CAV
# (m+3)~(m+n+2): following vehicles
ID = np.zeros(m + n + 2)
if mix:
    ID[m + 1] = 1

X = np.zeros((2 * (m + n + 1), NumStep))
u = np.zeros((NumStep, 1))
V_diff = np.zeros((NumStep, m + n + 1))  #Velocity Difference
D_diff = np.zeros((NumStep, m + n + 1))  #Velocity Difference

#---------------------------------------------------------
# Simulation starts
#---------------------------------------------------------
for k in range(0, NumStep - 1):
    # Update acceleration
    V_diff[k, :] = S[k, 0:-1, 1] - S[k, 1:, 1]
    D_diff[k, :] = S[k, 0:-1, 0] - S[k, 1:, 0]
    cal_D = D_diff[k, :].copy() # For the boundary of Optimal Veloicity Calculation
    for i in range(m + n + 1):
        if cal_D[i] > s_go:
            cal_D[i] = s_go
        elif cal_D[i] < s_st:
            cal_D[i] = s_st

    # OVM Model
    acel = alpha * (v_max / 2 * (1 - np.cos(pi * (cal_D - s_st) / (s_go - s_st))) - S[k, 1:, 1]) + beta * V_diff[k, :]
    acel = np.where(acel > acel_max, acel_max, acel)
    acel = np.where(acel < dcel_max, dcel_max, acel)
    # SD as ADAS to prevent crash
    acel_sd = (S[k, 1:, 1] ** 2 - S[k, 0:-1, 1] ** 2) / (2 * D_diff[k, :])
    acel[acel_sd > abs(dcel_max)] = dcel_max

    S[k, 1:, 2] = acel
    # the preceding vehicle
    S[k, 0, 2] = 0 

    # Perturbation
    if PerturbedType == 1:
        P_A = 0.2
        P_T = 12
        if 20 < k * Tstep < 20 + P_T:
            S[k, PerturbedID, 2] = P_A * cos(2 * pi / P_T * (k * Tstep - 20))
    elif PerturbedType == 2:
        if 20 < k * Tstep < 21:
            S[k, PerturbedID, 2] = 5
    elif PerturbedType == 3:
        if k * Tstep == 20:
            S[k, PerturbedID, 1] = v_star - 0.4

    temp = np.reshape(D_diff[k, :], (m + n + 1,1)) - s_star
    print(temp.shape)
    temp = temp.flatten()
    print(temp.shape)
    X[0::2, k] = temp
    temp1 = np.reshape(S[k - 1, 1:, 1], (m + n + 1,1)) - v_star
    temp1 = temp1.flatten()
    X[1::2, k] = temp1 
    print("hi")
    print(X[1::2, k - 1])
    print(X[:, k - 1])
    if k > ActuationTime / Tstep:
        u[k] = K @ X[:, k]  
        if u[k] > acel_max:
            u[k] = acel_max
        elif u[k] < dcel_max:
            u[k] = dcel_max
        S[k, m + 1, 2] += u[k]

    S[k + 1, :, 1] = S[k, :, 1] + Tstep * S[k, :, 2]
    S[k + 1, :, 0] = S[k, :, 0] + Tstep * S[k, :, 1]

# ------------------------------------------------------------------------
# Plot the results
# ------------------------------------------------------------------------
print("-----------------------------------------------------------")
print("    Now plot the velocity profiles for demonstration, please wait ... ")

Wsize = 22
plt.figure()
i = 0
p1, = plt.plot(np.arange(Tstep, TotalTime + Tstep, Tstep), S[:, i, 1], '-', linewidth=2, color=[190/255, 190/255, 190/255])
for i in range(1, m + 1):
    p2, = plt.plot(np.arange(Tstep, TotalTime + Tstep, Tstep), S[:, i, 1], '-', linewidth=2, color=[90/255, 90/255, 90/255])
i = m + 1
p3, = plt.plot(np.arange(Tstep, TotalTime + Tstep, Tstep), S[:, i, 1], '-', linewidth=2, color=[244/255, 53/255, 124/255])
for i in range(m + 2, n + m + 2):
    p4, = plt.plot(np.arange(Tstep, TotalTime + Tstep, Tstep), S[:, i, 1], '-', linewidth=2, color=[67/255, 121/255, 227/255])
plt.gca().tick_params(labelsize=Wsize - 4)
plt.grid(True)
plt.xlabel('t [s]', fontsize=Wsize, color='k', usetex=True)
plt.ylabel('Velocity [m/s]', fontsize=Wsize, color='k', usetex=True)
plt.xlim([20, 45])

if PerturbedType == 1:
    plt.ylim([14.5, 15.5])
elif PerturbedType == 2:
    plt.ylim([0, 30])
elif PerturbedType == 3:
    plt.ylim([14.5, 15.5])
    plt.xlim([20, 30])

if m == 0:
    l = plt.legend([p1, p3, p4], ['Head vehicle', 'CAV', 'HDVs behind'], loc='upper right')
else:
    l = plt.legend([p1, p2, p3, p4], ['Head vehicle', 'HDVs ahead', 'CAV', 'HDVs behind'], loc='upper right')

plt.gcf().set_size_inches(480 / 96, 350 / 96) 
plt.gcf().set_dpi(96)
plt.setp(l.get_texts(), fontsize=Wsize-4, usetex=True)
plt.grid(True)
plt.show()


