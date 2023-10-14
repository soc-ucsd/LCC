import math
import numpy as np
import time
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.colors import LinearSegmentedColormap
from matplotlib.colors import CSS4_COLORS
from matplotlib import animation

# =========================================================================
#               Video DEMO: Free Driving LCC
# Scenario:
#       One CAV is leading the motion of n HDVs behind
#       One sudden disturbance happens at one HDV behind the CAV
#
# See Section V of the following paper for details
#   Title : Leading Cruise Control in Mixed Traffic Flow:
#                      System Modeling,Controllability,and String Stability
#   Author:Jiawei Wang, Yang Zheng, Chaoyi Chen, Qing Xu and Keqiang Li
# =========================================================================


# -------------------------------------------------------------------------
#   Parameter setup
# -------------------------------------------------------------------------
# mode of the LCC system
FD_bool = 0        # 0. CF-LCC; 1. FD-LCC

m       = 0        # number of preceding vehicles
n       = 10       # number of following vehicles
PerturbedID = 2    # perturbation on vehicle
                   # 0. Head vehicle
                   # 1 - m. Preceding vehicles
                   # m+2 - n+m+1. Following vehicles
PerturbedType = 2  # perturbation type
                   # 1:Sine-wave Perturbation;  2: Braking

# ------------------------------------------
# Parameters in the car-following model
# ------------------------------------------
alpha = 0.6  # Driver Model: OVM
beta  = 0.9 
s_st  = 5 
s_go  = 35 

# Traffic equilibrium
v_star   = 15    # Equilibrium velocity
acel_max = 2 
dcel_max = -5 
v_max    = 30 
s_star   = math.acos(1-v_star/v_max*2)/math.pi*(s_go-s_st)+s_st  #Equilibrium spacing

# linearized model
alpha1 = alpha*v_max/2*math.pi/(s_go-s_st)*math.sin(math.pi*(s_star-s_st)/(s_go-s_st)) 
alpha2 = alpha+beta 
alpha3 = beta 

# Simulation length
TotalTime = 100 
Tstep     = 0.01 
NumStep   = int(TotalTime/Tstep)

# ------------------------------------------------------------------------
# Some output information
# ------------------------------------------------------------------------
print('============================================================\n')
print('    Demo: Car-Following Leading Cruise Control (Python Version) \n')
print('          By Haoxing Du, Meihui Liu, Yue Yin                \n\n')
print('       Converted from Matlab version   ')
print('          By Jiawei Wang, Yang Zheng \n')
print('============================================================\n')
print('Number of HDV vehicles behind: %d\n' % (n))
print('Perturbation vehicle Id      : %d\n' % (PerturbedID))
print('---------------------------\n')
print('HDV car-following model: optimal velocity model (OVM) \n')
print('Parameter setup in HDV car-following model: \n')
print('    alpha  beta  s_st  s_go  v_max \n    %4.2f  %4.2f  %4.2f  %4.2f  %4.2f\n'% (alpha,beta,s_st,s_go,v_max))
print('Coefficients in linearized HDV car-following model: \n')
print('    alpha1  alpha2  alpha3 \n    %4.2f    %4.2f    %4.2f \n'% (alpha1,alpha2,alpha3))
print('---------------------------\n')
print('Simulation length (time step): %d  (%4.2f)\n' % (TotalTime,Tstep))  # this can be improved
print('-----------------------------------------------------------\n')
print('   Simulation beigns ...')

# ------------------------------------------------------------------------
# Experiment starts here
# ------------------------------------------------------------------------
# Mix or not
#   mix  = 0: All HDVs 
#   mix  = 1: there exists one CAV -- Free-driving LCC

start_time = time.time()
for mix in range(0, 2):
    if mix == 0:
        ActuationTime = 99999
    else:
        ActuationTime = 0; # LCC controller work or not: 0 - Controller Work; Large: won't work

    # -----------------------------------------------
    # Define state variables
    # -----------------------------------------------
    # Initial State for each vehicle
    S     = np.zeros((NumStep,m+n+2,3))
    dev_s = 0
    dev_v = 0
    co_v  = 1.0
    v_ini = co_v*v_star  #Initial velocity
    # from - dev to dev
    S[0, :, 0] = (np.linspace(0, -(m + n + 1) * s_star, m + n + 2) + (np.random.rand(m + n + 2) * 2 * dev_s - dev_s)).reshape((12,))
    # The vehicles are uniformly distributed on the straight road with a random deviation
    S[0,:,1] = (v_ini * np.ones((m+n+2,1)) + (np.random.rand(m+n+2,1)*2*dev_v-dev_v)).reshape((12,))

    # meaning of parameters
    # 1:        head vehicle
    # 2-(m+1):  Preceding vehicles
    # m+2:      CAV
    # (m+3)-(m+n+2): Following vehicles

    ID = np.zeros((1,m+n+2))
    if mix:
        ID[0, m+1] = 1
    
    X = np.zeros((2*(m+n+1),NumStep))
    u = np.zeros((NumStep,1))                # 0. HDV  1. CAV
    V_diff = np.zeros((NumStep,m+n+1))       # Velocity Difference
    D_diff = np.zeros((NumStep,m+n+1))       # Following Distance

    # ---------------------------------------------------------
    # LCC controller: the following choice is used in our paper
    # ---------------------------------------------------------
    K = np.zeros((1,2*(n+1)))
    if FD_bool:
        K[0, 0:6] = [0,-0.5,-0.2,0.05,-0.1,0.05]
    else:
        K[0, 0:6] = [0.1,-0.5,-0.2,0.05,-0.1,0.05]

    # ---------------------------------------------------------
    # Simulation starts here
    # ---------------------------------------------------------
    for k in range(0, NumStep - 1):
        V_diff[k, :] = S[k, 0:-1, 1] - S[k, 1:, 1]
        D_diff[k, :] = S[k, 0:-1, 0] - S[k, 1:, 0]
        cal_D = D_diff[k, :]
        for i in range(0, m+n+1):
            if cal_D[i] > s_go:
                cal_D[i] = s_go
            elif cal_D[i] < s_st:
                cal_D[i] = s_st
        
        # nonlinear OVM Model
        # V_d = v_max/2*(1-cos(pi*(h-h_st)/(h_go-h_st)));
        # a2 = alpha*(V_d-v2)+beta*(v1-v2);
        acel = alpha * ((v_max / 2) * (1 - np.cos(np.pi * (cal_D - s_st) / (s_go - s_st))) - S[k, 1:, 1]) + beta * V_diff[k, :]
        acel[acel > acel_max] = acel_max
        acel[acel < dcel_max] = dcel_max

        S[k, 1:, 2] = acel
        S[k, 0, 2] = 0

        if PerturbedType == 1:
            P_A = 0.2
            P_T = 15
            if (k * Tstep) > 20 and (k * Tstep) < (20 + P_T):
                S[k, PerturbedID, 2] = P_A * math.cos(2 * math.pi / P_T * (k * Tstep-20))
        elif PerturbedType == 2:
            if (k * Tstep > 20) and (k * Tstep < 21):
                S[k, PerturbedID, 2] = -5

        X[0::2, k] = (np.reshape(D_diff[k, :], (m + n + 1, 1)) - s_star).reshape((11,))
        X[1::2, k] = (np.reshape(S[k, 1:, 1], (m + n + 1, 1)) - v_star).reshape((11,))
        if k > ActuationTime/Tstep:
            u[k] = np.dot(K, X[:, k])
            if u[k] > acel_max:
                u[k] = acel_max
            elif u[k] < dcel_max:
                u[k] = dcel_max
            S[k, m+1, 2] = u[k]

        #
        # SD as ADAS to prevent crash
        # 
        acel_sd = (S[k, 1:, 1]**2 - S[k, 0:-1, 1]**2) / (2 * D_diff[k, :])
        acel_temp = S[k, :, 2]
        acel_temp_first11 = acel_temp[:-1]
        acel_temp_first11[acel_sd > np.abs(dcel_max)] = dcel_max
        acel_temp[:-1] = acel_temp_first11
        S[k, :, 2] = acel_temp
            
        S[k+1, :, 1] = S[k, :, 1] + Tstep * S[k, :, 2]
        S[k+1, :, 0] = S[k, :, 0] + Tstep * S[k, :, 1]

    # Data Recording
    if mix:
        S_LCC = S
    else:
        S_HDV = S

end_time = time.time()
tsim = end_time - start_time

print('  ends at %6.4f seconds \n' % (tsim))
print('   Computing average abosulte velocity error ... \n')

# -------------------------------------------------------------------------
# Calculate Average Abosulte Velocity Error
# -------------------------------------------------------------------------
VelocityDeviation_HDV = 0
VelocityDeviation_LCC = 0

for i in range(int(20/Tstep-1), int(40/Tstep)):
    VelocityDeviation_HDV += np.sum(np.abs(S_HDV[i, 1:, 1] - v_star))
    VelocityDeviation_LCC += np.sum(np.abs(S_LCC[i, 1:, 1] - v_star))

VelocityDeviation_HDV = VelocityDeviation_HDV*Tstep/20/11
VelocityDeviation_LCC = VelocityDeviation_LCC*Tstep/20/11

print(' Looking ahead only (all HDVs)  |   LCC       |  Improvement rate \n')
print(' %.2f                           |  %.2f       |  %6.4f \n' % (VelocityDeviation_HDV,VelocityDeviation_LCC,(VelocityDeviation_HDV-VelocityDeviation_LCC)/VelocityDeviation_HDV))

print('   Computing total fuel consumption ... \n')
# -------------------------------------------------------------------------
# Calculate Fuel Consumption
# -------------------------------------------------------------------------
FuelConsumption_HDV = 0
FuelConsumption_LCC = 0

for i in range(int(20/Tstep-1), int(40/Tstep)):
    R_HDV = 0.333 + 0.00108*S_HDV[i, 1:, 1]**2 + 1.2*S_HDV[i, 1:, 2]
    F_HDV = 0.444 + 0.09 * R_HDV * S_HDV[i, 1:, 1] + 0.054 * np.maximum(0, S_HDV[i, 1:, 2])**2 * S_HDV[i, 1:, 1]
    F_HDV[R_HDV <= 0] = 0.444
    FuelConsumption_HDV = FuelConsumption_HDV + np.sum(F_HDV)*Tstep
    R_LCC  = 0.333 + 0.00108*S_LCC[i,1:,1]**2 + 1.2*S_LCC[i,1:,2]
    F_LCC  = 0.444 + 0.09*R_LCC*S_LCC[i, 1:,1] + 0.054 * np.maximum(0,S_LCC[i,1:,2])**2*S_LCC[i,1:,1]
    F_LCC[R_LCC <= 0] = 0.444
    FuelConsumption_LCC = FuelConsumption_LCC + np.sum(F_LCC)*Tstep
print(' Looking ahead only (all HDVs)  |   LCC       |  Improvement rate \n')
print(' %.2f                         |  %.2f     |  %6.4f \n' % (FuelConsumption_HDV,FuelConsumption_LCC,(FuelConsumption_HDV-FuelConsumption_LCC)/FuelConsumption_HDV))

# ------------------------------------------------------------------------
#  Plot Video 
# ------------------------------------------------------------------------
videoOutput = 0  # whether save into a video file
vehicleSize = 12 # MarkerSize
FSize = 16
VehicleColor = [93 / 255, 40 / 255, 132 / 255]
if FD_bool:
    videoFile ='FDLCC_Comparison_BrakeID_' + str(PerturbedID) + '.gif'
else:
    videoFile ='CFLCC_Comparison_BrakeID_' + str(PerturbedID) + '.gif'

#-------------------Begin Animation----------------------------------------
Position1 = [0.1,0.6,0.8,0.3]
Position2 = [0.1,0.15,0.8,0.3]
fig, (ax1, ax2) = plt.subplots(2, 1)
ax1.set_position(Position1)
ax2.set_position(Position2)
fig.add_subplot(111, frameon=False)
plt.tick_params(labelcolor='none', which='both', top=False, bottom=False, left=False, right=False)
line1 = [None] * (n+1)
position1 = [None] * (n+1)
line2 = [None] * (n+1)
position2 = [None] * (n+1)
dt = 0.02
title1 = plt.text(0.5,0.85, "Time = 0 s", bbox={'facecolor':'w', 'alpha':0.5, 'pad':5},
                transform=ax1.transAxes, ha="center")

original_x1 = S_HDV[0, 1, 0] + 20
original_x2 = S_LCC[0, 1, 0] + 20 
pstart1 = ax1.plot(original_x1, 0, 'o', markersize=vehicleSize/2, markerfacecolor='k', markeredgecolor='none')[0]
pstart2 = ax2.plot(original_x2, 0, 'o', markersize=vehicleSize/2, markerfacecolor='k', markeredgecolor='none')[0]

def init():
    x = np.linspace(0, 2000, 2000)
    y = np.zeros(2000)
    ax1.plot(x, y, '--', linewidth=0.5, color='k')
    ax2.plot(x, y, '--', linewidth=0.5, color='k')

    for id in range(n+1):
        line1[id] = ax1.plot(np.linspace(S_HDV[0, id - 1, 0], S_HDV[0, id, 0], 10), -5 * np.ones(10),  linewidth=1, color=VehicleColor)[0]
        line2[id] = ax2.plot(np.linspace(S_LCC[0, id - 1, 0], S_LCC[0, id, 0], 10), 5 * np.ones(10),  linewidth=1, color=VehicleColor)[0]
    for id in range(n+1):
        position1[id] = ax1.plot(S_HDV[0, id, 0], -5, marker='o', markersize=vehicleSize)[0]
        position2[id] = ax2.plot(S_LCC[0, id, 0], 5, marker='o', markersize=vehicleSize)[0]
        if id == 0:
            position1[id].set_markerfacecolor([0 / 255, 176 / 255, 240 / 255])
            position1[id].set_markeredgecolor('none')
            position2[id].set_markerfacecolor([0 / 255, 176 / 255, 240 / 255])
            position2[id].set_markeredgecolor('none')
        else:
            position1[id].set_markerfacecolor([0.7, 0.7, 0.7])
            position1[id].set_markeredgecolor('none')
            position2[id].set_markerfacecolor([0.7, 0.7, 0.7])
            position2[id].set_markeredgecolor('none')

    if FD_bool:
        line1[0].set_visible(False)
        line2[0].set_visible(False)
        pstart1.set_visible(False)
        pstart2.set_visible(False)
    
    ax1.axis([original_x1 - 250, original_x1, -6, 6])
    ax2.axis([original_x2 - 250, original_x2, -6, 6])
    fig.gca().set_ylabel('Velocity Perturbation ($\mathrm{m/s}$)', fontsize=FSize)
    fig.gca().set_xlabel('Position ($\mathrm{m}$)', fontsize=FSize)
    ax1.set_yticks([-6, 0, 6])
    ax2.set_yticks([-6, 0, 6])
    fig.gca().tick_params(axis='both', which='both', labelsize=FSize)
    title1.set_text("Time = 0 s")
    

def update(frame):
    i = int(15 / Tstep) + int(dt / Tstep) * frame

    for id in range(1, n+2):
        line1[id-1].set_xdata(np.linspace(S_HDV[i-1,id-1,0], S_HDV[i-1,id,0], 10))
        line1[id-1].set_ydata(np.linspace(S_HDV[i-1,id-1,1]-15, S_HDV[i-1,id,1]-15, 10))
        position1[id-1].set_xdata(S_HDV[i-1, id, 0])
        position1[id-1].set_ydata(S_HDV[i-1, id, 1]-15)
        line2[id-1].set_xdata(np.linspace(S_LCC[i-1,id-1,0], S_LCC[i-1,id,0], 10))
        line2[id-1].set_ydata(np.linspace(S_LCC[i-1,id-1,1]-15, S_LCC[i-1,id,1]-15, 10))
        position2[id-1].set_xdata(S_LCC[i-1, id, 0])
        position2[id-1].set_ydata(S_LCC[i-1, id, 1]-15)
    
    original_x1 = S_HDV[0, 1, 0] + 20 + 15 * 15 + 15 * dt * frame
    original_x2 = S_LCC[0, 1, 0] + 20 + 15 * 15 + 15 * dt * frame
    pstart1.set_xdata(original_x1)
    pstart2.set_xdata(original_x2)
    ax1.set_xlim([original_x1 - 250, original_x1])
    ax2.set_xlim([original_x2 - 250, original_x2])

    title1.set_text(f'Time = {i * Tstep:.1f} s')
       
ani = animation.FuncAnimation(fig, update, frames = 1750, interval = 5, init_func=init, blit=False, repeat=False)

#Comment out the following lines to save the result as a gif image
#print('-----------------------------------------------------------\n')
#print('    Now record a video for demonstration, please wait ... \n')
#writergif = animation.PillowWriter(fps=30)
#ani.save('filename.gif',writer=writergif)

plt.show()