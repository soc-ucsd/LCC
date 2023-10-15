# =========================================================================
#               Control Energy of Free-Driving LCC
#
# Numerically calculate the three energy-related metrics of the FD-LCC 
# system at different system sizes and time lengths
#
# See Section III.B of the following paper for details
#   Title : Leading Cruise Control in Mixed Traffic Flow:
#                      System Modeling,Controllability,and String Stability
#   Author:Jiawei Wang, Yang Zheng, Chaoyi Chen, Qing Xu and Keqiang Li
# =========================================================================

import numpy as np
import matplotlib.pyplot as plt
import time as my_time
from tqdm import tqdm  # For creating progress bars
from scipy.linalg import expm, inv, eig
import sys
sys.path.insert(0, '/Users/yueyin/LCC/python/_model')
from SystemModel_CF import SystemModel_CF
from SystemModel_FD import SystemModel_FD

# -------------------------------------------------------------------------
# Parameter setup
# -------------------------------------------------------------------------
FD_bool = 0 # mode of the LCC system
            # 0. CF-LCC; 1. FD-LCC
s_star = 20 # Equilibrium spacing
            # correspond to an equilibrium velocity of 15 m/s

# ------------------------------------------
# Parameters in the car-following model
# ------------------------------------------
# Driver Model: OVM
alpha = 0.6
beta = 0.9
s_st = 5
s_go = 35
v_max = 30

# linearized model
alpha1 = alpha * v_max / 2 * np.pi / (s_go - s_st) * np.sin(np.pi * (s_star - s_st) / (s_go - s_st))
alpha2 = alpha + beta
alpha3 = beta

# ------------------------------------------
# Parameters for calculating the energy
# ------------------------------------------
T_collected = [10, 20, 30]        # Time length: t
N_collected = list(range(1, 6))   # System size: n

# -------------------------------------------------------------------------
# Energy-related metrics
# -------------------------------------------------------------------------
# Metric 1: smallest eigenvalue of Controllability Gramian: lambda_min(W(t))
MinEnergy = np.zeros((len(N_collected), len(T_collected)))
# Metric 2: trace of inverse Controllability Gramian: Tr(W(t)^(-1))
AvgEnergy = np.zeros((len(N_collected), len(T_collected)))
# Metric 3: minimum transfer energy: E_min(t)
TransferEnergy = np.zeros((len(N_collected), len(T_collected)))

# -------------------------------------------------------------------------
# Some output information
# -------------------------------------------------------------------------
print('=====================================================================')
print('           Control Energy of Free-Driving LCC (Python Version)')
print('                 By Haoxin Du, Meihui Liu, Yue Yin ')
print()  
print('                   Converted from Matlab Version ')
print('                    By Jiawei Wang, Yang Zheng ')
print('=====================================================================')
print('System size (number of HDVs behind) :', ' '.join(map(str, N_collected)))
print('Time length                         :', ' '.join(map(str, T_collected)))
print('HDV car-following model: optimal velocity model (OVM)')
print('Parameter setup in HDV car-following model:')
print(f'    alpha  beta  s_st  s_go  v_max \n    {alpha:.2f}  {beta:.2f}  {s_st:.2f}  {s_go:.2f}  {v_max:.2f}')
print('Coefficients in linearized HDV car-following model:')
print(f'    alpha1  alpha2  alpha3 \n    {alpha1:.2f}    {alpha2:.2f}    {alpha3:.2f}')
print('-----------------------------------------------------------')
print('   Numerical calculation begins ...')

# -------------------------------------------------------------------------
# Numerical calculation starts here
# -------------------------------------------------------------------------
start_time = my_time.time()
total_iterations = len(N_collected) * len(T_collected)
with tqdm(total=total_iterations, desc="Processing") as pbar:
    iTest = 0
    for iN, N in enumerate(N_collected):
        for iT, time in enumerate(T_collected):

            iTest = iTest+1
            N = N_collected[iN]
            time = T_collected[iT]

            if FD_bool:
                A, B = SystemModel_FD(N, alpha1, alpha2, alpha3)
            else:
                A, B = SystemModel_CF(N, alpha1, alpha2, alpha3)
            # Controllability Gramian
            # If the system is asymptotically stable, then Wc = lyap(A,B*B');
            # Here, we use the definition to calculate Wc.
            Wc = np.zeros_like(A)
            tau = 0.1
            for x in np.arange(0, time + tau, tau):
                expAx = expm(A * x)
                expAxt = expm(A.T * x)
                Wc += np.dot(np.dot(expAx, np.dot(B, B.T)), expAxt) * tau

            MinEnergy[iN, iT] = min(np.linalg.eigvals(Wc))
            AvgEnergy[iN, iT] = np.trace(inv(Wc))

            # Equilibrium spacing with one-unit higher equilibrium velocity
            v = 1
            s = (alpha2 - alpha3) / alpha1 * v
            temp = np.vstack((s, v))
            x = np.tile(temp, (N + 1, 1)) #Target state
            TransferEnergy[iN, iT] = np.dot(np.dot(x.T, np.linalg.inv(Wc)), x)

            pbar.update(1)
# Calculate execution time
tsim = my_time.time() - start_time
print(f'ends at {tsim:.4f} seconds')

# -------------------------------------------------------------------------
# Plot Results 
# -------------------------------------------------------------------------
print('-----------------------------------------------------------')
print('    Now plot the results for control energy, please wait ... ')

Wsize = 22  # word size
Lwidth = 1.5
Msize = 8
color1 = [215 / 255, 51 / 255, 201 / 255]
color2 = [242 / 255, 60 / 255, 84 / 255]
color3 = [67 / 255, 121 / 255, 227 / 255]

plt.figure(1)
p1, = plt.semilogy(N_collected, MinEnergy[:, 0], color=color1, linewidth=Lwidth, label=f'$t={T_collected[0]}$')
plt.semilogy(N_collected, MinEnergy[:, 0], 'o', color=color1, markersize=Msize, linewidth=Lwidth)
p2, = plt.semilogy(N_collected, MinEnergy[:, 1], color=color2, linewidth=Lwidth, label=f'$t={T_collected[1]}$')
plt.semilogy(N_collected, MinEnergy[:, 1], '^', color=color2, markersize=Msize, linewidth=Lwidth)
p3, = plt.semilogy(N_collected, MinEnergy[:, 2], color=color3, linewidth=Lwidth, label=f'$t={T_collected[2]}$')
plt.semilogy(N_collected, MinEnergy[:, 2], 'x', color=color3, markersize=Msize, linewidth=Lwidth)
plt.grid(True)
plt.gca().tick_params(labelsize=Wsize-5)
plt.legend(handles=[p1, p2, p3], loc='upper right')
plt.xlabel('$n$', fontsize=Wsize, color='k')
plt.title('$\lambda_{\mathrm{min}}(W(t))$', fontsize=Wsize, color='k')
plt.gcf().set_size_inches(6, 9)
plt.gca().set_ylim([1e-15, 10])
plt.gca().set_xticks(np.arange(0, 7, 2))

plt.figure(2)
p1, = plt.semilogy(N_collected, AvgEnergy[:, 0], color=color1, linewidth=Lwidth, label=f'$t={T_collected[0]}$')
plt.semilogy(N_collected, AvgEnergy[:, 0], 'o', color=color1, markersize=Msize, linewidth=Lwidth)
p2, = plt.semilogy(N_collected, AvgEnergy[:, 1], color=color2, linewidth=Lwidth, label=f'$t={T_collected[1]}$')
plt.semilogy(N_collected, AvgEnergy[:, 1], '^', color=color2, markersize=Msize, linewidth=Lwidth)
p3, = plt.semilogy(N_collected, AvgEnergy[:, 2], color=color3, linewidth=Lwidth, label=f'$t={T_collected[2]}$')
plt.semilogy(N_collected, AvgEnergy[:, 2], 'x', color=color3, markersize=Msize, linewidth=Lwidth)
plt.grid(True)
plt.gca().tick_params(labelsize=Wsize-5)
plt.legend(handles=[p1, p2, p3], loc='upper right')
plt.xlabel('$n$', fontsize=Wsize, color='k')
plt.title('$\mathrm{Tr}(W(t)^{-1})$', fontsize=Wsize, color='k')
plt.gcf().set_size_inches(6, 9)
plt.gca().set_xticks(np.arange(0, 7, 2))

plt.figure(3)
plt.semilogy(N_collected, TransferEnergy[:, 0], color=color1, linewidth=Lwidth, label=f't={T_collected[0]}')
plt.semilogy(N_collected, TransferEnergy[:, 0], 'o', color=color1, markersize=Msize, linewidth=Lwidth)
plt.semilogy(N_collected, TransferEnergy[:, 1], color=color2, linewidth=Lwidth, label=f't={T_collected[1]}')
plt.semilogy(N_collected, TransferEnergy[:, 1], '^', color=color2, markersize=Msize, linewidth=Lwidth)
plt.semilogy(N_collected, TransferEnergy[:, 2], color=color3, linewidth=Lwidth, label=f't={T_collected[2]}')
plt.semilogy(N_collected, TransferEnergy[:, 2], 'x', color=color3, markersize=Msize, linewidth=Lwidth)
plt.grid(True)
plt.tick_params(labelsize=Wsize - 5)
plt.legend(loc='upper right')
plt.xlabel('n', fontsize=Wsize, color='k')
plt.title('$E_{\mathrm{min}}(t)$', fontsize=Wsize, color='k')
plt.gcf().set_size_inches(6, 9)
plt.ylim(1e-2, 1e9)
plt.xticks(np.arange(0, 7, 2))
plt.gca().spines['top'].set_visible(False)
plt.gca().spines['right'].set_visible(False)
plt.tight_layout()

# Show the plot
plt.show()




