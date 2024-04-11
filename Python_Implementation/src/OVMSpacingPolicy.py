import numpy as np
import math
import matplotlib.pyplot as plt

# =========================================================================
#           Illustration for Desired Velocity Function in OVM
#
# Numerically calculate the three energy-related metrics of the FD-LCC 
# system at different system sizes and time lengths
#
# See II.A of the following paper for details
#   Title : Leading Cruise Control in Mixed Traffic Flow:
#                      System Modeling,Controllability,and String Stability
#   Author:Jiawei Wang, Yang Zheng, Chaoyi Chen, Qing Xu and Keqiang Li
#   Python Version implemented by Haoxin Du, Meihui Liu, Yue Yin
# =========================================================================

# -------------------------------------------------------------------------
#   Parameter setup in OVM model
# -------------------------------------------------------------------------
alpha = 0.6
beta = 0.9
s_st = 5
s_go = 35
v_max = 30

s = np.arange(0, 40 + 0.1, 0.1) # spacing
v = v_max/2*(1-np.cos(np.pi*(s-s_st)/(s_go-s_st)));   # desired velocity
v[s < s_st] = 0
v[s>s_go] = v_max

# -------------------------------------------------------------------------
# Some output information
# -------------------------------------------------------------------------
print('=====================================================================\n')
print('      Illustration for Desired Velocity Function in OVM \n')
print('          By Meihui Liu, Yue Yin                \n\n')
print('       Converted from Matlab version   ')
print('          By Jiawei Wang, Yang Zheng \n')
print('=====================================================================\n')
print('HDV car-following model: optimal velocity model (OVM) \n')
print('Parameter setup in HDV car-following model: \n')
print('    alpha  beta  s_st  s_go  v_max \n    %4.2f  %4.2f  %4.2f  %4.2f  %4.2f\n'% (alpha,beta,s_st,s_go,v_max))
print('-----------------------------------------------------------\n')


# -------------------------------------------------------------------------
#  Plot Results 
# -------------------------------------------------------------------------
print('    Now plot the illustration for desired velocity function, please wait ... \n')

plt.rcParams['font.family'] = 'serif'
plt.rcParams['font.serif'] = 'Times New Roman'
plt.rcParams["mathtext.fontset"] = 'dejavuserif'
plt.rcParams["text.usetex"] = False

plt.figure(figsize=(6, 5))
plt.plot(s, v, linewidth=3, label=r'Desired Velocity')
plt.plot(s, [v_max] * len(s), '--k', linewidth=2, label=r'$v_{max}$')
plt.grid(True)

plt.text(1, v_max + 1, r'$v_{max}$', fontsize=20, color='k')
plt.text(31, 2, r'$s_{go}$', fontsize=20, color='k')
plt.text(4.5, 2, r'$s_{st}$', fontsize=20, color='k')
plt.plot([5, 5], [-1, 1], '-k', linewidth=2)
plt.plot([35, 35], [0, 30], '--k', linewidth=2)

plt.xlabel(r'Spacing', fontsize=24, color='k')
plt.ylabel(r'Desired Velocity', fontsize=24, color='k')
plt.xticks([])
plt.yticks([])
plt.xlim(0, 40)
plt.ylim(0, 35)
plt.tight_layout()

plt.show()