import math
import sympy as sp
import numpy as np
import time
import matplotlib.pyplot as plt

# =========================================================================
#         Frequency-domain Response of LCC for a Perturbation Ahead
#
# Numerically calculate the magnitude of the head-to-tail transfer function 
# of LCC at various excitation frequencies, i.e., |G(jw)|, at different 
# feedback cases 
#
# See Section V.A of the following paper for details
#   Title : Leading Cruise Control in Mixed Traffic Flow:
#                      System Modeling,Controllability,and String Stability
#   Author:Jiawei Wang, Yang Zheng, Chaoyi Chen, Qing Xu and Keqiang Li
#   Python Version implemented by Haoxin Du, Meihui Liu, Yue Yin
# =========================================================================

# -------------------------------------------------------------------------
#   Parameter setup
# -------------------------------------------------------------------------

m = 2          # the number of preceding HDVs ahead
n = 2          # the number of following HDVs behind

s_star = 20    # Equilibrium spacing
               # correspond to an equilibrium velocity of 15 m/s

# ------------------------------------------
# Parameters in the car-following model
# ------------------------------------------
# Driver Model: OVM
alpha = 0.6
beta = 0.9
v_max  = 30
s_st   = 5
s_go   = 35

# linearized model
alpha1 = alpha * v_max / 2 * math.pi / (s_go - s_st) * math.sin(math.pi * (s_star - s_st) / (s_go - s_st))
alpha2 = alpha + beta
alpha3 = beta

# -------------------------------------------------------------------------
# Parameters for feedback gains
# -------------------------------------------------------------------------
# feedback gains corresponding to vehicle -2
id_ahead2 = -2
mu_ahead2 = 1
k_ahead2 = -1
# feedback gains corresponding to vehicle -1
id_ahead1 = -1
mu_ahead1 = 1
k_ahead1 = -1
# feedback gains corresponding to vehicle 1
id_behind1 = 1
mu_behind1 = -1
k_behind1 = -1
# feedback gains corresponding to vehicle 2
id_behind2 = 2
mu_behind2 = -1
k_behind2 = -1

# -------------------------------------------------------------------------
# Some output information
# -------------------------------------------------------------------------
print('-----------------------------------------------------------\n')
print('           Frequency-domain Response of LCC \n')
print('          By Meihui Liu, Yue Yin                \n\n')
print('       Converted from Matlab version   ')
print('          By Jiawei Wang, Yang Zheng \n')
print('Number of HDVs behind: %d\n'% (n))
print('Number of HDVs ahead : %d\n'% (m))
print('HDV car-following model  : %4.2f  %4.2f\n' % (alpha,beta))  # this can be improved
print('-----------------------------------------------------------\n')
print('   Numerical calculation beigns ...')

# -------------------------------------------------------------------------
# Calculate the head-to-tail transfer function of LCC
# -------------------------------------------------------------------------
w = np.linspace(1e-8, 3, 1000)
start_time = time.time()
# HDV-only case
HeadTail_Tf0 = lambda w: abs(((alpha3 * 1j * w + alpha1) / (-w ** 2 + alpha2 * 1j * w + alpha1)) ** (n + m + 1))

# Case A: consider the information of vehicle -2
HeadTail_Tf1 = lambda w: abs(((alpha3 * 1j * w + alpha1) / (-w ** 2 + alpha2 * 1j * w + alpha1)) ** (n + m)) * abs(
    (alpha3 * 1j * w + alpha1 + (mu_ahead2 * ((-w ** 2 + alpha2 * 1j * w + alpha1) / (alpha3 * 1j * w + alpha1)) -
                                   mu_ahead2 + k_ahead2 * 1j * w) * (
                 (alpha3 * 1j * w + alpha1) / (-w ** 2 + alpha2 * 1j * w + alpha1)) ** (id_ahead2 + 1)) /
    (-w ** 2 + alpha1 + 1j * w * alpha2))

# Case B: consider the information of vehicles -2, -1
HeadTail_Tf2 = lambda w: abs(((alpha3 * 1j * w + alpha1) / (-w ** 2 + alpha2 * 1j * w + alpha1)) ** (n + m)) * abs(
    (alpha3 * 1j * w + alpha1 + (mu_ahead2 * ((-w ** 2 + alpha2 * 1j * w + alpha1) / (alpha3 * 1j * w + alpha1)) -
                                   mu_ahead2 + k_ahead2 * 1j * w) *
                                  ((alpha3 * 1j * w + alpha1) / (-w ** 2 + alpha2 * 1j * w + alpha1)) ** (
                                              id_ahead2 + 1) +
                               (mu_ahead1 * ((-w ** 2 + alpha2 * 1j * w + alpha1) / (alpha3 * 1j * w + alpha1)) -
                                mu_ahead1 + k_ahead1 * 1j * w) *
                                  ((alpha3 * 1j * w + alpha1) / (-w ** 2 + alpha2 * 1j * w + alpha1)) ** (
                                              id_ahead1 + 1)) /(-w ** 2 + alpha1 + 1j * w * alpha2))

# Case C: consider the information of vehicles -2, -1, 1
HeadTail_Tf3 = lambda w: abs(((alpha3 * 1j * w + alpha1) / (-w ** 2 + alpha2 * 1j * w + alpha1)) ** (n + m)) * abs(
    (alpha3 * 1j * w + alpha1 + (mu_ahead2 * ((-w ** 2 + alpha2 * 1j * w + alpha1) / (alpha3 * 1j * w + alpha1)) -
                                   mu_ahead2 + k_ahead2 * 1j * w) *
                                  ((alpha3 * 1j * w + alpha1) / (-w ** 2 + alpha2 * 1j * w + alpha1)) ** (
                                              id_ahead2 + 1) +
                               (mu_ahead1 * ((-w ** 2 + alpha2 * 1j * w + alpha1) / (alpha3 * 1j * w + alpha1)) -
                                mu_ahead1 + k_ahead1 * 1j * w) *
                                  ((alpha3 * 1j * w + alpha1) / (-w ** 2 + alpha2 * 1j * w + alpha1)) ** (
                                              id_ahead1 + 1)) / (-w ** 2 + alpha1 + 1j * w * alpha2 -
     (mu_behind1 * ((-w ** 2 + alpha2 * 1j * w + alpha1) / (alpha3 * 1j * w + alpha1)) -
      mu_behind1 + k_behind1 * 1j * w) *
     ((alpha3 * 1j * w + alpha1) / (-w ** 2 + alpha2 * 1j * w + alpha1)) ** id_behind1))

# Case D: consider the information of vehicles -2, -1, 1, 2
HeadTail_Tf4 = lambda w: abs(((alpha3 * 1j * w + alpha1) / (-w ** 2 + alpha2 * 1j * w + alpha1)) ** (n + m)) * abs(
    (alpha3 * 1j * w + alpha1 + (mu_ahead2 * ((-w ** 2 + alpha2 * 1j * w + alpha1) / (alpha3 * 1j * w + alpha1)) -
                                   mu_ahead2 + k_ahead2 * 1j * w) *
                                  ((alpha3 * 1j * w + alpha1) / (-w ** 2 + alpha2 * 1j * w + alpha1)) ** (
                                              id_ahead2 + 1) +
                               (mu_ahead1 * ((-w ** 2 + alpha2 * 1j * w + alpha1) / (alpha3 * 1j * w + alpha1)) -
                                mu_ahead1 + k_ahead1 * 1j * w) *
                                  ((alpha3 * 1j * w + alpha1) / (-w ** 2 + alpha2 * 1j * w + alpha1)) ** (
                                              id_ahead1 + 1)) /
    (-w ** 2 + alpha1 + 1j * w * alpha2 -
     (mu_behind1 * ((-w ** 2 + alpha2 * 1j * w + alpha1) / (alpha3 * 1j * w + alpha1)) -
      mu_behind1 + k_behind1 * 1j * w) *
     ((alpha3 * 1j * w + alpha1) / (-w ** 2 + alpha2 * 1j * w + alpha1)) ** id_behind1 -
     (mu_behind2 * ((-w ** 2 + alpha2 * 1j * w + alpha1) / (alpha3 * 1j * w + alpha1)) -
      mu_behind2 + k_behind2 * 1j * w) *
     ((alpha3 * 1j * w + alpha1) / (-w ** 2 + alpha2 * 1j * w + alpha1)) ** id_behind2))
HeadTail_Tf0(w)
HeadTail_Tf1(w)
HeadTail_Tf2(w)
HeadTail_Tf3(w)
HeadTail_Tf4(w)
end_time = time.time()
tsim = end_time - start_time

print('  ends at %6.8f seconds \n'% (tsim))

# -------------------------------------------------------------------------
#  Plot Results 
# -------------------------------------------------------------------------
print('-----------------------------------------------------------\n')
print('    Now plot the results for frequency-domain response, please wait ... \n')

#w = np.linspace(1e-8, 3, 1000)
plt.rcParams['font.family'] = 'serif'
plt.rcParams['font.serif'] = 'Times New Roman'
plt.rcParams["mathtext.fontset"] = 'dejavuserif'
plt.rcParams["text.usetex"] = False
plt.figure(figsize=(8, 6))
ax = plt.axes() 

plt.plot(w, HeadTail_Tf0(w), linewidth=2, color='k', label='HDV only')
plt.plot(w, HeadTail_Tf1(w), linewidth=2, color='#F4357C', label='Case A')
plt.plot(w, HeadTail_Tf2(w), linewidth=2, color='#FFB000', label='Case B')
plt.plot(w, HeadTail_Tf3(w), linewidth=2, color='#4379E3', label='Case C')
plt.plot(w, HeadTail_Tf4(w), linewidth=2, color='#8333EC', label='Case D')
plt.plot(np.linspace(0, 3, 10), np.ones(10), 'k--')

plt.xlabel(r'$\omega$', fontsize=22, color='k')
plt.ylabel(r'$|\Gamma(j\omega)|$', fontsize=22, color='k')
plt.tick_params(labelsize=18)
plt.ylim([0, 1.2])
plt.xlim([0, 3])
plt.grid()
ax.yaxis.set_ticks([0, 0.5, 1.0])
ax.xaxis.set_ticks(np.arange(0, 4, 1))

# Legend
legend_labels = ['HDV only', 'Case A', 'Case B', 'Case C', 'Case D']
legend_colors = ['k', '#F4357C', '#FFB000', '#4379E3', '#8333EC']
plt.legend(legend_labels, loc='right', fontsize=22, frameon=False)

plt.show()