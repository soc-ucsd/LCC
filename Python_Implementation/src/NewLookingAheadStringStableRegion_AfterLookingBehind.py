import math
import numpy as np
import scipy.io
from joblib import Parallel, delayed
from tqdm import tqdm
from scipy.optimize import minimize_scalar
from scipy.io import loadmat
from scipy.io import savemat
from datetime import datetime
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import multiprocessing as mp

# =========================================================================
#   New "Looking Ahead" String Stable Region of LCC after Looking Behind
#
# Find the new "looking ahead" head-to-tail string stable regions after
# incorporating the motion of one vehicle behind
#
# See Section IV.B (Fig. 7(c)-(f)) of the following paper for details
#   Title : Leading Cruise Control in Mixed Traffic Flow:
#                      System Modeling,Controllability,and String Stability
#   Author:Jiawei Wang, Yang Zheng, Chaoyi Chen, Qing Xu and Keqiang Li
#   Python Version implemented by Haoxin Du, Meihui Liu, Yue Yin
# =========================================================================

# -------------------------------------------------------------------------
#   Parameter setup
# -------------------------------------------------------------------------
id_ahead = -2;          # id of the preceding HDV which is under 
                        # investigation of the string stability region
id_behind = 1;          # id of the following HDV which is incorporated

m = 2;                  # number of preceding vehicles
n = 2;                  # number of following vehicles

generate_data_bool = 0; # 0. Use existing data; 1. Generate new data

# ------------------------------------------
# Feedback gain setup for the following HDV
# ------------------------------------------
if id_behind == 1:
    mu_behind = -1
    k_behind = -1
elif id_behind == 2:
    mu_behind = -1
    k_behind = -1

# ------------------------------------------
# Parameters in the car-following model
# ------------------------------------------
s_star = 20; # Equilibrium spacing
DriverDynamics = 1
if DriverDynamics == 1:
        alpha = 0.6
        beta = 0.9
elif DriverDynamics == 2:
        alpha = 0.4
        beta = 0.6                               

v_max  = 30
s_st   = 5
s_go   = 35
alpha1 = alpha*v_max/2*math.pi/(s_go-s_st)*math.sin(math.pi*(s_star-s_st)/(s_go-s_st))
alpha2 = alpha+beta
alpha3 = beta

# -------------------------------------------------------------------------
# Some output information
# -------------------------------------------------------------------------
print('=====================================================================\n')
print('New "Looking Ahead" String Stable Region of LCC after Looking Behind \n')
print('          By Meihui Liu, Yue Yin                \n\n')
print('       Converted from Matlab version   ')
print('          By Jiawei Wang, Yang Zheng \n')
print('=====================================================================\n')
print('Number of HDV vehicles ahead: %d\n'%(m))
print('Number of HDV vehicles behind: %d\n'%(n))
print('---------------------------\n')
print('HDV car-following model: optimal velocity model (OVM) \n')
print('Parameter setup in HDV car-following model: \n')
print('    alpha  beta  s_st  s_go  v_max \n    %4.2f  %4.2f  %4.2f  %4.2f  %4.2f\n'%(alpha,beta,s_st,s_go,v_max))
print('Coefficients in linearized HDV car-following model: \n')
print('    alpha1  alpha2  alpha3 \n    %4.2f    %4.2f    %4.2f \n'%(alpha1,alpha2,alpha3))
print('---------------------------\n')
print('Index of the HDV ahead under investigation: %d \n'%(id_ahead))
print('---------------------------\n')
print('Index of the HDV behind under incorporation: %d \n'%(id_behind))
print('Fixed "looking-behind" feedback gain: \n    mu = %4.2f, k = %4.2f \n'%(mu_behind,k_behind))
print('-----------------------------------------------------------\n')

if generate_data_bool:
    print('    Now calculate the head-to-tail string stable region for "looking-ahead" feedback policies\n')
    print('    Please wait ... \n')
else:
    print('    Now skip the calculation for head-to-tail string stable regions for "looking-ahead" feedback policies\n')

# ------------------------------------------------------------------------
# Experiment starts here
# Find the new "looking ahead" head-to-tail string stable region
# ------------------------------------------------------------------------
if generate_data_bool:
    # ------------------------------------------
    # Range of the investigated feedback gain
    # ------------------------------------------
    K = np.arange(-10, 10.02, 0.02)
    Mu = np.arange(-10, 10.02, 0.02)

    # Whether this setup is string stable
    SS_bool = np.zeros((len(K), len(Mu)))
    print(len(K))
    print(len(Mu))
    TestNumber = len(K) * len(Mu)
    iTest = 1

#    pbar = tqdm(total = len(K) * len(Mu))

    def find_HeadTail_Tf(ik1):
        global SS_bool
        k = K[ik0 - 1]
        mu = Mu[ik1 - 1]
        minus_HeadTail_Tf = lambda w: abs(((alpha3 * 1j * w + alpha1) / (-w**2 + alpha2 * 1j * w + alpha1))**(n + m) * 
                                      np.abs((alpha3*1j*w + alpha1 + (mu * ((-w**2 + alpha2*1j*w + alpha1) / (alpha3*1j*w + alpha1)) - mu + k*1j*w) *
                   ((alpha3*1j*w + alpha1) / (-w**2 + alpha2*1j*w + alpha1))**(id_ahead + 1)) /
                   (-w**2 + alpha1 + 1j*w*alpha2 -
                   (mu_behind * ((-w**2 + alpha2*1j*w + alpha1) / (alpha3*1j*w + alpha1)) - mu_behind + k_behind*1j*w) *
                   ((alpha3*1j*w + alpha1) / (-w**2 + alpha2*1j*w + alpha1))**id_behind)))
        
        result = minimize_scalar(minus_HeadTail_Tf, bounds=(1e-8, 100))
        fval = -result.fun
        x = result.x
        if fval <= 1:
        #    print()
            #SS_bool[ik0 - 1, ik1 - 1] = 1
            SS_bool[1 - 1, ik1 - 1] = 1
        #return fval, x

    #return_val = Parallel(n_jobs = -1)(delayed(find_HeadTail_Tf)(ik0, ik1) for ik1 in np.arange(1, len(Mu) + 1) for ik0 in range(1, len(K) + 1))
    #fval = [item[0] for item in return_val]
    #x = [item[1] for item in return_val]
    pool = mp.Pool(mp.cpu_count())

    #for ik0 in range(1, len(K) + 1):
    pool.map(find_HeadTail_Tf, np.arange(1, 3))
    pool.close()
#   pbar.update(1)
#   pbar.set_description(f"DriverDynamics={DriverDynamics}; ID={id_ahead}; Processing...{ik0/len(K)*100}%")

#    pbar.close()

    data = {"DriverDynamics": DriverDynamics, "K": K, "Mu": Mu, "SS_bool": SS_bool, "TestNumber": TestNumber, "alpha": alpha,
                     "alpha1": alpha1, "alpha2": alpha2, "alpha3": alpha3, "beta": beta, "generate_data_bool": generate_data_bool,
                     "iTest":iTest, "id_ahead": id_ahead, "id_behind": id_behind, "ik0": ik0, "k_behind": k_behind, "m": m,
                     "mu_behind": mu_behind, "n":n, "s_go": s_go, "s_st": s_st, "s_star": s_star, "v_max": v_max}
    current_date = datetime.now().strftime('%d-%b-%Y')
    savemat('./_data/' + current_date + '_SSRegion_WithLookingBehind_id_behind_' + str(id_behind) + 
        '_mu_behind_' + str(mu_behind) + '_k_behind_' + str(k_behind) + 
        '_n_' + str(n) + '_m_' + str(m) + 
        '_DriverDynamics_' + str(DriverDynamics) + '_FeedbackID_' + str(id_ahead) + '.mat', data)

# ------------------------------------------
# Data for original "looking ahead" string stable region
# ------------------------------------------
load_data1 = loadmat('./_data/30-Mar-2020_SSRegion_n_' + str(n) + '_m_' + str(m) +
    '_DriverDynamics_' + str(DriverDynamics) + '_FeedbackID_' + str(id_ahead) + '.mat')
OriginalSS_bool = load_data1['SS_bool']

# ------------------------------------------
# Data for new "looking ahead" string stable region after looking behind
# ------------------------------------------
if not generate_data_bool:
    load_data2 = loadmat('./Python_Implementation/_data/20240404_SSRegion_WithLookingBehind_id_behind_' + str(id_behind) + 
        '_mu_behind_' + str(mu_behind) + '_k_behind_' + str(k_behind) +
        '_n_' + str(n) + '_m_' + str(m) + 
        '_DriverDynamics_' + str(DriverDynamics) + '_FeedbackID_' + str(id_ahead) + '.mat')
    FinalSS_bool = OriginalSS_bool + load_data2['SS_bool']
    K = load_data2['K']
    Mu = load_data2['Mu']
else:
    # Utilize the newly generate data
    # The data name might need to be changed
    load_data2 = loadmat('./Python_Implementation/_data/' + current_date + '_SSRegion_WithLookingBehind_id_behind_' + str(id_behind) + 
        '_mu_behind_' + str(mu_behind) + '_k_behind_' + str(k_behind) + 
        '_n_' + str(n) + '_m_' + str(m) + 
        '_DriverDynamics_' + str(DriverDynamics) + '_FeedbackID_' + str(id_ahead) + '.mat')
    SS_bool = load_data2['SS_bool']
    K = load_data2['K']
    Mu = load_data2['Mu']
    FinalSS_bool = OriginalSS_bool + SS_bool; #added this line cause otherwise FinalSS_bool is not recognzied
    
# ------------------------------------------------------------------------
#  Plot the String Stable Region
# ------------------------------------------------------------------------
print('-----------------------------------------------------------\n')
if not generate_data_bool:
    print('    Now plot the string stable region utilizing existing data, please wait ... \n')
else:
    print('    Now plot the string stable region utilizing newly generated data, please wait ... \n')

Mu_3d, K_3d = np.meshgrid(K, Mu)
Wsize = 18

plt.rcParams['font.family'] = 'serif'
plt.rcParams['font.serif'] = 'Times New Roman'
plt.rcParams["mathtext.fontset"] = 'dejavuserif'
plt.rcParams["text.usetex"] = False
fig = plt.figure(figsize=(20, 20))
ax =  plt.axes(projection ='3d') 
surf = ax.plot_surface(K_3d, Mu_3d, FinalSS_bool, cmap='viridis', edgecolor='none', antialiased=False)

mymap = np.array([[235, 235, 235], [255, 181, 190], [113, 178, 246]]) / 255
surf.set_cmap(plt.cm.colors.ListedColormap(mymap))

if id_ahead == -2:
    ax.set_xlabel(r'$k_{-2}$', fontsize=Wsize, color='k')
    ax.set_ylabel(r'$\mu_{-2}$', fontsize=Wsize, color='k')
elif id_ahead == -1:
    ax.set_xlabel(r'$k_{-1}$', fontsize=Wsize, color='k')
    ax.set_ylabel(r'$\mu_{-1}$', fontsize=Wsize, color='k')
elif id_ahead == 1:
    ax.set_xlabel(r'$k_{1}$', fontsize=Wsize, color='k')
    ax.set_ylabel(r'$\mu_{1}$', fontsize=Wsize, color='k')
elif id_ahead == 2:
    ax.set_xlabel(r'$k_{2}$', fontsize=Wsize, color='k')
    ax.set_ylabel(r'$\mu_{2}$', fontsize=Wsize, color='k')

# Set tick label font properties
for tick in ax.xaxis.get_major_ticks():
    tick.label.set_fontsize(Wsize - 2)
for tick in ax.yaxis.get_major_ticks():
    tick.label.set_fontsize(Wsize - 2)

# Set axis limits
ax.set_xlim(np.min(K), np.max(K))
ax.set_ylim(np.min(Mu), np.max(Mu))
ax.yaxis.set_ticks(np.arange(-10, 11, 10))
ax.xaxis.set_ticks(np.arange(-10, 11, 10))
ax.zaxis.set_ticks(np.arange(0))

ax.view_init(azim=-90, elev=90)

fig.set_size_inches(8, 6)
fig.subplots_adjust(top=0.95, bottom=0.15)

plt.show()





