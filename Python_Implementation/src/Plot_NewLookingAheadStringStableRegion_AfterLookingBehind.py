from scipy.io import loadmat
import numpy as np
import matplotlib.pyplot as plt

# Description
# Plot the new "looking ahead" head-to-tail string stable after incorporation of one vehicle behind 
# Correspond to Fig. 7(c)-(f) in our paper.

# id of HDVs ahead
id_ahead = -1 # -1 or -2
# id of HDVs behind
id_behind = 1 # 1 or 2

# Parameters

n = 2
m = 2

if id_behind == 1:
    mu_behind = -1
    k_behind = -1
elif id_behind == 2:
    mu_behind = -1
    k_behind = -1

DriverDynamics = 1

# Load Data
load_data = loadmat('./Python_Implementation/_data/SSRegion_n_' + str(n) + '_m_' + str(m) +
    '_DriverDynamics_' + str(DriverDynamics) + '_FeedbackID_' + str(id_ahead) + '.mat')
SS_bool = load_data['SS_bool']
K = load_data['K']
Mu = load_data['Mu']
OriginalSS_bool = SS_bool

if id_ahead<0:
    load_data = loadmat('./Python_Implementation/_data/20240404_SSRegion_WithLookingBehind_id_behind_' + str(id_behind) + 
        '_mu_behind_' + str(mu_behind) + '_k_behind_' + str(k_behind) +
        '_n_' + str(n) + '_m_' + str(m) + 
        '_DriverDynamics_' + str(DriverDynamics) + '_FeedbackID_' + str(id_ahead) + '.mat')
    SS_bool = load_data['SS_bool']
    K = load_data['K']
    Mu = load_data['Mu']
    FinalSS_bool = OriginalSS_bool+SS_bool
    
    for i in range(1, SS_bool.shape[0] + 1):
        for j in range(1, SS_bool.shape[1] + 1):
            if OriginalSS_bool[i - 1, j - 1] == 1 and SS_bool[i - 1,j - 1] == 0:
                FinalSS_bool[i - 1, j - 1] = 2

# String Stability Area After Looking Behind

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