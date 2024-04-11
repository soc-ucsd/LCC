import numpy as np
import matplotlib.pyplot as plt
from scipy.io import loadmat
from matplotlib.colors import ListedColormap

# Key Parameters
id = 2  # -2, -1, 1, 2

# Parameters
n = 2
m = 2
DriverDynamics = 1

# Load data (adjust as per your data file's structure and path)
# Replace this part with the appropriate loading mechanism for your .mat files
mat_data_ps = loadmat(f'./Python_Implementation/_data/20240404_PSRegion_n_{n}_m_{m}_DriverDynamics_{DriverDynamics}_FeedbackID_{id}.mat')
PS_bool = mat_data_ps['PS_bool']
K, Mu = mat_data_ps['K'].flatten(), mat_data_ps['Mu'].flatten()

SSPS_bool = np.zeros_like(PS_bool)
Mu_3d, K_3d = np.meshgrid(K, Mu)

a = np.where(PS_bool == 0)
PS_Y = Mu_3d[a]
PS_X = K_3d[a]
PS_Z = np.full(len(a[0]), 2)

mat_data_ss = loadmat(f'./Python_Implementation/_data/SSRegion_n_{n}_m_{m}_DriverDynamics_{DriverDynamics}_FeedbackID_{id}.mat')
SS_bool = mat_data_ss['SS_bool']
K, Mu = mat_data_ss['K'].flatten(), mat_data_ss['Mu'].flatten()
Mu_3d, K_3d = np.meshgrid(K, Mu)





# Plotting
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
#plt.figure(figsize=(10, 6))
# Colormap
mymap = ListedColormap([(235/255, 235/255, 235/255), (113/255, 178/255, 246/255)])

# Surface Plot
p = ax.plot_surface(K_3d, Mu_3d, SS_bool, cmap=mymap, edgecolor='none')
#plt.pcolormesh(K_3d, Mu_3d, SS_bool, cmap=mymap, shading='auto')

# View
ax.view_init(elev=90, azim=-90)   # Rotate the plot for a top-down view
ax.set_zticks([]) 
# Formatting
Wsize = 18
ax.tick_params(labelsize=Wsize - 2, which='both', axis='both', labelcolor='black')
ax.set_xlim([np.min(K), np.max(K)])
ax.set_ylim([np.min(Mu), np.max(Mu)])
 # Hiding the Z-axis

# Labels based on id
labels = { 
    -2: ('$k_{-2}$', '$\mu_{-2}$'), 
    -1: ('$k_{-1}$', '$\mu_{-1}$'), 
    1: ('$k_1$', '$\mu_1$'), 
    2: ('$k_2$', '$\mu_2$') 
}
ax.set_xlabel(labels[id][0], fontsize=Wsize, labelpad=15)
ax.set_xticks([-10, 0, 10])
ax.set_ylabel(labels[id][1], fontsize=Wsize, labelpad=15)

# Scatter plots
s = ax.scatter(PS_X, PS_Y, PS_Z, color='black', alpha=0.05, s=2 )

if id > 0:
    b = ax.scatter(-1, -1, 2, color='black', marker='x', s=50)
    ax.text(-8, -1, 2, '(-1, -1)', fontsize=Wsize-2)
    

plt.show()

# For saving the figure, uncomment the below line
# fig.savefig(f'Fig3_Driver_{DriverDynamics}_id_{id}.eps', format='eps', dpi=300)
