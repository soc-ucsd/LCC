import numpy as np
from scipy.linalg import eig
import os
from tqdm import tqdm 
from datetime import date
import datetime
import scipy.io as sio
from SystemModel_GeneralLCC import SystemModel_GeneralLCC

## Description
# Find the plant stable region when monitoring one HDV

## Parameters
# number of the following HDVs
n = 2  
# number of the preceding HDVs
m = 2  

DriverDynamics = 1

if m != 0 and n != 0:
    ID = np.zeros(m + n, dtype=int)
    ID[:m] = np.arange(-m, 0, 1)
    ID[m:] = np.arange(1, n + 1, 1)
elif m == 0:
    ID = np.arange(1, n + 1, 1)
elif n == 0:
    ID = np.arange(-m, 0, 1)

# id of the HDV that has feedback
for id in ID:
    K = np.arange(-10, 10.1, 0.1)
    Mu = np.arange(-10, 10.1, 0.1)
    s_star = 20

    ##### Please Change #####
    if DriverDynamics == 1:
        alpha, beta = 0.6, 0.9
    elif DriverDynamics == 2:
        alpha, beta = 0.4, 0.6

    ##### No change #####
    v_max = 30
    s_st = 5
    s_go = 35
    alpha1 = alpha * v_max / 2 * np.pi / (s_go - s_st) * np.sin(np.pi * (s_star - s_st) / (s_go - s_st))
    alpha2 = alpha + beta
    alpha3 = beta

    A, B = SystemModel_GeneralLCC(n, m, alpha1, alpha2, alpha3)
    PS_bool = np.zeros((len(K), len(Mu)))

    total_tests = len(K) * len(Mu)
    pbar = tqdm(total=total_tests, desc=f'DriverDynamics={DriverDynamics}, ID={id}')

    for ik0 in range(1, len(K)+1):
        for ik1 in range(1, len(Mu)+1):
            k = K[ik0 - 1]
            mu = Mu[ik1 - 1]
            Feedback = np.zeros(2 * (m + n + 1))
            Feedback[2 * m ] = alpha1
            Feedback[2 * m + 1] = -alpha2
            Feedback[2 * m - 1] = alpha3

            Feedback[2 * (m + id)] += k
            Feedback[2 * (m + id) + 1] += mu

            Feedback = Feedback.reshape(1, 10)
            A_closed = A + np.dot(B, Feedback)

            if not np.any(np.real(np.linalg.eigvals(A_closed)) > 0):
                PS_bool[ik0 - 1, ik1 - 1] = 1
            
            pbar.update(1)
    pbar.close()

    home_dir = os.path.expanduser("~")
    data_dir = os.path.join(home_dir, "LCC", "Python_Implementation", "_data")

    #np.save(os.path.join(data_dir, f"{date.today()}_PSRegion_n_{n}_m_{m}_DriverDynamics_{DriverDynamics}_FeedbackID_{id}.npy"), PS_bool)
    filename =  datetime.datetime.now().strftime('%Y%m%d') 
    filename += "_PSRegion_n_{}".format(n)
    filename += "_m_{}".format(m)
    filename += "_DriverDynamics_{}".format(DriverDynamics)
    filename += "_FeedbackID_{}.mat".format(id)
    data_to_save = {
        "id": id,
        "DriverDynamics": DriverDynamics,
        "n": n,
        "m": m,
        "K": K,
        "Mu": Mu,
        "s_star": s_star,
        "alpha": alpha,
        "beta": beta,
        "v_max": v_max,
        "s_st": s_st,
        "s_go": s_go,
        "alpha1": alpha1,
        "alpha2": alpha2,
        "alpha3": alpha3,
        "PS_bool": PS_bool
    }
    full_path = os.path.join(data_dir, filename)
    sio.savemat(full_path, data_to_save)
