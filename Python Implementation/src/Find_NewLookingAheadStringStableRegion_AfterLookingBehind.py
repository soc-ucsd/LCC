import numpy as np
from scipy.optimize import fminbound
import os
from datetime import date
import scipy.io as sio
import datetime
from tqdm import tqdm 
# =========================================================================
#               New "Looking Ahead" LCC
#
# Find the new "looking ahead" head-to-tail string stable regions after 
# incorporating the motion of the vehicles behind 
#
# See Section IV.B (Fig. 7(c)-(f)) of the following paper for details
#   Title : Leading Cruise Control in Mixed Traffic Flow:
#                      System Modeling,Controllability,and String Stability
#   Author:Jiawei Wang, Yang Zheng, Chaoyi Chen, Qing Xu and Keqiang Li
# =========================================================================

## Parameters
id_behind = 1
DriverDynamics = 1

# Number of the following HDVs
n = 2
# Number of the preceding HDVs
m = 2

if id_behind == 1:
    mu_behind = -1
    k_behind = -1
elif id_behind == 2:
    mu_behind = -1
    k_behind = -1

save_data_bool = 1  
# 0. Not save the data; 1. Save the data

# ID of the HDV that has feedback
for id_ahead in range(-2, 0):

    K = np.arange(-10, 10.02, 0.02)
    Mu = np.arange(-10, 10.02, 0.02)
    s_star = 20

    ## OVM parameters
    if DriverDynamics == 1:
        alpha = 0.6
        beta = 0.9
    elif DriverDynamics == 2:
        alpha = 0.4
        beta = 0.6

    # Other OVM parameters
    v_max = 30
    s_st = 5
    s_go = 35
    alpha1 = alpha * v_max / 2 * np.pi / (s_go - s_st) * np.sin(np.pi * (s_star - s_st) / (s_go - s_st))
    alpha2 = alpha + beta
    alpha3 = beta

    SS_bool = np.zeros((len(K), len(Mu)))
    ## Find the string stable region
    TestNumber = len(K) * len(Mu)
    pbar = tqdm(total=TestNumber, desc=f'DriverDynamics={DriverDynamics}; ID={id_ahead}')
   
    for ik0 in range(1, len(K)+1):
        for ik1 in range(1, len(Mu)+1):
            k = K[ik0 - 1]
            mu = Mu[ik1 - 1]
            HDV_Tf = lambda w: -abs(((alpha3 * 1j * w + alpha1) / (-w ** 2 + alpha2 * 1j * w + alpha1)) ** n)
            minus_HeadTail_Tf = lambda w:-abs(
                    ((alpha3 * 1j * w + alpha1) / (-w ** 2 + alpha2 * 1j * w + alpha1)) ** (n + m) *
                    (alpha3 * 1j * w + alpha1 +
                     (mu * ((-w ** 2 + alpha2 * 1j * w + alpha1) / (alpha3 * 1j * w + alpha1)) - mu + k * 1j * w) *
                     ((alpha3 * 1j * w + alpha1) / (-w ** 2 + alpha2 * 1j * w + alpha1)) ** (id_ahead + 1)) /
                    (-w ** 2 + alpha1 + 1j * w * alpha2 -
                     (mu_behind * ((-w ** 2 + alpha2 * 1j * w + alpha1) / (alpha3 * 1j * w + alpha1)) - mu_behind + k_behind * 1j * w) *
                     ((alpha3 * 1j * w + alpha1) / (-w ** 2 + alpha2 * 1j * w + alpha1)) ** id_behind)
                )

            x, fval = fminbound(minus_HeadTail_Tf, 1e-8, 100, full_output=True)[0:2]
            fval = -fval
            if fval <= 1:
                SS_bool[ik0-1, ik1-1] = 1
            pbar.update(1)
    pbar.close()

    if save_data_bool:
        home_dir = os.path.expanduser("~")
        data_dir = os.path.join(home_dir, "Desktop", "LCC", "python", "_data")
       
        filename =  datetime.datetime.now().strftime('%Y%m%d') 
        filename += "_SSRegion_WithLookingBehind_id_behind_{}".format(id_behind)
        filename += "_mu_behind_{}".format(mu_behind)
        filename += "_k_behind_{}".format(k_behind)
        filename += "_n_{}".format(n)
        filename += "_m_{}".format(m)
        filename += "_DriverDynamics_{}".format(DriverDynamics)
        filename += "_FeedbackID_{}.mat".format(id_ahead)
        data_to_save = {
            "id_behind": id_behind,
            "DriverDynamics": DriverDynamics,
            "n": n,
            "m": m,
            "mu_behind": mu_behind,
            "k_behind": k_behind,
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
            "SS_bool": SS_bool
        }
        full_path = os.path.join(data_dir, filename)
        sio.savemat(full_path, data_to_save)
