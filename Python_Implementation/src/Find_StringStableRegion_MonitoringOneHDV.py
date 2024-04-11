import numpy as np
from scipy.optimize import minimize_scalar
from scipy.optimize import fminbound
import os
import scipy.io as sio
from tqdm import tqdm 

# Parameters
n = 2  # number of the following HDVs
m = 2  # number of the preceding HDVs
DriverDynamics = 1

if m != 0 and n != 0:
    ID = np.zeros(m + n, dtype=int)
    ID[:m] = np.arange(-m, 0, 1)
    ID[m:] = np.arange(1, n + 1, 1)
    print (ID)
    #ID = list(range(-m, 0)) + list(range(1, n+1))
elif m == 0:
    ID = np.arange(1, n + 1, 1)
    #ID = list(range(1, n+1))
elif n == 0:
    ID = np.arange(-m, 0, 1)
    #ID = list(range(-m, 0))

# Iterate over HDVs with feedback
for id in ID:
    K = np.arange(-10, 10.02, 0.02)
    Mu = np.arange(-10, 10.02, 0.02)
    s_star = 20

    # OVM parameters
    if DriverDynamics == 1:
        alpha, beta = 0.6, 0.9
    elif DriverDynamics == 2:
        alpha, beta = 0.4, 0.6

    v_max = 30
    s_st = 5
    s_go = 35
    alpha1 = alpha * v_max / 2 * np.pi / (s_go - s_st) * np.sin(np.pi * (s_star - s_st) / (s_go - s_st))
    alpha2 = alpha + beta
    alpha3 = beta

    SS_bool = np.zeros((len(K), len(Mu)))
    TestNumber = len(K) * len(Mu)
    pbar = tqdm(total=TestNumber, desc=f'DriverDynamics={DriverDynamics}; ID={id}')
    # Progress bar with tqdm
    for ik0 in range(0, len(K)):
        for ik1 in range(0, len(Mu)):
            k = K[ik0 ]
            mu = Mu[ik1 ]
            def HDV_Tf(w):
                return (-np.abs(((alpha3*1j*w+alpha1)/(-w**2+alpha2*1j*w+alpha1))**n))

            if id < 0:
                minus_HeadTail_Tf = lambda w: (-abs(((alpha3*1j*w+alpha1)/(-w**2+alpha2*1j*w+alpha1))**(n+m))
                                   * abs((alpha3*1j*w+alpha1 + (mu * ((-w**2+alpha2*1j*w+alpha1)/(alpha3*1j*w+alpha1)) - mu + k*1j*w)
                                      * ((alpha3*1j*w+alpha1)/(-w**2+alpha2*1j*w+alpha1))**(id+1))
                                      / (-w**2 + alpha1 + 1j*w*alpha2)))
                '''''
                def minus_HeadTail_Tf(w):
                    return -np.abs(((alpha3*1j*w+alpha1)/(-w**2+alpha2*1j*w+alpha1))**(n+m)
                                   * ((alpha3*1j*w+alpha1 + (mu * ((-w**2+alpha2*1j*w+alpha1)/(alpha3*1j*w+alpha1)) - mu + k*1j*w)
                                      * ((alpha3*1j*w+alpha1)/(-w**2+alpha2*1j*w+alpha1))**(id+1))
                                      / (-w**2 + alpha1 + 1j*w*alpha2)))
                '''''
            else:
                minus_HeadTail_Tf = lambda w: (-abs(((alpha3*1j*w+alpha1)/(-w**2+alpha2*1j*w+alpha1))**(n+m))
                                   * abs((alpha3*1j*w+alpha1)/(-w**2 + alpha1 + 1j*w*alpha2 
                                                          - (mu * ((-w**2+alpha2*1j*w+alpha1) / (alpha3*1j*w+alpha1)) - mu + k*1j*w) 
                                                          * ((alpha3*1j*w+alpha1)/(-w**2+alpha2*1j*w+alpha1))**id)))
                '''''
                def minus_HeadTail_Tf(w):
                    return -np.abs(((alpha3*1j*w+alpha1)/(-w**2+alpha2*1j*w+alpha1))**(n+m)
                                   * (alpha3*1j*w+alpha1)/(-w**2 + alpha1 + 1j*w*alpha2 
                                                          - (mu * ((-w**2+alpha2*1j*w+alpha1) / (alpha3*1j*w+alpha1)) - mu + k*1j*w) 
                                                          * ((alpha3*1j*w+alpha1)/(-w**2+alpha2*1j*w+alpha1))**id))
                '''''

            # Minimize the function
            x, fval = fminbound(minus_HeadTail_Tf, 1e-8, 100, full_output=True)[0:2]

            
            fval = -fval

            if id == -1 and ik0 == 417 and ik1 == 594:
                print(f"fval: {fval}")
            if id == 1 and ik0 == 425 and ik1 == 848:
                print(f"fval: {fval}")
            if id == 1 and ik0 == 649 and ik1 == 604:
                print(f"fval: {fval}")
                    
            if fval <= 1.00000000000001:
                SS_bool[ik0 , ik1] = 1
            pbar.update(1)
    pbar.close()

    # Save the result as .npy file
    home_dir = os.path.expanduser("~")
    data_dir = os.path.join(home_dir, "LCC", "Python_Implementation", "_data")
    filename = "SSRegion_n_{}".format(n)
    filename += "_m_{}".format(m)
    filename += "_DriverDynamics_{}".format(DriverDynamics)
    filename += "_FeedbackID_{}.mat".format(id)
       
    data_to_save = {
            "ID": ID,
            "id": id,
            "TestNumber": TestNumber,
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
            "ik0" : ik0,
            "SS_bool": SS_bool
        }
    full_path = os.path.join(data_dir, filename)
    sio.savemat(full_path, data_to_save)
    #np.save(os.path.join(data_dir, f"SSRegion_n_{n}_m_{m}_DriverDynamics_{DriverDynamics}_FeedbackID_{id}.npy"), SS_bool)
