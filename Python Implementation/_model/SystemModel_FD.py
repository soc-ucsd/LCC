import numpy as np
# Linear Free Driving LCC model
# Input
#    N: the number of vehicles in FD-LCC
#    alpha1, alpha2, alpha3: parameters from the linearized car-following model
# Output
#    state-space model: [A, B]
#                       \dot{x} = Ax + Bu
#
#   See Section II of the following paper for modeling details
#   Title : Leading Cruise Control in Mixed Traffic Flow:
#                      System Modeling,Controllability,and String Stability
#   Author:Jiawei Wang, Yang Zheng, Chaoyi Chen, Qing Xu and Keqiang Li
#   Python Version implemented by Haoxin Du, Meihui Liu, Yue Yin

def SystemModel_FD(N, alpha1, alpha2, alpha3):
    A = np.zeros((2*N+2, 2*N+2))
    B = np.zeros((2*N+2, 1))

    A[:2, :2] = [[0, 1], [0, 0]]
    A1 = np.array([[0, -1], [alpha1, -alpha2]])
    A2 = np.array([[0, 1], [0, alpha3]])
    for i in range(2, N+2):
        A[2*i-2:2*i, 2*i-2:2*i] = A1 
        A[2*i-2:2*i, 2*i-4:2*i-2] = A2 
    B[1, 0] = 1
    return A, B
