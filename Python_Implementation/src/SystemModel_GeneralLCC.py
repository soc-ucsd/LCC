import numpy as np

def SystemModel_GeneralLCC(N, M, alpha1, alpha2, alpha3):
    # Generate LTI system model for LCC
    # Input:
    #     N: number of the following HDVs
    #     M: number of the preceding HDVs
    #     alpha1, alpha2, alpha3: linearized coefficients
    # Output:
    #    A, B: system matrices
    
    A1 = np.array([[0, -1], [alpha1, -alpha2]])
    A2 = np.array([[0, 1], [0, alpha3]])

    A = np.zeros((2 * N + 2 * M + 2, 2 * N + 2 * M + 2))
    B = np.zeros((2 * N + 2 * M + 2, 1))

    A[:2, :2] = A1

    for i in range(2, M + N + 2):
        A[2*i-2:2*i, 2*i-2:2*i] = A1
        A[2*i-2:2*i, 2*i-4:2*i-2] = A2

    B[2*M + 1] = 1
    A[2*M + 1, :] = 0

    return A, B

