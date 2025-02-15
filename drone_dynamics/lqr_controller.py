from scipy.linalg import solve_continuous_are
from drone_dynamics.dynamics import get_state_space_matrices
import numpy as np

def lqr(A, B, Q, R):
    """
    Solve the continuous-time Algebraic Riccati Equation for LQR.
    :param A: State matrix
    :param B: Input matrix
    :param Q: State weighting matrix
    :param R: Control weighting matrix
    :return: Gain matrix K
    """
    # Check controllability
    controllability_matrix = np.hstack([B, A @ B, A @ A @ B])
    rank = np.linalg.matrix_rank(controllability_matrix)

    if rank < A.shape[0]:
        print("Warning: System is not fully controllable!")
    
    # Solve the Riccati equation
    P = solve_continuous_are(A, B, Q, R)
    K = np.linalg.inv(R) @ B.T @ P
    return K

def tune_lqr_gains(A, B):

    # Define weighting matrices for position and orientation
    Q = np.diag([10, 10, 10, 1, 1, 1, 10, 10, 10, 1, 1, 1])  # State weights
    R = np.diag([1, 1, 1, 1])  # Control effort weights

    # Compute LQR gains
    K = lqr(A, B, Q, R)
    return K
