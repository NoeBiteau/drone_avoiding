import numpy as np
from scipy.linalg import solve_continuous_are

def lqr(A, B, Q, R):
    """
    Solve the continuous-time LQR problem.
    """
    P = solve_continuous_are(A, B, Q, R)
    K = np.linalg.inv(R) @ B.T @ P
    return K
