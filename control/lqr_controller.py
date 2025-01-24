from scipy.linalg import solve_continuous_are
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
    P = solve_continuous_are(A, B, Q, R)
    K = np.linalg.inv(R) @ B.T @ P
    return K

def tune_lqr_gains(mass, gravity, inertia, drag_coefficient):
    """
    Compute LQR gains for the drone's linearized dynamics.
    :param mass: Mass of the drone
    :param gravity: Gravitational acceleration
    :param inertia: Moment of inertia matrix
    :param drag_coefficient: Drag coefficient
    :return: LQR gain matrix K
    """
    from drone_dynamics.dynamics import get_state_space_matrices

    A, B, _, _ = get_state_space_matrices(mass, gravity, inertia, drag_coefficient)

    # Define weighting matrices
    Q = np.diag([10, 10, 10, 1, 1, 1, 10, 10, 10, 1, 1, 1])  # Penalize states
    R = np.diag([1, 1, 1, 1])  # Penalize control effort

    K = lqr(A, B, Q, R)
    return K

