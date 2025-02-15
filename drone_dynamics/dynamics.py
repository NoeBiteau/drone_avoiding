import numpy as np
from scipy.linalg import block_diag

def get_state_space_matrices(mass, gravity, inertia):
    """
    Compute enhanced state-space matrices for the drone linearized around hover condition.
    :param mass: Mass of the drone
    :param gravity: Gravitational acceleration
    :param inertia: Moment of inertia matrix (diagonal for simplicity)
    :return: A, B, C, D matrices suitable for LQR control
    """
    # State vector: [x, y, z, x_dot, y_dot, z_dot, phi, theta, psi, p, q, r]
    # Control vector: [thrust, tau_phi, tau_theta, tau_psi]

    g = gravity
    Ix, Iy, Iz = np.diag(inertia)

    # Define the state matrix A
    A = np.zeros((12, 12))
    A[0:3, 3:6] = np.eye(3)  # Position-velocity coupling
    A[3, 7] = -g  # Theta affects X acceleration
    A[4, 6] = g   # Phi affects Y acceleration

    # Angular velocity coupling for rotational dynamics
    A[6:9, 9:12] = np.eye(3)

    # Define the input matrix B
    B = np.zeros((12, 4))
    B[5, 0] = 1 / mass  # Thrust affects z_dot
    B[9, 1] = 1 / Ix    # Torque affects roll rate
    B[10, 2] = 1 / Iy   # Torque affects pitch rate
    B[11, 3] = 1 / Iz   # Torque affects yaw rate

    # Output matrix (all states as outputs)
    C = np.eye(12)
    D = np.zeros((12, 4))

    return A, B, C, D
