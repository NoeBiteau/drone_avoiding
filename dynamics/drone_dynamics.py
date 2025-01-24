import numpy as np
from scipy.linalg import block_diag

def get_state_space_matrices(mass, gravity, inertia, drag_coefficient):
    """
    Compute state-space matrices for the drone linearized around hover condition.
    :param mass: Mass of the drone
    :param gravity: Gravitational acceleration
    :param inertia: Moment of inertia matrix (3x3 diagonal)
    :param drag_coefficient: Aerodynamic drag coefficient
    :return: A, B, C, D matrices
    """
    # Linear motion dynamics
    A_linear = np.zeros((6, 6))
    A_linear[:3, 3:] = np.eye(3)  # Position-velocity coupling
    A_linear[3:, 3:] = -drag_coefficient / mass * np.eye(3)  # Drag effect

    B_linear = np.zeros((6, 4))
    B_linear[3:, 0] = np.array([0, 0, 1]) / mass  # Thrust affects z_dot

    # Rotational motion dynamics
    A_rotational = np.zeros((6, 6))
    A_rotational[:3, 3:] = np.eye(3)  # Angular velocity coupling
    B_rotational = np.zeros((6, 4))
    B_rotational[3:, 1:] = np.linalg.inv(inertia)  # Torques affect angular velocities

    # Combine linear and rotational dynamics
    A = block_diag(A_linear, A_rotational)
    B = np.vstack((B_linear, B_rotational))

    # Output matrix (position and orientation)
    C = np.eye(12)  # All states as outputs
    D = np.zeros((12, 4))  # No direct feedthrough

    return A, B, C, D
