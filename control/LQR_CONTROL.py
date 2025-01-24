from scipy.linalg import solve_continuous_are
import numpy as np

def lqr(A, B, Q, R):
    """
    Solve the continuous-time Algebraic Riccati Equation for LQR.
    :param A: State transition matrix
    :param B: Control input matrix
    :param Q: State weighting matrix
    :param R: Control weighting matrix
    :return: Gain matrix K
    """
    # Solve the Riccati equation
    P = solve_continuous_are(A, B, Q, R)
    # Compute the LQR gain
    K = np.linalg.inv(R) @ B.T @ P
    return K

# Example usage
if __name__ == "__main__":
    # Define A, B matrices based on a simplified linearized drone model
    A = np.array([
        [0, 1, 0, 0],
        [0, 0, 0, 0],
        [0, 0, 0, 1],
        [0, 0, 0, 0]
    ])
    B = np.array([
        [0, 0],
        [1, 0],
        [0, 0],
        [0, 1]
    ])
    Q = np.diag([10, 1, 10, 1])  # Penalize position and velocity errors
    R = np.diag([1, 1])          # Penalize control effort

    K = lqr(A, B, Q, R)
    print("LQR Gain Matrix K:")
    print(K)
