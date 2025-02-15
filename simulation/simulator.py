import numpy as np

def simulate_drone(A, B, K, initial_state, target_state, dt, steps):
    """
    Simulate drone dynamics with LQR controller.
    :param A: State matrix
    :param B: Input matrix
    :param K: LQR gain matrix
    :param initial_state: Initial state vector
    :param target_state: Desired state vector
    :param dt: Time step
    :param steps: Number of simulation steps
    """
    state = np.array(initial_state)
    trajectory = [state]

    for _ in range(steps):
        # Compute error
        error = state - target_state

        # Compute control input using LQR
        u = -K @ error

        # Update state (discretized dynamics)
        state = state + (A @ state + B @ u) * dt
        trajectory.append(state)

    return np.array(trajectory)
