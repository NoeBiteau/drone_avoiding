from drone_dynamics.dynamics import get_state_space_matrices
from drone_dynamics.lqr_controller import tune_lqr_gains
from simulation.simulator import simulate_drone
from simulation.visualize_3d import plot_trajectory
import numpy as np

def main():
    # Drone parameters
    mass = 1.0
    gravity = 9.81
    inertia = np.diag([0.01, 0.01, 0.02])
    drag_coefficient = 0.1

    # Get state-space matrices
    A, B, C, D = get_state_space_matrices(mass, gravity, inertia)

    # Get LQR gains
    K = tune_lqr_gains(A, B)

    # Initial and target states
    initial_state = np.zeros(12)
    target_state = np.array([1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0])


    # Simulate
    dt = 0.01
    steps = 1000
    trajectory = simulate_drone(A, B, K, initial_state, target_state, dt, steps)

    # Visualize
    plot_trajectory(trajectory)

if __name__ == "__main__":
    main()
