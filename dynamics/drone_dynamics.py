import numpy as np

class DroneDynamics:
    def __init__(self, mass=1.0, inertia=np.diag([0.01, 0.01, 0.02]), gravity=9.81, drag_coefficient=0.1):
        """
        Initialize drone dynamics with configurable parameters.
        :param mass: Mass of the drone (kg)
        :param inertia: Moment of inertia matrix (3x3 diagonal)
        :param gravity: Gravitational acceleration (m/s^2)
        :param drag_coefficient: Aerodynamic drag coefficient
        """
        self.mass = mass
        self.inertia = inertia
        self.gravity = gravity
        self.drag_coefficient = drag_coefficient

        # Drone state: position, velocity, orientation, angular velocity
        self.state = {
            "position": np.zeros(3),  # [x, y, z]
            "velocity": np.zeros(3),  # [vx, vy, vz]
            "angles": np.zeros(3),    # [roll, pitch, yaw]
            "angular_velocity": np.zeros(3)  # [wx, wy, wz]
        }

    def update_parameters(self, mass=None, inertia=None, gravity=None, drag_coefficient=None):
        """
        Update drone parameters dynamically.
        """
        if mass is not None:
            self.mass = mass
        if inertia is not None:
            self.inertia = inertia
        if gravity is not None:
            self.gravity = gravity
        if drag_coefficient is not None:
            self.drag_coefficient = drag_coefficient

    def compute_forces(self, thrust, angles):
        """
        Compute forces acting on the drone.
        """
        roll, pitch, yaw = angles
        R = self.rotation_matrix(roll, pitch, yaw)
        gravity_force = np.array([0, 0, -self.mass * self.gravity])
        thrust_force = np.dot(R, np.array([0, 0, thrust]))
        drag_force = -self.drag_coefficient * self.state["velocity"]
        return gravity_force + thrust_force + drag_force

    def compute_torques(self, desired_angular_acceleration):
        """
        Compute torques required for angular acceleration.
        """
        return np.dot(self.inertia, desired_angular_acceleration)

    def update_state(self, forces, torques, dt):
        """
        Update the drone's state using forces and torques over time step dt.
        """
        # Linear motion
        acceleration = forces / self.mass
        self.state["velocity"] += acceleration * dt
        self.state["position"] += self.state["velocity"] * dt

        # Angular motion
        angular_acceleration = np.linalg.inv(self.inertia).dot(torques)
        self.state["angular_velocity"] += angular_acceleration * dt
        self.state["angles"] += self.state["angular_velocity"] * dt

    def rotation_matrix(self, roll, pitch, yaw):
        """
        Compute rotation matrix from roll, pitch, yaw angles.
        """
        Rx = np.array([
            [1, 0, 0],
            [0, np.cos(roll), -np.sin(roll)],
            [0, np.sin(roll), np.cos(roll)]
        ])
        Ry = np.array([
            [np.cos(pitch), 0, np.sin(pitch)],
            [0, 1, 0],
            [-np.sin(pitch), 0, np.cos(pitch)]
        ])
        Rz = np.array([
            [np.cos(yaw), -np.sin(yaw), 0],
            [np.sin(yaw), np.cos(yaw), 0],
            [0, 0, 1]
        ])
        return Rz @ Ry @ Rx
