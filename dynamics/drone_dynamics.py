import numpy as np

class DroneDynamics:
    def __init__(self, mass=1.0, inertia=np.diag([0.01, 0.01, 0.02]), gravity=9.81):
        """
        Initialize drone dynamics with configurable mass, inertia, and gravity.
        :param mass: Mass of the drone (kg)
        :param inertia: Moment of inertia matrix (3x3 diagonal for simplicity)
        :param gravity: Gravitational acceleration (m/s^2)
        """
        self.mass = mass
        self.inertia = inertia
        self.gravity = gravity

        # Drone state: position, velocity, orientation, angular velocity
        self.state = {
            "position": np.zeros(3),  # [x, y, z]
            "velocity": np.zeros(3),  # [vx, vy, vz]
            "angles": np.zeros(3),    # [roll, pitch, yaw]
            "angular_velocity": np.zeros(3)  # [wx, wy, wz]
        }

    def update_parameters(self, mass=None, inertia=None, gravity=None):
        """
        Update drone parameters dynamically.
        :param mass: New mass value (kg)
        :param inertia: New inertia matrix (3x3)
        :param gravity: New gravity value (m/s^2)
        """
        if mass is not None:
            self.mass = mass
        if inertia is not None:
            self.inertia = inertia
        if gravity is not None:
            self.gravity = gravity

    def compute_forces(self, thrust, angles):
        """
        Compute forces acting on the drone based on thrust and tilt.
        :param thrust: Total thrust (N)
        :param angles: Orientation angles (roll, pitch, yaw) in radians
        :return: Net force vector (N) in the inertial frame
        """
        roll, pitch, yaw = angles
        R = self.rotation_matrix(roll, pitch, yaw)
        gravity_force = np.array([0, 0, -self.mass * self.gravity])
        thrust_force = np.dot(R, np.array([0, 0, thrust]))
        return gravity_force + thrust_force

    def compute_torques(self, desired_angular_acceleration):
        """
        Compute torques required to achieve a desired angular acceleration.
        :param desired_angular_acceleration: Angular acceleration vector [alpha_x, alpha_y, alpha_z]
        :return: Torque vector (Nm)
        """
        return np.dot(self.inertia, desired_angular_acceleration)

    def rotation_matrix(self, roll, pitch, yaw):
        """
        Compute rotation matrix from roll, pitch, yaw angles.
        :param roll: Roll angle (radians)
        :param pitch: Pitch angle (radians)
        :param yaw: Yaw angle (radians)
        :return: 3x3 rotation matrix
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
