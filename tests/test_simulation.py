import pytest
import numpy as np
from dynamics.drone_dynamics import DroneDynamics

def test_gravity_force():
    drone = DroneDynamics()
    forces = drone.compute_forces(thrust=0, angles=np.zeros(3))
    assert np.allclose(forces, np.array([0, 0, -9.81 * drone.mass]))
