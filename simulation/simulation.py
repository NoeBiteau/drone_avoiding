import pybullet as p
import pybullet_data
import numpy as np
import time

def setup_simulation():
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.loadURDF("plane.urdf")
    p.setGravity(0, 0, -9.81)

    # Load drone
    drone = p.loadURDF("samurai.urdf", basePosition=[0, 0, 1])

    # Add obstacle
    obstacle = p.loadURDF("cube.urdf", basePosition=[2, 0, 0.5], globalScaling=0.5)

    return drone, obstacle

def apply_wind_force(drone_id):
    wind_force = np.random.normal(0, 0.1, 3)  # Simulates random wind
    p.applyExternalForce(drone_id, -1, wind_force, [0, 0, 0], p.WORLD_FRAME)

if __name__ == "__main__":
    drone, obstacle = setup_simulation()

    for i in range(1000):
        apply_wind_force(drone)
        time.sleep(0.01)
