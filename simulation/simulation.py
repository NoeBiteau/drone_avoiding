import pybullet as p
import time

def run_simulation():
    """
    Basic Pybullet simulation environment for the drone.
    """
    physicsClient = p.connect(p.GUI)
    p.setGravity(0, 0, -9.81)
    # Load a simple plane and drone model (adjust paths to your URDF files)
    planeId = p.loadURDF("plane.urdf")
    droneId = p.loadURDF("drone.urdf", [0, 0, 1])

    for _ in range(1000):
        p.stepSimulation()
        time.sleep(1./240.)
    p.disconnect()
