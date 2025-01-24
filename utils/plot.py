import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def plot_trajectory(trajectory):
    """
    Plot the drone trajectory in 3D.
    :param trajectory: Array of states over time
    """
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    x = trajectory[:, 0]
    y = trajectory[:, 1]
    z = trajectory[:, 2]

    ax.plot(x, y, z, label="Drone Trajectory")
    ax.set_xlabel('X Position')
    ax.set_ylabel('Y Position')
    ax.set_zlabel('Z Position')
    ax.legend()
    plt.show()
