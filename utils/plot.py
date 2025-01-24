import plotly.graph_objects as go
import numpy as np

def plot_3d_trajectory(trajectory):
    trajectory = np.array(trajectory)
    fig = go.Figure()

    fig.add_trace(go.Scatter3d(
        x=trajectory[:, 0],
        y=trajectory[:, 1],
        z=trajectory[:, 2],
        mode='lines',
        name='Trajectory'
    ))

    fig.update_layout(scene=dict(
        xaxis_title='X',
        yaxis_title='Y',
        zaxis_title='Z'
    ))

    fig.show()

# Example Usage
if __name__ == "__main__":
    # Example trajectory data
    trajectory = np.array([[0, 0, 1], [1, 1, 1.5], [2, 1, 2], [3, 2, 2.5]])
    plot_3d_trajectory(trajectory)

