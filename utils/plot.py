import matplotlib.pyplot as plt

def plot_response(time, response, title):
    """
    Plot the time response of the system.
    """
    plt.figure()
    plt.plot(time, response)
    plt.title(title)
    plt.xlabel('Time (s)')
    plt.ylabel('Response')
    plt.grid()
    plt.show()
