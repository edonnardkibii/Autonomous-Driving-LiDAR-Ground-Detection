from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt

def plot_graph(x_axis, y_axis, z_axis):
    fig = plt.figure()

    # syntax for 3D projection
    ax = plt.axes(projection = '3d')

    ax.scatter(x_axis, y_axis, z_axis)

    ax.set_xlabel('x axis', labelpad=20)
    ax.set_ylabel('y axis', labelpad=20)
    ax.set_zlabel('z axis', labelpad=20)

    # ax.axes.set_xlim3d(left=-250, right=250)
    # ax.axes.set_ylim3d(bottom=50, top=300)
    # ax.axes.set_zlim3d(bottom=-200, top=20)

    # plot
    plt.show()