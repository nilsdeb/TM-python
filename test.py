import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def plot_points(x_coords, y_coords, z_coords):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(x_coords, y_coords, z_coords, c='k', marker='o')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    plt.show()


x_coords = [1, 2, 3, 4, 5]
y_coords = [2, 3, 4, 5, 6]
z_coords = [3, 4, 5, 6, 7]

plot_points(x_coords, y_coords, z_coords)
