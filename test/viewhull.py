import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

# Load the CSV file
points = np.loadtxt("points.csv", delimiter=",")
hull_indices = np.loadtxt("hull.csv", delimiter=",")

# Extract x and y columns
nb_points, nb_dims = np.shape(points)

has_mesh = len(hull_indices) > 0

if has_mesh:
    nb_faces, nb_dims = np.shape(hull_indices)
    hull_vertices = [[points[int(hull_indices[i][j])] for j in range(0, nb_dims)] for i in range(0, nb_faces)]

# Plot the points
step_size = 1
if nb_dims == 3:
    x, y, z = points[:, 0], points[:, 1], points[:, 2]
    plot = plt.figure().add_subplot(projection='3d')
    plot.scatter(x[::step_size], y[::step_size], z[::step_size])

    if has_mesh:
        poly3d = Poly3DCollection(hull_vertices, alpha=0.5, edgecolor='k')
        plot.add_collection(poly3d)

else:
    x, y = points[::step_size, 0], points[::step_size, 1]
    plot = plt
    plot.scatter(x, y)
    if has_mesh:
        for segment in hull_vertices:
            (x0, y0), (x1, y1) = segment
            plot.plot([x0, x1], [y0, y1])



plt.show()