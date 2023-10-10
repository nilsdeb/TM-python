import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

# Liste des coordonnées x, y et z des points
x_coords = [1, 2, 3, 4, 5]  # Exemple des coordonnées x
y_coords = [2, 3, 4, 5, 6]  # Exemple des coordonnées y
z_coords = [3, 4, 5, 6, 7]  # Exemple des coordonnées z

# Liste des composantes x, y et z des vecteurs de vitesse
u_velocities = [0.5, 1, -0.5, 1.5, -1]  # Exemple des composantes x des vecteurs de vitesse
v_velocities = [1, -0.5, 1.5, -1, 0.5]  # Exemple des composantes y des vecteurs de vitesse
w_velocities = [-1, 0.5, -1.5, 1, -0.5]  # Exemple des composantes z des vecteurs de vitesse

# Calcul des vitesses absolues
velocities = np.sqrt(np.array(u_velocities)**2 + np.array(v_velocities)**2 + np.array(w_velocities)**2)

# Indice du point avec la plus grande vitesse
max_velocity_idx = np.argmax(velocities)

# Création de la figure
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Tracé des points
ax.scatter(x_coords, y_coords, z_coords, c='k', marker='o')  # Points en noir

# Tracé du point avec la plus grande vitesse en rouge
ax.scatter(x_coords[max_velocity_idx], y_coords[max_velocity_idx], z_coords[max_velocity_idx], c='r', marker='o')

# Tracé des vecteurs de vitesse
ax.quiver(x_coords, y_coords, z_coords, u_velocities, v_velocities, w_velocities)

# Suppression du cadre et des axes
ax.axis('off')

# Affichage du graphe
plt.show()
