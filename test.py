import numpy as np

# Matrice de rotation 1 (autour de l'axe x)
angle1 = 1  # Angle de rotation en radians
rotationx = np.array([[1, 0, 0],
                      [0, np.cos(angle1), -np.sin(angle1)],
                      [0, np.sin(angle1), np.cos(angle1)]])

# Matrice de rotation 2 (autour de l'axe y)
angle2 = 1  # Angle de rotation en radians
rotationy = np.array([[np.cos(angle2), 0, np.sin(angle2)],
                      [0, 1, 0],
                      [-np.sin(angle2), 0, np.cos(angle2)]])

# Matrice de rotation 3 (autour de l'axe z)
angle3 = 1  # Angle de rotation en radians
rotationz = np.array([[np.cos(angle3), -np.sin(angle3), 0],
                      [np.sin(angle3), np.cos(angle3), 0],
                      [0, 0, 1]])

# Multiplication des matrices de rotation
resultatyxz = np.dot(np.dot(rotationy, rotationx), rotationz)
resultatzxy = np.dot(np.dot(rotationz, rotationx), rotationy)
resultatxyz = np.dot(np.dot(rotationx, rotationy), rotationz)
resultatzyx = np.dot(np.dot(rotationz, rotationy), rotationx)
resultatyzx = np.dot(np.dot(rotationy, rotationz), rotationx)
resultatxzy = np.dot(np.dot(rotationx, rotationz), rotationy)

vecteur = (0,0,1)





print(np.dot(resultatyxz, vecteur))
print( )
print(np.dot(resultatzxy, vecteur))
print( )
print(np.dot(resultatxyz, vecteur))
print( )
print(np.dot(resultatzyx, vecteur))
print( )
print(np.dot(resultatyzx, vecteur))
print( )
print(np.dot(resultatxzy, vecteur))





