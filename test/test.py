# pour les vecteurs et les quaternions
import numpy as np
from numpy import linalg as LA



# pour les maths
import math as m










# Fonction pour multiplier deux quaternions (utiliser dans rotation)
def quaternion_multiply(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    return np.array([w, x, y, z])



# Fonction pour effectuer la rotation d'un vecteur à l'aide de quaternions
def rotationVecteur(vector, vectorAngle):
    """Permet de tourner un vecteur dans l'espace avec les angles x,y,z. 
    
    !les angles doivent etre en radians!
    
    retourne directement le vecteur qui a fait la rotation
    """


    alpha = vectorAngle[0]
    beta = vectorAngle[1]
    gamma = vectorAngle[2]

    #optimisation
    sinAlpha = np.sin(alpha/2)
    cosBeta = np.cos(beta/2)
    cosGamma = np.cos(gamma/2)
    cosAlpha = np.cos(alpha/2)
    sinBeta = np.sin(beta/2)
    sinGamma = np.sin(gamma/2)

    # Calcul des composantes du quaternion
    qx = sinAlpha * cosBeta * cosGamma + cosAlpha * sinBeta * sinGamma
    qy = cosAlpha * sinBeta * cosGamma - sinAlpha * cosBeta * sinGamma
    qz = cosAlpha * cosBeta * sinGamma + sinAlpha * sinBeta * cosGamma
    qw = cosAlpha * cosBeta * cosGamma - sinAlpha * sinBeta * sinGamma
 
    # Normalisation du quaternion
    quaternion = np.array([qw, qx, qy, qz]) / LA.norm([qw, qx, qy, qz])
 
    # Conversion du vecteur en un quaternion
    vector_quaternion = np.append([0], vector)
 
    # Calcul de la conjugaison du quaternion de rotation
    conjugate_quaternion = np.array([quaternion[0], -quaternion[1], -quaternion[2], -quaternion[3]])
 
    # Calcul du quaternion résultant
    rotated_quaternion = quaternion_multiply(quaternion_multiply(quaternion, vector_quaternion), conjugate_quaternion)
 
    # Extraction du vecteur transformé à partir du quaternion
    rotated_vector = rotated_quaternion[1:]
    return rotated_vector
 

vec = (0,4,0)

vec2 = (0,0,0)

vec3 = rotationVecteur(vec,vec2)

print(vec3)
