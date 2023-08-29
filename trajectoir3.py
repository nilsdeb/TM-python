#######################################  librairie  ######################################################################################



# graphique
import folium
from folium.plugins import MarkerCluster

# pour les maths
import numpy as np
from numpy import linalg as LA


################################### Fonction pour effectuer la rotation d'un vecteur à l'aide de quaternions  #############################
#https://pastebin.com/9aVXyUK8



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
def rotationVecteur(vectorAngle, vector):
    """les angles doivent etre en radians"""


    alpha = vectorAngle[0]
    beta = vectorAngle[1]
    gamma = vectorAngle[2]


    # Calcul des composantes du quaternion
    qx = np.sin(alpha/2) * np.cos(beta/2) * np.cos(gamma/2) + np.cos(alpha/2) * np.sin(beta/2) * np.sin(gamma/2)
    qy = np.cos(alpha/2) * np.sin(beta/2) * np.cos(gamma/2) - np.sin(alpha/2) * np.cos(beta/2) * np.sin(gamma/2)
    qz = np.cos(alpha/2) * np.cos(beta/2) * np.sin(gamma/2) + np.sin(alpha/2) * np.sin(beta/2) * np.cos(gamma/2)
    qw = np.cos(alpha/2) * np.cos(beta/2) * np.cos(gamma/2) - np.sin(alpha/2) * np.sin(beta/2) * np.sin(gamma/2)
 
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
 


#######################################  class point  ######################################################################################

# c est plus simple de cree deux class pour ne pas faire de confusion

#class des point IMU
class PointIMU :
    # liste de tout les point cree
    point = [] 
    
    def __init__(self,vec_r,vec_v,vec_t):
        self.__class__.point.append(self)
        self.r = vec_r
        self.v = vec_v
        self.t = vec_t
        self.label = 'IMU ' + str(len(self.__class__.point))

    #print
    def __str__(self):  
        return f"Point {self.label} = position : {self.r}, vitesse : {self.v}, pos angulaire : {self.t}"



# class des points GPS

class PointGPS :

    # liste de tout les point cree
    point = []

    # attention, les vecteurs seront surement en 2d
    def __init__(self,vec_r,vec_v):
        self.__class__.point.append(self)
        self.r = vec_r
        self.v = vec_v
        self.label = 'gps ' + str(len(self.__class__.point))

    # print
    def __str__(self):  
        return f"Point = position : {self.r}, vitesse : {self.v}"









def angleG(vec):
    """creer le referentiel unique"""

    theta_x = np.arccos(vec[0] / np.linalg.norm(vec))
    theta_y = np.arccos(vec[1] / np.linalg.norm(vec))
    theta_z = np.arccos(vec[2] / np.linalg.norm(vec))   
    
    

    angle =  np.array([-(theta_x-np.pi/2), -theta_y, -(theta_z-np.pi/2)])
    return angle







vec = np.array([2,6,7])
vec2 = angleG(vec)
print(vec)
print(vec2)

vec3 = rotationVecteur(vec2,vec)

print(vec3)