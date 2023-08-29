#######################################  initialisation  ######################################################################################



# graphique
import folium
from folium.plugins import MarkerCluster

# pour les maths
import numpy as np      


# code vecteurs 3d et ces operations : vecteurs d'angle compris
class Vec :     

    # defni quand on le cree
    def __init__(self,x,y,z):   
        self.x = x
        self.y = y
        self.z = z

    # addition
    def __add__(self,other):
        return Vec(self.x + other.x, self.y + other.y, self.z + other.z)

    # soustraction
    def __sub__(self,other):    
        return Vec(self.x - other.x, self.y - other.y, self.z - other.z)

    # multiplication
    def __mul__(self,x):    
        return Vec(self.x * x, self.y * x, self.z * x)

    # division
    def __truediv__(self,x):    
        return Vec(self.x / x, self.y / x, self.z / x)

    # pour print
    def __str__(self):      
        return f"({self.x},{self.y},{self.z})"

    # pour tester les matrice
    def norme (self) :
        return np.sqrt(self.x**2+self.y**2+self.z**2)




a = Vec(0,3,4)

veca = a.norme()

print(veca)