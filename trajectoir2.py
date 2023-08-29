
#######################################  structure du code  ######################################################################################
#
#
# initialisation :
# package
# liste et variable
# en ce moment donnée
# class vecteur
#
# analyse des donnees de l IMU :
# class pointIMU
# fonction de la creation du referentiel unique
# pas de recurence
#
# analyse des donnees du gps :
# class gps
#
# creation du graphe :
# 
# fonction graphe
    # fonction interne passe des point IMU a gps
#
# main :
# main
#
#
#######################################  a faire  ##############################################################################################
#
#
# mettre le premier point imu au referentiel 0,0,0       matrice
# de quoi pouvoir lire le dossier de l'arduino et en extraire les données pour l IMU et le gps    
# matrice de merde       matrice
# enlever le g permannent       matrice
# comment savoir la direction sur la carte des points IMU?
# 
# optimisation du code genre limitation de vitesse, de chaangement d'angle
#
#
#################################################################################################################################################


#######################################  initialisation  ######################################################################################



# graphique
import folium
from folium.plugins import MarkerCluster

# pour les maths
import numpy as np      

# temp entre chaque mesure des acceleration utiliser dans step
t = 5

# Accélérations
accs = [
    [0, 0, 0],
    [1.5, 0.2, -0.5],
    [0.8, -0.3, 0.9],
    [-0.7, 0.5, -1.2],
    [0.3, -0.6, 1.4],
    [-1.2, 0.7, -0.9],
    [0.5, -0.4, 0.3],
    [-0.6, 0.3, -0.8],
    [0.9, -1.2, 0.5],
    [-0.4, 0.5, -0.3],
    [0.3, -0.2, 0.1],
    [0.5, -0.6, 0.9],
    [-0.8, 1.0, -0.7],
    [0.6, -0.4, 0.5],
    [-0.3, 0.1, -0.2],
    [0.7, -0.5, 0.6],
    [-0.4, 0.3, -0.5],
    [0.2, -0.1, 0.3],
    [-0.5, 0.4, -0.2],
    [0.3, -0.3, 0.4],
    [-0.1, 0.2, -0.3]
]

liste_vecacc = []



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
        return  np.sqrt(self.x*self.x+self.y*self.y+self.z*self.z)

#######################################  a changer  ######################################################################################



    def matrice (self, a) :     # a = vecteur angle  multiplication d'un vecteur par une matrice de rotation (les angles sont donner par un angle de rotation) le plus simple est de faire matrice axe x ensuite matrice axe y ... 
        x1 = self.x     # matrice x
        y1 = self.y * np.cos(-a.x) - self.z * np.sin(-a.x)     # matrice x        - devant tout les angle parce que les matrices sont R(-alpha)+...
        z1 = self.y * np.sin(-a.x) + self.z * np.cos(-a.x)     # matrice x
        x2 = x1 * np.cos(-a.y) + z1 * np.sin(-a.y)     # matrice y
        y2 = y1     # matrice y
        z2 = - x1 * np.sin(-a.y) + z1 * np.cos(-a.y)     # matrice y
        x3 = x2 * np.cos(-a.z) - y2 * np.sin(-a.z)     # matrice z
        y3 = x2 * np.sin(-a.z) + y2 * np.cos(-a.z)     # matrice z
        z3 = z2     # matrice z
        return Vec(x3, y3, z3)












#######################################  traitement des donnees de l IMU  ######################################################################################


# point de chaque prise de données
class Point :

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



def ref_unique ():
    pass



# pas de recurence  //  acc pour acceleration 
def step (point_n, acc_n, omega_n):     

    # equation angulaire pour obtenir theta_n+1   //  new_t = thetan+1
    new_t = omega_n*t + point_n.t

    # passage de l'acceleration de l'IMU dans le referentiel unique
    uni_acc = acc_n  #.matrice(nt)

    # equation horaire pour obtenir r_n+1 et v_n+1
    new_r = uni_acc*1/2*t*t + point_n.v*t + point_n.r
    new_v = uni_acc*t + point_n.v

    #creation du point_n+1
    return Point(new_r, new_v, new_t)














#######################################  traitement des donnees du gps  ######################################################################################



# point gps et IMU relier au gps pour le graphe
class Pointgps :

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












#######################################  creation de la carte  ######################################################################################


# creation du graphe
def graphe ():

    # le point 0,0,0 du referentiel est lier a la premiere coordonnées gps
    ref_lat = Pointgps.point[0].r.x
    ref_lon = Pointgps.point[0].r.y

    # creation de la carte
    m = folium.Map(location=[ref_lat, ref_lon], zoom_start=15)

    # passer de tout les point IMU a des coordonée gps, fonction interrieur
    def IMU_gps ():

        for point in Point.point :

            # calcule la difference d angle entre le point de ref(0,0,0) et le point donner  /111111 parce que c est 1 metre en degrer pour le gps
            lat_deplacement = point.r.y / 111111

            # le cos et pour corriger les ligne qui se raproche ne fonction de la lat geometrie de la terre...
            lon_deplacement = point.r.x / (111111 * np.cos(np.radians(ref_lat)))

            # additionne la deifference avec le point de reference
            lat = ref_lat + lat_deplacement
            lon = ref_lon + lon_deplacement

            # creer les point en rouge pour l IMU et l ajoute dans son cluster
            folium.Marker(location=[lat, lon], popup=point.label, icon=folium.Icon(color='red')).add_to(m)

    # faire la fonction IMU pour avoir les coordonnée gps
    IMU_gps()

    # boucle pour chaque point gps
    for point in Pointgps.point :

        # sors les coordonnées
        lat = point.r.x
        lon = point.r.y

        # creer les point en bleu pour le gps et l ajoute dans son cluster
        folium.Marker(location=[lat, lon], popup=point.label, icon=folium.Icon(color='blue')).add_to(m)

    # affichage de la carte
    m.save('map4.html')




#######################################  main  ######################################################################################

def main():

    #creer un vecteur 0 pour qq truc
    vec0 = Vec(0,0,0)

    #avant de creer la def, creer le referentiel unique
    point1 = Point(vec0,vec0,vec0)

    #transforme tout la liste d'acceleration en vecteur
    for acc in accs :
        liste_vecacc.append(Vec(acc[0],acc[1],acc[2]))

    #creer tout les points
    for vecacc in liste_vecacc :
        step(Point.point[-1], vecacc, vec0)

    #creer coordonnée gps
    gps0 = Vec(46.2319,6.8524,0)

    # creation du point gps 0
    pointgps0 = Pointgps(gps0,vec0)

    #creer le graphe gps
    graphe()




    

    

    

    

    





    

if __name__ == '__main__':
    main()
