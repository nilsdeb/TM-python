
#######################################  structure du code  ######################################################################################
#
#
# package
# liste et variable
# class vecteur
# class point
# pas de recurence
# extraction position et vitesse pour les listes pour le graphe
# creation du graphe
# main
#
#
#######################################  a faire  ##############################################################################################
#
# verifier le code
# mettre le premier point imu au referentiel 0,0,0
# de quoi pouvoir lire le dossier de l'arduino et en extraire les données pour l IMU et le gps
# matrice de merde
# essayer d'afficher les vecteur vitesses sur la carte
#enlever le g permannent
# 
# optimisation du code genre limitation de vitesse, de chaangement d'angle
#
#
#################################################################################################################################################








# graphique
import folium
from folium.plugins import MarkerCluster

# pour les maths
import numpy as np      

# temp entre chaque mesure des acceleration utiliser dans step
t = 0.5  

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










# verifier a par matrice
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


#######################################  a changer  ######################################################################################





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
    uni_acc = acc_n.matrice(nt)

    # equation horaire pour obtenir r_n+1 et v_n+1
    new_r = uni_acc*1/2*t*t + point_n.v*t + point_n.r
    new_v = uni_acc*t + point_n.v

    #creation du point_n+1
    return Point(new_r, new_v, new_t)






# point gps et IMU relier au gps pour le graphe
class Pointgps :

    # liste de tout les point cree
    point = []

    # attention, les vecteurs seront surement en 2d
    def __init__(self,vec_r,vec_v,label):
        self.__class__.point.append(self)
        self.r = vec_r
        self.v = vec_v
        self.label = 'gps ' + len(self.__class__.point)

    # print
    def __str__(self):  
        return f"Point = position : {self.r}, vitesse : {self.v}"





# passer de tout les point IMU a des coordonée gps
def IMU_gps ():
    
    #creer la liste de tout les vitesses

    liste_vit = []
    # le point 0,0,0 du referentiel est lier a la premiere coordonnées gps
    ref_lat = Pointgps.point[0].r.x
    ref_long = Pointgps.point[0].r.y

    # Creer un marker cluster et le sauve sur la carte  //  ca permet une meilleur lisibiliter sur la carte
    IMU_cluster = MarkerCluster().add_to(m)


    for point in Point.point :

        # calcule la difference d angle entre le point de ref(0,0,0) et le point donner
        lat_deplacement = point.r.y / 111111

        # le cos et pour corriger les ligne qui se raproche ne fonction de la lat
        lon_deplacement = point.r.x / (111111 * np.cos(np.radians(ref_lat)))

        # additionne la deifference avec le point de reference
        lat = ref_lat + lat_deplacement
        lon = ref_lon + lon_deplacement
        
        # calcule le norme de la vitesse en 2d
        vitesse = np.sqrt(point.v.x**2 + point.v.y**2)

        #ajoute la vitesse a la liste
        liste_vit.append(vitesse)

        # creer les point en rouge pour IMU, son vecteur de vitesse et l ajoute dans un cluster
        folium.PolyLine(locations=[position, (lat + point.v.x, lon + point.v.y)],
                    color='red',
                    weight=vitesse,
                    arrowhead=True,
                    popup=point.label.format(vector)).add_to(marker_cluster)




# creation du graphe
def graphe ():

    # creation de la carte
    m = folium.Map(location=[ref_gps_lat, ref_gps_lon], zoom_start=15)

    # faire la fonction IMU pour avoir les coordonnée gps
    IMU_gps()

    # creer un cluster pour les point gps
    gps_cluster = MarkerCluster().add_to(m)

    # boucle pour chaque point gps
    for point in Pointgps.point :

        # sors les coordonnées
        lat = point.r.y
        lon = point.r.x

        # creer les point en bleu pour le gps et l ajoute dans son cluster
    folium.Marker(location=[lat, lon], popup=point.label, icon=folium.Icon(color='blue')).add_to(gps_cluster)

    # affichage de la carte
    m.save('map.html')




















# fonction pour extraire les coordonnées et les vitesses des points
def list_graphe ():
    
    for point in Point.point:

        # extraction des coordonnées de position
        x_coords.append(point.r.x)
        y_coords.append(point.r.y)
        z_coords.append(point.r.z)
        
        # extraction des vitesses
        vx_velo.append(point.v.x)
        vy_velo.append(point.v.y)
        vz_velo.append(point.v.z)




# creation d un graphique //  utilisation des listes de positions et de vitesses    //  utilisable pour gps et IMU
def graphics (listrx, listry, listrz, listvx, listvy, listvz):

    # calcule la norme de chaque vitesse
    velocities = np.sqrt(np.array(u_velocities)**2 + np.array(v_velocities)**2 + np.array(w_velocities)**2)

    # indique le point avec la plus grande vitesse
    max_velocity_idx = np.argmax(velocities)

    # creation de la figure
    fig = plt.figure()

    # module de la figure
    ax = fig.add_subplot(111, projection='3d')
    
    # enleve axes et le fond
    ax.axis('off')
    
    # cree les points // c='k' fait que les points sont en noir
    ax.scatter(listrx, listry, listrz, c='k', marker='o')

    # cree le point avec la plus grande vitesse en rouge
    ax.scatter(listrx[max_velocity_idx], listry[max_velocity_idx], listrz[max_velocity_idx], c='r', marker='o')

    # cree les vecteurs de vitesse pour chaque points
    ax.quiver(listrx, listry, listrz, listvx, listvy, listvz)

    # affiche le graphe
    plt.show()      

















def main():
    vec0 = Vec(0,0,0)
    point1 = Point(vec0,vec0,vec0)
    print(point1)
    for acc in accs :
        liste_vecacc.append(Vec(acc[0],acc[1],acc[2]))

    print(liste_vecacc)
    





    

if __name__ == '__main__':
    main()
