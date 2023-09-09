#######################################  a faire  ######################################################################################
#
#
#
# verifier chaque une des fonction dans avec toutes les possibilité et tout expliquer
#
#
#lire fichier
#quaternion / quelle angle vraiment est a utiliser dans Point imu. theta?
#
#
# attention, quelle vecteur = a quoi, genre surtout axe pour les angle x y etc... vas faloir tester et etre au claire
#
#
#######################################  librairie  ######################################################################################





# graphique
import folium
from folium.plugins import MarkerCluster



# pour les vecteurs et les quaternions
import numpy as np
from numpy import linalg as LA



# pour les maths
import math as m





#######################################  liste et variable  ######################################################################################





#constente du temps entre deux mesures
temps = 1

#0.15, -0.25, 0.85

#liste donne, a organiser de cette magniere: [[t,accx....][t2,acc....]]
#ordre de la liste dans la liste[accx,accy,accz,gyrox,gir0y,giroz,lat,long]
donne = [
    [1,0,0, 0.213, -0.076, 0.122, 46.5223, 6.6332],
    [-0.2, 0.6, -0.4, 0.065, 0.158, -0.128, 46.5193, 6.6339],
    [0.35, 0.3, 0.05, -0.092, 0.126, -0.057, 46.5268, 6.6148],
    [-0.08, -0.18, 0.13, 0.142, -0.109, 0.084, 46.5239, 6.6257],
    [0.12, -0.08, -0.02, 0.077, -0.061, -0.045, 46.5251, 6.6263],
    [0.18, -0.06, 0.32, -0.115, 0.124, 0.176, 46.5241, 6.6267],
    [-0.28, 0.17, -0.07, 0.032, -0.067, -0.146, 46.5249, 6.6249],
    [0.03, 0.12, -0.22, 0.128, -0.149, 0.055, 46.5215, 6.6319],
    [-0.22, -0.35, 0.25, -0.149, 0.078, 0.115, 46.5261, 6.6354],
    [0.05, 0.18, -0.28, 0.095, -0.167, 0.022, 46.5229, 6.6317],
    [-0.16, -0.27, 0.23, 0.073, 0.101, 0.032, 46.5234, 6.6351],
    [0.23, -0.08, 0.08, -0.065, -0.032, 0.128, 46.5235, 6.6312],
    [-0.15, 0.62, -0.37, 0.098, 0.159, -0.121, 46.5275, 6.6257],
    [0.28, 0.23, 0.07, -0.099, 0.142, -0.045, 46.5232, 6.6272],
    [-0.07, -0.19, 0.14, 0.118, -0.093, 0.096, 46.5238, 6.6251],
    [0.11, -0.12, -0.01, 0.051, -0.038, -0.022, 46.5248, 6.6255],
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 46.5240, 6.6248],
    [0.2, -0.07, 0.35, -0.125, 0.138, 0.189, 46.5236, 6.6265],
    [-0.32, 0.16, -0.09, 0.028, -0.071, -0.155, 46.5247, 6.6250],
    [0.06, 0.11, -0.24, 0.115, -0.163, 0.033, 46.5212, 6.6321],
    [-0.18, -0.29, 0.21, -0.151, 0.085, 0.122, 46.5260, 6.6356],
    [0.08, 0.14, -0.26, 0.085, -0.172, 0.011, 46.5228, 6.6315],
    [-0.12, -0.21, 0.19, 0.095, 0.075, 0.041, 46.5233, 6.6352],
    [0.26, -0.07, 0.13, -0.055, -0.017, 0.168, 46.5234, 6.6310],
    [-0.14, 0.64, -0.42, 0.082, 0.163, -0.136, 46.5274, 6.6258],
    [0.31, 0.28, 0.03, -0.108, 0.133, -0.061, 46.5231, 6.6275],
    [-0.09, -0.20, 0.16, 0.134, -0.100, 0.074, 46.5237, 6.6252],
    [0.13, -0.10, -0.03, 0.066, -0.028, -0.010, 46.5247, 6.6256],
    [0.17, -0.05, 0.37, -0.109, 0.131, 0.198, 46.5237, 6.6266],
    [-0.30, 0.13, -0.12, 0.012, -0.073, -0.165, 46.5246, 6.6251],
    [0.04, 0.10, -0.27, 0.102, -0.166, 0.044, 46.5213, 6.6322],
    [-0.20, -0.33, 0.28, -0.143, 0.091, 0.130, 46.5259, 6.6353],
    [0.07, 0.16, -0.31, 0.075, -0.179, 0.000, 46.5227, 6.6313],
]



#######################################  class point  ######################################################################################





#class verifiée
#class des points IMU
class PointIMU :

    # liste de tout les point cree, cela permet de les appeler ou  de les compters
    point = [] 
    
    # Les point IMU représente l'etat dans lequel est le capteur au moment de prendre cette mesure. Pour pouvoir appliquer les equations horaire a partire de ce moment précis, il faut stocker dans chaqu'un des point, la position, la vitesse, et l'angle dans lequel se trouve le capteur au moment de la prise de la mesure.
    def __init__(self,vec_r,vec_v,vec_t):
        self.__class__.point.append(self)
        self.r = vec_r  #position
        self.v = vec_v  #vitesse
        self.t = vec_t  #position angulaire
        self.label = 'IMU ' + str(len(self.__class__.point))

    #print
    def __str__(self):  
        return f"Point {self.label} = position : {self.r}, vitesse : {self.v}, pos angulaire : {self.t}"



#class verifiée
# class des points GPS
class PointGPS :

    # liste de tout les point cree
    point = []

    # Pour cette class, stocker les donnees de lattitude et de longitude suffise. Il serait possible de ne pas creer cette class et d'utiliser les donnée brut, mais cela compliquerai grandement la compréension du code plus tard. Comme la mesures des point IMU et des pointGps se fait en meme temp, il est plus simple de creer un point IMU et gps pour chaque point de mesure.  !!!!text a changer!!!!
    def __init__(self,vec_r):
        self.__class__.point.append(self)
        self.r = vec_r  #position
        self.label = 'gps ' + str(len(self.__class__.point))

    # print
    def __str__(self):  
        return f"Point {self.label} = position : {self.r}"





################################### Fonction pour effectuer la rotation d'un vecteur à l'aide de quaternions  #############################
# code pris de https://pastebin.com/9aVXyUK8 puis adapté


#ce que j ai comme angle c est pas pile les angles, c est la difference d angle dans chaque plan entre les deux vecteur. Est ce que que on ne dois pas repasser par le cercle trigonometrique est reposer des condition, si angle entre 0 et 90 alors le vecteur sera dans ce secteur, si blabla? je dis pas les quaternions sont inutile car surement a la fin pour effectuer la rotation il y en auras peut etre besoin mais il faut savoir avec quel angle et pk

# en attendant d avoir suffisament reflechi au probleme, je dis que cette rotation et fonctionnel et corrige le reste du code en ignorant celui ci. Ce n est que cette fonction a modifier mais le reste du cheminnement et du code reste juste meme avec cette rotation fausse.


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
 




#######################################  fonction calule d'angle entre vecteurs  ######################################################################################





# fonction verifiée
def anglesysteme(vecteurX,vecteurY):
    """calule l'angle par rapport a l'axe des abcisses du plan

    !besoin des composant du vectuer en 2D!"""


    #calcule la norme du vecteur (vecteurX, vecteurY)
    norme = m.sqrt(vecteurX**2+vecteurY**2)


    #si la norme du vecteur = 0, il doit / 0 ce qui est impossible. On peut physiquement dire que si la norme du vecteur est nul, alors l'angle entre un point et l'axe des absices est nul. La condition est donc pausée pour eviter ce problèmes

    # condition que la norme soie differente que 0
    if norme != 0:

        #calcule sin et cos du vecteur avec l'axe des abcisses
        sin = vecteurY/norme
        cos = vecteurX/norme


        #comme on cherche un angle sur 2 pi et non que sur pi/2, il faut donc ajouter l'information du sin et passé par le cercle trigonometrique pour savoir dans quelle cardan et le vecteur. Avec cette information en +, on peut déduire l'angle sur 2 pi
        
        #condition pour que l'angle soie dans le cadran 1 ou 2
        if sin >= 0 and cos > 0 or sin > 0 and cos < 0:      

            angle = m.acos(cos)

        #il considere sin90 = -0, donc si mon vecteur est pile a l angle droit, il me mets 270 au lieu de 90. Je rajoute cette condition pour le cas précis ou l'angle est a 90 degrer par rapport au l'axe des absisse
        if sin == 1 and cos == 0 : 

            angle = m.pi/2

        #condition pour le cadran 3 ou 4
        else :
            angle = m.pi*2-m.acos(cos)
    
    # suite de la condition si norme = 0
    else :

        angle = 0

    # return angle entre le vecteur et l'axe des abcisses
    return angle
    


# fonction verifiée
def diffAnglePlan (vecteur1,vecteur2):
    """angle pour passer du vecteur 1 au vecteur 2 dans un plan"""


    #calcule l'angle entre les vecteurs et l'axde des abcisses du plan
    angle1 = anglesysteme(vecteur1[0],vecteur1[1])
    angle2 = anglesysteme(vecteur2[0],vecteur2[1])

    #Comme on veut l'angle pour passer du vecteur1 au vecteur2, il faut soustraire l'angle 2 à l'angle 1

    # return l'angle entre les deux vecteurs. 
    return  (angle2-angle1)



# fonction verifiée
def diffAngle3D(vecteur1,vecteur2):
    """angle pour chaque plan entre deux vecteurs dans l'espace
    
    return un vecteur d'angle [alpha,beta,gamma]"""

    #le but est de généraliser en 3D ce que on avait avant fait en 2D. Le principe reste le meme, calculer dans chaque plan XY,YZ,ZX, la difference d'angle entre les deux vecteurs. Alpha est l'angle du plan XY, beta du plan YZ et gamma du plan ZX

    #calule non optimiser de chaque angle pour chaque plan pour les deux vecteurs.
    angleAlpha1 = anglesysteme(vecteur1[0],vecteur1[1])
    angleBeta1 = anglesysteme(vecteur1[1],vecteur1[2])
    angleGamma1 = anglesysteme(vecteur1[2],vecteur1[0])

    angleAlpha2 = anglesysteme(vecteur2[0],vecteur2[1])
    angleBeta2 = anglesysteme(vecteur2[1],vecteur2[2])
    angleGamma2 = anglesysteme(vecteur2[2],vecteur2[0])

    #Comme on veut les angles pour passer du vecteur1 au vecteur2, il faut soustraire les angles du veteur 2 au angle du vecteur 1

    # return les angles entre les deux vecteurs.
    return np.array([angleAlpha2-angleAlpha1,angleBeta2-angleBeta1,angleGamma2-angleGamma1])





#######################################  initialisation #####################################################################################



#fonction vérifiée
def initialisation(vecacc):
    """creer le referentiel unique

    !mettre le premier vecteur acceleration!
    
    return le point IMU 0"""

    # Pour savoir dans quelle orientation le capteur est au début de la mesure, il faut que au debut de la mesure, le capteur ne bouge pas. Les seuls acceleration qu il mesurera sera alors que g. je fais donc la norme du vecteur(qui devrai toujours etre egal a g) et je creer un vecteur artificiel g avec comme composant(0,g,0). EN faisant la difference d'angle entre les deux vecteurs, on a la difference de position angulaire entre le capteur et le referentiel unique.

    # calcule la norme
    norme = np.linalg.norm(vecacc)

    #creer le vecteur artificiel g
    g = np.array([0,-norme,0])

    #calule la difference d'angle entre g et le vecteurs d'acceleration mesurer par le capteur. Logiquement les angles doivent s'aditionner.
    angleDiff = diffAngle3D(vecacc,g)      

    #comme c'est le premier point et que au début de la mesure, il faut pauser la capteur quelque seconde sans le bouger, alors nous pouvons dire que sa vitesse initial = 0. Comme on creer le referentiel avec ce point, pour ne pas me compliquer la vis, je dis que ce premier point est le point central de mon referentiel, le point (0,0,0). 

    #creation d'un vecteur (0,0,0) pour pouvoir correctement completer les informations pour creer le premier point imu.
    vec0 = np.array([0,0,0])

    #creation de premier point imu
    return PointIMU(vec0,vec0,angleDiff)



#######################################  Point IMU to Point GPS #####################################################################################



def pointIMUtoGPS(point):
        
    #Comme on est certains que le premier point imu et Gps sont au meme endroit dans l'espace, Pour caluler la positon gps d un point imu, il faut faire la difference entre le pointimu0 et le point imu sur x et y pour connaitre la difference de position(cette operation n est pas dan sle code car la position du point0 = (0,0,0), donc cela reviens a faire -0). Pour transforer la differnece de mettre a degrer gps, il faut diviser la difference par un chiffre. Ensuite il suffi d'additionner les degrer relatif au deplacement avec la position gps initial pour trouver la position gps final

    #dans la fonction allignement, je defini que l'axe x de mon referentiel corespond au lattitude et que mon axe z on longitude

    # calcule la difference d angle entre le point de ref(0,0,0) et le point donner  /111111 parce que c est 1 metre en degrer pour le gps
    lat_deplacement = point.r[0] / 111111

    # Comme plus on se raproche des poles, plus les degres de longitude devienne plus petit, la difference entre m et degrer devient variable. Le cos de la position permet de corriger cette variation. Comme je suis sur des changement de position petit, je neglige le changement de position et prend la position de reference pour corriger la variation et non celle juste avant.
    lon_deplacement = point.r[2] / (111111 * np.cos(np.radians(donne[0][-2])))

    # additionne la difference avec le point de reference
    lat = donne[0][-2] + lat_deplacement     
    lon = donne[0][-1] + lon_deplacement


    #return un point Gps avec les coordonné gps que a le point imu
    return creationPointGps(lat,lon)




#######################################  fonction  ######################################################################################

def lireFichier():
    pass






def allignement():
    """"allignement entre les point gps et imu"""

    initialisation(np.array([donne[0][0],donne[0][1],donne[0][2]]))

    #nescecite que l initialisation sois faite  
    nombrePoint = 1 

    #permet de creer les point uniquement jusqu il y aie un deplacement de 20 m
    while np.linalg.norm(PointIMU.point[nombrePoint].r[:1])> 20: 

        #construi le point IMU il faut changer les indice suivant comment est construite la liste
        recurence(PointIMU.point[nombrePoint],donne[nombrePoint][0],donne[nombrePoint][1])   

        #construi le point gps de la meme mesure   changer les indices
        creationPointGps(donne[nombrePoint][3],donne[nombrePoint][4])

        # metrs a jour la constente  
        nombrePoint = nombrePoint + 1

    gps2 = pointIMUtoGPS(PointIMU.point[-1])

    gps1 = creationPointGps(donne[nombrePoint][3],donne[nombrePoint][4])        # a changer les indices

    gps0 = creationPointGps(donne[0][3],donne[0][4])        #a changer

    vecteur1 = gps1.r-gps0.r
    vecteur2 = gps2.r-gps0.r

    angle = diffAnglePlan(vecteur2,vecteur1)

    list.clear(PointGPS.point)
    list.clear(PointIMU.point)

    premierPoint = initialisation(donne[0][0],donne[0][1])

    premierPoint.t[0] = premierPoint.t[0]+angle







def recurence(pointIMU,vecteurAcc,vecteurAng):
    """passage entre n et n+1"""

    # equation angulaire pour obtenir theta_n+1   //  new_t = thetan+1
    newOmega = vecteurAng*temps + pointIMU.t

    # passage de l'acceleration de l'IMU dans le referentiel unique
    accelUnique = rotationVecteur(newOmega,vecteurAcc)

    # equation horaire pour obtenir r_n+1 et v_n+1
    new_r = accelUnique*1/2*(temps**2) + pointIMU.v*temps + pointIMU.r
    new_v = accelUnique*temps + pointIMU.v

    #creation du point_n+1
    return PointIMU(new_r, new_v, newOmega)




def creationPointGps (long, lat):

    position = np.array([long,lat])

    return PointGPS(position)










def creationGraphe():
    # creation de la carte
    m = folium.Map(location=[ref_lat, ref_lon], zoom_start=15)

    for point in PointIMU.point :

       #imutogps a bien verifier quoi fait quoi

        # creer les point en rouge pour l IMU et l ajoute dans son cluster
        folium.Marker(location=[lat, lon], popup=point.label, icon=folium.Icon(color='red')).add_to(m)

    # boucle pour chaque point gps
    for point in PointGPS.point :

        # sors les coordonnées
        lat = point.r[0]
        lon = point.r[1]

        # creer les point en bleu pour le gps et l ajoute dans son cluster
        folium.Marker(location=[lat, lon], popup=point.label, icon=folium.Icon(color='blue')).add_to(m)

    # affichage de la carte
    m.save('map4.html')










def main():

    #lireFichier()
    #allignement()
    #for instances in donne :
        #vecacc = np.array([donne[instances][0],donne[instances][1],donne[instances][2]])        #a cahnger
        #vecang = np.array([donne[instances][4],donne[instances][5],donne[instances][6]])        #a cahnger

        #recurence(PointIMU.point[-1],vecacc,vecang)

        #creationPointGps(donne[instances][7],donne[instances][8])

    #creationGraphe()

    vec = np.array([donne[0][0],donne[0][1],donne[0][2]])


    
    a = initialisation(vec)

    b = rotationVecteur(vec,a.t)

    print(a.t*360/(2*m.pi), b)

    

    






if __name__ == '__main__':
    main()
