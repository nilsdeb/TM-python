#######################################  librairie  ######################################################################################


# graphique
import folium



# pour les vecteurs et les quaternions
import numpy as np
from numpy import linalg as LA



# pour les maths
import math as m



#######################################  liste et variable  ######################################################################################





#constente du temps entre deux mesures
temps = 10


#liste donne, a organiser de cette magniere: [[t,accx....][t2,acc....]]
#ordre de la liste dans la liste[accx,accy,accz,gyrox,giroy,giroz,lat,long]
donne = []



#######################################  class point  ######################################################################################





#class des points IMU
class PointIMU :

    # liste de tout les point cree, cela permet de les appeler ou  de les compters
    point = [] 
    
    # Les point IMU représente l'etat dans lequel est le capteur au moment de prendre cette mesure. Pour pouvoir appliquer les equations horaire a partire de ce moment précis, il faut stocker dans chaqu'un des point, la position, la vitesse, et l'angle dans lequel se trouve le capteur au moment de la prise de la mesure.
    def __init__(self,vec_r,vec_v,vec_t):
        self.__class__.point.append(self)
        self.r = vec_r  #position
        self.v = vec_v  #vitesse
        vec_t[0] = vec_t[0]%(2*m.pi)
        vec_t[1] = vec_t[1]%(2*m.pi)
        vec_t[2] = vec_t[2]%(2*m.pi)
        self.t = vec_t  #position angulaire
        self.label = 'IMU ' + str(len(self.__class__.point)-1)

    #print
    def __str__(self):  
        return f"Point {self.label} = position : {self.r}, vitesse : {self.v}, pos angulaire : {self.t}"



# class des points GPS
class PointGPS :

    # liste de tout les point cree
    point = []

    # Pour cette class, stocker les donnees de lattitude et de longitude suffise. Il serait possible de ne pas creer cette class et d'utiliser les donnée brut, mais cela compliquerai grandement la compréension du code plus tard. Comme la mesures des point IMU et des pointGps se fait en meme temp, il est plus simple de creer un point IMU et gps pour chaque point de mesure.  !!!!text a changer!!!!
    def __init__(self,vec_r):
        self.__class__.point.append(self)
        self.r = vec_r  #position
        self.label = 'gps ' + str(len(self.__class__.point)-1)

    # print
    def __str__(self):  
        return f"Point {self.label} = position : {self.r}"





################################### Fonction pour effectuer la rotation d'un vecteur à l'aide de quaternions  #############################
# code pris de https://pastebin.com/9aVXyUK8 puis adapté



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
 




#######################################  fonction calule d'angle entre vecteurs  ######################################################################################





def diffAngleVecteur(vecteur1, vecteur2, precision):
    """difference d'angle entre vecteur 2 et vecteur 1

    !!c'est pour passer du vecteur2 au vecteur1, pas l'inverse!!
    
    vecteur1 = par example g
    
    vecteur2 = par example premiere acceleration
    
    precision = nombre d'angle que on test pour x, y et z"""


    #angle de base defini a 0 
    angleX,angleY,angleZ = 0,0,0

    #liste qui me permete de stocker tout les normes
    liste = []

    #dico qui me premet de stocker les angles avec comme indice la norme
    dico = {}

    #boucle qui teste tout les angles
    for _ in range(precision):  # Première boucle
    
        for _ in range(precision):  # Deuxième boucle
        
            for _ in range(precision):  # Troisième boucle
            
                #creer un vecteur angle avec les angle tester a ce moment
                vectorAngle = np.array([angleX,angleY,angleZ])

                # calcule le vecteur en tournant avec les angles tests
                f2 = rotationVecteur(vecteur2,vectorAngle)

                # Calculez la norme de la différence entre f2 et g
                diff_norm = np.linalg.norm(f2-vecteur1)               

                liste.append(diff_norm)

                dico[diff_norm] = vectorAngle

                #print("vector_angle {0}//////// f2 {1}//////// g-f2 {2}//////// diff_norme {3}".format(vectorAngle,f2,vecteur1-f2,diff_norm))

                angleZ += 2*m.pi / precision


            angleY += 2*m.pi / precision

            #besoin de remettre a 0 les angle a chaque boucles
            angleZ = 0  

        angleX += 2*m.pi / precision

        angleY = 0

    #cherche la plus petite norme
    a = min(liste)

    #retourne
    return dico[a]





#######################################  initialisation #####################################################################################



def initialisation(vecacc):
    """creer le referentiel unique

    !mettre le premier vecteur acceleration!
    
    return le point IMU 0"""

    # Pour savoir dans quelle orientation le capteur est au début de la mesure, il faut que au debut de la mesure, le capteur ne bouge pas. Les seuls acceleration qu il mesurera sera alors que g. je fais donc la norme du vecteur(qui devrai toujours etre egal a g) et je creer un vecteur artificiel g avec comme composant(0,g,0). EN faisant la difference d'angle entre les deux vecteurs, on a la difference de position angulaire entre le capteur et le referentiel unique.

    # calcule la norme

    norme = np.linalg.norm(vecacc)

    #creer le vecteur artificiel g
    g = np.array([0,0,-norme])


    #calule la difference d'angle entre g et le vecteurs d'acceleration mesurer par le capteur. Logiquement les angles doivent s'aditionner.
    angleDiff = diffAngleVecteur(g,vecacc,50) 


    #comme c'est le premier point et que au début de la mesure, il faut pauser la capteur quelque seconde sans le bouger, alors nous pouvons dire que sa vitesse initial = 0. Comme on creer le referentiel avec ce point, pour ne pas me compliquer la vis, je dis que ce premier point est le point central de mon referentiel, le point (0,0,0). 

    #creation d'un vecteur (0,0,0) pour pouvoir correctement completer les informations pour creer le premier point imu.
    vec0 = np.array([0,0,0])


    #creation de premier point imu
    return PointIMU(vec0,vec0,angleDiff)



#######################################  Point IMU to Point GPS #####################################################################################



def pointIMUtoGPS(point):
        
    #Comme on est certains que le premier point imu et Gps sont au meme endroit dans l'espace, Pour caluler la positon gps d un point imu, il faut faire la difference entre le pointimu0 et le point imu sur x et y pour connaitre la difference de position(cette operation n est pas dans le code car la position du point0 = (0,0,0), donc cela reviens a faire -0). Pour transforer la differnece de mettre a degrer gps, il faut diviser la difference par un chiffre. Ensuite il suffi d'additionner les degrer relatif au deplacement avec la position gps initial pour trouver la position gps final

    #dans la fonction allignement, je defini que l'axe x de mon referentiel corespond au lattitude et que mon axe z on longitude

    # calcule la difference d angle entre le point de ref(0,0,0) et le point donner  /111111 parce que c est 1 metre en degrer pour le gps
    lat_deplacement = point.r[0] / 111111

    # Comme plus on se raproche des poles, plus les degres de longitude devienne plus petit, la difference entre m et degrer devient variable. Le cos de la position permet de corriger cette variation. Comme je suis sur des changement de position petit, je neglige le changement de position et prend la position de reference pour corriger la variation et non celle juste avant.
    lon_deplacement = point.r[1] / (111111 * np.cos(np.radians(donne[0][-2])))

    # additionne la difference avec le point de reference
    lat = donne[0][-2] + lat_deplacement     
    lon = donne[0][-1] + lon_deplacement


    #return un point Gps avec les coordonné gps que a le point imu
    return creationPointGps(lat,lon)




#######################################  fonction  ######################################################################################





def lireFichier(nom_fichier):
    try:
        # Ouvrir le fichier en mode lecture
        with open(nom_fichier, 'r') as fichier:
            # Lire toutes les lignes du fichier
            lignes = fichier.readlines()

            # Initialiser des listes vides pour stocker les valeurs alternées
            liste1 = []
            liste2 = []
            liste3 = []
            liste4 = []
            liste5 = []
            liste6 = []
            liste7 = []
            liste8 = []


            # Parcourir chaque ligne du fichier
            for i, ligne in enumerate(lignes):
                # Supprimer les espaces en début et en fin de ligne
                ligne = ligne.strip()
            # Ajouter la ligne aux listes alternatives en fonction de la parité de l'indice
                if i % 11 == 0:
                    liste1.append(ligne)
                if i % 11 == 1:
                    liste2.append(ligne)
                if i % 11 == 2:
                    liste3.append(ligne)
                if i % 11 == 4:
                    liste4.append(ligne)
                if i % 11 == 5:
                    liste5.append(ligne)
                if i % 11 == 6:
                    liste6.append(ligne)
                if i % 11 == 8:
                    liste7.append(ligne)
                if i % 11 == 10:
                    liste8.append(ligne)

            #print(liste1,liste2,liste3,liste4,liste5,liste6,liste7,liste8)
            
            for i in range(len(liste8)):
                donne.append([float(liste1[i]),float(liste2[i]),float(liste3[i]),float(liste4[i])/180*m.pi,float(liste5[i])/180*m.pi,float(liste6[i])/180*m.pi,float(liste7[i]),float(liste8[i])])
    except FileNotFoundError:
        print(f"Le fichier '{nom_fichier}' n'a pas été trouvé.")
        return [], []
    except Exception as e:
        print(f"Une erreur s'est produite : {e}")
        return [], []


def allignement():
    """"allignement entre les point gps et imu"""

    #Si on n allligne pas les point IMu et Gps, les point gps peuvent partire par exemple au nord comm ele trajet réel, mais les point imu vers l'est. Pour calibrer les point imu, il faut calculer l angle de decalage et le corriger en l'incluant dans le point0, dans le theta.gamma
    #verifier
    #faire l'initialisation sans calibrage pour donner le point0
    initialisation(np.array([donne[0][0],donne[0][1],donne[0][2]]))


    #variable qui compte le nombre de mesure passé, on commnece a 1 car l'initialisation a deja ete faite
    nombrePoint = 0

    #permet de creer les point uniquement jusqu il y aie un deplacement de 10 m. le but et de ne pas tout calculer mais d'avoir une distence assez grande pour pouvoir etre au dessus de l'incertitude du gps.
    while np.linalg.norm(PointIMU.point[nombrePoint].r[0:2])< 50:


        #construit les vecteur d'acceleration et de vitesse angulaire pour la recurence
        vecacc = np.array([donne[nombrePoint][0],donne[nombrePoint][1],donne[nombrePoint][2]])
        vecvitang = np.array([donne[nombrePoint][3],donne[nombrePoint][4],donne[nombrePoint][5]])

        #construit le point IMU suivant. 
        recurence(PointIMU.point[nombrePoint],vecacc,vecvitang)

        # mets a jour la constente  
        nombrePoint += 1 

    #point gps du point 0
    gps0 = creationPointGps(donne[0][-2],donne[0][-1])

    # Pour le dernier point imu creer, le point gps1 et le point gps mesurer a cette instant de mesure
    gps1 = creationPointGps(donne[nombrePoint][-2],donne[nombrePoint][-1])

    # Creer le point gps obtenu grace a l'imu
    gps2 = pointIMUtoGPS(PointIMU.point[-1])

    #creation du vecteur representant le GPS
    vecteur1 = np.array([gps1.r[0]-gps0.r[0],gps1.r[1]-gps0.r[1],0])
    norme1 = np.linalg.norm(vecteur1)

    #avoir le vecteur normer
    vecteur2 = np.array([vecteur1[0]/norme1,vecteur1[1]/norme1,0])

    #creation du vecteur representant l IMU
    vecteur3 = np.array([gps2.r[0]-gps0.r[0],gps2.r[1]-gps0.r[1],0])
    norme3 = np.linalg.norm(vecteur3)

    #avoir le vecteur normer
    vecteur4 = np.array([vecteur3[0]/norme3,vecteur3[1]/norme3,0])

    #donne la difference d'angle entre les deux vecteurs. C'est cette difference d'angle qui permettra d'alligner els point gps et les point imu.
    angle = diffAngleVecteur(vecteur2,vecteur4,50)

    #comme tout les point imu doive etre corriger avec le nouvelle angle, il est plus simple d'effacer tout les point construit jusque la et recommnecer la creation de tout les point.
    PointGPS.point.clear()
    PointIMU.point.clear()

    #creation du premier point pour la deuxieme fois
    premierPoint = initialisation(np.array([donne[0][0],donne[0][1],donne[0][2]]))

    #correction de l'angle theta dans le point0, cela allignera les chemin du gps et celui de l'imu   ±???, attention, prendre que l'angle z, le reste est faux car se sont dans vecteur 2d passé artificiellement en 3d
    premierPoint.t[2] -= angle[2]



def recurence(pointIMU,vecteurAcc,vecteurAng):
    """passage entre n et n+1"""

    #en utilisant les equation horaires, avec le point de mesure, le vecteur acceleration et celui de la vitesse angulaire mesurer au point de mesure, il est physiquement possible de savoir ou se fera la prochaine mesure.

    # equation angulaire pour obtenir theta_n+1   //  new_t = theta n+1
    newOmega = vecteurAng*temps + pointIMU.t
    newOmega[0] = newOmega[0]%(2*m.pi)
    newOmega[1] = newOmega[1]%(2*m.pi)
    newOmega[2] = newOmega[2]%(2*m.pi)

    # comme le capteur n est pas fixer a un rail, il peut faire des mouvements dotatif. Cela implique que il perd le referentiel de la mesure n-1. avec les equation angulaire, tetha et omega, cela permet de faire une projection du vecteur de le referentiel de mesure au referentiel unique.

    # passage de l'acceleration de l'IMU dans le referentiel unique
    accelUnique = rotationVecteur(vecteurAcc,newOmega)

    #comme cette acceleration mesure en permanence g, il faut l'enlever, mais comme g = -1, il faut faire +1
    accelUnique[2] = accelUnique[2]+1

    #utilisation du vecteur d'acceleration projeter dans le referentiel unique pour deduire le prochain point.

    # equation horaire pour obtenir r_n+1 et v_n+1
    new_r = accelUnique*1/2*(temps**2) + pointIMU.v*temps + pointIMU.r
    new_v = accelUnique*temps + pointIMU.v

    #creation du point_n+1
    return PointIMU(new_r, new_v, newOmega)





# permet plus simplement de creer des point gps
def creationPointGps (long, lat):

    # creer un vecteur avec deux coordonnées, lattitude et longitude
    position = np.array([long,lat])

    #return un point gps
    return PointGPS(position)



def creationGraphe():

    # creation de la carte
    m = folium.Map(location=[donne[0][-2],donne[0][-1]], zoom_start=15)

    
    for point in PointIMU.point :

        #cette etape revient a faire la meme chose que dans point imu to gps mais nous ne pouvons pas utiliser cette fonction ici car cela crereai des point gps, qui seront inditengable des point imu. la carte en devient donc incompresible. Pour comprendre les 4 prochaine ligne de code, il faut se referer a la fonction ImuToGps
        lat_deplacement = point.r[0] / 111111

        lon_deplacement = point.r[2] / (111111 * np.cos(np.radians(donne[0][-2])))

        lat = donne[0][-2] + lat_deplacement      
        lon = donne[0][-1] + lon_deplacement

        # creer les point en rouge Qui symbolise les point imu, mais il les rentres sous forme gps pour que il puisse l'afficher sur la carte. Il creer un cluster et cela permet de mettre tout les point imu dans ce cluster
        folium.Marker(location=[lat, lon], popup=point.label, icon=folium.Icon(color='red')).add_to(m)

    # boucle pour chaque point gps
    for point in PointGPS.point :

        # creer les point en bleu pour le gps et l ajoute dans son cluster
        folium.Marker(location=[point.r[0], point.r[1]], popup=point.label, icon=folium.Icon(color='blue')).add_to(m)

    # sauve la carte sous le nom de map4
    m.save('map2.html')








def main():
    #verifier, fonctionnel
    lireFichier("donnes.tex")


    #permet de creer le premier avec toute ces correction
    allignement()

    #boucle qui crée tout les poin IMU par recurence et tout les point gps

    #pas possible de faire sans passé par cette petite variable a
    a = 0
    for i in donne :
        
        vecacc = np.array([donne[a][0],donne[a][1],donne[a][2]])

        #attention,les angles sont en degrer
        vecang = np.array([donne[a][3],donne[a][4],donne[a][5]])

        recurence(PointIMU.point[-1],vecacc,vecang)


        if donne[a][-2] > 2:

            creationPointGps(donne[a][-2],donne[a][-1])
        else :
            creationPointGps(PointGPS.point[a-1].r[0],PointGPS.point[a-1].r[0])

        a += 1

    #creation du plan
    creationGraphe()

    

    






if __name__ == '__main__':
    main()
