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
temps = 0.12


#liste donne, a organiser de cette magniere: [[t,accx....][t2,acc....]]
#ordre de la liste dans la liste[accx,accy,accz,gyrox,giroy,giroz,lat,long]
donne = []


#variable de stockage pour la verification du signe de l acceleration corrigee
accelerationglobale = 0


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





######################################### fonction de rotation d un vectuer #####################################################################################################




def rotate_vector_with_angle_and_axis(axis, vector_to_rotate):

    """rotation d'un vecteur autour d'un vecteur angulaire"""

    #fais la norme du vecteur de rotation = angle que l on veut tourner
    angle = LA.norm(axis)


    #si il y a pas d angle alors il n y a pas de rotation
    if angle == 0 :
        return vector_to_rotate

    #normalise l axe de rotation (angle etant la norme de axis)
    axis2 = axis/angle

    #creation des parametres pour la matrice
    ux, uy, uz = axis2
    cos_theta = np.cos(angle)
    sin_theta = np.sin(angle)
    one_minus_cos_theta = 1 - cos_theta

    # construction de la matrice
    R = np.array([
        [cos_theta + ux**2 * one_minus_cos_theta, ux * uy * one_minus_cos_theta - uz * sin_theta, ux * uz * one_minus_cos_theta + uy * sin_theta],
        [uy * ux * one_minus_cos_theta + uz * sin_theta, cos_theta + uy**2 * one_minus_cos_theta, uy * uz * one_minus_cos_theta - ux * sin_theta],
        [uz * ux * one_minus_cos_theta - uy * sin_theta, uz * uy * one_minus_cos_theta + ux * sin_theta, cos_theta + uz**2 * one_minus_cos_theta]
    ])

    #retourne la rotation par R du vecteur a tourner
    return np.dot(R, vector_to_rotate)




#######################################  fonction calule d'angle entre vecteurs  ######################################################################################




def diffAngleVecteur(vecteur1, vecteur2, precision):

    """difference d'angle entre vecteur 2 et vecteur 1

    !!c'est pour passer du vecteur2 au vecteur1, pas l'inverse!!
    
    vecteur1 = par exemple g
    
    vecteur2 = par exemple premiere acceleration
    
    precision = nombre d'angle que on test pour x, y et z"""

    #angle de base defini, pas a 0 sinon dans la fonction rotate il y a une dividion par 0
    angleX,angleY,angleZ = 0,0,0

    #liste qui me permete de stocker tout les normes
    liste = []

    #dico qui me premet de stocker les angles avec comme indice la norme
    dico = {}

    #boucle qui teste tout les angles
    # Première boucle
    for _ in range(precision-1):  
        
        # Deuxième boucle
        for _ in range(precision-1):  
            
            # Troisième boucle
            for _ in range(precision-1):  
            
                #creer un vecteur angle avec les angle tester a ce moment
                vectorAngle = np.array([angleX,angleY,angleZ])

                # calcule le vecteur en tournant avec les angles tests
                f2 = rotate_vector_with_angle_and_axis(vectorAngle,vecteur2)

                # Calculez la norme de la différence entre f2 et g
                diff_norm = LA.norm(f2-vecteur1)               

                liste.append(diff_norm)

                dico[diff_norm] = vectorAngle

                #print("diffAngleVecteur : vector_angle {0}//////// f2 {1}//////// g-f2 {2}//////// diff_norme {3}".format(vectorAngle,f2,vecteur1-f2,diff_norm))

                angleZ += 2*m.pi / precision


            angleY += 2*m.pi / precision

            #besoin de remettre a 0 les angle a chaque boucles
            angleZ = 0  

        angleX += 2*m.pi / precision

        angleY = 0

    #cherche la plus petite norme
    a = min(liste)

    #retourne le vecteur angle
    return dico[a]




#######################################  calibrage gyro #####################################################################################




def calibrage(nombrepoint):

    """calibre le gyroscope en soustrayant la moyenne des premier point a l'ensemble des donnes gyro"""

    nombredonne = len(donne)-1

    x,y,z = 0,0,0

    for i in range(nombrepoint):
        x += donne[i][3]
        y += donne[i][4]
        z += donne[i][5]

    #fait les moyennes
    cx,cy,cz = x/4,y/4,z/4

    #corrige l'ensemble de la base de donnees
    for i in range(nombredonne):
        donne[i][3] -= cx
        donne[i][4] -= cy
        donne[i][5] -= cz




#######################################  initialisation #####################################################################################




def alignemntG(vecacc):

    """creer le referentiel unique

    !mettre le premier vecteur acceleration!
    
    return le point IMU 0"""

    # Pour savoir dans quelle orientation le capteur est au début de la mesure, il faut que au debut de la mesure, le capteur ne bouge pas. Les seuls acceleration qu il mesurera sera alors que g. je fais donc la norme du vecteur(qui devrai toujours etre egal a g) et je creer un vecteur artificiel g avec comme composant(0,g,0). EN faisant la difference d'angle entre les deux vecteurs, on a la difference de position angulaire entre le capteur et le referentiel unique.

    # calcule la norme
    norme = LA.norm(vecacc)

    #creer le vecteur artificiel g
    g = np.array([0,0,-norme])


    #calule la difference d'angle entre g et le vecteurs d'acceleration mesurer par le capteur. Logiquement les angles doivent s'aditionner.
    angleDiff = diffAngleVecteur(g,vecacc,50) 

    #comme c'est le premier point et que au début de la mesure, il faut pauser la capteur quelque seconde sans le bouger, alors nous pouvons dire que sa vitesse initial = 0. Comme on creer le referentiel avec ce point, pour ne pas me compliquer la vis, je dis que ce premier point est le point central de mon referentiel, le point (0,0,0). 
    vec0 = np.array([0,0,0])

    #creation de premier point imu
    return PointIMU(vec0,vec0,angleDiff)




#######################################  creation Point GPS #####################################################################################




# permet plus simplement de creer des point gps
def creationPointGps (long, lat):

    # creer un vecteur avec deux coordonnées, lattitude et longitude
    position = np.array([long,lat])

    #return un point gps
    return PointGPS(position)




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




#######################################  lire fichier  #####################################################################################################





def lireFichier(nom_fichier):
        
    # Initialiser des listes vides pour stocker les valeurs alternées
    liste1 = []
    liste2 = []
    liste3 = []
    liste4 = []
    liste5 = []
    liste6 = []
    liste7 = []
    liste8 = []

    
    # Ouvrir le fichier en mode lecture
    with open(nom_fichier, 'r') as fichier:
        
        # Lire toutes les lignes du fichier
        lignes = fichier.readlines()

        # Parcourir chaque ligne du fichier
        for i, ligne in enumerate(lignes):
        
            #split en differents ellement des que il y a une virgule
            split = ligne.split(",")

            for i in split :

                # Supprimer les espaces en début et en fin d objet
                i = i.strip()

                #ne prends que les donnees et les stocke dans la bonne liste
                if i[:3] == "ax=" :
                    ni = i[3:]
                    liste1.append(ni)

                if i[:3] == "ay=" :
                    ni = i[3:]
                    liste2.append(ni)

                if i[:3] == "az=" :
                    ni = i[3:]
                    liste3.append(ni)

                if i[:3] == "gx=" :
                    ni = i[3:]
                    liste4.append(ni)
                
                if i[:3] == "gy=" :
                    ni = i[3:]
                    liste5.append(ni)
                
                if i[:3] == "gz=" :
                    ni = i[3:]
                    liste6.append(ni)
                
                if i[:4] == "lat=" :
                    ni = i[4:]
                    liste7.append(ni)
                
                if i[:4] == "lon=" :
                    ni = i[4:]
                    liste8.append(ni)

    #creer la liste donnee dans le bon ordre
    for i in range(len(liste1)):
        donne.append([float(liste1[i]),float(liste2[i]),float(liste3[i]),float(liste4[i])/360*(2*m.pi),float(liste5[i])/360*(2*m.pi),float(liste6[i])/360*(2*m.pi),float(liste7[i]),float(liste8[i])]) #/360*(2*m.pi)




########################################################## initialisation ##########################################################################################




def initialisation():
    """"allignement entre les point gps et imu"""

    #Si on n allligne pas les point IMu et Gps, les point gps peuvent partire par exemple au nord comm ele trajet réel, mais les point imu vers l'est. Pour calibrer les point imu, il faut calculer l angle de decalage et le corriger en l'incluant dans le point0, dans le theta.gamma
    #verifier
    #faire l'initialisation sans calibrage pour donner le point0
    alignemntG(np.array([donne[0][0],donne[0][1],donne[0][2]]))

    print("initialisation",PointIMU.point[0].t)

    #permet d eviter de refaire alignementG plus bas
    stockeangle = PointIMU.point[0].t

    #variable qui compte le nombre de mesure passé, on commnece a 1 car l'initialisation a deja ete faite
    nombrePoint = 0

    #permet de creer les point uniquement jusqu il y aie un deplacement de 20 m. le but et de ne pas tout calculer mais d'avoir une distence assez grande pour pouvoir etre au dessus de l'incertitude du gps.
    while LA.norm(PointIMU.point[nombrePoint].r[0:2])< 1:


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
    norme1 = LA.norm(vecteur1)

    #avoir le vecteur normer
    vecteur2 = np.array([vecteur1[0]/norme1,vecteur1[1]/norme1,0])

    #creation du vecteur representant l IMU
    vecteur3 = np.array([gps2.r[0]-gps0.r[0],gps2.r[1]-gps0.r[1],0])
    norme3 = LA.norm(vecteur3)

    #avoir le vecteur normer
    vecteur4 = np.array([vecteur3[0]/norme3,vecteur3[1]/norme3,0])

    #donne la difference d'angle entre les deux vecteurs. C'est cette difference d'angle qui permettra d'alligner els point gps et les point imu.
    angle = diffAngleVecteur(vecteur2,vecteur4,100)

    print("initialisation , angle",angle)


    #comme tout les point imu doive etre corriger avec le nouvelle angle, il est plus simple d'effacer tout les point construit jusque la et recommnecer la creation de tout les point.
    PointGPS.point.clear()
    PointIMU.point.clear()

    #corection de l'angle theta z
    stockeangle[2] + angle[2]

    #creation du premier point pour la deuxieme fois
    premierPoint = PointIMU(np.array([0,0,0]),np.array([0,0,0]),stockeangle)

    print("initialisation, avant",premierPoint.t)




############################################################# recurence ################################################################################




def recurence(pointIMU,vecteurAcc,vecteurAng):
    """passage entre n et n+1"""

    #en utilisant les equation horaires, avec le point de mesure, le vecteur acceleration et celui de la vitesse angulaire mesurer au point de mesure, il est physiquement possible de savoir ou se fera la prochaine mesure.

    # equation angulaire pour obtenir theta_n+1   //  new_t = theta n+1
    
    newOmega = -vecteurAng*temps + pointIMU.t
    print(f"recurence, base : {pointIMU.t},vecteurang{vecteurAng} iteration {-vecteurAng*temps}, total {newOmega}")
    # comme le capteur n est pas fixer a un rail, il peut faire des mouvements dotatif. Cela implique que il perd le referentiel de la mesure n-1. avec les equation angulaire, tetha et omega, cela permet de faire une projection du vecteur de le referentiel de mesure au referentiel unique.

    # passage de l'acceleration de l'IMU dans le referentiel unique
    accelUnique = rotate_vector_with_angle_and_axis(newOmega,vecteurAcc)
    #accelUnique = vecteurAcc

    print("recurence")
    print("recurence, vecteur acc : {0},norme {3}, vecteur corriger ; {1}, norme {2}".format(vecteurAcc,accelUnique,LA.norm(accelUnique),LA.norm(vecteurAcc)))
    print("recurence")

    #comme cette acceleration mesure en permanence g, il faut l'enlever, mais comme g = -1, il faut faire +1
    accelUnique[2] = accelUnique[2]+1

    #comme g != 1 mais 9.81, il faut multiplier toute l'acceleration par g
    accelUnique = 9.81*accelUnique

    #permet de stocker ces acceleration
    global accelerationglobale 
    accelerationglobale = accelUnique

    #utilisation du vecteur d'acceleration projeter dans le referentiel unique pour deduire le prochain point.

    # equation horaire pour obtenir r_n+1 et v_n+1
    new_r = accelUnique*1/2*(temps**2) + pointIMU.v*temps + pointIMU.r
    new_v = accelUnique*temps + pointIMU.v

    #creation du point_n+1
    return PointIMU(new_r, new_v, newOmega)




################################################################ creation graphe ########################################################




def creationGraphe():

    # creation de la carte
    m = folium.Map(location=[donne[0][-2],donne[0][-1]], zoom_start=15)

    
    for point in PointIMU.point :

        #cette etape revient a faire la meme chose que dans point imu to gps mais nous ne pouvons pas utiliser cette fonction ici car cela crereai des point gps, qui seront inditengable des point imu. la carte en devient donc incompresible. Pour comprendre les 4 prochaine ligne de code, il faut se referer a la fonction ImuToGps
        lat_deplacement = point.r[0] / 111111

        lon_deplacement = point.r[1] / (111111 * np.cos(np.radians(donne[0][-2])))

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




########################################################## plus moins ################################################################




def plusmoins(ab,plus,moins):

    if ab > 0 :

        plus += 1
    if ab < 0 :

        moins += 1


    return plus,moins




########################################################## main ################################################################




def main():
    #verifier, fonctionnel
    lireFichier("test.tex")


    #permet de corrige le calibrage des gyros
    calibrage(2)


    #permet de creer le premier avec toute ces correction
    initialisation()
    #alignemntG(np.array([donne[0][0],donne[0][1],donne[0][2]]))
    
    #variable de stockage pour plusmoin et comprendre quand l'accélération est postive ou negative sur quelle axe
    plusx,plusy,plusz = 0,0,0
    moinsx,moinsy,moinsz = 0,0,0
    aplusx,aplusy,aplusz = 0,0,0
    amoinsx,amoinsy,amoinsz = 0,0,0

    #boucle qui crée tout les poin IMU par recurence et tout les point gps
    for a in range(len(donne)) :
        
        #crer le vecteur acceleration pour l'iteration
        vecacc = np.array([donne[a][0],donne[a][1],donne[a][2]])

        #creer le vecteur omega
        vecang = np.array([donne[a][3],donne[a][4],donne[a][5]])

        #fait la recurence
        recurence(PointIMU.point[-1],vecacc,vecang)

        #comprendre les accélération
        plusx,moinsx = plusmoins(accelerationglobale[0],plusx,moinsx)
        plusy,moinsy = plusmoins(accelerationglobale[1],plusy,moinsy)
        plusz,moinsz = plusmoins(accelerationglobale[2],plusz,moinsz)
        aplusx,amoinsx = plusmoins(vecacc[0],aplusx,amoinsx)
        aplusy,amoinsy = plusmoins(vecacc[1],aplusy,amoinsy)
        aplusz,amoinsz = plusmoins(vecacc[2],aplusz,amoinsz)

        print("main",PointIMU.point[-1])      

        #si il n'y a pas de donnes gps sur cette iteration, prends la derniere donnes valide
        if donne[a][-2] > 2:
            creationPointGps(donne[a][-2],donne[a][-1])

        else :
            creationPointGps(PointGPS.point[a-1].r[0],PointGPS.point[a-1].r[0])

    print("main",temps*len(PointIMU.point))

    print(("main",PointIMU.point[4].t-PointIMU.point[-1].t))

    print(f"mains,vecteur base x {amoinsx} , plus {aplusx} /// y {amoinsy} , plus {aplusy} ///  z {amoinsz} , plus {aplusz}")
    print(f"main,vecteur unique x {moinsx} , plus {plusx} /// y {moinsy} , plus {plusy} ///  z {moinsz} , plus {plusz}")

    #creation du plan
    creationGraphe()

    





if __name__ == '__main__':
    main()