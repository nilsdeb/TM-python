# graphique
import folium
from folium.plugins import MarkerCluster



# pour les vecteurs et les quaternions
import numpy as np
from numpy import linalg as LA



# pour les maths
import math as m



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








def find_rotation_angles_bruteforce(vecteur1, vecteur2, precision):
    """Trouver en testant plein d'angle celui qui permet de se raprocher le plus pour passer du vecteur 2 au vecteur 1
    
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

                print("vector_angle {0}//////// f2 {1}//////// g-f2 {2}//////// diff_norme {3}".format(vectorAngle,f2,vecteur1-f2,diff_norm))

                angleZ += 2*m.pi / precision


            angleY += 2*m.pi / precision

            angleZ = 0

        angleX += 2*m.pi / precision

        angleY = 0

    #cherche la plus petite norme
    a = min(liste)

    #retourne
    return dico[a]



                



    

def main():

    vec = np.array([4,0,3])
    vec1 = np.array([0,0,5])
    a = find_rotation_angles_bruteforce(vec1,vec,50)
    vec2 = rotationVecteur(vec,a)

    print("angle = {0} //// vecteur test = {1}".format(a,vec2))



    



    

    

    






if __name__ == '__main__':
    main()
