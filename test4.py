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








def find_rotation_angles_bruteforce(g, f):
    #definition des angle tester

    angleX,angleY,angleZ = 0,0,0

    vectorAngle = np.array([angleX,angleY,angleZ])


    for _ in range(50):  # Première boucle
        angleX += m.pi / 50
    
        for _ in range(50):  # Deuxième boucle
            angleY += m.pi / 50
        
            for _ in range(50):  # Troisième boucle
                angleZ += m.pi / 50
            
                # Calculez f en utilisant les angles actuels (angleX, angleY, angleZ)
                vectorAngle = (angleX, angleY, angleZ)
                # Vous devez calculer f en utilisant la fonction rotationVecteur ici, en passant vectorAngle

                f2 = rotationVecteur(f,vectorAngle)

                # Calculez la norme de la différence entre f2 et g
                diff_norm = np.linalg.norm(f2 - g)

                print(f2-g)
                

                if diff_norm < 0.3:
                    return np.array([-vectorAngle[0],-vectorAngle[1],-vectorAngle[2]])



    

def main():

    vec = np.array([4,0,3])
    vec1 = np.array([0,0,5])
    a = find_rotation_angles_bruteforce(vec,vec1)
    print (a)

    vec2 = rotationVecteur(vec,a)

    print(vec2)



    



    

    

    






if __name__ == '__main__':
    main()
