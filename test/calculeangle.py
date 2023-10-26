# pour les vecteurs et les quaternions
import numpy as np
from numpy import linalg as LA



# pour les maths
import math as m



#fonction lire fichier et liste de données done identique au code tm
donne = []

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


            
        for i in range(len(liste1)):
            donne.append([float(liste1[i]),float(liste2[i]),float(liste3[i]),float(liste4[i])/360*(4*m.pi),float(liste5[i])/360*(4*m.pi),float(liste6[i])/360*(4*m.pi),float(liste7[i]),float(liste8[i])])



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




#fonction de rotation autour d'un vecteur angle dont la norme = l'angle de rotation

def rotate_vector_with_angle_and_axis(axis, vector_to_rotate):


    # Ensure that the axis is a unit vector



    norme = LA.norm(axis)

    angle = norme
    axis /= norme
    
    ux, uy, uz = axis
    cos_theta = np.cos(angle)
    sin_theta = np.sin(angle)
    one_minus_cos_theta = 1 - cos_theta

    # Construct the rotation matrix
    R = np.array([
        [cos_theta + ux**2 * one_minus_cos_theta, ux * uy * one_minus_cos_theta - uz * sin_theta, ux * uz * one_minus_cos_theta + uy * sin_theta],
        [uy * ux * one_minus_cos_theta + uz * sin_theta, cos_theta + uy**2 * one_minus_cos_theta, uy * uz * one_minus_cos_theta - ux * sin_theta],
        [uz * ux * one_minus_cos_theta - uy * sin_theta, uz * uy * one_minus_cos_theta + ux * sin_theta, cos_theta + uz**2 * one_minus_cos_theta]
    ])

    # Rotate the vector using matrix-vector multiplication
    rotated_vector = np.dot(R, vector_to_rotate)

    return rotated_vector




#test
def rotate_vector(vector, axis):
    # Conversion de l'angle en radians
    theta = LA.norm(axis)

    # Calcul de cos(theta) et sin(theta)
    cos_theta = np.cos(theta)
    sin_theta = np.sin(theta)

    # Création de la matrice de croix-produit à partir de l'axe
    u_cross = np.array([[0, -axis[2], axis[1]],
                        [axis[2], 0, -axis[0]],
                        [-axis[1], axis[0], 0]])

    # Création de la matrice de produit externe à partir de l'axe
    u_outer = np.outer(axis, axis)

    # Création de la matrice identité
    I = np.identity(3)

    # Calcul de la matrice de rotation
    R = cos_theta * I + sin_theta * u_cross + (1 - cos_theta) * u_outer

    # Application de la rotation au vecteur
    rotated_vector = np.dot(R, vector)

    return rotated_vector



#rotation par les 3 matrice autour de chaque axes
def rotate_vector_around_axes(vector, angles):
    # Conversion des angles en radians
    theta_x = angles[0]
    theta_y = angles[1]
    theta_z = angles[2]

    # Matrices de rotation autour des axes
    rotation_matrix_x = np.array([[1, 0, 0],
                                  [0, np.cos(theta_x), -np.sin(theta_x)],
                                  [0, np.sin(theta_x), np.cos(theta_x)]])

    rotation_matrix_y = np.array([[np.cos(theta_y), 0, np.sin(theta_y)],
                                  [0, 1, 0],
                                  [-np.sin(theta_y), 0, np.cos(theta_y)]])

    rotation_matrix_z = np.array([[np.cos(theta_z), -np.sin(theta_z), 0],
                                  [np.sin(theta_z), np.cos(theta_z), 0],
                                  [0, 0, 1]])

    # Rotation du vecteur autour de chaque axe
    rotated_vector = np.dot(rotation_matrix_z, np.dot(rotation_matrix_y, np.dot(rotation_matrix_x, vector)))

    return rotated_vector



print()
def main():
    lireFichier("testmruax.tex")

    #print(donne)
    #calibrage()

    nombredonne = len(donne)-1


    vecteurbase = np.array([1,0,0])

    vecteurbase2 = np.array([1,0,0])
    

    x,y,z = 0,0,0

    rx,ry,rz = 0,0,0

    scalaire = 360/(4*m.pi)

    for i in range(nombredonne):
        x += donne[i][3]-0.18/scalaire
        y += donne[i][4]-0.12/scalaire
        z += donne[i][5]-0.18/scalaire
        rx += (donne[i][3]-0.18/scalaire)*0.05
        ry += (donne[i][4]-0.12/scalaire)*0.05
        rz += (donne[i][5]-0.18/scalaire)*0.05
        vecteurangle = np.array([rx,ry,rz])
        vecteurbase = rotationVecteur(vecteurbase,vecteurangle)

   
    cx,cy,cz = x/nombredonne,y/nombredonne,z/nombredonne

    t = nombredonne*0.05
    print("temps de la mesures : {0}".format(t))
    print("moyenne vitesses angle x : {0} /// y : {1} /// z : {2} /// angle final x : {3} /// y : {4} /// z : {5}".format(cx,cy,cz,rx,ry,rz))

    vecteurangle = np.array([rx,ry,rz])


    print("quaternion : {0} /// matrice simple : {1} /// matrice ++ : {2} /// 3d matrice :{3}".format(rotationVecteur(vecteurbase2,vecteurangle),rotate_vector_with_angle_and_axis(vecteurangle,vecteurbase),rotate_vector(vecteurbase,vecteurangle),rotate_vector_around_axes(vecteurbase,vecteurangle)))




if __name__ == '__main__':
    main()