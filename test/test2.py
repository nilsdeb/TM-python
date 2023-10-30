



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

    nombreligne = 0
    
    # Ouvrir le fichier en mode lecture
    with open(nom_fichier, 'r') as fichier:
        
        # Lire toutes les lignes du fichier
        lignes = fichier.readlines()


        # Parcourir chaque ligne du fichier
        for i, ligne in enumerate(lignes):

            #split en differents ellement des que il y a une virgule
            split = ligne[nombreligne].split(",")

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

            nombreligne +=


            
        for i in range(len(liste8)):
            donne.append([float(liste1[i]),float(liste2[i]),float(liste3[i]),float(liste4[i])/180*m.pi,float(liste5[i])/180*m.pi,float(liste6[i])/180*m.pi,float(liste7[i]),float(liste8[i])])
