# data est une liste qui contient les gx,gy,gz avec encore aucune conversion.
outfile = 'finalout.csv'
with open(outfile, 'w') as out:
for k,row in enumerate(data):
# conversion en rad/s (hypothese: la source est en degres par secondes)
dps = np.array(row)
w = dps/180*math.pi
# !!! Il y a un signe - car on veut 'annuler' la rotation
d_theta = -w*dt
# Soit on intègre le vecteur theta et alors on 
# recalcul chaque fois la position de v1 par rapport
# au vecteur original de v1 (1,0,0), soit on tourne chaque
# fois le v1 de la quantité d_theta, je choisis ici la 
# deuxième possibilité.
v1 = rotate_vector_with_angle_and_axis(d_theta,v1)
out.write( f"{k}\t{v1[0]}\t{v1[1]}\t{v1[2]}\n")