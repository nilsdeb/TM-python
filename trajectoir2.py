
import numpy as np      #pour les matrices

t = 0.5    #temp entre chaque mesure des acceleration utiliser dans step

x_coords = []       #liste des position pour l'affichage graphique
y_coords = []
z_coords = []


vx_velo = []        #liste des vitesse pour graph
vy_velo = []
vz_velo = []

tx_coords = []       #liste des position angulaire pour graph / theta x....
ty_coords = []
tz_coords = []







class Vec :     #code vecteurs 3d et ces operations : vecteurs d'angle compris
    def __init__(self,x,y,z):   #defni quand on le cree
        self.x = x
        self.y = y
        self.z = z

    def __add__(self,other):    #addition
        return Vec(self.x + other.x, self.y + other.y, self.z + other.z)

    def __sub__(self,other):    #soustraction
        return Vec(self.x - other.x, self.y - other.y, self.z - other.z)

    def __mul__(self,x):    #multiplication
        return Vec(self.x * x, self.y * x, self.z * x)

    def __truediv__(self,x):    #division
        return Vec(self.x / x, self.y / x, self.z / x)

    def __str__(self):      #pour print
        return f"({self.x},{self.y},{self.z})"

    def norme (self):       #pour tester les matrice car norme a = norme a1
        return  np.sqrt(self.x*self.x+self.y*self.y+self.z*self.z)




#######################################  a changer  ######################################################################################



    def matrice (self, a) :     #a = vecteur angle  multiplication d'un vecteur par une matrice de rotation (les angles sont donner par un angle de rotation) le plus simple est de faire matrice axe x ensuite matrice axe y ... 
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
        


class Point :       #point de chaque prise de données

    point = [] #liste de tout les point cree

    def __init__(self,r,v,t):     #r = vec de position // v = vec de vitesse // t = vec position angulaire
        self.__class__.point.append(self)
        self.r = r   #r pour r(t)=...
        self.v = v   #v pour v(t)=...
        self.t = t   #t pour theta(t)=...

    def step (self, acc, omega):     #acc pour acceleration    le but est de cree n+1 aves les donné de n et les accelerations
        new_t = omega*t + self.t       #new_t = nouveau theta     cration de theta.n+1
        uni_acc = acc.matrice(nt)      #uni_acc = image de l'acceleration de l'imu dans le referentiel unique
        new_r = uni_acc*1/2*t*t + self.v*t + self.r    #new_r = nouvelle position    equation horaire // ne peux pas mettre t^2 beug de int et float
        new_v = uni_acc*t + self.v      #new_v = nouvelle vitesse  equation horaire pour la vitesse
        return Point(new_r, new_v, new_t)        #retourne le point d'apres  le pas de recurrence est construit


    def __str__(self):  #print
        return f"Point = position : {self.r}, vitesse : {self.v}, pos angulaire : {self.t}"




def graphics ():






#premiere partie : sortire les info du fichier creer lpar l ardu
#analyser ces fichier
#afichage et conclusion..





def main():
    r0 = Vec(4, 5, 6)
    v0 = Vec(7 , 8, 9)
    t0 = Vec(1,1,1)
    base = Point(r0, v0, t0)
    o1 = Vec(1,2,3)
    a1 = Vec(1,2,3)
    step1 = base.step(a1, o1)
    #print(step1)
    #print(step1.r.norme())
    unit = Vec(9.4567,6.6324,3.6234)
    angle = Vec(5,34,76)
    nunit = unit.matrice(angle)
    #print(unit, unit.norme())
    #print(nunit, nunit.norme())

if __name__ == '__main__':
    main()
