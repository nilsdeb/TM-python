import math as m



t = 0.5    #temp entre chaque mesure des acceleration utiliser dans step
n = 0    #defini combien de position on a calculer

class Vec :     #code vecteurs 3d et ces operations : vecteurs d'angle compris

    def __init__(self,x,y,z):   #defni quand on le cree
        self.x = x
        self.y = y
        self.z = z

    def __add__(self,other):    #addition
        return Vec(self.x+other.x, self.y+other.y, self.z+other.z)

    def __sub__(self,other):    #soustraction
        return Vec(self.x-other.x, self.y-other.y, self.z-other.z)

    def __mul__(self,x):    #multiplication
        return Vec(self.x*x, self.y*x, self.z*x)

    def __truediv__(self,x):    #division
        return Vec(self.x/x, self.y/x, self.z/x)

    def __str__(self):      #pour print
        return f"({self.x},{self.y},{self.z})"

    def norme (self):
        return  m.sqrt(self.x*self.x+self.y*self.y+self.z*self.z)


    def matrice (self, a) :     #a = vecteur angle  multiplication d'un vecteur par une matrice de rotation (les angles sont donner par un angle de rotation) le plus simple est de faire matrice axe x ensuite matrice axe y ... 

        x1 = self.x     # matrice x
        y1 = self.y * m.cos(-a.x) - self.z * m.sin(-a.x)     # matrice x        - devant tout les angle parce que les matrices sont R(-alpha)+...
        z1 = self.y * m.sin(-a.x) + self.z * m.cos(-a.x)     # matrice x
        x2 = x1 * m.cos(-a.y) + z1 * m.sin(-a.y)     # matrice y
        y2 = y1     # matrice y
        z2 = - x1 * m.sin(-a.y) + z1 * m.cos(-a.y)     # matrice y
        x3 = x2 * m.cos(-a.z) - y2 * m.sin(-a.z)     # matrice z
        y3 = x2 * m.sin(-a.z) + y2 * m.cos(-a.z)     # matrice z
        z3 = z2     # matrice z
    

        return Vec(x3, y3, z3)







class Point :       #point de chaque prise de données

    point = [] #liste de tout les point cree

    def __init__(self,r,v,t):     #r = vec de position // v = vec de vitesse // t = vec position angulaire
        self.__class__.point.append(self)
        self.r = r   #r pour r(t)=...
        self.v = v   #v pour v(t)=...
        self.t = t   #t pour theta(t)=...






    def step (self, a, o):     #a pour acceleration // o pour omega    le but est de cree n+1 aves les donné de n et les accelerations
        nt = o*t + self.t       #nt = nouveau theta     cration de theta.n+1
        print(a.norme())
        a1 = a.matrice(nt)      #a1 = image de l'acceleration de l'imu dans le referentiel unique
        print(a1.norme())
        nr = a1*1/2*t*t + self.v*t + self.r    #nr = nouvelle position    equation horaire // ne peux pas mettre t^2 beug de int et float
        nv = a1*t + self.v      #nv = nouvelle vitesse  equation horaire pour la vitesse
        return Point(nr, nv, nt)        #retourne le point d'apres  le pas de recurrence est construit

    def __str__(self):  #print
        return f"Point = position : {self.r}, vitesse : {self.v}, pos angulaire : {self.t}"



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