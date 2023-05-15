import math as m



t = 0.04    #temp entre chaque mesure des acceleration utiliser dans step
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

    def matrice (self, a) :     #multiplication d'un vecteur par une matrice de rotation (les angles sont donner par un angle de rotation) a = vecteur angle

        xp = self.x*(m.cos(a.y)*m.cos(a.z))+self.y*(m.sin(a.x)*m.sin(a.y)*m.cos(a.z)-m.cos(a.x)*m.sin(a.z))+self.z*(m.cos(a.x)*m.sin(a.y)*m.cos(a.z)+m.cos(a.x)*m.sin(a.z))
        yp = self.x*(m.cos(a.y)*m.cos(a.z))+self.y*(m.sin(a.x)*m.sin(a.y)*m.cos(a.z)+m.cos(a.x)*m.sin(a.z))+self.z*(m.cos(a.x)*m.sin(a.y)*m.cos(a.z)-m.cos(a.x)*m.sin(a.z))
        zp = -self.x*(m.sin(a.y))+self.y*(m.sin(a.x)*m.cos(a.y))+self.z*(m.cos(a.x)*m.cos(a.y))

        return Vec(xp,yp,zp)







class Point :       #point de chaque prise de données

    point = [] #liste de tout les point cree

    def __init__(self,r,v,t,o):     #r = vec de position // v = vec de vitesse // t = vec position angulaire // o  = vec vitesse angulaire
        self.__class__.point.append(self)
        self.r = r   #r pour r(t)=...
        self.v = v   #v pour v(t)=...
        self.t = t   #t pour theta(t)=...
        self.o = o   #o pour omega(t)=...


    def step (self, a, al):     #a pour acceleration // al pour alpha    le but est de cree n+1 aves les donné de n et les accelerations
        nr = a/2*t*t + self.v*t + self.r    #equation horaire // ne peux pas mettre t^2 beug de int et float
        nv = a*t + self.v
        nt = al/2*t*t + self.o*t + self.t
        no = al*t + self.o
        return Point(nr, nv, nt, no)

    def __str__(self):  #print
        return f"Point = position : {self.r}, vitesse : {self.v}, pos angulaire : {self.t}, vitesse ang : {self.o}"






def nextp (p0, a, al):  #p0 pour le point de base // a pour acceleration // al pour alpha

    a1 = a.matrice(p0.t)   #a1 et l'image de a dans le referentiel unique
    al1 = al.matrice(p0.t)    #al1 et l'image de al dans le referentiel unique
    p1 = p0.step(a1, al1)   #p1 et le point n+1, le pas de récurence est construit
    



def main():
    vec = Vec(5,4,3)
    a  = Vec(m.pi/2,m.pi/2,m.pi/2)
    vac= vec.matrice(a)
    point = Point(vac, vac, vac, vac)
    print(point)
    print(len(Point.point))


    point.step(vec, vec)
    print(Point.point[1])

        #regarder quelle forme prenne les mesure pour ecrire la boucle










if __name__ == '__main__':

    main()