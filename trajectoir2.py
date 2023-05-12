import math as m





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
        return Vec(self.x/x, self.y/x, self.z/zx)

    def __str__(self):      #pour print
        return f"Vec = ({self.x},{self.y},{self.z})"

    def matrice (self, a) :     #multiplication d'un vecteur par une matrice de rotation (les angles sont donner par un angle de rotation) a = vecteur angle

        xp = self.x*(m.cos(a.y)*m.cos(a.z))+self.y*(m.sin(a.x)*m.sin(a.y)*m.cos(a.z)-m.cos(a.x)*m.sin(a.z))+self.z*(m.cos(a.x)*m.sin(a.y)*m.cos(a.z)+m.cos(a.x)*m.sin(a.z))
        yp = self.x*(m.cos(a.y)*m.cos(a.z))+self.y*(m.sin(a.x)*m.sin(a.y)*m.cos(a.z)+m.cos(a.x)*m.sin(a.z))+self.z*(m.cos(a.x)*m.sin(a.y)*m.cos(a.z)-m.cos(a.x)*m.sin(a.z))
        zp = -self.x*(m.sin(a.y))+self.y*(m.sin(a.x)*m.cos(a.y))+self.z*(m.cos(a.x)*m.cos(a.y))

        return Vec(xp,yp,zp)







class Point :       #point de chaque prise de données

    point = [] #liste de tout les point cree

    def __init__(self,r,v,t,o):     #r = vec de position // v = vec de vitesse // t = vec position angulaire // o  = vec vitesse angulaire
        self.__class__.point.append(self)
        self.r = r   #r pour r(t)...
        self.v = v   #v pour v(t)...
        self.t = t   #t pour theta(t)...
        self.o = o   #o pour omega(t)...

    def __str__(self):  #print
        return f"P = (position : {self.r}, vitesse : {self.v}, pos angulaire : {self.t}, vitesse ang : {self.o},)"



def main():
    vec = Vec(5,4,3)
    a  = Vec(m.pi/2,m.pi/2,m.pi/2)
    vac= vec.matrice(a)
    point = Point(vac, vac, vac, vac)
    print(point)
    print(len(Point.point))








if __name__ == '__main__':

    main()