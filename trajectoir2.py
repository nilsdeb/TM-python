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






class Matrice :     # plus simple que une fonction
    pass






class Point :       #point de chaque prise de données

    point = [] #liste de tout les point cree

    def __init__(self,pos,vit,posang,vitang):     #pos = vec de position // vit = vec de vitesse // posang = vec position angulaire // vitang  = vec vitesse angulaire
        self.__class__.point.append(self)
        self.r = pos   #r pour r(t)...
        self.v = vit   #v pour v(t)...
        self.t = posang   #t pour theta(t)...
        self.o = vitang   #o pour omega(t)...

    def __str__(self):  #print
        return f"P = (position : {self.r}, vitesse : {self.v}, pos angulaire : {self.t}, vitesse ang : {self.o},)"



def main():
    vec = Vec(1, 2, 2)
    point = Point(vec, vec, vec, vec)
    print(point)
    print(len(Point.point))








if __name__ == '__main__':

    main()