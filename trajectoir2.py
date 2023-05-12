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

    def __init__(self,Vecr,Vecv,Vect,Veco):     #Vecr = vec de position // vecv = vec de vitesse // vect = vec position angulaire // vec0  = vec vitesse angulaire
        self.__class__.point.append(self)
        self.r = Vecr   #r pour r(t)...
        self.v = Vecv   #v pour v(t)...
        self.t = Vect   #t pour theta(t)...
        self.o = Veco   #o pour omega(t)...

    def __str__(self):  #print
        return f"P = ({self.r},{self.v},{self.t},{self.o},)"

        