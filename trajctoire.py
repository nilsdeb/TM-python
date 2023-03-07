import math as m



# position "0" en vitesse, accelereation, position 0 de gps, vecteur acceleration, vecteur du gps(attention 2d) => changement one step => postion un => loop
# position gps confirme celle des accelerometre et gyro




class Point :       #creation de point pour chaque mesure

    point = []      #liste des points pour pouvoir les compter

    def __init__(self,x,y,z,gx,gy,gz,vx,vy,vz):

        self.__class__.point.append(self)       #pour les ajouter a la liste
        self.x = x      #position
        self.y = y
        self.z = z
        self.gx = gx        #postion angulaire dans l'espace
        self.gy = gy
        self.gz = gz
        self.vx = vx        #vitesse a se point
        self.vy = vy
        self.vz = vz

    def __str__(self):      #pour print
        return f"P({self.x},{self.y},{self.z},{self.gx},{self.gy},{self.gz},{self.vx},{self.vy},{self.vz})"



class Vec :
    def __init__(self,x,y,z):
        self.x = x
        self.y = y
        self.z = z      #que besoin du vecteur qui se corrigera avec le points d'avant

    def norme(other):   #norme d'un vecteur
        return m.sqrt(other.x**2+other.y**2+other.z**2)

    def angle(self,normeself):
        angx = m.acos(self.x/normeself)     #x sur la norme qui est l'hypotenuse
        angy = m.acos(self.y/normeself)
        angz = m.acos(self.z/normeself)
        return (angx,angy,angz)     #angle x,y,z

    def retangle(normeself,listeang):      # liste angle (x,y,z) 
        x = m.cos(listeang[0])*normeself
        y = m.cos(listeang[1])*normeself
        z = m.cos(listeang[2])*normeself
        return Vec(x,y,z)       #redonne un vecteur

    def __add__(self,other):
        return Vec(self.x+other.x, self.y+other.y, self.z+other.z)

    def __sub__(self,other):
        return Vec(self.x-other.x, self.y-other.y, self.z-other.z)

    def __mul__(self,x):
        return Vec(self.x*x, self.y*x, self.z*x)

    def __truediv__(self,x):
        return Vec(self.x/x, self.y/x, self.z/zx)

    def __str__(self):      #pour print
        return f"P({self.x},{self.y},{self.z})"


def base():        #point de base
    point0 = Point(0, 0, 0, gx, gy, gz, 0, 0, 0)        #le faite que l'acceleration et la vitesse soie a 0 signifie que je dois commencer ma mesure immobile

def nextacc():     #prochain point accelerometre gyroscope
    pass


def nextgps():     #verification + plus tard regrouper avecc accelerometre et gyro
    pass


def main():
    pass









if __name__ == '__main__':

    main()



