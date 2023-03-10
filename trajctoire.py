import math as m


# position "0" en vitesse, accelereation, position 0 de gps, vecteur acceleration, vecteur du gps(attention 2d) => changement one step => postion un => loop
# position gps confirme celle des accelerometre et gyro



Accx = (1,2,3,4,5,6)       # liste accelerometre

Accy = (1,2,3,4,5,6)

Accz = (1,2,3,4,5,6)

Girx = (1,2,3,4,5,6)       #liste gyroscope

Giry = (1,2,3,4,5,6)

Girz = (1,2,3,4,5,6)

Long = (1,2,3,4,5,6)       #coordonnée gps

Lati = (1,2,3,4,5,6)

girx0 = Girx[0]   #pour avoir l'angle de base qui mets a 0 tout les gyro

giry0 = Giry[0]

girz0 = Girz[0]

incplat = ()    #angle en fonction du temp du plateau de mesure

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

class Ang :
    def __init__(self,x,y,z):
        self.x = x
        self.y = y
        self.z = z

    def __add__(self,other):
        return Vec(self.x+other.x, self.y+other.y, self.z+other.z)

    def __sub__(self,other):
        return Vec(self.x-other.x, self.y-other.y, self.z-other.z)


    


class Vec :
    def __init__(self,x,y,z):
        self.x = x
        self.y = y
        self.z = z      
    def norme(other):   #norme d'un vecteur
        return m.sqrt(other.x**2+other.y**2+other.z**2)

    def angle(self,normeself):
        angx = m.acos(self.x/normeself)     #x sur la norme qui est l'hypotenuse
        angy = m.acos(self.y/normeself)
        angz = m.acos(self.z/normeself)
        return Ang(angx,angy,angz)     #angle x,y,z

    def retangle(normeself,Ang):      # liste angle (x,y,z) 
        x = m.cos(Ang.x)*normeself
        y = m.cos(Ang.y)*normeself
        z = m.cos(Ang.z)*normeself
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
    point0 = Point(0, 0, 0, 0, 0, 0, 0, 0, 0)        #le faite que l'acceleration et la vitesse soie a 0 signifie que je dois commencer ma mesure immobile et avec un angle le plus plat possible

def nextacc():     #prochain point accelerometre gyroscope   graviter comment faire?
    ndp = len(Point.point)       #nombre de point, len commence a 1 vu que il y a le point de base
    angledif = Ang(Girx[ndp]-girx0-Girx[ndp-1],Giry[ndp]-giry0-Giry[ndp-1],Girz[ndp]-girz0-Girz[ndp-1])   #corresction d'angle par raport a la mesure d'avant
    accel = Vec(Accx[ndp],Accy[ndp],Accz[ndp])      #vecteur brut des donnée d'accelerometre
    norme = norme(accel)
    ang = angle(acce, norme)

def nextgps():     #verification + plus tard regrouper avecc accelerometre et gyro
    pass


def main():
    base()
    nextacc()










if __name__ == '__main__':

    main()



