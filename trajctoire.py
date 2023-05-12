import math as m


# position "0" en vitesse, accelereation, position 0 de gps, vecteur acceleration => changement one step => postion un => loop
# position gps confirme celle des accelerometre et gyro / temoin

temp = 0.25     #en seconde

Accx = (1,2,3,4,5,6)       # liste accelerometre

Accy = (1,2,3,4,5,6)

Accz = (1,2,3,4,5,6)

Girx = (1,2,3,4,5,6)       #liste gyroscope

Giry = (1,2,3,4,5,6)

Girz = (1,2,3,4,5,6)


incplat = []   #angle en fonction du temp du plateau de mesure

class Point :       #creation de point pour chaque mesure

    point = []      #liste des points pour pouvoir les compter

    def __init__(self,Vec,Ang,Vecv):    #Vec = vec acceleration/ Vecv = vec vitesse

        self.__class__.point.append(self)       #pour les ajouter a la liste
        self.x = Vec.x      #position
        self.y = Vec.y
        self.z = Vec.z
        self.vx = Vecv.x       #vitesse a se point
        self.vy = Vecv.y
        self.vz = Vecv.z
        self.gx = Ang.x        #postion angulaire dans l'espace
        self.gy = Ang.y
        self.gz = Ang.z
        

    def __str__(self):      #pour print
        return f"P({self.x},{self.y},{self.z},{self.gx},{self.gy},{self.gz},{self.vx},{self.vy},{self.vz})"

class Ang :

    angle = []

    def __init__(self,x,y,z):
        self.__class__.angle.append(self)       #pour les ajouter a la liste
        self.x = x
        self.y = y
        self.z = z

    def __add__(self,other):
        return Ang(self.x+other.x, self.y+other.y, self.z+other.z)

    def __sub__(self,other):
        return Ang(self.x-other.x, self.y-other.y, self.z-other.z)


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

    def retangle(Ang,normeself):      # liste angle (x,y,z) 
        x = m.cos(Ang.x)*normeself
        y = m.cos(Ang.y)*normeself
        z = m.cos(Ang.z)*normeself
        return Vec(x,y,z)       #redonne un vecteur

    def horaire (self,point,ang):    #applique equa horaire
        vecp = Vec(1/2*self.x*temp**2 + point.vx*temp + point.x, 1/2*self.x*temp**2 + point.vy*temp + point.y, 1/2*self.x*temp**2 + point.vz*temp + point.z)
        vecv = Vec(self.x*temp + point.vx, self.y*temp + point.vy, self.z*temp + point.vz)
        return Point(vecp, ang, vecv)


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


def base():     #point de base
    acc0 = Vec(0, 0, 0)
    vit0 = Vec(0, 0, 0)     
    ang0 = Ang(0, 0, 0)
    point0 = Point(acc0,vit0,ang0)        #le faite que l'acceleration et la vitesse soie a 0 signifie que je dois commencer ma mesure immobile et avec un angle le plus plat possible

def nextacc():     #prochain point accelerometre gyroscope   graviter comment faire?
    ndp = len(Point.point)       #nombre de point, len commence a 1 vu que il y a le point de base
    angledif = Ang(Girx[ndp]-Girx[ndp-1],Giry[ndp]-Giry[ndp-1],Girz[ndp]-Girz[ndp-1])-gir0  #corresction d'angle par raport a la mesure d'avant
    accel = Vec(Accx[ndp],Accy[ndp],Accz[ndp])      #vecteur brut des donnée d'accelerometre
    norme = Vec.norme(accel)
    ang = Vec.angle(accel, norme)
    ang2 = ang-angledif   #compense la rotation du capteur sur l'acceleration
    newvec = Vec.retangle(ang2, norme)   #vecteur force corriger
    #comment chercher l'angle du point d'avant?


    




def nextgps():     #verification + plus tard regrouper avecc accelerometre et gyro
    pass


def main():
    base()
    nextacc()
    nextacc()
    nextacc()
    nextacc()











if __name__ == '__main__':

    main()



