import folium
from folium.plugins import MarkerCluster

# Classe pour les points relatifs en 3D
class RelativePoint3D:
    def __init__(self, dx, dy, dz, label):
        self.dx = dx
        self.dy = dy
        self.dz = dz
        self.label = label

# Coordonnées du point GPS de référence
ref_gps_lat = 48.8588377
ref_gps_lon = 2.2770206

# Liste des points relatifs en 3D
relative_points_3d = [
    RelativePoint3D(0, 0, 0, "Point 1"),
    RelativePoint3D(10, 5, 2, "Point 2"),
    RelativePoint3D(15, -3, 1, "Point 3"),
    # Ajoutez d'autres points ici...
]

# Création de la carte centrée sur le point GPS de référence
m = folium.Map(location=[ref_gps_lat, ref_gps_lon], zoom_start=15)

# Ajout du marqueur pour le point GPS de référence
folium.Marker(location=[ref_gps_lat, ref_gps_lon], popup="Référence GPS").add_to(m)

# Relier les points relatifs à leurs positions GPS respectives en 3D
for point in relative_points_3d:
    gps_lat = ref_gps_lat + point.dx
    gps_lon = ref_gps_lon + point.dy
    folium.Marker(location=[gps_lat, gps_lon], popup=point.label, icon=folium.Icon(color='blue', icon='cube')).add_to(m)

# Affichage de la carte avec le cluster de marqueurs
marker_cluster = MarkerCluster().add_to(m)

# Affichage de la carte
m.save('map.html')
