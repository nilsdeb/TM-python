import folium
from folium import plugins
import random
import math

# Point de chaque prise de données
class Point:
    # Liste de tous les points créés
    point = []

    def __init__(self, lat, lon, label):
        self.__class__.point.append(self)
        self.lat = lat
        self.lon = lon
        self.label = label


# Fonction pour générer la carte avec les points
def generate_map():
    # Création de la carte centrée sur une position initiale
    m = folium.Map(location=[48.8588377, 2.2770206], zoom_start=12)

    # Ajout des marqueurs pour chaque point
    for point in Point.point:
        folium.Marker(location=[point.lat, point.lon], popup=point.label).add_to(m)

    # Affichage de la carte avec la fonction de clusterisation
    plugins.MarkerCluster().add_to(m)

    # Affichage de la carte
    return m


# Générer un trajet de 100 points sur une distance de 2 km
distance = 2  # en kilomètres
num_points = 100
start_lat = 48.8588377
start_lon = 2.2770206

# Calculer la distance entre chaque point
distance_per_point = distance / num_points

# Générer les points du trajet
for i in range(num_points):
    # Calculer la latitude et la longitude pour chaque point
    d_lat = random.uniform(-1, 1) * distance_per_point / 111.32
    d_lon = random.uniform(-1, 1) * distance_per_point / (111.32 * math.cos(math.radians(start_lat)))
    lat = start_lat + d_lat
    lon = start_lon + d_lon
    label = f"Point {i+1}"

    # Créer un objet Point pour chaque point du trajet
    point = Point(lat, lon, label)

# Générer la carte avec les points du trajet
map = generate_map()

# Enregistrer la carte dans un fichier HTML
map.save("map.html")
