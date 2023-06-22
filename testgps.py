import folium

# Point GPS de départ
gps_lat = 48.859693
gps_lon = 2.347237

# Accélérations
accelerations = [
    (1.5, 0.2, -0.5),
    (0.8, -0.3, 0.9),
    (-0.7, 0.5, -1.2),
    (0.3, -0.6, 1.4),
    (-1.2, 0.7, -0.9),
    (0.5, -0.4, 0.3),
    (-0.6, 0.3, -0.8),
    (0.9, -1.2, 0.5),
    (-0.4, 0.5, -0.3),
    (0.3, -0.2, 0.1),
    (0.5, -0.6, 0.9),
    (-0.8, 1.0, -0.7),
    (0.6, -0.4, 0.5),
    (-0.3, 0.1, -0.2),
    (0.7, -0.5, 0.6),
    (-0.4, 0.3, -0.5),
    (0.2, -0.1, 0.3),
    (-0.5, 0.4, -0.2),
    (0.3, -0.3, 0.4),
    (-0.1, 0.2, -0.3)
]

# Liste des positions
positions = [(gps_lat, gps_lon)]
for acceleration in accelerations:
    lat = positions[-1][0] + acceleration[0] * 0.5
    lon = positions[-1][1] + acceleration[1] * 0.5
    positions.append((lat, lon))

# Création de la carte
m = folium.Map(location=[gps_lat, gps_lon], zoom_start=15)

# Trajet
folium.PolyLine(locations=positions, color='blue', weight=2.5, opacity=1).add_to(m)

# Affichage de la carte
m.save('trajet.html')
