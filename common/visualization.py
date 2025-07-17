import folium
import os
import webbrowser

def mostrar_mapa_interactivo(rutas, nombres, colores, ubicaciones, coordenadas, filename="mapa_ruta.html"):
    """
    rutas: lista de listas de coordenadas [[lat, lng]]
    nombres: lista de nombres para cada ruta
    colores: lista de colores para cada ruta
    ubicaciones: lista de nombres originales de puntos a visitar (en orden)
    coordenadas: lista de coordenadas originales [[lat, lng]] en el mismo orden que ubicaciones
    """
    if not rutas:
        print("❌ No se proporcionaron rutas para el mapa.")
        return

    # Crear mapa centrado en el punto inicial
    mapa = folium.Map(location=rutas[0][0], zoom_start=6)

    for i, ruta in enumerate(rutas):

        # Línea de ruta
        folium.PolyLine(
            locations=ruta,
            color=colores[i % len(colores)],
            weight=4,
            opacity=0.8,
            tooltip=nombres[i]
        ).add_to(mapa)

        nombres_puntos = ubicaciones[i]
        coords_puntos = coordenadas[i]

        # Añadir marcadores sólo en las ubicaciones originales
        for j, (nombre_punto, coord) in enumerate(zip(nombres_puntos, coords_puntos)):
            icon_color = (
                'green' if j == 0 else
                'red' if j == len(nombres_puntos) - 1 else
                'blue'
            )
            folium.Marker(
                location=[coord[1], coord[0]],
                tooltip=nombre_punto,
                popup=nombre_punto,
                icon=folium.Icon(color=icon_color)
            ).add_to(mapa)
            
    mapa.save(filename)
    webbrowser.open(f"file://{os.path.abspath(filename)}")
