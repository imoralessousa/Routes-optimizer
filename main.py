import openrouteservice
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
from dotenv import load_dotenv
import os
import folium
import webbrowser


load_dotenv()
api_key = os.getenv("ORS_API_KEY")
client = openrouteservice.Client(key=api_key)

# 🌍 Geocodifica una dirección en coordenadas [lng, lat]
def geocode_address(address):
    geocode = client.pelias_search(text=address)
    try:
        coords = geocode['features'][0]['geometry']['coordinates']
        return coords  # [lng, lat]
    except (IndexError, KeyError):
        print(f"❌ Dirección no encontrada: {address}")
        return None

# 💬 Pedir direcciones al usuario
def get_addresses():
    print("🟢 Introduce dirección de salida (A):")
    start = input("> ")

    print("\n🟡 Introduce dirección de llegada (B):")
    end = input("> ")

    print("\n🔵 Introduce direcciones intermedias (una por línea, ENTER vacío para terminar):")
    middle = []
    while True:
        line = input("> ")
        if not line.strip():
            break
        middle.append(line.strip())

    return start, middle, end

# 📏 Obtener matriz de distancias
def get_distance_matrix(locations):
    matrix = client.distance_matrix(
        locations=locations,
        profile='driving-car',
        metrics=['distance'],
        units='m'
    )
    return matrix['distances']

# 🧠 Resolver TSP con origen y destino fijos
def solve_tsp(matrix):
    size = len(matrix)
    manager = pywrapcp.RoutingIndexManager(size, 1, [0], [size-1])  # 1 vehículo, empieza en 0

    routing = pywrapcp.RoutingModel(manager)

    def distance_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return int(matrix[from_node][to_node])

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # 💡 Restringir punto final (último nodo) como destino fijo
    # routing.AddDisjunction([manager.NodeToIndex(size - 1)], 0)
    # routing.SetFixedCostOfAllVehicles(0)

    search_params = pywrapcp.DefaultRoutingSearchParameters()
    search_params.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC

    solution = routing.SolveWithParameters(search_params)

    if not solution:
        print("❌ No se encontró solución")
        return

    index = routing.Start(0)
    route = []
    while not routing.IsEnd(index):
        node = manager.IndexToNode(index)
        route.append(node)
        index = solution.Value(routing.NextVar(index))
    route.append(manager.IndexToNode(index))

    return route

# Obtener ruta
def get_full_route_geometry(coordinates):
    """
    Pide a ORS la ruta completa (carretera) entre los puntos dados en orden.
    """
    geometry = client.directions(
        coordinates,
        profile='driving-car',
        format='geojson'
    )['features'][0]['geometry']['coordinates']

    # ORS devuelve [lng, lat], pero folium necesita [lat, lng]
    return [[lat, lng] for lng, lat in geometry]

# Mostrar mapa interactivo
def mostrar_mapa_interactivo(coordinates, route, filename="mapa_ruta.html"):
    
    # Obtener coordenadas ordenadas por la ruta óptima
    ordered_coords = [coordinates[i] for i in route]

    # Pedir la ruta real a ORS
    full_route = get_full_route_geometry(ordered_coords)

    # Crear el mapa centrado en el punto inicial
    start_coord = ordered_coords[0][::-1]
    mapa = folium.Map(location=start_coord, zoom_start=6)

    # Añadir marcadores
    for i, coord in enumerate(ordered_coords):
        latlon = coord[::-1]
        label = "A (Inicio)" if i == 0 else "B (Fin)" if i == len(ordered_coords) - 1 else f"Punto {i}"
        folium.Marker(
            location=latlon,
            popup=label,
            tooltip=label,
            icon=folium.Icon(color='green' if i == 0 else 'red' if i == len(ordered_coords) - 1 else 'blue')
        ).add_to(mapa)

    # Dibujar ruta real
    folium.PolyLine(locations=full_route, color="blue", weight=4).add_to(mapa)

    # Guardar y abrir el mapa
    mapa.save(filename)
    print(f"\n🗺️  Mapa interactivo guardado en: {filename}")
    webbrowser.open(f"file://{os.path.abspath(filename)}")


# ▶️ MAIN
def main():
    start_addr, middle_addrs, end_addr = get_addresses()

    # Geocodificar direcciones
    print("\n📍 Geocodificando direcciones...")
    addresses = [start_addr] + middle_addrs + [end_addr]
    coordinates = []

    for addr in addresses:
        coord = geocode_address(addr)
        if coord is None:
            print("⛔ Abortando por error en geocodificación.")
            return
        coordinates.append(coord)

    # Matriz de distancias reales
    print("📐 Calculando matriz de distancias reales...")
    distance_matrix = get_distance_matrix(coordinates)

    # Comprobar si hay None en la matriz de distancias
    for i, row in enumerate(distance_matrix):
        for j, dist in enumerate(row):
            if dist is None:
                print(f"⛔ Distancia inválida (None) detectada entre los puntos {i} y {j}. Abortando.")
                return

    # Optimización
    print("🧮 Calculando ruta óptima...")
    route = solve_tsp(distance_matrix)

    print("\n✅ Orden óptimo de direcciones:")
    for idx in route:
        label = "A (Inicio)" if idx == 0 else "B (Fin)" if idx == len(coordinates) - 1 else f"Punto {idx}"
        print(f" - {label}: {addresses[idx]}")

    # Mostrar mapa
    mostrar_mapa_interactivo(coordinates, route)

if __name__ == "__main__":
    main()