import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from common.ors_client import get_ors_client
from common.geocoding import geocode_address
from common.visualization import mostrar_mapa_interactivo
from ortools.constraint_solver import routing_enums_pb2, pywrapcp

# 💬 Pedir datos al usuario
def get_input_data():
    print("🚚 ¿Cuántos vehículos usarás?")
    n_vehicles = int(input("> "))

    print("🏭 Introduce dirección de salida (inicio/fin de todos los vehículos):")
    depot = input("> ")

    print("\n📁 Leyendo direcciones desde archivo 'destinations.txt'...")
    try:
        with open("vrp/destinations.txt", "r", encoding="utf-8") as f:
            locations = [line.strip() for line in f if line.strip()]
        print(f"✔️ Se cargaron {len(locations)} direcciones.")
    except FileNotFoundError:
        print("❌ No se encontró el archivo 'destinations.txt'. Asegúrate de que existe.")
        return None, None, None

    return n_vehicles, depot, locations


# 📏 Matriz de distancias
def get_distance_matrix(client, coordinates):
    matrix = client.distance_matrix(
        locations=coordinates,
        profile='driving-car',
        metrics=['distance'],
        units='m'
    )
    return matrix['distances']


# 🧠 Resolver problema VRP
def solve_vrp(matrix, n_vehicles, depot_index):
    size = len(matrix)
    manager = pywrapcp.RoutingIndexManager(size, n_vehicles, depot_index)
    routing = pywrapcp.RoutingModel(manager)

    def distance_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return int(matrix[from_node][to_node])

    transit_index = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_index)

    # ✅ Añadir dimensión de distancia para balancear rutas
    routing.AddDimension(
        transit_index,
        0,                # sin slack
        10_000_000,       # límite muy alto (10.000 km)
        True,             # inicia en cero
        "Distance"
    )

    distance_dimension = routing.GetDimensionOrDie("Distance")

    # ✅ Esto fuerza a equilibrar las rutas (coste global por diferencia)
    distance_dimension.SetGlobalSpanCostCoefficient(100)

    # ❌ NO añadimos penalizaciones, obligamos a visitar todos
    # Esto hace que todos los puntos sean asignados sí o sí

    # ✅ Búsqueda
    search_params = pywrapcp.DefaultRoutingSearchParameters()
    search_params.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    )
    search_params.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    )
    search_params.time_limit.seconds = 30

    solution = routing.SolveWithParameters(search_params)
    if not solution:
        print("❌ No se encontró solución.")
        return []

    # ✅ Extraer rutas
    routes = []
    for vehicle_id in range(n_vehicles):
        index = routing.Start(vehicle_id)
        route = []
        while not routing.IsEnd(index):
            route.append(manager.IndexToNode(index))
            index = solution.Value(routing.NextVar(index))
        route.append(manager.IndexToNode(index))
        routes.append(route)

    return routes



# 🧭 Ruta y duración por vehículo
def get_vehicle_routes_geometries(client, full_coords, routes):
    geometries = []
    durations = []
    distances_km = []

    for route in routes:
        coords = [full_coords[i] for i in route]
        response = client.directions(coords, profile='driving-car', format='geojson')
        
        features = response.get('features', [])
        if not features:
            print("❌ No se recibieron características para esta ruta.")
            geometries.append([])
            durations.append(0)
            distances_km.append(0)
            continue
        
        properties = features[0].get('properties', {})
        summary = properties.get('summary', {})
        duration_sec = summary.get('duration', 0)
        distance_m = summary.get('distance', 0)
        
        geometry = features[0].get('geometry', {}).get('coordinates', [])
        latlng = [[lat, lng] for lng, lat in geometry] if geometry else []

        geometries.append(latlng)
        durations.append(duration_sec / 60 if duration_sec else 0)
        distances_km.append(distance_m / 1000 if distance_m else 0)

    return geometries, durations, distances_km


# ▶️ MAIN
def main():
    client = get_ors_client()
    n_vehicles, depot, locs = get_input_data()
    if not all([n_vehicles, depot, locs]):
        return
    addresses = [depot] + locs

    print("\n📍 Geocodificando direcciones...")
    coordinates = []
    for addr in addresses:
        coord = geocode_address(client, addr)
        if coord is None:
            print("⛔ Abortando.")
            return
        coordinates.append(coord)

    print("📐 Calculando matriz de distancias...")
    matrix = get_distance_matrix(client, coordinates)

    for i, row in enumerate(matrix):
        for j, val in enumerate(row):
            if val is None:
                print(f"⛔ Distancia inválida entre {i} y {j}")
                return

    print("🚚 Calculando rutas para vehículos...")
    routes = solve_vrp(matrix, n_vehicles, depot_index=0)

    colors = ["blue", "green", "red", "purple", "orange", "darkred", "cadetblue", "black"]
    route_geoms, durations, distances_km = get_vehicle_routes_geometries(client, coordinates, routes)

    for i, (r, d, km) in enumerate(zip(routes, durations, distances_km)):
        print(f"\n🛻 Ruta Vehículo {i+1} (Duración: {d:.1f} min | Distancia: {km:.2f} km):")
        for idx in r:
            print(f" - {addresses[idx]}")

    # Construir ubicaciones y coordenadas por ruta para mostrar marcadores correctos
    ubicaciones_por_ruta = []
    coordenadas_por_ruta = []
    for route in routes:
        ubicacion_ruta = [addresses[idx] for idx in route]
        coord_ruta = [coordinates[idx] for idx in route]
        ubicaciones_por_ruta.append(ubicacion_ruta)
        coordenadas_por_ruta.append(coord_ruta)

    mostrar_mapa_interactivo(
        rutas=route_geoms,
        nombres=[f"Vehículo {i+1}" for i in range(len(route_geoms))],
        colores=colors,
        ubicaciones=ubicaciones_por_ruta,
        coordenadas=coordenadas_por_ruta
    )

if __name__ == "__main__":
    main()
