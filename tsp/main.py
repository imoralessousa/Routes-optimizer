import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from common.ors_client import get_ors_client
from common.geocoding import geocode_address
from common.visualization import mostrar_mapa_interactivo
from ortools.constraint_solver import routing_enums_pb2, pywrapcp

# 💬 Pedir direcciones
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

# 📏 Matriz de distancias
def get_distance_matrix(client, locations):
    matrix = client.distance_matrix(
        locations=locations,
        profile='driving-car',
        metrics=['distance'],
        units='m'
    )
    return matrix['distances']

# 🧠 Resolver TSP
def solve_tsp(matrix):
    size = len(matrix)
    manager = pywrapcp.RoutingIndexManager(size, 1, [0], [size - 1])
    routing = pywrapcp.RoutingModel(manager)

    def distance_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return int(matrix[from_node][to_node])

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    search_params = pywrapcp.DefaultRoutingSearchParameters()
    search_params.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC

    solution = routing.SolveWithParameters(search_params)

    if not solution:
        print("❌ No se encontró solución")
        return

    index = routing.Start(0)
    route = []
    while not routing.IsEnd(index):
        route.append(manager.IndexToNode(index))
        index = solution.Value(routing.NextVar(index))
    route.append(manager.IndexToNode(index))

    return route

# 🧭 Obtener ruta real y duración
def get_full_route_geometry(client, coordinates):
    response = client.directions(coordinates, profile='driving-car', format='geojson')
    geometry = response['features'][0]['geometry']['coordinates']
    duration_sec = response['features'][0]['properties']['summary']['duration']
    duration_min = duration_sec / 60
    return [[lat, lng] for lng, lat in geometry], duration_min

# ▶️ MAIN
def main():
    client = get_ors_client()
    start, middle, end = get_addresses()
    addresses = [start] + middle + [end]

    print("\n📍 Geocodificando direcciones...")
    coordinates = []
    for addr in addresses:
        coord = geocode_address(client, addr)
        if coord is None:
            print("⛔ Abortando por error en geocodificación.")
            return
        coordinates.append(coord)

    print("📐 Calculando matriz de distancias...")
    matrix = get_distance_matrix(client, coordinates)

    for i, row in enumerate(matrix):
        for j, dist in enumerate(row):
            if dist is None:
                print(f"⛔ Distancia inválida entre {i} y {j}")
                return

    print("🧮 Calculando ruta óptima...")
    route = solve_tsp(matrix)

    print("\n✅ Orden óptimo de direcciones:")
    for idx in route:
        label = "A (Inicio)" if idx == 0 else "B (Fin)" if idx == len(coordinates) - 1 else f"Punto {idx}"
        print(f" - {label}: {addresses[idx]}")

    ordered_coords = [coordinates[i] for i in route]
    full_route, duration = get_full_route_geometry(client, ordered_coords)

    print(f"\n⏱️  Duración estimada: {duration:.1f} minutos")

    mostrar_mapa_interactivo(
        rutas=[full_route],
        nombres=["Ruta TSP"],
        colores=["blue"],
        ubicaciones=[addresses],
        coordenadas=[ordered_coords]
    )

if __name__ == "__main__":
    main()
