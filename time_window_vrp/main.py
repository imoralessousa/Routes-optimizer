import sys
import os
import datetime
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from ortools.constraint_solver import routing_enums_pb2, pywrapcp
from common.ors_client import get_ors_client
from common.geocoding import geocode_address
from common.visualization import mostrar_mapa_interactivo


SERVICE_TIME_MIN = 5  # Tiempo de parada por destino en minutos
MAX_MINUTES = 1440    # Horizonte de planificación (1 día)


def parse_time_window_line(line):
    parts = line.strip().split('|')
    address = parts[0].strip()
    if len(parts) == 3:
        fmt = "%H:%M"
        start = parts[1].strip()
        end = parts[2].strip()
        start_minutes = int(datetime.datetime.strptime(start, fmt).hour) * 60 + int(datetime.datetime.strptime(start, fmt).minute)
        end_minutes = int(datetime.datetime.strptime(end, fmt).hour) * 60 + int(datetime.datetime.strptime(end, fmt).minute)
    else:
        start_minutes = 0
        end_minutes = MAX_MINUTES
    return address, start_minutes, end_minutes


def load_time_window_data(file_path):
    addresses = []
    time_windows = []
    with open(file_path, "r", encoding="utf-8") as f:
        for line in f:
            if line.strip():
                addr, start, end = parse_time_window_line(line)
                addresses.append(addr)
                time_windows.append((start, end))
    return addresses, time_windows


def get_time_matrix(client, coordinates):
    matrix = client.distance_matrix(
        locations=coordinates,
        profile='driving-car',
        metrics=['duration'],
        units='m'
    )
    return matrix['durations']


def solve_vrp_with_time_windows(time_matrix, time_windows, num_vehicles, depot_index, salida_minutos):
    size = len(time_matrix)
    manager = pywrapcp.RoutingIndexManager(size, num_vehicles, depot_index)
    routing = pywrapcp.RoutingModel(manager)

    def time_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return int(time_matrix[from_node][to_node]) + SERVICE_TIME_MIN  # duración + tiempo de servicio

    transit_callback_index = routing.RegisterTransitCallback(time_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    routing.AddDimension(
        transit_callback_index,
        60,             # slack
        MAX_MINUTES,    # máximo tiempo total
        False,          # no inicia en cero (lo fijamos nosotros)
        "Time"
    )
    time_dimension = routing.GetDimensionOrDie("Time")

    for location_idx, (open_time, close_time) in enumerate(time_windows):
        index = manager.NodeToIndex(location_idx)
        time_dimension.CumulVar(index).SetRange(open_time, close_time)

    # Salida fija desde almacén
    for vehicle_id in range(num_vehicles):
        start_index = routing.Start(vehicle_id)
        time_dimension.CumulVar(start_index).SetRange(salida_minutos, salida_minutos)

    # Penalización para poder saltarse puntos si no es viable
    for i in range(size):
        routing.AddDisjunction([manager.NodeToIndex(i)], 100000)

    # Parámetros de búsqueda
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
    search_parameters.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH)
    search_parameters.time_limit.seconds = 30

    solution = routing.SolveWithParameters(search_parameters)
    if not solution:
        print("❌ No se encontró solución.")
        return []

    # Extraer rutas
    routes = []
    for vehicle_id in range(num_vehicles):
        index = routing.Start(vehicle_id)
        route = []
        while not routing.IsEnd(index):
            route.append(manager.IndexToNode(index))
            index = solution.Value(routing.NextVar(index))
        route.append(manager.IndexToNode(index))
        routes.append(route)
    return routes


def main():
    client = get_ors_client()

    print("🚚 ¿Cuántos vehículos usarás?")
    num_vehicles = int(input("> "))

    print("🏭 Dirección de salida de todos los camiones:")
    depot_address = input("> ")

    print("⏰ ¿A qué hora saldrán del almacén? (formato HH:MM)")
    hora_salida = input("> ").strip()
    h, m = map(int, hora_salida.split(":"))
    salida_minutos = h * 60 + m

    print("📁 Leyendo archivo 'time_windows.txt'...")
    addresses, time_windows = load_time_window_data("time_window_vrp/time_windows.txt")

    all_addresses = [depot_address] + addresses
    all_time_windows = [(salida_minutos, MAX_MINUTES)] + time_windows

    print("📍 Geocodificando...")
    coordinates = []
    for addr in all_addresses:
        coord = geocode_address(client, addr)
        if coord is None:
            print(f"⛔ Error en geocodificación: {addr}")
            return
        coordinates.append(coord)

    print("🕒 Calculando matriz de tiempos...")
    time_matrix = get_time_matrix(client, coordinates)

    print("🚚 Calculando rutas...")
    routes = solve_vrp_with_time_windows(time_matrix, all_time_windows, num_vehicles, depot_index=0, salida_minutos=salida_minutos)

    if not routes:
        return

    colors = ["blue", "green", "red", "purple", "orange", "darkred", "cadetblue", "black"]

    for i, route in enumerate(routes):
        print(f"\n🛻 Ruta Vehículo {i+1}:")
        for idx in route:
            addr = all_addresses[idx]
            window = all_time_windows[idx]
            print(f" - {addr} (⏰ {window[0]//60:02d}:{window[0]%60:02d} - {window[1]//60:02d}:{window[1]%60:02d})")

    # Visualización opcional
    rutas_geometria = []
    ubicaciones_por_ruta = []
    coordenadas_por_ruta = []

    for route in routes:
        coords = [coordinates[idx] for idx in route]
        response = client.directions(coords, profile='driving-car', format='geojson')
        geometry = response['features'][0]['geometry']['coordinates']
        latlng = [[lat, lng] for lng, lat in geometry]
        rutas_geometria.append(latlng)
        ubicaciones_por_ruta.append([all_addresses[idx] for idx in route])
        coordenadas_por_ruta.append(coords)

    mostrar_mapa_interactivo(
        rutas=rutas_geometria,
        nombres=[f"Vehículo {i+1}" for i in range(len(routes))],
        colores=colors,
        ubicaciones=ubicaciones_por_ruta,
        coordenadas=coordenadas_por_ruta
    )


if __name__ == "__main__":
    main()
