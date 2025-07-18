import sys
import os
from datetime import datetime, timedelta
from ortools.constraint_solver import routing_enums_pb2, pywrapcp

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from common.ors_client import get_ors_client
from common.geocoding import geocode_address
from common.visualization import mostrar_mapa_interactivo

SERVICE_TIME_MIN = 0  # minutos

def read_time_window_addresses(filename):
    addresses = []
    windows = []
    with open(filename, 'r', encoding='utf-8') as f:
        for line in f:
            parts = [p.strip() for p in line.strip().split('|')]
            addr = parts[0]
            if len(parts) == 3:
                start = datetime.strptime(parts[1], "%H:%M")
                end = datetime.strptime(parts[2], "%H:%M")
                minutes_start = start.hour * 60 + start.minute
                minutes_end = end.hour * 60 + end.minute
            else:
                minutes_start = 0
                minutes_end = 24 * 60
            addresses.append(addr)
            windows.append((minutes_start, minutes_end))
    return addresses, windows

def get_time_matrix(client, coords):
    matrix = client.distance_matrix(
        locations=coords,
        profile='driving-car',
        metrics=['duration'],
        units='m'
    )
    return [[int(cell / 60) for cell in row] for row in matrix['durations']]  # en minutos

def solve_time_window_tsp(time_matrix, time_windows, start_time_min):
    size = len(time_matrix)
    manager = pywrapcp.RoutingIndexManager(size, 1, [0], [size - 1])
    routing = pywrapcp.RoutingModel(manager)

    def time_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return time_matrix[from_node][to_node] + SERVICE_TIME_MIN

    transit_callback_index = routing.RegisterTransitCallback(time_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Crear dimensión de tiempo
    routing.AddDimension(
        transit_callback_index,
        720,           # Slack máximo
        1440,         # Tiempo máximo (24h)
        False,        # No force start cumul to zero
        "Time"
    )

    time_dimension = routing.GetDimensionOrDie("Time")

    # Forzar que la salida ocurra exactamente a la hora inicial
    start_index = routing.Start(0)
    time_dimension.CumulVar(start_index).SetValue(start_time_min)

    # Aplicar ventanas de tiempo
    for location_idx, (start, end) in enumerate(time_windows):
        if location_idx == 0 or location_idx == len(time_windows) - 1:
            continue  # No aplicar a inicio ni fin
        index = manager.NodeToIndex(location_idx)
        time_dimension.CumulVar(index).SetRange(start, end)

    # Minimizar tiempo final
    routing.AddVariableMinimizedByFinalizer(time_dimension.CumulVar(routing.Start(0)))
    routing.AddVariableMinimizedByFinalizer(time_dimension.CumulVar(routing.End(0)))

    search_params = pywrapcp.DefaultRoutingSearchParameters()
    search_params.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    search_params.local_search_metaheuristic = routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    search_params.time_limit.seconds = 30

    solution = routing.SolveWithParameters(search_params)

    if not solution:
        return None, None

    route = []
    arrival_times = []

    index = routing.Start(0)
    while not routing.IsEnd(index):
        node = manager.IndexToNode(index)
        route.append(node)
        time_var = time_dimension.CumulVar(index)
        arrival_times.append(solution.Value(time_var))
        index = solution.Value(routing.NextVar(index))

    # Añadir nodo final
    node = manager.IndexToNode(index)
    route.append(node)
    time_var = time_dimension.CumulVar(index)
    arrival_times.append(solution.Value(time_var))

    return route, arrival_times


def main():
    client = get_ors_client()

    print("📍 Dirección de salida:")
    start_address = input("> ").strip()

    print("🏁 Dirección de llegada:")
    end_address = input("> ").strip()

    print("⏰ ¿A qué hora comienza la ruta? (formato HH:MM)")
    start_time_str = input("> ")
    start_dt = datetime.strptime(start_time_str, "%H:%M")
    start_min = start_dt.hour * 60 + start_dt.minute

    print("📁 Leyendo archivo 'time_windows.txt'...")
    middle_addresses, middle_windows = read_time_window_addresses("time_window_tsp/time_windows.txt")

    addresses = [start_address] + middle_addresses + [end_address]
    time_windows = [(start_min, 24 * 60)] + middle_windows + [(start_min, 24 * 60)]

    print("📍 Geocodificando direcciones...")
    coordinates = []
    for addr in addresses:
        coord = geocode_address(client, addr)
        if coord is None:
            print(f"❌ Error geocodificando {addr}")
            return
        coordinates.append(coord)

    print("🕒 Calculando matriz de tiempos...")
    time_matrix = get_time_matrix(client, coordinates)

    print("🧠 Resolviendo TSP con ventanas temporales...")
    route, arrival_times = solve_time_window_tsp(time_matrix, time_windows, start_min)

    if route is None:
        print("❌ No se pudo encontrar una solución.")
        return

    print("\n✅ Orden óptimo de visita:")
    ordered_addresses = []
    ordered_coords = []
    horas_llegada = []

    for i, idx in enumerate(route):
        addr = addresses[idx]
        coord = coordinates[idx]
        arrival_dt = start_dt + timedelta(minutes=(arrival_times[i] - start_min))
        hora_str = arrival_dt.strftime("%H:%M")

        if i == 0:
            label = f"📍 Inicio: {addr}"
        elif i == len(route) - 1:
            label = f"🏁 Fin: {addr}"
        else:
            label = f"🔹 {addr}"

        print(f" - {label} (🕒 {hora_str})")
        ordered_addresses.append(addr)
        ordered_coords.append(coord)
        horas_llegada.append(hora_str)


    # Obtener ruta real
    response = client.directions(ordered_coords, profile='driving-car', format='geojson')
    geometry = response['features'][0]['geometry']['coordinates']
    folium_coords = [[lat, lng] for lng, lat in geometry]

    print(f"\n🗺️ Mostrando mapa interactivo...")

    mostrar_mapa_interactivo(
        rutas=[folium_coords],
        nombres=["Ruta con ventanas temporales"],
        colores=["blue"],
        ubicaciones=[ordered_addresses],
        coordenadas=[[ [c[0], c[1]] for c in ordered_coords ]]
    )

if __name__ == "__main__":
    main()
