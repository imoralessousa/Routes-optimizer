import sys
import os
from datetime import datetime, timedelta

from ortools.constraint_solver import routing_enums_pb2, pywrapcp

# Añadir ruta al módulo común
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

# Importaciones de módulos propios
from common.ors_client import get_ors_client
from common.geocoding import geocode_address
from common.visualization import mostrar_mapa_interactivo

# Tiempo fijo de servicio en cada parada (en minutos)
SERVICE_TIME_MIN = 10


def parse_time_windows(filepath):
    """
    Lee direcciones con ventanas de tiempo opcionales desde un archivo separado por '|'
    Devuelve: lista de direcciones, lista de ventanas de tiempo en minutos [(inicio, fin)]
    """
    addresses = []
    time_windows = []

    with open(filepath, "r", encoding="utf-8") as f:
        for line in f:
            parts = [p.strip() for p in line.strip().split("|")]
            address = parts[0]

            if len(parts) == 3:
                try:
                    start = datetime.strptime(parts[1], "%H:%M")
                    end = datetime.strptime(parts[2], "%H:%M")
                    start_min = start.hour * 60 + start.minute
                    end_min = end.hour * 60 + end.minute
                    time_windows.append((start_min, end_min))
                except ValueError:
                    print(f"⚠️ Error en el formato de hora en línea: {line}")
                    time_windows.append((0, 24 * 60))
            else:
                time_windows.append((0, 24 * 60))

            addresses.append(address)

    return addresses, time_windows


def create_time_matrix(client, coordinates):
    """
    Devuelve una matriz de tiempos (en minutos) entre todos los puntos.
    """
    matrix = client.distance_matrix(
        locations=coordinates,
        profile='driving-car',
        metrics=['duration'],
        units='m'
    )
    durations_sec = matrix['durations']
    return [[int(val / 60) for val in row] for row in durations_sec]


def main():
    print("🚚 ¿Cuántos vehículos usarás?")
    n_vehicles = int(input("> "))

    print("🏭 Dirección de salida de todos los camiones:")
    depot_address = input("> ")

    print("⏰ ¿A qué hora saldrán del almacén? (formato HH:MM)")
    start_time_str = input("> ")
    start_dt = datetime.strptime(start_time_str, "%H:%M")
    start_minutes = start_dt.hour * 60 + start_dt.minute

    print("📁 Leyendo archivo 'time_windows.txt'...")
    all_addresses, time_windows = parse_time_windows("time_window_vrp/time_windows.txt")

    addresses = [depot_address] + all_addresses
    time_windows = [(start_minutes, 24 * 60)] + time_windows

    print("📍 Geocodificando...")
    client = get_ors_client()
    coordinates = []
    for address in addresses:
        coord = geocode_address(client, address)
        if coord is None:
            print(f"⛔ No se pudo geocodificar: {address}")
            return
        coordinates.append(coord)

    print("🕒 Calculando matriz de tiempos...")
    time_matrix = create_time_matrix(client, coordinates)

    size = len(time_matrix)
    manager = pywrapcp.RoutingIndexManager(size, n_vehicles, 0)
    routing = pywrapcp.RoutingModel(manager)

    def time_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return time_matrix[from_node][to_node] + SERVICE_TIME_MIN

    transit_callback_index = routing.RegisterTransitCallback(time_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    routing.AddDimension(
        transit_callback_index,
        30,
        24 * 60,
        False,
        "Time"
    )
    time_dimension = routing.GetDimensionOrDie("Time")

    for i, (start, end) in enumerate(time_windows):
        index = manager.NodeToIndex(i)
        time_dimension.CumulVar(index).SetRange(start, end)

    search_params = pywrapcp.DefaultRoutingSearchParameters()
    search_params.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    search_params.local_search_metaheuristic = routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    search_params.time_limit.seconds = 30

    print("🚚 Calculando rutas...")
    solution = routing.SolveWithParameters(search_params)
    if not solution:
        print("❌ No se encontró solución.")
        return

    rutas = []
    horas_llegada = []

    for vehicle_id in range(n_vehicles):
        index = routing.Start(vehicle_id)
        print(f"\n🛻 Ruta Vehículo {vehicle_id + 1}:")

        ruta = []
        horas = []
        previous_arrival = None
        previous_address = None

        while not routing.IsEnd(index):
            node_index = manager.IndexToNode(index)
            ruta.append(node_index)
            address = addresses[node_index]

            time_var = time_dimension.CumulVar(index)
            arrival_min = solution.Value(time_var)
            arrival_dt = start_dt + timedelta(minutes=(arrival_min - start_minutes))
            hora_llegada = arrival_dt.strftime("%H:%M")
            horas.append(hora_llegada)

            print(f" - {address} (🕒 Llega a las {hora_llegada})")
            if previous_arrival is not None:
                travel_time = arrival_min - previous_arrival - SERVICE_TIME_MIN
                print(f"     ⏱️ Tiempo desde '{previous_address}': {travel_time} min + {SERVICE_TIME_MIN} min de servicio")

            previous_arrival = arrival_min
            previous_address = address
            index = solution.Value(routing.NextVar(index))

        node_index = manager.IndexToNode(index)
        ruta.append(node_index)
        address = addresses[node_index]
        time_var = time_dimension.CumulVar(index)
        arrival_min = solution.Value(time_var)
        arrival_dt = start_dt + timedelta(minutes=(arrival_min - start_minutes))
        hora_llegada = arrival_dt.strftime("%H:%M")
        horas.append(hora_llegada)
        print(f" - {address} (🕒 Llega a las {hora_llegada})")
        if previous_arrival is not None:
            travel_time = arrival_min - previous_arrival - SERVICE_TIME_MIN
            print(f"     ⏱️ Tiempo desde '{previous_address}': {travel_time} min + {SERVICE_TIME_MIN} min de servicio")

        rutas.append(ruta)
        horas_llegada.append(horas)

    # 🔄 Preparar datos para visualización
    rutas_coordenadas = []
    ubicaciones_por_ruta = []
    coordenadas_por_ruta = []

    for ruta in rutas:
        rutas_coordenadas.append([[coord[1], coord[0]] for coord in [coordinates[nodo] for nodo in ruta]])
        ubicaciones_por_ruta.append([addresses[nodo] for nodo in ruta])
        coordenadas_por_ruta.append([[coord[1], coord[0]] for coord in [coordinates[nodo] for nodo in ruta]])

    nombres_rutas = [f"Vehículo {i + 1}" for i in range(n_vehicles)]
    colores = ['red', 'blue', 'green', 'purple', 'orange', 'darkred', 'lightblue'][:n_vehicles]

    # 🗺️ Mostrar mapa
    mostrar_mapa_interactivo(
        rutas=rutas_coordenadas,
        nombres=nombres_rutas,
        colores=colores,
        ubicaciones=ubicaciones_por_ruta,
        coordenadas=coordenadas_por_ruta,
        filename="ruta_vehiculos.html"
    )


if __name__ == "__main__":
    main()
