def geocode_address(client, address):
    geocode = client.pelias_search(text=address)
    try:
        coords = geocode['features'][0]['geometry']['coordinates']
        return coords  # [lng, lat]
    except (IndexError, KeyError):
        print(f"❌ Dirección no encontrada: {address}")
        return None
