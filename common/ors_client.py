import openrouteservice
from common.config import load_api_key

def get_ors_client():
    api_key = load_api_key()
    return openrouteservice.Client(key=api_key)
