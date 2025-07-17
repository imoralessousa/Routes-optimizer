from dotenv import load_dotenv
import os

def load_api_key():
    load_dotenv()
    api_key = os.getenv("ORS_API_KEY")
    if not api_key:
        raise ValueError("❌ ORS_API_KEY no encontrado en el archivo .env")
    return api_key
