import os

VALID_ROLES = ["bot", "cv"]
CLIENT_IDS = {"bot": "1", "cv": "2"}
ROLE_BY_CLIENT_ID = {client_id: role for role, client_id in CLIENT_IDS.items()}
HOST = os.getenv("WS_HOST", "127.0.0.1")
PORT = int(os.getenv("WS_PORT", "8765"))
