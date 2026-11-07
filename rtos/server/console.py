# one terminal for everything: shows the robots imu data, anything else it sends us,
# and whatever you type gets sent to the robot
# usage: .venv/bin/python console.py   (stop server.py first since they use the same port)

import asyncio
import json
import sys
import threading
import config

from logger import print_server_message, print_bot_message, print_cv_message
from messages import RegistrationRequestMessage, RegistrationResponseMessage

from websockets.asyncio.server import serve
from websockets.exceptions import ConnectionClosed

# Connected clients as a dictionary of websockets
connected_clients = {}

latest_imu = None  # newest imu reading
imu_count = 0  # how many imu messages came in this second


async def handle_connection(websocket):

    print_server_message("Handling incoming websocket connection")

    # Wait for 5 seconds to receive registration message
    raw_message = await asyncio.wait_for(
        websocket.recv(),
        timeout=5,
    )

    client_id = None

    try:
        # Parse registration message
        registration = RegistrationRequestMessage.from_json(raw_message)

        # Capture client ID and role
        client_id = registration.client_id
        role = registration.role

        # Check if this client is already connected
        if client_id in connected_clients:

            reason = f"Client ID {client_id} already conected"

            print_server_message(reason)

            await websocket.close(
                code=1008,
                reason=reason,
            )
            return

        # Validate role and expected client ID
        if role not in config.VALID_ROLES or config.CLIENT_IDS.get(role) != client_id:

            reason = f"Invalid role ({role}) or client ID ({client_id})"

            print_server_message(reason)

            await websocket.close(
                code=1008,
                reason=reason,
            )
            return

        # Register the client
        connected_clients[client_id] = websocket

        success_message = f"Client {client_id} ({role}) connected"
        print_server_message(success_message)

        # Inform client of successful registration
        response = RegistrationResponseMessage(
            success=True,
            message=success_message,
        )

        await websocket.send(response.to_json())

        # Keep handling messages until disconnection
        async for raw in websocket:
            await process_message(client_id, raw)

    except asyncio.TimeoutError:

        reason = "Registration timed out"

        print_server_message(reason)

        await websocket.close(
            code=1008,
            reason=reason,
        )

    except (ValueError, TypeError) as e:

        reason = f"Registration failed - {e}"
        print_server_message(reason)
        await websocket.close(
            code=1008,
            reason=reason,
        )

    except ConnectionClosed:
        print_server_message("Conncetion closed")
        pass

    finally:
        # Only remove the connection owned by this handler
        if client_id is not None and connected_clients.get(client_id) is websocket:
            del connected_clients[client_id]
            print_server_message(f"Client {client_id} disconnected")


async def process_message(client_id, raw):
    # TODO: Parse and route messages.

    role = config.ROLE_BY_CLIENT_ID.get(client_id)

    match role:
        case _ if config.VALID_ROLES[0]:
            print_bot_message(raw)

        case _ if config.VALID_ROLES[1]:
            print_cv_message(raw)

        case _:
            print_server_message(f"Message from unknown client {client_id}: {raw}")


async def main():
    async with serve(handle_connection, "0.0.0.0", 8765) as server:
        print_server_message("Server listening on port 8765")
        await server.serve_forever()


if __name__ == "__main__":
    asyncio.run(main())


# async def handle_robot(ws):
#     global robot, latest_imu, imu_count
#     robot = ws
#     print(f"robot connected from {ws.remote_address[0]}")
#     try:
#         async for msg in ws:
#             if msg.startswith("{"):
#                 latest_imu = msg
#                 imu_count += 1
#             else:
#                 print(f"robot: {msg}")
#     except Exception as e:
#         print(f"connection error: {e}")
#     finally:
#         if robot is ws:
#             robot = None
#         print("robot disconnected, waiting for it to come back...")


# # once a second instead of 50 times a second so you can still read it and type
# async def show_imu():
#     global imu_count
#     while True:
#         await asyncio.sleep(1)
#         if robot is None or latest_imu is None:
#             continue
#         d = json.loads(latest_imu)
#         a, g = d["a"], d["g"]
#         print(
#             f"imu {imu_count}/s   accel {a[0]:7.1f} {a[1]:7.1f} {a[2]:7.1f} mg   "
#             f"gyro {g[0]:6.2f} {g[1]:6.2f} {g[2]:6.2f} deg/s"
#         )
#         imu_count = 0


# async def send_typed_lines():
#     # reading the keyboard blocks so it gets its own thread that hands lines to us
#     loop = asyncio.get_running_loop()
#     lines = asyncio.Queue()

#     def read_keyboard():
#         for line in sys.stdin:
#             loop.call_soon_threadsafe(lines.put_nowait, line.strip())

#     threading.Thread(target=read_keyboard, daemon=True).start()

#     while True:
#         line = await lines.get()
#         if not line:
#             continue
#         if robot is None:
#             print("no robot connected yet")
#             continue
#         await robot.send(line)
#         print(f"sent: {line}")


# async def main():
#     async with serve(handle_robot, "0.0.0.0", PORT):
#         print(
#             f"waiting for the robot on port {PORT}. type a message and hit enter to send it"
#         )
#         await asyncio.gather(show_imu(), send_typed_lines())


# if __name__ == "__main__":
#     try:
#         asyncio.run(main())
#     except KeyboardInterrupt:
#         pass
