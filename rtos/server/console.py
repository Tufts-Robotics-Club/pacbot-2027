# one terminal for everything: live imu/encoder panel, a scrolling log of everything the
# robot and cv send us, and a command box at the bottom that sends to the robot
# usage: .venv/bin/python console.py   (stop server.py first since they use the same port)
# keys: F1 stop (brake), F2 motors off (coast), ctrl+l clear log, ctrl+q quit

import asyncio
import json

import config
import logger
from logger import (
    print_server_message,
    print_bot_message,
    print_cv_message,
    print_sent_message,
)
from messages import RegistrationRequestMessage, RegistrationResponseMessage

from rich.text import Text
from textual.app import App, ComposeResult
from textual.binding import Binding
from textual.widgets import Footer, Header, Input, RichLog, Static
from websockets.asyncio.server import serve
from websockets.exceptions import ConnectionClosed

BOT_ID = config.CLIENT_IDS["bot"]
CV_ID = config.CLIENT_IDS["cv"]

# Connected clients as a dictionary of websockets
connected_clients = {}

# high rate telemetry goes to the panel instead of the log so the log stays readable
latest_imu = None  # newest imu reading as a dict
imu_count = 0  # imu messages since the last rate update
imu_rate = 0  # imu messages per second
latest_enc = None  # newest "enc L: .. R: .." line


async def handle_connection(websocket):

    print_server_message("Handling incoming websocket connection")

    client_id = None

    try:
        # Wait for 5 seconds to receive registration message
        raw_message = await asyncio.wait_for(
            websocket.recv(),
            timeout=5,
        )

        # Parse registration message
        registration = RegistrationRequestMessage.from_json(raw_message)

        # Capture client ID and role
        client_id = registration.client_id
        role = registration.role

        # Check if this client is already connected
        if client_id in connected_clients:

            reason = f"Client ID {client_id} already conected"

            print_server_message(reason)

            # clear it so the finally block doesn't remove the existing connection
            client_id = None

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
        print_server_message("Connection closed")

    finally:
        # Only remove the connection owned by this handler
        if client_id is not None and connected_clients.get(client_id) is websocket:
            del connected_clients[client_id]
            print_server_message(f"Client {client_id} disconnected")


# returns True if raw was imu/encoder telemetry and got stored for the panel
def handle_bot_telemetry(raw):
    global latest_imu, imu_count, latest_enc

    if raw.startswith("{"):
        try:
            data = json.loads(raw)
        except json.JSONDecodeError:
            return False
        if isinstance(data, dict) and "a" in data and "g" in data:
            latest_imu = data
            imu_count += 1
            return True
        return False

    if raw.startswith("enc "):
        latest_enc = raw
        return True

    return False


async def process_message(client_id, raw):
    # TODO: Parse and route messages.

    role = config.ROLE_BY_CLIENT_ID.get(client_id)

    if role == "bot":
        if not handle_bot_telemetry(raw):
            print_bot_message(raw)

    elif role == "cv":
        print_cv_message(raw)

    else:
        print_server_message(f"Message from unknown client {client_id}: {raw}")


async def send_to_bot(command):
    bot = connected_clients.get(BOT_ID)
    if bot is None:
        print_server_message("No robot connected, command not sent")
        return
    try:
        await bot.send(command)
        print_sent_message(command)
    except ConnectionClosed:
        print_server_message("Robot disconnected, command not sent")


SOURCE_STYLES = {
    "SERVER": "yellow",
    "BOT": "green",
    "CV": "magenta",
    "SENT": "cyan",
}


class ConsoleApp(App):
    TITLE = "pacbot console"

    CSS = """
    #status { height: 1; padding: 0 1; background: $panel; }
    #telemetry { height: 4; padding: 0 1; border: round $accent; }
    #log { height: 1fr; border: round $primary; }
    """

    BINDINGS = [
        Binding("f1", "stop", "Stop (brake)", priority=True),
        Binding("f2", "off", "Motors off", priority=True),
        Binding("ctrl+l", "clear_log", "Clear log", priority=True),
    ]

    def compose(self) -> ComposeResult:
        yield Header()
        yield Static(id="status")
        yield Static(id="telemetry")
        yield RichLog(id="log", wrap=True, max_lines=5000)
        yield Input(placeholder="command → robot   d <duty> [right] | s | x")
        yield Footer()

    def on_mount(self):
        self.log_view = self.query_one("#log", RichLog)
        self.status = self.query_one("#status", Static)
        self.telemetry = self.query_one("#telemetry", Static)
        self.query_one(Input).focus()

        logger.set_sink(self.write_log)

        self.set_interval(1.0, self.update_rate)
        self.set_interval(0.2, self.refresh_panels)
        self.refresh_panels()

        # websocket server runs on the same event loop as the ui
        self.run_worker(self.run_server(), exclusive=True)

    def on_unmount(self):
        logger.set_sink(None)

    async def run_server(self):
        async with serve(handle_connection, "0.0.0.0", config.PORT) as server:
            print_server_message(f"Server listening on port {config.PORT}")
            await server.serve_forever()

    def write_log(self, time, source, message):
        line = Text.assemble(
            (time, "dim"),
            " ",
            (f"[{source}]", f"bold {SOURCE_STYLES.get(source, 'white')}"),
            " ",
            message,
        )
        self.log_view.write(line)

    def update_rate(self):
        global imu_count, imu_rate
        imu_rate = imu_count
        imu_count = 0

    def refresh_panels(self):
        def dot(client_id, name):
            if client_id in connected_clients:
                return (f"● {name}", "bold green")
            return (f"○ {name}", "dim")

        self.status.update(
            Text.assemble(
                dot(BOT_ID, "bot"),
                "   ",
                dot(CV_ID, "cv"),
                ("   port ", "dim"),
                (str(config.PORT), "dim"),
            )
        )

        if latest_imu is None:
            imu = Text("IMU  waiting for data", style="dim")
        else:
            a, g = latest_imu["a"], latest_imu["g"]
            imu = Text(
                f"IMU  {imu_rate:3d}/s   "
                f"accel {a[0]:7.1f} {a[1]:7.1f} {a[2]:7.1f} mg   "
                f"gyro {g[0]:7.2f} {g[1]:7.2f} {g[2]:7.2f} deg/s"
            )

        enc = (
            Text(latest_enc.replace("enc", "ENC ", 1))
            if latest_enc
            else Text("ENC  waiting for data", style="dim")
        )

        self.telemetry.update(Text("\n").join([imu, enc]))

    async def on_input_submitted(self, event: Input.Submitted):
        command = event.value.strip()
        event.input.value = ""
        if command:
            await send_to_bot(command)

    async def action_stop(self):
        await send_to_bot("0")  # "s" is south now

    async def action_off(self):
        await send_to_bot("x")

    def action_clear_log(self):
        self.log_view.clear()


if __name__ == "__main__":
    ConsoleApp().run()
