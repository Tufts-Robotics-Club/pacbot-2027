from datetime import datetime

# Where log lines go. None means plain print() (server.py, scripts);
# console.py sets this so lines land in its log pane instead of stdout.
_sink = None


# Routes log lines to sink(time, source, message) instead of stdout. Pass None to reset.
def set_sink(sink):
    global _sink
    _sink = sink


# Returns the current human-readable time as YYYY-MM-DD HH:MM:SS.
def get_time():
    return datetime.now().strftime("%Y-%m-%d %H:%M:%S")


def _emit(source, message):
    time = get_time()
    if _sink is None:
        print(f"{time} [{source}]: {message}")
    else:
        _sink(time, source, str(message))


# Prints a server-designated message.
def print_server_message(message):
    _emit("SERVER", message)


# Prints a bot-designated message.
def print_bot_message(message):
    _emit("BOT", message)


# Prints a CV-designated message.
def print_cv_message(message):
    _emit("CV", message)


# Prints a command we sent out.
def print_sent_message(message):
    _emit("SENT", message)
