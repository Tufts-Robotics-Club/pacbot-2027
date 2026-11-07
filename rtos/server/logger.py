from datetime import datetime


# Returns the current human-readable time as YYYY-MM-DD HH:MM:SS.
def get_time():
    return datetime.now().strftime("%Y-%m-%d %H:%M:%S")


# Prints a server-designated message.
def print_server_message(message):
    print(f"{get_time()} [SERVER]: {message}")


# Prints a bot-designated message.
def print_bot_message(message):
    print(f"{get_time()} [BOT]: {message}")


# Prints a CV-designated message.
def print_cv_message(message):
    print(f"{get_time()} [CV]: {message}")
