"""Turns the short commands you type in console.py / tune.py into typed messages for the robot.

    d <duty%> [right]   DutyMessage        open loop duty, -100..100 (one number = both wheels)
    v <mm/s> [right]    SpeedMessage       closed loop speed
    0 | stop            StopMessage        brake
    x                   StopMessage        motors off (coast)
    n | e | s | w       DirectionMessage   go that way on the board
    tcw <deg> [mm/s]    TurnMessage        turn clockwise in place
    tccw <deg> [mm/s]   TurnMessage        turn counter clockwise in place
    o <duty%> [ms]      StepTestMessage    duty step test (default 1000 ms)
    p <mm/s> [ms]       StepTestMessage    speed step test
    kp <x> | ki <x>     GainsMessage       set a PI gain
    show                GainsMessage       report the PI gains
    {...}               any message as raw JSON, checked before it is sent
"""

from messages import (
    BaseMessage,
    Direction,
    DirectionMessage,
    DutyMessage,
    GainsMessage,
    SpeedMessage,
    StepMode,
    StepTestMessage,
    StopMessage,
    TurnMessage,
    parse_message,
)

USAGE = "d <duty> [right] | v <mm/s> [right] | n e s w | tcw/tccw <deg> [mm/s] | 0 | x | kp/ki <x> | show"


def _numbers(args: list[str], name: str, count_min: int, count_max: int, cast=int) -> list:
    if not count_min <= len(args) <= count_max:
        raise ValueError(f"usage: {name}")
    try:
        return [cast(a) for a in args]
    except ValueError:
        raise ValueError(f"usage: {name}  (numbers only)") from None


def parse_command(text: str) -> BaseMessage:
    """Raises ValueError with a usage hint if the command isn't recognised."""
    text = text.strip()

    if text.startswith("{"):
        return parse_message(text)

    command, *args = text.split()
    command = command.lower()

    if command in ("0", "stop") and not args:
        return StopMessage(coast=False)

    if command == "x" and not args:
        return StopMessage(coast=True)

    if command in ("n", "e", "s", "w") and not args:
        return DirectionMessage(direction=Direction(command))

    if command == "d":
        left, *right = _numbers(args, "d <duty%> [right duty%]", 1, 2)
        return DutyMessage(left=left, right=right[0] if right else left)

    if command == "v":
        left, *right = _numbers(args, "v <mm/s> [right mm/s]", 1, 2)
        return SpeedMessage(left=left, right=right[0] if right else left)

    if command in ("tcw", "tccw"):
        degrees, *speed = _numbers(args, f"{command} <degrees> [wheel mm/s]", 1, 2, cast=float)
        return TurnMessage(degrees=degrees, clockwise=command == "tcw", speed=speed[0] if speed else None)

    if command in ("o", "p"):
        value, *duration = _numbers(args, f"{command} <value> [ms]", 1, 2)
        return StepTestMessage(
            mode=StepMode.SPEED if command == "p" else StepMode.DUTY,
            value=value,
            duration_ms=duration[0] if duration else 1000,
        )

    if command in ("kp", "ki"):
        (gain,) = _numbers(args, f"{command} <gain>", 1, 1, cast=float)
        if gain < 0:
            raise ValueError("gains must be 0 or more")
        return GainsMessage(**{command: gain})

    if command == "show" and not args:
        return GainsMessage()

    raise ValueError(f"unknown command '{text}'. try: {USAGE}")
