"""Typed JSON messages between the server and its clients.

Every message on the wire is one JSON object whose "type" field picks the class, e.g.
    {"type": "duty", "left": 30, "right": 30}

The robot's side of this lives in include/messages.h and src/messages.c. If you add or change a
type here, change it there too.

    to send:     DutyMessage(left=30, right=30).to_json()
    to receive:  parse_message(raw)  -> whichever class matches "type"
"""

import json
from abc import ABC, abstractmethod
from dataclasses import asdict, dataclass
from enum import StrEnum
from typing import ClassVar, Self, TypeVar

from config import VALID_ROLES


class MessageType(StrEnum):
    """Every "type" string the server and robot understand."""

    # any client <-> server
    REGISTRATION = "registration"
    REGISTRATION_RESPONSE = "registration_response"
    ERROR = "error"
    INFO = "info"
    INSTRUCTION = "instruction"

    # server -> robot (motor commands)
    STOP = "stop"
    DIRECTION = "direction"
    DUTY = "duty"
    SPEED = "speed"
    TURN = "turn"
    STEP_TEST = "step_test"
    GAINS = "gains"

    # robot -> server (telemetry)
    ENCODERS = "encoders"
    STEP_LOG = "step_log"
    STEP_DONE = "step_done"
    IMU = "imu"


class Direction(StrEnum):
    NORTH = "n"
    EAST = "e"
    SOUTH = "s"
    WEST = "w"


class StepMode(StrEnum):
    DUTY = "duty"
    SPEED = "speed"


# "type" string -> class, filled in automatically by BaseMessage.__init_subclass__
MESSAGE_CLASSES: dict[str, type["BaseMessage"]] = {}


# ---------- field validation helpers ----------
# Each one pulls a field out of a decoded JSON dict and raises ValueError if it's the wrong kind.
# bool is checked for separately because in Python True is also an int.


def _number(data: dict, key: str, optional: bool = False) -> float | None:
    value = data.get(key)
    if value is None and optional:
        return None
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{key} must be a number")
    return value


def _int(data: dict, key: str) -> int:
    value = data.get(key)
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError(f"{key} must be an integer")
    return value


def _bool(data: dict, key: str, default: bool | None = None) -> bool:
    value = data.get(key, default)
    if not isinstance(value, bool):
        raise ValueError(f"{key} must be true or false")
    return value


def _str(data: dict, key: str) -> str:
    value = data.get(key)
    if not isinstance(value, str):
        raise ValueError(f"{key} must be a string")
    return value


E = TypeVar("E", bound=StrEnum)


def _enum(data: dict, key: str, enum: type[E]) -> E:
    value = data.get(key)
    try:
        return enum(value)
    except ValueError:
        choices = ", ".join(e.value for e in enum)
        raise ValueError(f"{key} must be one of {choices}") from None


def _numbers(data: dict, key: str, length: int) -> list[float]:
    value = data.get(key)
    if (
        not isinstance(value, list)
        or len(value) != length
        or any(isinstance(v, bool) or not isinstance(v, (int, float)) for v in value)
    ):
        raise ValueError(f"{key} must be a list of {length} numbers")
    return value


def _load_object(raw: str | bytes) -> dict:
    try:
        data = json.loads(raw)
    except json.JSONDecodeError as exc:
        raise ValueError("Invalid JSON") from exc

    if not isinstance(data, dict):
        raise ValueError("Message must be a JSON object")

    return data


# ---------- base classes ----------


@dataclass
class BaseMessage(ABC):
    """The base class that all message types inherit from."""

    # TODO: Add timestamp support
    # TODO: Add client identification support

    TYPE: ClassVar[str]

    def __init_subclass__(cls, **kwargs):
        # Any subclass that sets TYPE is registered so parse_message can find it
        super().__init_subclass__(**kwargs)
        if "TYPE" in cls.__dict__:
            if cls.TYPE in MESSAGE_CLASSES:
                raise TypeError(f"Message type '{cls.TYPE}' is defined twice")
            MESSAGE_CLASSES[cls.TYPE] = cls

    def to_dict(self) -> dict:
        """Convert the message to a JSON-compatible dictionary. Fields set to None are left out."""
        fields = {key: value for key, value in asdict(self).items() if value is not None}
        return {"type": str(self.TYPE), **fields}

    def to_json(self) -> str:
        """Serialize the message to a compact JSON string (the robot's buffers are small)."""
        return json.dumps(self.to_dict(), separators=(",", ":"))

    @classmethod
    def from_json(cls, raw: str | bytes) -> Self:
        """Validates the JSON structure and that it is this class's type before allowing its use."""
        data = _load_object(raw)

        if data.get("type") != cls.TYPE:
            raise ValueError(f"Expected message type '{cls.TYPE}'")

        return cls.from_dict(data)

    @classmethod
    @abstractmethod
    def from_dict(cls, data: dict) -> Self:
        """Validate fields and construct the message."""
        pass


def parse_message(raw: str | bytes) -> BaseMessage:
    """Decode any message, picking the class from its "type" field.

    Raises ValueError if it isn't valid JSON, the type is unknown, or a field is wrong.
    """
    data = _load_object(raw)
    msg_type = data.get("type")

    cls = MESSAGE_CLASSES.get(msg_type)
    if cls is None:
        raise ValueError(f"Unknown message type {msg_type!r}")

    return cls.from_dict(data)


# ---------- text and registration ----------


@dataclass
class TextMessage(BaseMessage):
    """Base class for messages containing a text payload."""

    message: str

    @classmethod
    def from_dict(cls, data: dict) -> Self:
        return cls(message=_str(data, "message"))


@dataclass
class InfoMessage(TextMessage):
    """Represents a message containing information as a text payload"""

    TYPE = MessageType.INFO


@dataclass
class ErrorMessage(TextMessage):
    """Represents a message containing an error as a text payload"""

    TYPE = MessageType.ERROR


@dataclass
class RegistrationRequestMessage(BaseMessage):
    client_id: str
    role: str

    TYPE = MessageType.REGISTRATION

    @classmethod
    def from_dict(cls, data: dict) -> Self:
        client_id = data.get("client_id")
        role = data.get("role")

        if not isinstance(client_id, str) or not client_id.strip():
            raise ValueError("client_id must be a nonempty string")

        if role not in VALID_ROLES:
            raise ValueError("Invalid client role")

        return cls(
            client_id=client_id,
            role=role,
        )


@dataclass
class RegistrationResponseMessage(BaseMessage):
    """Reports whether a client's registration succeeded."""

    success: bool
    message: str

    TYPE = MessageType.REGISTRATION_RESPONSE

    @classmethod
    def from_dict(cls, data: dict) -> Self:
        return cls(success=_bool(data, "success"), message=_str(data, "message"))


# ---------- server -> robot: motor commands ----------


@dataclass
class StopMessage(BaseMessage):
    """Stop both motors. Brakes by default; coast=True turns the motor driver off so they spin freely."""

    coast: bool = False

    TYPE = MessageType.STOP

    @classmethod
    def from_dict(cls, data: dict) -> Self:
        return cls(coast=_bool(data, "coast", default=False))


@dataclass
class DirectionMessage(BaseMessage):
    """Drive that way on the board, turning 90 degrees first if needed (the opposite way reverses)."""

    direction: Direction

    TYPE = MessageType.DIRECTION

    @classmethod
    def from_dict(cls, data: dict) -> Self:
        return cls(direction=_enum(data, "direction", Direction))


@dataclass
class DutyMessage(BaseMessage):
    """Open loop duty per wheel, -100..100 %."""

    left: int
    right: int

    TYPE = MessageType.DUTY

    @classmethod
    def from_dict(cls, data: dict) -> Self:
        return cls(left=_int(data, "left"), right=_int(data, "right"))


@dataclass
class SpeedMessage(BaseMessage):
    """Closed loop speed per wheel in mm/s (negative = backward)."""

    left: int
    right: int

    TYPE = MessageType.SPEED

    @classmethod
    def from_dict(cls, data: dict) -> Self:
        return cls(left=_int(data, "left"), right=_int(data, "right"))


@dataclass
class TurnMessage(BaseMessage):
    """Turn in place. speed is the wheel speed in mm/s; None uses the robot's default."""

    degrees: float
    clockwise: bool
    speed: float | None = None

    TYPE = MessageType.TURN

    @classmethod
    def from_dict(cls, data: dict) -> Self:
        return cls(
            degrees=_number(data, "degrees"),
            clockwise=_bool(data, "clockwise"),
            speed=_number(data, "speed", optional=True),
        )


@dataclass
class StepTestMessage(BaseMessage):
    """Step test for tune.py. value is a duty % (mode duty) or mm/s (mode speed)."""

    mode: StepMode
    value: int
    duration_ms: int = 1000

    TYPE = MessageType.STEP_TEST

    @classmethod
    def from_dict(cls, data: dict) -> Self:
        return cls(
            mode=_enum(data, "mode", StepMode),
            value=_int(data, "value"),
            duration_ms=_int(data, "duration_ms"),
        )


@dataclass
class GainsMessage(BaseMessage):
    """Set the speed PI gains. Leave both as None to just have the robot report them."""

    kp: float | None = None
    ki: float | None = None

    TYPE = MessageType.GAINS

    @classmethod
    def from_dict(cls, data: dict) -> Self:
        kp = _number(data, "kp", optional=True)
        ki = _number(data, "ki", optional=True)
        if (kp is not None and kp < 0) or (ki is not None and ki < 0):
            raise ValueError("gains must be 0 or more")
        return cls(kp=kp, ki=ki)


# ---------- robot -> server: telemetry ----------


@dataclass
class EncodersMessage(BaseMessage):
    """Raw encoder counts for each wheel."""

    left: int
    right: int

    TYPE = MessageType.ENCODERS

    @classmethod
    def from_dict(cls, data: dict) -> Self:
        return cls(left=_int(data, "left"), right=_int(data, "right"))


@dataclass
class StepLogMessage(BaseMessage):
    """One sample of a step test.

    t is ms since the test started, value is the commanded duty % or mm/s, left/right are the
    measured speeds (counts/s for duty tests, mm/s for speed tests). Speed tests also include the
    duty the PI controller chose for each wheel.
    """

    t: int
    value: int
    left: int
    right: int
    left_duty: float | None = None
    right_duty: float | None = None

    TYPE = MessageType.STEP_LOG

    @classmethod
    def from_dict(cls, data: dict) -> Self:
        return cls(
            t=_int(data, "t"),
            value=_int(data, "value"),
            left=_int(data, "left"),
            right=_int(data, "right"),
            left_duty=_number(data, "left_duty", optional=True),
            right_duty=_number(data, "right_duty", optional=True),
        )


@dataclass
class StepDoneMessage(BaseMessage):
    """Sent after the last StepLogMessage of a step test."""

    TYPE = MessageType.STEP_DONE

    @classmethod
    def from_dict(cls, data: dict) -> Self:
        return cls()


@dataclass
class ImuMessage(BaseMessage):
    """One IMU reading: t in ms since boot, a = accel [x, y, z] in mg, g = gyro [x, y, z] in deg/s."""

    t: int
    a: list[float]
    g: list[float]

    TYPE = MessageType.IMU

    @classmethod
    def from_dict(cls, data: dict) -> Self:
        return cls(t=_int(data, "t"), a=_numbers(data, "a", 3), g=_numbers(data, "g", 3))
