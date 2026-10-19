"""The link to the boat.

Everything above this layer talks to a :class:`BoatLink` and never knows
whether the boat is real.  A link carries two things: actuator commands up to
the boat, and :class:`~usvgs.core.telemetry.Telemetry` back down.

Only the simulator exists so far.  A real boat — an ArduPilot Rover in boat
frame over MAVLink is the usual choice — goes in as one more subclass.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from collections.abc import Callable

from ..core.telemetry import Telemetry

TelemetryHandler = Callable[[Telemetry], None]


class BoatLink(ABC):
    #: Shown in the status bar.
    name = "link"
    simulated = False

    def __init__(self) -> None:
        self.on_telemetry: TelemetryHandler | None = None

    @abstractmethod
    def open(self) -> None: ...

    @abstractmethod
    def close(self) -> None: ...

    @abstractmethod
    def send_command(self, port: float, stbd: float, armed: bool) -> None:
        """Motor levels in [-1, 1].  Sent continuously; the boat stops on silence."""

    def send_payload(self, name: str, value) -> None:  # noqa: B027 - optional
        """A payload command, such as ``("skimmer", True)``.  Ignored by default."""

    def _deliver(self, telemetry: Telemetry) -> None:
        handler = self.on_telemetry
        if handler is not None:
            handler(telemetry)
