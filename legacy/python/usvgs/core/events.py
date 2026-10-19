"""A small typed event bus.

The vehicle layer publishes; the user interface subscribes.  Keeping this
deliberately minimal — no priorities, no filtering, no async — because the only
requirement is that a worker thread can announce something without knowing who
is listening or on which thread they live.

Subscribers run on the publishing thread.  A subscriber that needs to touch a
graphical widget must marshal to its own thread; the Qt layer wraps this bus in
a queued signal for exactly that reason.
"""

from __future__ import annotations

import enum
import logging
import threading
import time
from collections.abc import Callable, Iterator
from dataclasses import dataclass, field
from typing import Any, TypeVar

log = logging.getLogger(__name__)


class EventKind(enum.Enum):
    """What happened.  Used for routing and for the on-disk session log."""

    LINK_STATE = "link_state"
    ARMING_STATE = "arming_state"
    MODE = "mode"
    TELEMETRY = "telemetry"
    COMMAND = "command"
    SAFETY = "safety"
    MISSION = "mission"
    MESSAGE = "message"


class Level(enum.IntEnum):
    """Operator-facing importance, matching the telemetry alert scale."""

    DEBUG = -1
    INFO = 0
    NOTICE = 1
    WARNING = 2
    ERROR = 3
    CRITICAL = 4

    @property
    def label(self) -> str:
        return self.name.lower()


@dataclass(frozen=True)
class Event:
    """Something that happened, with enough context to log or display it."""

    kind: EventKind
    message: str = ""
    level: Level = Level.INFO
    #: Structured payload.  Must be JSON-serialisable: the recorder writes it.
    data: dict[str, Any] = field(default_factory=dict)
    #: Wall-clock time, for the log.  Monotonic time is unhelpful across runs.
    timestamp: float = field(default_factory=time.time)

    def to_dict(self) -> dict[str, Any]:
        return {
            "t": self.timestamp,
            "kind": self.kind.value,
            "level": self.level.label,
            "message": self.message,
            "data": self.data,
        }


Handler = Callable[[Event], None]
T = TypeVar("T")


class EventBus:
    """Publish/subscribe with no delivery guarantees beyond "we tried"."""

    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._handlers: dict[EventKind | None, list[Handler]] = {}
        self._history: list[Event] = []
        self._history_limit = 1000

    def subscribe(self, handler: Handler, kind: EventKind | None = None) -> Callable[[], None]:
        """Register ``handler``; pass ``kind`` to filter, or None for everything.

        Returns a callable that unsubscribes, so callers do not have to keep a
        reference to the handler in order to detach it.
        """
        with self._lock:
            self._handlers.setdefault(kind, []).append(handler)

        def unsubscribe() -> None:
            with self._lock:
                handlers = self._handlers.get(kind)
                if handlers and handler in handlers:
                    handlers.remove(handler)

        return unsubscribe

    def publish(self, event: Event) -> None:
        """Deliver ``event`` to every matching subscriber.

        A subscriber that raises is logged and skipped.  One misbehaving view
        must not be able to stop the vehicle layer from running, and this bus is
        called from the control loop.
        """
        with self._lock:
            self._history.append(event)
            if len(self._history) > self._history_limit:
                del self._history[: self._history_limit // 2]
            handlers = list(self._handlers.get(event.kind, ())) + list(
                self._handlers.get(None, ())
            )

        for handler in handlers:
            try:
                handler(event)
            except Exception:
                log.exception("event subscriber raised while handling %s", event.kind)

    # -- convenience ------------------------------------------------------

    def emit(
        self,
        kind: EventKind,
        message: str = "",
        level: Level = Level.INFO,
        **data: Any,
    ) -> Event:
        event = Event(kind=kind, message=message, level=level, data=data)
        self.publish(event)
        return event

    def message(self, text: str, level: Level = Level.INFO, **data: Any) -> Event:
        return self.emit(EventKind.MESSAGE, text, level, **data)

    def history(self, kind: EventKind | None = None, limit: int = 200) -> list[Event]:
        with self._lock:
            events = self._history if kind is None else [e for e in self._history if e.kind is kind]
            return list(events[-limit:])

    def __iter__(self) -> Iterator[Event]:
        with self._lock:
            return iter(list(self._history))
