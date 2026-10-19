//! A small event bus.
//!
//! The vehicle layer publishes; the user interface subscribes.  Deliberately
//! minimal — no priorities, no async — because the only requirement is that
//! a worker thread can announce something without knowing who is listening
//! or on which thread they live.  Each subscriber gets its own channel and
//! drains it when it likes; a subscriber that went away is dropped.

use std::collections::VecDeque;
use std::sync::mpsc::{channel, Receiver, Sender};
use std::sync::Mutex;

use serde_json::{Map, Value};

use super::clock;

/// What happened.  Used for routing and for the session log.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum EventKind {
    LinkState,
    ArmingState,
    Mode,
    Telemetry,
    Command,
    Safety,
    Mission,
    Message,
}

impl EventKind {
    pub fn as_str(self) -> &'static str {
        match self {
            EventKind::LinkState => "link_state",
            EventKind::ArmingState => "arming_state",
            EventKind::Mode => "mode",
            EventKind::Telemetry => "telemetry",
            EventKind::Command => "command",
            EventKind::Safety => "safety",
            EventKind::Mission => "mission",
            EventKind::Message => "message",
        }
    }
}

/// Operator-facing importance, matching the telemetry alert scale.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Level {
    Debug = -1,
    Info = 0,
    Notice = 1,
    Warning = 2,
    Error = 3,
    Critical = 4,
}

impl Level {
    pub fn label(self) -> &'static str {
        match self {
            Level::Debug => "debug",
            Level::Info => "info",
            Level::Notice => "notice",
            Level::Warning => "warning",
            Level::Error => "error",
            Level::Critical => "critical",
        }
    }

    pub fn value(self) -> i32 {
        self as i32
    }
}

/// Something that happened, with enough context to log or display it.
#[derive(Clone, Debug)]
pub struct Event {
    pub kind: EventKind,
    pub message: String,
    pub level: Level,
    /// Structured payload, JSON so the recorder can write it.
    pub data: Map<String, Value>,
    /// Wall-clock time, for the log.
    pub timestamp: f64,
}

impl Event {
    pub fn to_json(&self) -> Value {
        serde_json::json!({
            "t": self.timestamp,
            "kind": self.kind.as_str(),
            "level": self.level.label(),
            "message": self.message,
            "data": self.data,
        })
    }
}

const HISTORY_LIMIT: usize = 1000;

#[derive(Default)]
struct Inner {
    history: VecDeque<Event>,
    subscribers: Vec<(Option<EventKind>, Sender<Event>)>,
}

/// Publish/subscribe with no delivery guarantees beyond "we tried".
#[derive(Default)]
pub struct EventBus {
    inner: Mutex<Inner>,
}

impl EventBus {
    pub fn new() -> Self {
        Self::default()
    }

    /// A channel of every event (or only those of `kind`) from now on.
    pub fn subscribe(&self, kind: Option<EventKind>) -> Receiver<Event> {
        let (tx, rx) = channel();
        self.inner.lock().unwrap().subscribers.push((kind, tx));
        rx
    }

    pub fn publish(&self, event: Event) {
        let mut inner = self.inner.lock().unwrap();
        inner.history.push_back(event.clone());
        if inner.history.len() > HISTORY_LIMIT {
            inner.history.drain(..HISTORY_LIMIT / 2);
        }
        inner.subscribers.retain(|(kind, tx)| {
            if kind.is_some_and(|k| k != event.kind) {
                return true;
            }
            tx.send(event.clone()).is_ok()
        });
    }

    pub fn emit(&self, kind: EventKind, message: impl Into<String>, level: Level, data: Value) -> Event {
        let data = match data {
            Value::Object(map) => map,
            Value::Null => Map::new(),
            other => {
                let mut map = Map::new();
                map.insert("value".into(), other);
                map
            }
        };
        let event = Event { kind, message: message.into(), level, data, timestamp: clock::unix_time() };
        self.publish(event.clone());
        event
    }

    pub fn message(&self, text: impl Into<String>, level: Level) -> Event {
        self.emit(EventKind::Message, text, level, Value::Null)
    }

    pub fn history(&self, kind: Option<EventKind>, limit: usize) -> Vec<Event> {
        let inner = self.inner.lock().unwrap();
        let events: Vec<Event> =
            inner.history.iter().filter(|e| kind.is_none_or(|k| e.kind == k)).cloned().collect();
        let skip = events.len().saturating_sub(limit);
        events.into_iter().skip(skip).collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn subscribers_get_what_they_asked_for() {
        let bus = EventBus::new();
        let all = bus.subscribe(None);
        let modes = bus.subscribe(Some(EventKind::Mode));
        bus.message("hello", Level::Info);
        bus.emit(EventKind::Mode, "Manual", Level::Info, serde_json::json!({"mode": "manual"}));
        assert_eq!(all.try_iter().count(), 2);
        let got: Vec<Event> = modes.try_iter().collect();
        assert_eq!(got.len(), 1);
        assert_eq!(got[0].data["mode"], "manual");
        assert_eq!(bus.history(Some(EventKind::Message), 10).len(), 1);
    }
}
