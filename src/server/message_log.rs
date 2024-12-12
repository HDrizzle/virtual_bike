/// Created 2024-3-5
/// Chat log

use std::time::{SystemTime, UNIX_EPOCH};
use serde::{Serialize, Deserialize};

#[allow(unused_imports)]
use crate::prelude::*;
#[cfg(feature = "client")]
use bevy::prelude::Resource;

/// Types of message
#[derive(Serialize, Deserialize, Clone)]
pub enum MessageEnum {
    /// User chat
    UserChat {
        name: String,
        chat: String
    },
    /// When client is connected to the Renet server
    ClientConnected(String),
    /// When the Renet server loses a connection
    /// Username, Reason
    ClientDisconnected(String, String)
}

impl MessageEnum {
    pub fn to_string(&self) -> String {
        match &self {
            Self::UserChat{name, chat} => format!("{}: {}", name, chat),
            Self::ClientConnected(s) => format!("Client \"{}\" connected", s),
            Self::ClientDisconnected(name, reason) => format!("Client \"{}\" disconnected for reason: {}", name, reason)
        }
    }
}

#[derive(Serialize, Deserialize, Clone)]
pub struct Message {
    timestamp: u64,
    enum_: MessageEnum
}

impl Message {
    pub fn to_string(&self) -> String {
        self.enum_.to_string()// TODO: timestamp
    }
}

impl Message {
    pub fn new(enum_: MessageEnum) -> Self {
        Self {
            timestamp: SystemTime::now().duration_since(UNIX_EPOCH).unwrap().as_secs(),
            enum_
        }
    }
}

#[cfg_attr(feature = "client", derive(Resource))]
pub struct Log {
    pub messages: Vec<Message>
}

impl Log {
    pub fn new() -> Self {
        Self {
            messages: Vec::new()
        }
    }
    pub fn add(&mut self, msg: Message) {
        self.messages.push(msg);
    }
}