"""Event protocol definitions for remote viewer interaction."""

import json
from enum import Enum
from typing import Dict, Any, Optional, Union
from dataclasses import dataclass


class EventType(Enum):
    """Types of events supported by the remote viewer."""
    
    MOUSE_MOVE = "mouse_move"
    MOUSE_DOWN = "mouse_down"
    MOUSE_UP = "mouse_up"
    SCROLL = "scroll"
    KEY_DOWN = "key_down"
    KEY_UP = "key_up"
    COMMAND = "command"


@dataclass
class MouseEvent:
    """Mouse event data."""
    
    type: EventType
    x: int
    y: int
    buttons: int = 0  # Bitmask: 1=left, 2=right, 4=middle
    
    def to_dict(self) -> Dict[str, Any]:
        return {
            "type": self.type.value,
            "x": self.x,
            "y": self.y,
            "buttons": self.buttons,
        }


@dataclass 
class ScrollEvent:
    """Scroll/wheel event data."""
    
    type: EventType
    x: int
    y: int  
    dx: float
    dy: float
    
    def to_dict(self) -> Dict[str, Any]:
        return {
            "type": self.type.value,
            "x": self.x,
            "y": self.y,
            "dx": self.dx,
            "dy": self.dy,
        }


@dataclass
class KeyEvent:
    """Keyboard event data."""
    
    type: EventType
    code: str
    key: Optional[str] = None
    alt: bool = False
    ctrl: bool = False
    shift: bool = False
    meta: bool = False
    
    def to_dict(self) -> Dict[str, Any]:
        return {
            "type": self.type.value,
            "code": self.code,
            "key": self.key,
            "alt": self.alt,
            "ctrl": self.ctrl,
            "shift": self.shift,
            "meta": self.meta,
        }


@dataclass  
class CommandEvent:
    """Command event data."""
    
    type: EventType
    cmd: str
    params: Optional[Dict[str, Any]] = None
    
    def to_dict(self) -> Dict[str, Any]:
        result = {
            "type": self.type.value,
            "cmd": self.cmd,
        }
        if self.params:
            result["params"] = self.params
        return result


EventData = Union[MouseEvent, ScrollEvent, KeyEvent, CommandEvent]


class EventProtocol:
    """Protocol for handling remote viewer events."""
    
    @staticmethod
    def parse_event(data: Dict[str, Any]) -> Optional[EventData]:
        """Parse raw event data into typed event objects."""
        try:
            event_type = EventType(data.get("type"))
            
            if event_type in [EventType.MOUSE_MOVE, EventType.MOUSE_DOWN, EventType.MOUSE_UP]:
                return MouseEvent(
                    type=event_type,
                    x=data.get("x", 0),
                    y=data.get("y", 0),
                    buttons=data.get("buttons", 0),
                )
            
            elif event_type == EventType.SCROLL:
                return ScrollEvent(
                    type=event_type,
                    x=data.get("x", 0),
                    y=data.get("y", 0),
                    dx=data.get("dx", 0.0),
                    dy=data.get("dy", 0.0),
                )
            
            elif event_type in [EventType.KEY_DOWN, EventType.KEY_UP]:
                return KeyEvent(
                    type=event_type,
                    code=data.get("code", ""),
                    key=data.get("key"),
                    alt=data.get("alt", False),
                    ctrl=data.get("ctrl", False),
                    shift=data.get("shift", False),
                    meta=data.get("meta", False),
                )
            
            elif event_type == EventType.COMMAND:
                return CommandEvent(
                    type=event_type,
                    cmd=data.get("cmd", ""),
                    params=data.get("params"),
                )
                
        except (ValueError, KeyError) as e:
            print(f"Failed to parse event: {e}")
            return None
        
        return None
    
    @staticmethod
    def serialize_event(event: EventData) -> str:
        """Serialize event object to JSON string."""
        return json.dumps(event.to_dict())
    
    @staticmethod
    def deserialize_event(json_str: str) -> Optional[EventData]:
        """Deserialize JSON string to event object."""
        try:
            data = json.loads(json_str)
            return EventProtocol.parse_event(data)
        except json.JSONDecodeError as e:
            print(f"Failed to deserialize event: {e}")
            return None