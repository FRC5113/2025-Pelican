import socket
import time
import threading
from typing import Any, Callable, Dict, Optional
from ntcore import NetworkTableInstance, NetworkTableEntry

# Configuration
TWITCH_CHANNEL = 'ryanforce_'  # Change this to your desired channel (without #)
COMMAND_DURATION = 2.0  # Duration in seconds to hold each command

class TwitchChatMonitor:
    def __init__(self, channel):
        self.channel = channel
        self.sock = socket.socket()
        
    def connect(self):
        """Connect to Twitch IRC"""
        self.sock.connect(('irc.chat.twitch.tv', 6667))
        self.sock.send(f"PASS oauth:justinfan12345\n".encode('utf-8'))
        self.sock.send(f"NICK justinfan12345\n".encode('utf-8'))
        self.sock.send(f"JOIN #{self.channel}\n".encode('utf-8'))
        print(f'Connected to #{self.channel}')
        print('Listening for messages...')
        print('=' * 50)
        
    def get_messages(self):
        """Receive and parse messages"""
        response = self.sock.recv(2048).decode('utf-8', errors='ignore')
        
        # Handle PING to keep connection alive
        if response.startswith('PING'):
            self.sock.send("PONG\n".encode('utf-8'))
            return []
        
        messages = []
        for line in response.split('\r\n'):
            if 'PRIVMSG' in line:
                try:
                    # Parse username
                    username = line.split('!')[0][1:]
                    # Parse message
                    message = line.split('PRIVMSG')[1].split(':')[1]
                    messages.append((username, message))
                except:
                    pass
        
        return messages
    
    def run(self, message_callback=None):
        """Main loop with optional callback"""
        self.connect()
        
        try:
            while True:
                messages = self.get_messages()
                for username, message in messages:
                    print(f'{username}: {message}')
                    if message_callback:
                        message_callback(username, message)
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("\n\nDisconnecting...")
            self.sock.close()
        except Exception as e:
            print(f"Error: {e}")
            self.sock.close()

class SmartNT:
    def __init__(
        self, root_table: str = "/", verbose: bool = False, poll_period: float = 0.02
    ):
        self.nt = NetworkTableInstance.getDefault()
        self.nt.startClient4("SmartNTClient")
        self.nt.setServer("10.51.13.2")
        self.table = self.nt.getTable(root_table.strip("/"))
        self._entries: Dict[str, NetworkTableEntry] = {}
        self._properties: Dict[str, Dict[str, Callable]] = {}
        self.verbose = verbose
        self.poll_period = poll_period
        self._running = False

    def _get_entry(self, key: str) -> NetworkTableEntry:
        key = str(key)
        if key not in self._entries:
            path_parts = key.strip("/").split("/")
            table = self.table
            for part in path_parts[:-1]:
                table = table.getSubTable(part)
            entry = table.getEntry(path_parts[-1])
            self._entries[key] = entry
            if self.verbose:
                print(f"[SmartNT] Created entry: /{'/'.join(path_parts)}")
        return self._entries[key]

    def set_struct_array(self, key: str, value: list, type):
        publisher = self.nt.getStructArrayTopic(f"{key}", type)
        publisher.publish(value)

    def put(self, key: str, value: Any):
        entry = self._get_entry(key)
        if isinstance(value, (float, int)):
            entry.setDouble(float(value))
        elif isinstance(value, bool):
            entry.setBoolean(value)
        elif isinstance(value, str):
            entry.setString(value)
        else:
            raise TypeError(f"Unsupported value type for key '{key}': {type(value)}")
        if self.verbose:
            print(f"[SmartNT] Set {key} = {value} (type: {type(value).__name__})")

    def get(self, key: str, default: Any = None) -> Any:
        entry = self._get_entry(key)
        if isinstance(default, (float, int)):
            return entry.getDouble(default)
        elif isinstance(default, bool):
            return entry.getBoolean(default)
        elif isinstance(default, str):
            return entry.getString(default)
        else:
            raise TypeError(
                f"Unsupported default type for key '{key}': {type(default)}"
            )

    def add_double_property(
        self, key: str, getter: Callable[[], float], setter: Callable[[float], None]
    ):
        self._properties[key] = {"getter": getter, "setter": setter, "type": "double"}
        self._get_entry(key)

    def add_boolean_property(
        self, key: str, getter: Callable[[], bool], setter: Callable[[bool], None]
    ):
        self._properties[key] = {"getter": getter, "setter": setter, "type": "boolean"}
        self._get_entry(key)

    def add_string_property(
        self, key: str, getter: Callable[[], str], setter: Callable[[str], None]
    ):
        self._properties[key] = {"getter": getter, "setter": setter, "type": "string"}
        self._get_entry(key)

    def start(self):
        if not self._running:
            self._running = True
            self._thread = threading.Thread(target=self._update_loop, daemon=True)
            self._thread.start()
            if self.verbose:
                print("[SmartNT] Update thread started")

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join()
            if self.verbose:
                print("[SmartNT] Update thread stopped")

    def _update_loop(self):
        while self._running:
            for key, funcs in self._properties.items():
                entry = self._get_entry(key)
                getter = funcs["getter"]
                setter = funcs["setter"]
                typ = funcs["type"]

                try:
                    # Push current getter value to NetworkTables
                    val = getter()
                    if typ == "double":
                        entry.setDouble(float(val))
                    elif typ == "boolean":
                        entry.setBoolean(bool(val))
                    elif typ == "string":
                        entry.setString(str(val))
                except Exception as e:
                    if self.verbose:
                        print(f"[SmartNT] Getter error for '{key}': {e}")

                try:
                    # Get the value from NT and update local property if different
                    if typ == "double":
                        nt_val = entry.getDouble(getter())
                        if abs(nt_val - getter()) > 1e-6:
                            setter(nt_val)
                    elif typ == "boolean":
                        nt_val = entry.getBoolean(getter())
                        if nt_val != getter():
                            setter(nt_val)
                    elif typ == "string":
                        nt_val = entry.getString(getter())
                        if nt_val != getter():
                            setter(nt_val)
                except Exception as e:
                    if self.verbose:
                        print(f"[SmartNT] Setter error for '{key}': {e}")

            time.sleep(self.poll_period)

    def put_number(self, key: str, value: float):
        self.put(key, float(value))

    def put_boolean(self, key: str, value: bool):
        self.put(key, value)

    def put_string(self, key: str, value: str):
        self.put(key, value)

    def get_number(self, key: str, default: float = 0.0) -> float:
        return float(self.get(key, default))

    def get_boolean(self, key: str, default: bool = False) -> bool:
        return bool(self.get(key, default))

    def get_string(self, key: str, default: str = "") -> str:
        return str(self.get(key, default))

    def value(self, key: str, val_or_default: Any = None, set: bool = False) -> Any:
        if set:
            self.put(key, val_or_default)
            return val_or_default
        else:
            return self.get(key, val_or_default)

class NetworkTableHandler:
    def __init__(self, table_name='SmartDashboard', command_duration=COMMAND_DURATION):
        self.smart_nt = SmartNT(root_table=table_name, verbose=True)
        self.smart_nt.start()
        self.command_duration = command_duration
        self.active_timers = {}
        self.lock = threading.Lock()
        
        # Command mapping: command -> (key, value, reset_value)
        self.command_map = {
            'forward': ('LeftY', 1.0, 0.0),
            'backward': ('LeftY', -1.0, 0.0),
            'back': ('LeftY', -1.0, 0.0),
            'left': ('LeftX', -1.0, 0.0),
            'right': ('LeftX', 1.0, 0.0),
            'l': ('RightX', -1.0, 0.0),
            'r': ('RightX', 1.0, 0.0),
            'l4': ('L4', True, False),
            'l3': ('L3', True, False),
            'l2': ('L2', True, False),
            'l1': ('L1', True, False),
            'intake': ('Intake', True, False),
            'stop': ('LeftY', 0.0, 0.0),  # Emergency stop
        }
        self.smart_nt.put('LeftY', 0.0)
        self.smart_nt.put('LeftX', 0.0)
        self.smart_nt.put('RightX', 0.0)
        self.smart_nt.put_boolean('L4', False)
        self.smart_nt.put_boolean('L3', False)
        self.smart_nt.put_boolean('L2', False)
        self.smart_nt.put_boolean('L1', False)
        self.smart_nt.put_boolean('Intake', False)


    def set_temporary_value(self, key: str, value: Any, reset_value: Any):
        """Set a value temporarily, then reset it after duration"""
        with self.lock:
            # Cancel existing timer for this key if any
            if key in self.active_timers:
                self.active_timers[key].cancel()
            
            # Set the value
            self.smart_nt.put(key, value)
            print(f"[Command] Set {key} = {value} for {self.command_duration}s")
            
            # Schedule reset
            timer = threading.Timer(self.command_duration, self._reset_value, args=(key, reset_value))
            self.active_timers[key] = timer
            timer.start()

    def _reset_value(self, key: str, reset_value: Any):
        """Reset a value back to default"""
        with self.lock:
            self.smart_nt.put(key, reset_value)
            print(f"[Command] Reset {key} = {reset_value}")
            if key in self.active_timers:
                del self.active_timers[key]

    def process_command(self, username: str, message: str):
        """Process a chat message as a command"""
        # Convert to lowercase and strip whitespace
        command = message.lower().strip()
        
        # Check if it's a valid command
        if command in self.command_map:
            key, value, reset_value = self.command_map[command]
            self.set_temporary_value(key, value, reset_value)
            print(f"[{username}] executed: {command}")
        # You can add more complex parsing here if needed

    def cancel_all(self):
        """Cancel all active timers"""
        with self.lock:
            for timer in self.active_timers.values():
                timer.cancel()
            self.active_timers.clear()

class TwitchToNetworkTables:
    def __init__(self, channel=TWITCH_CHANNEL, command_duration=COMMAND_DURATION):
        self.monitor = TwitchChatMonitor(channel)
        self.handler = NetworkTableHandler(command_duration=command_duration)
        
    def on_message(self, username: str, message: str):
        """Callback for when a message is received"""
        self.handler.process_command(username, message)
    
    def run(self):
        """Start the system"""
        print("Starting Twitch to NetworkTables bridge...")
        print(f"Command duration: {self.handler.command_duration}s")
        print("\nAvailable commands:")
        for cmd in sorted(self.handler.command_map.keys()):
            print(f"  - {cmd}")
        print("\n" + "=" * 50)
        
        try:
            self.monitor.run(message_callback=self.on_message)
        except KeyboardInterrupt:
            print("\n\nShutting down...")
            self.handler.cancel_all()
            self.handler.smart_nt.stop()

if __name__ == "__main__":
    bridge = TwitchToNetworkTables()
    bridge.run()