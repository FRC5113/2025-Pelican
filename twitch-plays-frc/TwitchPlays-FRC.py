from twitchchat import Twitch,YouTube
from networktable import NetworkTableHandler
import time
import concurrent.futures
import keyboard
import pyautogui
import re

# Configuration
TWITCH_CHANNEL = 'ryanforce_'  # Change this to your desired channel (without #)
COMMAND_DURATION = 2.0  # Duration in seconds to hold each command

SIM_IP = '127.0.0.1'
ROBO_IP = '10.51.13.2'
NT_IP  = SIM_IP  # Change to ROBO_IP when deploying to the robot

# MESSAGE_RATE controls how fast we process incoming Twitch Chat messages. It's the number of seconds it will take to handle all messages in the queue.
# This is used because Twitch delivers messages in "batches", rather than one at a time. So we process the messages over MESSAGE_RATE duration, rather than processing the entire batch at once.
# A smaller number means we go through the message queue faster, but we will run out of messages faster and activity might "stagnate" while waiting for a new batch. 
# A higher number means we go through the queue slower, and messages are more evenly spread out, but delay from the viewers' perspective is higher.
# You can set this to 0 to disable the queue and handle all messages immediately. However, then the wait before another "batch" of messages is more noticeable.
MESSAGE_RATE = 0.5
# MAX_QUEUE_LENGTH limits the number of commands that will be processed in a given "batch" of messages. 
# e.g. if you get a batch of 50 messages, you can choose to only process the first 10 of them and ignore the others.
# This is helpful for games where too many inputs at once can actually hinder the gameplay.
# Setting to ~50 is good for total chaos, ~5-10 is good for 2D platformers
MAX_QUEUE_LENGTH = 20
MAX_WORKERS = 100 # Maximum number of threads you can process at a time 



class TwitchToNetworkTables:
    def __init__(self, channel=TWITCH_CHANNEL, command_duration=COMMAND_DURATION):
        commands = {
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
            'turn': ('Turn', 0.0, 0.0),  # Value command handled separately
        }
        self.monitor = Twitch()
        self.monitor.twitch_connect(channel)
        self.handler = NetworkTableHandler(command_map=commands, command_duration=command_duration)

        self.last_time = time.time()
        self.message_queue = []
        self.thread_pool = concurrent.futures.ThreadPoolExecutor(max_workers=MAX_WORKERS)
        self.active_tasks = []
        pyautogui.FAILSAFE = False
        self.until_key = "setpoint"
    

    def handle_message(self, message: str):
        try:
            msg = message['message'].lower()
            username = message['username'].lower()

            print("Got this message from " + username + ": " + msg)

            match msg:
                case "forward":
                    self.handler.process_temp_command(username, msg)
                case "backward":
                    self.handler.process_temp_command(username, msg)
                case "back":
                    self.handler.process_temp_command(username, msg)
                case "left":
                    self.handler.process_temp_command(username, msg)
                case "right":
                    self.handler.process_temp_command(username, msg)
                case "l":
                    self.handler.process_temp_command(username, msg)
                case "r":
                    self.handler.process_temp_command(username, msg)
                case "stop":
                    self.handler.process_temp_command(username, msg)
                case "l4":
                    self.handler.process_temp_command(username, msg)
                case "l3":
                    self.handler.process_temp_command(username, msg)
                case "l2":
                    self.handler.process_temp_command(username, msg)
                case "l1":
                    self.handler.process_temp_command(username, msg)
            
            if msg.startswith("turn"):
                turn = msg.split(" ")
                self.handler.process_value_command("Turn", int(turn[1]))


        except Exception as e:
            print("Encountered exception: " + str(e))

    def run(self):
        """Start the system"""
        print("Starting Twitch to NetworkTables bridge...")
        print(f"Command duration: {self.handler.command_duration}s")
        print("\nAvailable commands:")
        for cmd in sorted(self.handler.command_map.keys()):
            print(f"  - {cmd}")
        print("\n" + "=" * 50)
        while True:
            try:
                self.active_tasks = [t for t in self.active_tasks if not t.done()]

                #Check for new messages
                new_messages = self.monitor.twitch_receive_messages();
                if new_messages:
                    self.message_queue += new_messages; # New messages are added to the back of the queue
                    self.message_queue = self.message_queue[-MAX_QUEUE_LENGTH:] # Shorten the queue to only the most recent X messages

                messages_to_handle = []
                if not self.message_queue:
                    # No messages in the queue
                    self.last_time = time.time()
                else:
                    # Determine how many messages we should handle now
                    r = 1 if MESSAGE_RATE == 0 else (time.time() - self.last_time) / MESSAGE_RATE
                    n = int(r * len(self.message_queue))
                    if n > 0:
                        # Pop the messages we want off the front of the queue
                        messages_to_handle = self.message_queue[0:n]
                        del self.message_queue[0:n]
                        self.last_time = time.time()

                if not messages_to_handle:
                    continue
                else:
                    for message in messages_to_handle:
                        if len(self.active_tasks) <= MAX_WORKERS:
                            self.active_tasks.append(self.thread_pool.submit(self.handle_message, message))
                        else:
                            print(f'WARNING: active tasks ({len(self.active_tasks)}) exceeds number of workers ({MAX_WORKERS}). ({len(self.message_queue)} messages in the queue)')
            except KeyboardInterrupt: 
                exit()
            except Exception as e:
                print("Encountered exception in main loop: " + str(e))
 

if __name__ == "__main__":
    bridge = TwitchToNetworkTables()
    bridge.run()
