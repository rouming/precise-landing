import time
import socket
import struct

import olympe
import subprocess
import time
from olympe.messages.skyctrl.CoPiloting import setPilotingSource
from olympe.messages.ardrone3.Piloting import TakeOff, Landing, PCMD
from olympe.messages.ardrone3.PilotingState import (
    PositionChanged,
    AlertStateChanged,
    FlyingStateChanged,
    SpeedChanged,
    AltitudeChanged,
    AttitudeChanged,
    NavigateHomeStateChanged,
)

from pynput.keyboard import Listener, Key, KeyCode
from collections import defaultdict
from enum import Enum

class Ctrl(Enum):
    (
        QUIT,
        TAKEOFF,
        LANDING,
        MOVE_LEFT,
        MOVE_RIGHT,
        MOVE_FORWARD,
        MOVE_BACKWARD,
        MOVE_UP,
        MOVE_DOWN,
        TURN_LEFT,
        TURN_RIGHT,

        UNBLOCK_OTHER_INPUT,
        BLOCK_OTHER_INPUT,
    ) = range(13)


QWERTY_CTRL_KEYS = {
    Ctrl.QUIT: Key.esc,
    Ctrl.TAKEOFF: "t",
    Ctrl.LANDING: "l",
    Ctrl.MOVE_LEFT: "a",
    Ctrl.MOVE_RIGHT: "d",
    Ctrl.MOVE_FORWARD: "w",
    Ctrl.MOVE_BACKWARD: "s",
    Ctrl.MOVE_UP: Key.up,
    Ctrl.MOVE_DOWN: Key.down,
    Ctrl.TURN_LEFT: Key.left,
    Ctrl.TURN_RIGHT: Key.right,

    Ctrl.UNBLOCK_OTHER_INPUT: "u",  # "unblock" other input
    Ctrl.BLOCK_OTHER_INPUT: "b", # "block" other input
}

AZERTY_CTRL_KEYS = QWERTY_CTRL_KEYS.copy()
AZERTY_CTRL_KEYS.update(
    {
        Ctrl.MOVE_LEFT: "q",
        Ctrl.MOVE_RIGHT: "d",
        Ctrl.MOVE_FORWARD: "z",
        Ctrl.MOVE_BACKWARD: "s",
    }
)

class event_type(Enum):
    ALTITUDE = 0
    ATTITUDE = 1
    VELOCITY = 2
    POSITION = 3

UDP_TELEMETRY_IP = '127.0.0.1'
UDP_TELEMETRY_PORT = 5556

UDP_COMMANDS_IP = '127.0.0.1'
UDP_COMMANDS_PORT = 5557

class FlightListener(olympe.EventListener):
    def __init__(self, drone):
        # Create telemetry sock
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        super().__init__(drone)

    def send_parrot_telemetry(self, event_type, fmt, data):
        ns = time.time_ns()
        usec = ns / 1000
        sec = usec / 1000000
        usec %= 1000000

        buf = struct.pack("iii" + fmt, int(event_type.value),
                          int(sec), int(usec), *data)
        self._sock.sendto(buf, (UDP_TELEMETRY_IP, UDP_TELEMETRY_PORT))

    @olympe.listen_event(FlyingStateChanged() | AlertStateChanged() | NavigateHomeStateChanged())
    def onStateChanged(self, event, scheduler):
        print("{} = {}".format(event.message.name, event.args["state"]))

    @olympe.listen_event(PositionChanged())
    def onPositionChanged(self, event, scheduler):
        print("latitude = {latitude:.3f} longitude = {longitude:.3f} altitude = {altitude:.3f}".format(**event.args))
        self.send_parrot_telemetry(event_type.POSITION, "fff",
                                   (event.args['latitude'],
                                    event.args['longitude'],
                                    event.args['altitude']))

    @olympe.listen_event(SpeedChanged())
    def onSpeedChanged(self, event, scheduler):
        print("velocity = [{speedX:.3f},{speedY:.3f},{speedZ:.3f}] ".format(**event.args))
        self.send_parrot_telemetry(event_type.VELOCITY, "fff",
                                   (event.args['speedX'],
                                    event.args['speedY'],
                                    event.args['speedZ']))

    @olympe.listen_event(AltitudeChanged())
    def onAltitudeChanged(self, event, scheduler):
        print("altitude = {altitude:.3f}".format(**event.args))
        self.send_parrot_telemetry(event_type.ALTITUDE, "f",
                                   (event.args['altitude'],))

    @olympe.listen_event(AttitudeChanged())
    def onAttitudeChanged(self, event, scheduler):
        print("roll = {roll:.3f}  pitch = {pitch:.3f}  yaw = {yaw:.3f}".format(**event.args))
        self.send_parrot_telemetry(event_type.ATTITUDE, "fff",
                                   (event.args['roll'],
                                    event.args['pitch'],
                                    event.args['yaw']))

class KeyboardCtrl(Listener):
    def __init__(self, ctrl_keys=None):
        self._other_input_blocked = True
        self._ctrl_keys = self._get_ctrl_keys(ctrl_keys)
        self._key_pressed = defaultdict(lambda: False)
        self._last_action_ts = defaultdict(lambda: 0.0)
        super().__init__(on_press=self._on_press, on_release=self._on_release)
        self.start()

    def _on_press(self, key):
        char = None
        if isinstance(key, KeyCode):
            char = key.char
        elif isinstance(key, Key):
            char = key

        self._key_pressed[char] = True

        # Handle enable/disable of other input separetely
        if char == self._ctrl_keys[Ctrl.UNBLOCK_OTHER_INPUT]:
            self._other_input_blocked = False
        elif char == self._ctrl_keys[Ctrl.BLOCK_OTHER_INPUT]:
            self._other_input_blocked = True

        if self._key_pressed[self._ctrl_keys[Ctrl.QUIT]]:
            return False
        else:
            return True

    def _on_release(self, key):
        char = None
        if isinstance(key, KeyCode):
            char = key.char
        elif isinstance(key, Key):
            char = key

        self._key_pressed[char] = False

        return True

    def other_input_blocked(self):
        return self._other_input_blocked

    def quit(self):
        return not self.running or self._key_pressed[self._ctrl_keys[Ctrl.QUIT]]

    def _axis(self, left_key, right_key):
        return 100 * (
            int(self._key_pressed[right_key]) - int(self._key_pressed[left_key])
        )

    def roll(self):
        return self._axis(
            self._ctrl_keys[Ctrl.MOVE_LEFT],
            self._ctrl_keys[Ctrl.MOVE_RIGHT]
        )

    def pitch(self):
        return self._axis(
            self._ctrl_keys[Ctrl.MOVE_BACKWARD],
            self._ctrl_keys[Ctrl.MOVE_FORWARD]
        )

    def yaw(self):
        return self._axis(
            self._ctrl_keys[Ctrl.TURN_LEFT],
            self._ctrl_keys[Ctrl.TURN_RIGHT]
        )

    def throttle(self):
        return self._axis(
            self._ctrl_keys[Ctrl.MOVE_DOWN],
            self._ctrl_keys[Ctrl.MOVE_UP]
        )

    def has_piloting_cmd(self):
        return (
            bool(self.roll())
            or bool(self.pitch())
            or bool(self.yaw())
            or bool(self.throttle())
        )

    def _rate_limit_cmd(self, ctrl, delay):
        now = time.time()
        if self._last_action_ts[ctrl] > (now - delay):
            return False
        elif self._key_pressed[self._ctrl_keys[ctrl]]:
            self._last_action_ts[ctrl] = now
            return True
        else:
            return False

    def takeoff(self):
        return self._rate_limit_cmd(Ctrl.TAKEOFF, 2.0)

    def landing(self):
        return self._rate_limit_cmd(Ctrl.LANDING, 2.0)

    def _get_ctrl_keys(self, ctrl_keys):
        # Get the default ctrl keys based on the current keyboard layout:
        if ctrl_keys is None:
            ctrl_keys = QWERTY_CTRL_KEYS
            try:
                # Olympe currently only support Linux
                # and the following only works on *nix/X11...
                keyboard_variant = (
                    subprocess.check_output(
                        "setxkbmap -query | grep 'variant:'|"
                        "cut -d ':' -f2 | tr -d ' '",
                        shell=True,
                    )
                    .decode()
                    .strip()
                )
            except subprocess.CalledProcessError:
                pass
            else:
                if keyboard_variant == "azerty":
                    ctrl_keys = AZERTY_CTRL_KEYS

        return ctrl_keys

class DronePilotingCmd():
    def __init__(self, roll, pitch, yaw, throttle):
        self._roll = roll
        self._pitch = pitch
        self._yaw = yaw
        self._throttle = throttle

    def roll(self):
        return self._roll

    def pitch(self):
        return self._pitch

    def yaw(self):
        return self._yaw

    def throttle(self):
        return self._throttle

class SockCtrl():
    def __init__(self):
        # Create commands sock
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((UDP_COMMANDS_IP, UDP_COMMANDS_PORT))
        sock.setblocking(0)

        self._sock = sock
        self._cmds = []

    def _process_sock_commands(self):
        # 4x signed chars
        fmt = "bbbb"
        sz = struct.calcsize(fmt)
        # Should be pow2
        assert not ((sz - 1) & sz)

        try:
            buf = self._sock.recv(4096)
        except:
            # EAGAIN
            return

        off = 0
        while off < len(buf):
            (roll, pitch, yaw, throttle) = struct.unpack_from(fmt, buf, off)
            self._cmds.append(DronePilotingCmd(roll, pitch, yaw, throttle))
            off += sz;

    def has_piloting_cmd(self):
        self._process_sock_commands()
        return len(self._cmds) > 0

    def get_piloting_cmd(self):
        return self._cmds.pop(0)

def send_drone_pcmd(drone, cmd):
    drone(PCMD(1,
               cmd.roll(),
               cmd.pitch(),
               cmd.yaw(),
               cmd.throttle(),
               timestampAndSeqNum=0)
    )

if __name__ == "__main__":
    # Reduce log level
    olympe.log.update_config({"loggers": {"olympe": {"level": "WARNING"}}})

    drone = olympe.Drone("192.168.53.1")
    with FlightListener(drone):
        drone.connect()
        drone(setPilotingSource(source="Controller")).wait()
        kb_ctrl = KeyboardCtrl()
        sock_ctrl = SockCtrl()
        while not kb_ctrl.quit():
            other_input_blocked = kb_ctrl.other_input_blocked()

            if kb_ctrl.takeoff():
                drone(TakeOff())
            elif kb_ctrl.landing():
                drone(Landing())

            if kb_ctrl.has_piloting_cmd():
                send_drone_pcmd(drone, kb_ctrl)
            elif sock_ctrl.has_piloting_cmd():
                cmd = sock_ctrl.get_piloting_cmd()
                if not other_input_blocked:
                    send_drone_pcmd(drone, cmd)
            elif other_input_blocked:
                # We send STOP cmd only if no other input
                drone(PCMD(0, 0, 0, 0, 0, timestampAndSeqNum=0))

            if not sock_ctrl.has_piloting_cmd():
                time.sleep(0.05)
