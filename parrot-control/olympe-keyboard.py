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
    ) = range(11)


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

UDP_TELEMETRY_IP = '127.0.0.1'
UDP_TELEMETRY_PORT = 5556


class FlightListener(olympe.EventListener):
    def __init__(self, drone):
        # Create telemetry sock
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self._alt   = 0.0
        self._roll  = 0.0
        self._pitch = 0.0
        self._yaw   = 0.0

        super().__init__(drone)

    def send_parrot_telemetry(self):
        ns = time.time_ns()
        usec = ns / 1000
        sec = usec / 1000000
        usec %= 1000000

        buf = struct.pack("iiffff", int(sec), int(usec), self._alt,
                          self._roll, self._pitch, self._yaw)
        self._sock.sendto(buf, (UDP_TELEMETRY_IP, UDP_TELEMETRY_PORT))

    @olympe.listen_event(FlyingStateChanged() | AlertStateChanged() | NavigateHomeStateChanged())
    def onStateChanged(self, event, scheduler):
        print("{} = {}".format(event.message.name, event.args["state"]))

    @olympe.listen_event(PositionChanged())
    def onPositionChanged(self, event, scheduler):
        print("latitude = {latitude} longitude = {longitude} altitude = {altitude}".format(**event.args))

    @olympe.listen_event(AltitudeChanged())
    def onAltitudeChanged(self, event, scheduler):
        print("altitude = {altitude}".format(**event.args))
        self._alt  = event.args['altitude']
        self.send_parrot_telemetry()

    @olympe.listen_event(AttitudeChanged())
    def onAttitudeChanged(self, event, scheduler):
        print("roll = {roll}  pitch = {pitch}  yaw = {yaw}".format(**event.args))
        self._roll  = event.args['roll']
        self._pitch = event.args['pitch']
        self._yaw   = event.args['yaw']
        self.send_parrot_telemetry()

class KeyboardCtrl(Listener):
    def __init__(self, ctrl_keys=None):
        self._ctrl_keys = self._get_ctrl_keys(ctrl_keys)
        self._key_pressed = defaultdict(lambda: False)
        self._last_action_ts = defaultdict(lambda: 0.0)
        super().__init__(on_press=self._on_press, on_release=self._on_release)
        self.start()

    def _on_press(self, key):
        if isinstance(key, KeyCode):
            self._key_pressed[key.char] = True
        elif isinstance(key, Key):
            self._key_pressed[key] = True
        if self._key_pressed[self._ctrl_keys[Ctrl.QUIT]]:
            return False
        else:
            return True

    def _on_release(self, key):
        if isinstance(key, KeyCode):
            self._key_pressed[key.char] = False
        elif isinstance(key, Key):
            self._key_pressed[key] = False
        return True

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

if __name__ == "__main__":
    # Create UDP sock
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)

    # Reduce log level
    olympe.log.update_config({"loggers": {"olympe": {"level": "WARNING"}}})

    drone = olympe.Drone("192.168.53.1")
    with FlightListener(drone):
        drone.connect()
        drone(setPilotingSource(source="Controller")).wait()
        control = KeyboardCtrl()
        while not control.quit():
            if control.takeoff():
                drone(TakeOff())
            elif control.landing():
                drone(Landing())
            if control.has_piloting_cmd():
                drone(
                    PCMD(
                        1,
                        control.roll(),
                        control.pitch(),
                        control.yaw(),
                        control.throttle(),
                        timestampAndSeqNum=0,
                    )
                )
            else:
                drone(PCMD(0, 0, 0, 0, 0, timestampAndSeqNum=0))
            time.sleep(0.05)
