import time
import socket
import struct

import olympe
import subprocess
import time
import math
from olympe.messages import gimbal
from olympe.messages.camera import (
    start_recording,
    stop_recording,
    set_camera_mode,
    set_recording_mode,
)
from olympe.messages.skyctrl.CoPiloting import setPilotingSource
from olympe.messages.ardrone3.Piloting import TakeOff, Landing, PCMD
from olympe.messages.ardrone3.SpeedSettings import MaxRotationSpeed
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
        START_RECORDING,
        STOP_RECORDING,

        UNBLOCK_OTHER_INPUT,
        BLOCK_OTHER_INPUT,
    ) = range(15)


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
    Ctrl.START_RECORDING: "r",
    Ctrl.STOP_RECORDING: "p",

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

def drone_cmd_pcmd(roll, pitch, yaw, throttle):
    return PCMD(1, roll, pitch, yaw, throttle, timestampAndSeqNum=0)

def drone_cmd_takeoff():
    return TakeOff()

def drone_cmd_land():
    return Landing()

def is_drone_cmd_land(cmd):
    return cmd.command_message.name == "Landing"

def drone_setup_gimbal():
    # Assume max gimbal speed
    drone(gimbal.set_max_speed(
        gimbal_id=0,
        yaw=90,
        pitch=90,
        roll=90)).wait()

    # Camera looks down
    drone(gimbal.set_target(
        gimbal_id=0,
        control_mode="position",
        yaw_frame_of_reference="none",   # None instead of absolute
        yaw=0.0,
        pitch_frame_of_reference="absolute",
        pitch=-90,
        roll_frame_of_reference="none",     # None instead of absolute
        roll=0.0,
    )).wait()

def drone_setup_rotation_speed():
    # In degrees/s
    drone(MaxRotationSpeed(current=90)).wait()

def drone_start_recording(drone):
    drone(set_camera_mode(cam_id=0, value="recording"))
    drone(set_recording_mode(cam_id=0,
                             mode="standard",
                             resolution="res_1080p",
                             framerate="fps_60",
                             hyperlapse="ratio_60"))
    drone(start_recording(cam_id=0))

def drone_stop_recording(drone):
    drone(stop_recording(cam_id=0))

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
        vel = (event.args['speedX'],
               event.args['speedY'],
               event.args['speedZ'])
        self.send_parrot_telemetry(event_type.VELOCITY, "fff", vel)

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

    def block_other_input(self):
        self._other_input_blocked = True

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

    def get_piloting_cmd(self):
        return drone_cmd_pcmd(self.roll(), self.pitch(),
                              self.yaw(), self.throttle())

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

    def start_recording(self):
        return self._rate_limit_cmd(Ctrl.START_RECORDING, 2.0)

    def stop_recording(self):
        return self._rate_limit_cmd(Ctrl.STOP_RECORDING, 2.0)

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
            # Looks clumsy, but does not introduce land commands
            if throttle == -128:
                self._cmds.append(drone_cmd_land())
                throttle = 0
            self._cmds.append(drone_cmd_pcmd(roll, pitch, yaw, throttle))
            off += sz;

    def has_piloting_cmd(self):
        self._process_sock_commands()
        return len(self._cmds) > 0

    def get_piloting_cmd(self):
        return self._cmds.pop(0)

    def drop_piloting_cmds(self):
        self._cmds = []


if __name__ == "__main__":
    # Reduce log level
    olympe.log.update_config({"loggers": {"olympe": {"level": "WARNING"}}})

    drone = olympe.Drone("192.168.53.1")
    with FlightListener(drone):
        drone.connect()
        drone(setPilotingSource(source="Controller")).wait()
        drone_setup_gimbal()
        drone_setup_rotation_speed()

        kb_ctrl = KeyboardCtrl()
        sock_ctrl = SockCtrl()
        while not kb_ctrl.quit():
            other_input_blocked = kb_ctrl.other_input_blocked()

            if kb_ctrl.takeoff():
                drone(drone_cmd_takeoff())
            elif kb_ctrl.landing():
                drone(drone_cmd_land())
                # Block other input after landing
                kb_ctrl.block_other_input()
            elif kb_ctrl.start_recording():
                drone_start_recording(drone)
            elif kb_ctrl.stop_recording():
                drone_stop_recording(drone)

            if kb_ctrl.has_piloting_cmd():
                drone(kb_ctrl.get_piloting_cmd())
            elif sock_ctrl.has_piloting_cmd() and not other_input_blocked:
                cmd = sock_ctrl.get_piloting_cmd()
                drone(cmd)
                if is_drone_cmd_land(cmd):
                    # Block other input after landing
                    kb_ctrl.block_other_input()
            elif other_input_blocked:
                # Drop the whole queue
                sock_ctrl.drop_piloting_cmds()
                # We send STOP cmd only if no other input
                drone(PCMD(0, 0, 0, 0, 0, timestampAndSeqNum=0))

            if not sock_ctrl.has_piloting_cmd():
                time.sleep(0.05)
