import time
import socket
import struct

import olympe
from olympe.messages.skyctrl.CoPiloting import setPilotingSource
from olympe.messages.ardrone3.Piloting import TakeOff, Landing, moveBy
from olympe.messages.ardrone3.PilotingState import (
    PositionChanged,
    AlertStateChanged,
    FlyingStateChanged,
    AltitudeChanged,
    AttitudeChanged,
    NavigateHomeStateChanged,
)

SKYCTRL_IP = "192.168.53.1"

UDP_IP = '127.0.0.1'
UDP_PORT = 5556

alt   = 0.0
roll  = 0.0
pitch = 0.0
yaw   = 0.0

def send_parrot_data(sock, alt, roll, pitch, yaw):
    ns = time.time_ns()
    usec = ns / 1000
    sec = usec / 1000000
    usec %= 1000000

    buf = struct.pack("iiffff", int(sec), int(usec), alt, roll, pitch, yaw)
    sock.sendto(buf, (UDP_IP, UDP_PORT))

class FlightListener(olympe.EventListener):

    @olympe.listen_event(FlyingStateChanged() | AlertStateChanged() | NavigateHomeStateChanged())
    def onStateChanged(self, event, scheduler):
        print("{} = {}".format(event.message.name, event.args["state"]))

    @olympe.listen_event(PositionChanged())
    def onPositionChanged(self, event, scheduler):
        print("latitude = {latitude} longitude = {longitude} altitude = {altitude}".format(**event.args))

    @olympe.listen_event(AltitudeChanged())
    def onAltitudeChanged(self, event, scheduler):
        global sock, alt, roll, pitch, yaw

        alt  = event.args['altitude']

        send_parrot_data(sock, alt, roll, pitch, yaw)

    @olympe.listen_event(AttitudeChanged())
    def onAttitudeChanged(self, event, scheduler):
        global sock, alt, roll, pitch, yaw

        roll  = event.args['roll']
        pitch = event.args['pitch']
        yaw   = event.args['yaw']

        send_parrot_data(sock, alt, roll, pitch, yaw)

# Create UDP sock
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)


#drone = olympe.Drone("10.202.0.1")
drone = olympe.Drone(SKYCTRL_IP)
with FlightListener(drone):
    drone.connect()
    drone(setPilotingSource(source="Controller")).wait()
    drone(
        FlyingStateChanged(state="hovering")
        | (TakeOff() & FlyingStateChanged(state="hovering"))
    ).wait()
    drone(moveBy(1, 0, 0, 0)).wait()
    drone(moveBy(0, 0, 0, 3.14)).wait()
    drone(moveBy(1, 0, 0, 0)).wait()
    drone(moveBy(0, 0, 0, 3.14)).wait()
    drone(Landing()).wait()
    drone(FlyingStateChanged(state="landed")).wait()
    drone.disconnect()
