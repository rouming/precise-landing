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

class FlightListener(olympe.EventListener):

    @olympe.listen_event(FlyingStateChanged() | AlertStateChanged() | NavigateHomeStateChanged())
    def onStateChanged(self, event, scheduler):
        print("{} = {}".format(event.message.name, event.args["state"]))

    @olympe.listen_event(PositionChanged())
    def onPositionChanged(self, event, scheduler):
        print(
            "latitude = {latitude} longitude = {longitude} altitude = {altitude}".format(
                **event.args
            )
        )

    @olympe.listen_event(AltitudeChanged())
    def onAltitudeChanged(self, event, scheduler):
        print(
            "altitude = {altitude}".format(
                **event.args
            )
        )

    @olympe.listen_event(AttitudeChanged())
    def onAttitudeChanged(self, event, scheduler):
        print(
            "roll = {roll} pitch = {pitch} yaw = {yaw}".format(
                **event.args
            )
        )


SKYCTRL_IP = "192.168.53.1"

#drone = olympe.Drone("10.202.0.1")
drone = olympe.Drone(SKYCTRL_IP)
with FlightListener(drone):
    drone.connect()
    drone(setPilotingSource(source="Controller")).wait()
    drone(
        FlyingStateChanged(state="hovering")
        | (TakeOff() & FlyingStateChanged(state="hovering"))
    ).wait()
    drone(moveBy(1.5, 0, 0, 0)).wait()
    drone(moveBy(0, 0, 0, 3.14)).wait()
    drone(moveBy(1.5, 0, 0, 0)).wait()
    drone(moveBy(0, 0, 0, 3.14)).wait()
    drone(Landing()).wait()
    drone(FlyingStateChanged(state="landed")).wait()
    drone.disconnect()
