#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
import evpylib

devices = {
    "left_motor": Motor(Port.C, positive_direction=Direction.CLOCKWISE),
    "right_motor": Motor(Port.B, positive_direction=Direction.COUNTERCLOCKWISE),
    "aux_motor_1": None,
    "aux_motor_2": None,
    "left_sensor": ColorSensor(Port.S2),
    "right_sensor": ColorSensor(Port.S1),
    "aux_sensor_1": None,
    "aux_sensor_2": None,
}

robot = evpylib.Robot(
    devices = devices,
    base_speed=1000,
    trace_speed=1000,
    max_speed=1200,
    turning_const=3.0,
    debug_mode=True,
)

laps = 0
main_start = robot.watch.time()

robot.ev3.speaker.beep(frequency=1000, duration=300)
robot.line_trace_time(
    10000,
    Kp=1, Kd=0.3,
    mode="balance",
)

# robot.turn_arc(98, radius_factor = 550, then = "STOP")
# robot.move_time(1000, ease_in = True, ease_out = True, then = "HOLD")
# while robot.watch.time() - main_start < 60000:
#     robot.move_time(1600, reverse=True, ease_in=True) 
#     robot.move_time(300, ease_in=True, ease_out=True)
#     robot.turn_arc(-117)
#     robot.move_time(2400, ease_in=True, ease_out=True)
#     robot.turn_arc(95)
#     robot.move_time(1050, ease_in=True, ease_out=True)
#     robot.move_time(1450, reverse=True, ease_in=True, ease_out=True)
#     robot.move_time(100, ease_in=True, ease_out=True)
#     robot.turn_arc(100, radius_factor = 300.0, then = "HOLD")
#     robot.move_time(1350, ease_in=True, ease_out=True)
#     robot.turn_arc(-100, radius_factor = 1000.0, then = "HOLD")
#     robot.move_time(650, ease_in=True, ease_out=True)
#     robot.ev3.speaker.beep(frequency=1000, duration=300)
#     laps += 1