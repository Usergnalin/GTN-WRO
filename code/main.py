#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, ColorSensor)
from pybricks.parameters import Port, Stop, Direction
from pybricks.tools import wait, StopWatch
from evpylib import Robot

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

robot = Robot(
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