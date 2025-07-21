#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, ColorSensor)
from pybricks.parameters import Port, Stop, Direction
from pybricks.tools import wait, StopWatch
from evpylib import Robot

devices = {
    "left_motor": Motor(Port.C, positive_direction=Direction.COUNTERCLOCKWISE),
    "right_motor": Motor(Port.B, positive_direction=Direction.CLOCKWISE),
    "aux_motor_1": Motor(Port.D, positive_direction=Direction.CLOCKWISE),
    "aux_motor_2": Motor(Port.A, positive_direction=Direction.CLOCKWISE),
    "left_sensor": ColorSensor(Port.S2),
    "right_sensor": ColorSensor(Port.S1),
    "aux_sensor_1": ColorSensor(Port.S3),
    "aux_sensor_2": ColorSensor(Port.S4),
}

def safe_routine():

    robot = Robot(
        devices = devices,
        base_speed=1000,
        trace_speed=800,
        max_speed=1200,
        aux_speed=100,
        turning_const=2.760,
        debug_mode=1,
    )

    robot.ev3.speaker.beep(frequency=1000, duration=300)
    robot.move_rotations(0.5)
    robot.line_trace_junction(2, 0.6)
    robot.turn_arc(-95)
    robot.line_trace_time(1000, 1, 0.3)
    robot.move_rotations(0.5)

def normal_routine():

    robot = Robot(
        devices = devices,
        base_speed=1000,
        trace_speed=800,
        max_speed=1200,
        aux_speed=100,
        turning_const=2.760,
        debug_mode=1,
    )

    robot.ev3.speaker.beep(frequency=1000, duration=200)

    #start routine
    robot.move_rotations(0.5, then="STOP")
    robot.line_trace_junction(1, 0.6)
    robot.turn_arc(-95)
    robot.line_trace_time(1000, 1, 0.3)
    robot.move_rotations(0.5)

def test():
    robot = Robot(
        devices = devices,
        base_speed=1000,
        trace_speed=800,
        max_speed=1200,
        aux_speed=100,
        turning_const=2.760,
        debug_mode=1,
    )
    robot.ev3.speaker.beep(frequency=1000, duration=300)

normal_routine()