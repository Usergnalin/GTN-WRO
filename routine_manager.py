from pybricks.hubs import InventorHub
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.parameters import Port, Direction
import json
import spikepylib

hub = InventorHub()
robot = spikepylib.Robot(
    devices = {
        "left_motor": Motor(Port.C, Direction.COUNTERCLOCKWISE),
        "right_motor": Motor(Port.E, Direction.CLOCKWISE),
        "aux_motor_1": None,
        "aux_motor_2": None,
        "left_sensor": None,
        "right_sensor": None,
        "aux_sensor_1": None,
        "aux_sensor_2": None,
        "line_sensor": ColorSensor(Port.A)
    },
    wheel_diameter=62.4,
    axle_track=175,
    base_speed=500,
    trace_speed=900,      
    aux_speed=100,
    base_acceleration=1000,
    debug_mode=2
)

COMMANDS = {
    "move_distance": robot.move_distance,
    "turn_arc": robot.turn_arc,
    "line_trace_distance": robot.line_trace_distance,
    "line_trace_junction": robot.line_trace_junction,
    "wall_align": robot.wall_align
}

with open("routines/test.json") as file:
    routine = json.load(file)['routine']

for step in routine:
    function = COMMANDS[step["fn"]]
    params = step.copy()
    params.pop("fn")
    function(**params)