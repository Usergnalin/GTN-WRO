from pybricks.hubs import InventorHub
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.parameters import Port, Direction
from pybricks.tools import wait
from pybricks.parameters import Stop, Axis
from pybricks.robotics import DriveBase
import spikepylib
import usys


# Initialize the hub
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
    # devices = {
    #     "left_motor": None,
    #     "right_motor": None,
    #     "aux_motor_1": None,
    #     "aux_motor_2": None,
    #     "left_sensor": None,
    #     "right_sensor": None,
    #     "aux_sensor_1": None,
    #     "aux_sensor_2": None,
    #     "line_sensor": None
    # },
    wheel_diameter=62.4,
    axle_track=175,
    base_speed=500,
    trace_speed=900,
    aux_speed=100,
    base_acceleration=1000,
    debug_mode=2
)

print(robot.hub.battery.voltage())

print(robot.wall_align(wall_sensitivity=0.1, acceleration=1000))

# robot.line_trace_junction(side='right', polling_rate=5, speed=1000)
# robot.move_distance(6)
# robot.turn_arc(90)
# robot.turn_arc(90, radius=-320)
# robot.line_trace_junction(side='left', junctions=2, polling_rate=5, speed=1000)
# robot.move_distance(6)
# robot.turn_arc(90)
# robot.turn_arc(90, radius=335)
# robot.line_trace_junction(side='right', junctions=2, polling_rate=5, speed=1000)
# robot.move_distance(6)
# robot.turn_arc(90)
# robot.line_trace_junction(side='right', junctions=2, polling_rate=5, speed=1000)
# robot.move_distance(6)
# robot.turn_arc(-90)
# robot.line_trace_distance(300, side='left', polling_rate=5, speed=1000)
# robot.turn_arc(180)
# for i in range(10):
#     robot.move_distance(30)
#     robot.turn_arc(90)
#     robot.turn_arc(90, radius=30)

# robot.turn_arc(3600)