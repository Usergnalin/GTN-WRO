from pybricks.hubs import InventorHub
from pybricks.pupdevices import Motor
from pybricks.parameters import Port, Direction
from pybricks.tools import wait, StopWatch
from pybricks.parameters import Stop, Color
from pybricks.robotics import DriveBase
from umath import pi

class Robot:
    def __init__(
        self, devices : dict, 
        wheel_diameter, 
        axle_track, 
        base_speed=1000, 
        trace_speed=700, 
        aux_speed = 200,
        base_acceleration = "auto",
        debug_mode=False
        ):

        self.hub = InventorHub()
        self.debug_mode = debug_mode

        self.left_motor = devices["left_motor"]
        self.right_motor = devices["right_motor"]
        self.aux_motor_1 = devices["aux_motor_1"]
        self.aux_motor_2 = devices["aux_motor_2"]

        self.left_sensor = devices["left_sensor"]
        self.right_sensor = devices["right_sensor"]
        self.aux_sensor_1 = devices["aux_sensor_1"]
        self.aux_sensor_2 = devices["aux_sensor_2"]
        self.line_sensor = devices["line_sensor"]

        self.wheel_diameter = wheel_diameter
        self.axle_track = axle_track

        if self.left_motor and self.right_motor:
            self.drive_base = DriveBase(
                self.left_motor, 
                self.right_motor, 
                wheel_diameter, 
                axle_track
            )

            self.drive_base.use_gyro(True)
        else:
            print("no motors connected, drivebase not initalised")

        self.watch = StopWatch()
        self.BASE_SPEED = base_speed
        self.TRACE_SPEED = trace_speed
        self.AUX_SPEED = aux_speed
        
        if base_acceleration == "auto":
            self.BASE_ACCELERATION = self.drive_base.settings()[1]
        else:
            self.BASE_ACCELERATION = base_acceleration

    def move_distance(self, distance, acceleration = None, speed=None, wait=True):
        self.drive_base.settings(
            straight_speed=self.BASE_ACCELERATION if speed is None else speed, 
            straight_acceleration=self.BASE_ACCELERATION if acceleration is None else acceleration
        )
        self.drive_base.straight(distance, then=Stop.HOLD, wait=True)
        self.drive_base.settings(straight_speed=self.BASE_SPEED, straight_acceleration=self.BASE_ACCELERATION)
    
    def turn_arc(self, angle, radius=0, acceleration=None, speed=None, wait=True):
        """Turn the robot clockwise with a specified radius."""
        self.drive_base.settings(
            turn_rate=self.BASE_SPEED / (abs(radius) + self.axle_track / 2) * (180 / pi) if speed is None else abs(speed) / (abs(radius) + self.axle_track / 2) * (180 / pi), 
            turn_acceleration=self.BASE_ACCELERATION if acceleration is None else acceleration
        )
        if radius == 0: self.drive_base.turn(angle, then=Stop.HOLD, wait=wait)
        else: self.drive_base.arc(radius, angle, then=Stop.HOLD, wait=wait)
        self.drive_base.settings(straight_speed=self.BASE_SPEED, straight_acceleration=self.BASE_ACCELERATION)

    def line_trace_distance(
        self,
        distance: float,
        side=None,
        speed=None,
        Kp=2,
        Kd=50,
        TRACE_TARGET=50,
        polling_rate=10,
        ease_duration=1000,
        stop=True,
    ):
        """
        Perform PD-controlled line tracing until the robot travels a specified distance.

        The function supports either a single line sensor (left or right side) or a dual
        sensor setup. It gradually increases speed at the start for smoother acceleration
        and maintains a constant base speed afterward. Steering is controlled using a
        proportional-derivative (PD) response to sensor error.

        Parameters:
            distance (float): Target travel distance in millimeters.
            side (str | None): 'left' or 'right' for single sensor setups. Ignored for dual sensors.
            speed (float | None): Base motor speed. Defaults to `self.TRACE_SPEED`.
            Kp (float): Proportional gain for PD control.
            Kd (float): Derivative gain for PD control.
            TRACE_TARGET (float): Target reflection value used for line following.
            polling_rate (int): Minimum time in milliseconds between sensor readings.
            ease_duration (int): Duration in milliseconds to ramp up from zero to full speed.
            stop (bool): Whether to stop the motors after reaching the target distance.
        """

        if self.debug_mode:
            print("Starting line trace until {} mm".format(distance))

        if not speed:
            speed = self.TRACE_SPEED

        using_single_sensor = self.line_sensor is not None
        using_dual_sensors = self.left_sensor is not None and self.right_sensor is not None

        if not using_single_sensor and not using_dual_sensors:
            raise ValueError("No sensors available for line tracing.")

        if using_single_sensor:
            if side is None:
                print("Parameter 'side' not specified. Defaulting to 'left'.")
                side = "left"
            elif side not in ("left", "right"):
                raise ValueError("Side must be 'left' or 'right' when using a single line sensor.")

            if side == "left":
                side_multiplier = 1
            else:
                side_multiplier = -1

        if using_dual_sensors and side is not None:
            print("Parameter 'side' is ignored when using two sensors.")

        last_error = None
        start_time = self.watch.time()
        start_distance = self.drive_base.distance()
        last_update = start_time

        while True:
            delta_time = self.watch.time() - last_update
            wait(max(0, polling_rate - delta_time))
            delta_time = self.watch.time() - last_update

            if using_single_sensor:
                reflection = self.line_sensor.reflection()
                if reflection is None:
                    continue

                error = reflection - TRACE_TARGET
                turn = Kp * error + Kd * ((error - last_error) / delta_time if delta_time and last_error is not None else 0)

                speed_left = speed
                speed_right = speed

                speed_left += turn * side_multiplier
                speed_right -= turn * side_multiplier

            else:  # using two sensors
                left_reflection = self.left_sensor.reflection()
                right_reflection = self.right_sensor.reflection()

                if left_reflection is None or right_reflection is None:
                    continue

                error = (left_reflection - TRACE_TARGET) - (right_reflection - TRACE_TARGET)
                turn = Kp * error + Kd * ((error - last_error) / delta_time if delta_time and last_error is not None else 0)

                speed_left = speed + turn
                speed_right = speed - turn

            elapsed = self.watch.time() - start_time
            ease_factor = min(1.0, elapsed / ease_duration) if ease_duration > 0 else 1.0
            speed_left *= ease_factor
            speed_right *= ease_factor

            if (self.drive_base.distance() - start_distance >= distance):
                if self.debug_mode == 2:
                    print("Error: {:.1f}, Derivative: {:.3f} Delta_time: {:.0f}".format(
                        error, (error - last_error) / delta_time if delta_time else 0, delta_time
                    ))
                break

            # Debug print
            if self.debug_mode == 2:
                print("Error: {:.1f}, Derivative: {:.3f} Delta_time: {:.0f}".format(
                    error, (error - last_error) / delta_time if delta_time else 0, self.watch.time() - last_update
                ))

            last_update = self.watch.time()
            last_error = error

            self.left_motor.run(speed_left)
            self.right_motor.run(speed_right)

        if stop:
            self.drive_base.stop()

    def line_trace_junction(
        self,
        junctions = 1,
        side = None,
        Kp = 2, Kd = 50,
        ease_in_duration = 1000,
        junction_sensitivity = 1,
        stop = True,
        TRACE_TARGET = 50,
        speed = None,
        polling_rate = 10,
        correct_junction_error = True,
        correction_grace_period = 200,
    ):
        """
        Perform PD-controlled line tracing until a specified number of junctions is detected.

        The function supports single or dual line sensors and gradually increases speed
        at the start for smoother acceleration. Junctions are detected based on sudden
        changes in sensor readings (derivative), and the robot can optionally correct
        its heading after stopping.

        Parameters:
            junctions (int): Number of junctions to detect before stopping.
            side (str | None): 'left' or 'right' for single sensor setups. Ignored for dual sensors.
            Kp (float): Proportional gain for PD control.
            Kd (float): Derivative gain for PD control.
            ease_in_duration (int): Duration in milliseconds to ramp up speed from zero.
            junction_sensitivity (float): Threshold for detecting junctions based on derivative.
            stop (bool): Whether to stop motors after reaching the target junctions.
            TRACE_TARGET (float): Target sensor value for line following.
            speed (float | None): Base motor speed. Defaults to `self.TRACE_SPEED`.
            polling_rate (int): Minimum time in ms between sensor readings.
            correct_junction_error (bool): Whether to correct heading after stopping.
            correction_grace_period (int): Time in ms to wait before heading correction.
        """
        if not speed:
            speed = self.TRACE_SPEED

        use_single_sensor = bool(self.line_sensor)
        use_dual_sensor   = bool(self.left_sensor and self.right_sensor)

        if use_single_sensor and use_dual_sensor:
            raise ValueError("Both single and dual sensor modes exist. Choose one explicitly.")

        if not use_single_sensor and not use_dual_sensor:
            raise ValueError("No sensors available for line tracing.")

        if use_single_sensor:
            if side is None:
                side = "left"
                print("Side not specified. Defaulting to 'left'.")
            if side not in ("left", "right"):
                raise ValueError("Side must be 'left' or 'right' for a single sensor.")

            if side == "left":
                side_direction = 1
            else:
                side_direction = -1

        last_error_value = None
        last_junction_timestamp = 0
        detected_junction_count = 0

        start_time = self.watch.time()
        last_update_time = start_time

        heading_history = []

        while True:
            current_time = self.watch.time()
            time_since_last_update = current_time - last_update_time

            wait(max(0, polling_rate - time_since_last_update))

            new_time = self.watch.time()
            delta_time = new_time - current_time
            last_update_time = new_time

            # Compute error ---------------------------------------------------------
            if use_single_sensor:
                sensor_value = self.line_sensor.reflection()
                if sensor_value is None:
                    continue
                error_value = sensor_value - TRACE_TARGET

            else:  # dual sensor mode
                left_value = self.left_sensor.reflection()
                right_value = self.right_sensor.reflection()
                if left_value is None or right_value is None:
                    continue

                error_value = (left_value - TRACE_TARGET) - (right_value - TRACE_TARGET)

            # PD derivative ---------------------------------------------------------
            if last_error_value is not None and delta_time:
                error_derivative = (error_value - last_error_value) / delta_time
            else:
                error_derivative = 0

            turn = Kp * error_value + Kd * error_derivative

            # Speed easing ----------------------------------------------------------
            elapsed_time = self.watch.time() - start_time
            if ease_in_duration > 0:
                ease_factor = min(1, elapsed_time / ease_in_duration)
            else:
                ease_factor = 1

            # Junction detection ----------------------------------------------------
            if ease_factor >= 1:
                derivative_threshold = 1 / junction_sensitivity
                junction_detected = abs(error_derivative) > derivative_threshold
                junction_cooldown_passed = (self.watch.time() - last_junction_timestamp) > 500

                heading_history.append(self.hub.imu.heading())
                if len(heading_history) > 100:
                    heading_history.pop(0)

                if junction_detected and (last_junction_timestamp == 0 or junction_cooldown_passed):
                    detected_junction_count += 1
                    last_junction_timestamp = self.watch.time()

                    if detected_junction_count >= junctions:
                        break

            # Motor control ---------------------------------------------------------
            if use_single_sensor:
                left_speed = speed * ease_factor + turn * side_direction
                right_speed = speed * ease_factor - turn * side_direction
            else:
                left_speed = speed * ease_factor + turn
                right_speed = speed * ease_factor - turn

            self.left_motor.run(left_speed)
            self.right_motor.run(right_speed)

            last_error_value = error_value

        # After loop ---------------------------------------------------------------
        if stop:
            self.drive_base.hold()
            wait(correction_grace_period)

            if correct_junction_error and heading_history:
                median_heading = sorted(heading_history)[len(heading_history) // 2]
                heading_difference = median_heading - self.hub.imu.heading()
                self.drive_base.turn(heading_difference)

    def wall_align(self, reversed = True, acceleration=None, speed = None, wall_sensitivity = 1, polling_rate=10):
        """
        Moves robot forward until it hits a wall (motor stalls).
        Applies easing and motor angle correction while moving.
        Uses debounce_duration to confirm wall hit.
        
        Parameters:
        - debounce_duration: int, ms to confirm stall before stopping
        - ease_in: bool, gradually increase speed at start
        - polling_rate: int, ms between control updates
        - correction: bool, motor angle correction to keep straight
        - then: str, one of "HOLD", "STOP", or "BRAKE" after movement ends
        """

        if not speed: speed = self.BASE_SPEED
        self.drive_base.settings(straight_acceleration=self.BASE_ACCELERATION if acceleration == None else acceleration)
        
        self.drive_base.drive(-speed if reversed else speed, turn_rate=0)
        wait(200)
        previous_load = max(self.right_motor.load(), self.left_motor.load())
        start_heading = self.hub.imu.heading()
        while True:
            current_load = max(self.right_motor.load(), self.left_motor.load())
            change = current_load - previous_load
            if self.debug_mode >= 2: print(change)
            if(change > wall_sensitivity ** -1): break
            previous_load = current_load
            wait(polling_rate)

        self.left_motor.dc(-50 if reversed else 50)
        self.right_motor.dc(-50 if reversed else 50)
        wait(200)
        self.left_motor.dc(0)
        self.right_motor.dc(0)
        wait(100)

        corrected_angle = self.hub.imu.heading() - start_heading

        self.drive_base.settings(straight_acceleration=self.BASE_ACCELERATION)
        return corrected_angle
                

