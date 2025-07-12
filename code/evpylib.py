from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor
from pybricks.parameters import Port, Stop
from pybricks.tools import wait, StopWatch

class Robot:
    def __init__(self, devices : dict, base_speed=1000, trace_speed=700, max_speed=1200, 
                 turning_const=2.2, debug_mode=False):

        self.ev3 = EV3Brick()
        self.debug_mode = debug_mode

        self.left_motor = devices["left_motor"]
        self.right_motor = devices["right_motor"]
        self.aux_motor_1 = devices["aux_motor_1"]
        self.aux_motor_2 = devices["aux_motor_2"]

        self.left_sensor = devices["left_sensor"]
        self.right_sensor = devices["right_sensor"]
        self.aux_sensor_1 = devices["aux_sensor_1"]
        self.aux_sensor_2 = devices["aux_sensor_2"]

        self.watch = StopWatch()
        self.BASE_SPEED = base_speed
        self.TRACE_SPEED = trace_speed
        self.MAX_SPEED = max_speed
        self.TURN_CONST = turning_const

    def line_trace_time(
        self,
        duration : int,
        Kp, Kd,
        ease_duration : int = 400,
        mode="balance",
        polling_rate : int = 5,
        then="HOLD",
        TRACE_TARGET=50,):
        """
        PD line tracing with multiple modes.

        Parameters:
        - duration: total running time in ms
        - Kp, Kd: PD constants
        - ease_duration: time in ms to gradually increase speed at start
        - mode: string, one of "balance", "left_only", "right_only", "left_minus_right", "right_minus_left"
        - polling_rate: wait time between updates in ms
        - then: what to do after duration ends: "HOLD", "STOP", or "BRAKE"
        """

        if self.debug_mode: print(f"Starting line trace for {duration} ms with mode '{mode}'")
        last_error = 0
        start = self.watch.time()  # Start stopwatch

        while (self.watch.time() - start) < duration:
            left_val = self.left_sensor.reflection()
            right_val = self.right_sensor.reflection()
            print(left_val, right_val)  # Debug output

            if left_val is not None and right_val is not None:
                # Calculate error based on mode
                if mode == "balance":
                    error = (TRACE_TARGET - left_val) - (TRACE_TARGET - right_val)
                elif mode == "left_only":
                    error = TRACE_TARGET - left_val
                elif mode == "right_only":
                    error = TRACE_TARGET - right_val
                elif mode == "left_minus_right":
                    error = left_val - right_val
                elif mode == "right_minus_left":
                    error = right_val - left_val
                else:
                    error = 0  # default no correction

                derivative = error - last_error
                turn = Kp * error + Kd * derivative

                ease_factor = min(1.0, self.watch.time() / ease_duration) if ease_duration > 0 else 1.0

                speed_left = min(max(self.TRACE_SPEED * ease_factor + turn, 0), self.MAX_SPEED)
                speed_right = min(max(self.TRACE_SPEED * ease_factor - turn, 0), self.MAX_SPEED)

                self.left_motor.run(speed_left)
                self.right_motor.run(speed_right)

                last_error = error

            wait(polling_rate)

        # After loop ends
        if then == "HOLD":
            self.left_motor.hold()
            self.right_motor.hold()
        elif then == "STOP":
            self.left_motor.stop()
            self.right_motor.stop()
        elif then == "BRAKE":
            self.left_motor.brake()
            self.right_motor.brake()

    def line_trace_junction(
        self,
        Kp, Kd,
        mode="balance",
        ease_duration: int = 400,
        polling_rate: int = 5,
        then="HOLD",
        TRACE_TARGET=50,
        ):
        """
        PD line tracing with multiple modes.

        Parameters:
        - ease_duration: time in ms to gradually increase speed at start
        - Kp, Kd: PD constants
        - mode: string, one of "balance", "left_only", "right_only", "left_minus_right", "right_minus_left"
        - polling_rate: wait time between updates in ms
        - then: what to do after duration ends: "HOLD", "STOP", or "BRAKE"
        """

        if self.debug_mode:
            print(f"Starting line trace until junction with mode '{mode}'")

        last_error = 0
        junction_threshold = 20  # Threshold to detect junction
        start_time = self.watch.time()  # Start stopwatch

        while True:
            left_val = self.left_sensor.reflection()
            right_val = self.right_sensor.reflection()

            if left_val < junction_threshold or right_val < junction_threshold:
                break

            if left_val is not None and right_val is not None:
                # Calculate error based on mode
                if mode == "balance":
                    error = (TRACE_TARGET - left_val) - (TRACE_TARGET - right_val)
                elif mode == "left_only":
                    error = TRACE_TARGET - left_val
                elif mode == "right_only":
                    error = TRACE_TARGET - right_val
                elif mode == "left_minus_right":
                    error = left_val - right_val
                elif mode == "right_minus_left":
                    error = right_val - left_val
                else:
                    error = 0

                derivative = error - last_error
                turn = Kp * error + Kd * derivative

                # Linear ease-in
                elapsed = self.watch.time() - start_time
                ease_factor = min(1.0, elapsed / ease_duration) if ease_duration > 0 else 1.0

                speed_left = min(max(self.TRACE_SPEED * ease_factor + turn, 0), self.MAX_SPEED)
                speed_right = min(max(self.TRACE_SPEED * ease_factor - turn, 0), self.MAX_SPEED)

                self.left_motor.run(speed_left)
                self.right_motor.run(speed_right)

                last_error = error

            wait(polling_rate)

        # After loop ends
        if then == "HOLD":
            self.left_motor.hold()
            self.right_motor.hold()
        elif then == "STOP":
            self.left_motor.stop()
            self.right_motor.stop()
        elif then == "BRAKE":
            self.left_motor.brake()
            self.right_motor.brake()

    def turn_arc(self, angle: float, radius_factor=0.0, then="HOLD"):
        """
        Turns the robot along an arc using differential wheel speeds.

        Parameters:
        - angle: float, turn amount in degrees. Positive for right, negative for left.
        - radius_factor: float, adjusts arc radius (0 = in-place turn, higher = wider arc).
        - then: action after turn ends. One of "HOLD", "STOP", or "BRAKE".
        """
        if self.debug_mode: print(f"Turning arc with angle {angle} and radius factor {radius_factor}")
        if then == "HOLD":
            then = Stop.HOLD
        elif then == "STOP":
            then = Stop.COAST
        elif then == "BRAKE":
            then = Stop.BRAKE

        left_angle = angle * self.TURN_CONST + radius_factor
        right_angle = -angle * self.TURN_CONST + radius_factor

        abs_left = abs(left_angle)
        abs_right = abs(right_angle)

        if abs_left > abs_right:
            ratio = abs_left / abs_right
            speed_left = self.BASE_SPEED
            speed_right = self.BASE_SPEED / ratio
        else:
            ratio = abs_right / abs_left
            speed_right = self.BASE_SPEED
            speed_left = self.BASE_SPEED / ratio

        self.left_motor.run_angle(speed_left if left_angle >= 0 else -speed_left, abs_left, wait=False, then=then)
        self.right_motor.run_angle(speed_right if right_angle >= 0 else -speed_right, abs_right, then=then)
    
    def move_rotations(self, rotations: float, then="HOLD"):
        """
        Moves the robot forward for a given number of wheel rotations.

        Parameters:
        - rotations: float, number of full rotations to move (positive = forward, negative = backward).
        - then: action after movement ends. One of "HOLD", "STOP", or "BRAKE".
        """
        if self.debug_mode: print(f"Moving {rotations} rotations with action '{then}'")

        if then == "HOLD":
            then = Stop.HOLD
        elif then == "STOP":
            then = Stop.COAST
        elif then == "BRAKE":
            then = Stop.BRAKE

        self.left_motor.run_angle(self.BASE_SPEED, rotations * 360, wait=False, then=then)
        self.right_motor.run_angle(self.BASE_SPEED, rotations * 360, then=then)

    def move_time(self, duration: float, reverse: bool = False, ease_in: bool = False, ease_out: bool = False, polling_rate: int = 10, then: str = "HOLD", correction: bool = True):
        """
        Moves the robot forward or backward for a specific time with optional easing and correction.

        Parameters:
        - duration: float, movement time in milliseconds.
        - reverse: bool, move backward if True.
        - ease_in: bool, gradually increase speed at start.
        - ease_out: bool, gradually decrease speed at end.
        - polling_rate: int, time in ms between control updates.
        - then: str, one of "HOLD", "STOP", or "BRAKE" after movement ends.
        - correction: bool, enable motor angle correction to keep a straight path.
        """
        if self.debug_mode: print(f"Moving for {duration} ms, reverse={reverse}, ease_in={ease_in}, ease_out={ease_out}, correction={correction}, then='{then}'")
        k = 0.5  # Correction strength

        # Determine easing duration
        if ease_in and ease_out:
            ease_duration = min(400, duration / 2)
        elif ease_in or ease_out:
            ease_duration = min(400, duration)
        else:
            ease_duration = 0

        ease_factor = 0 if ease_in else 1

        start = self.watch.time()
        delta_time = self.watch.time() - start

        self.left_motor.reset_angle(0)
        self.right_motor.reset_angle(0)

        while delta_time < duration:
            # Adjust easing factor
            if ease_in and delta_time < ease_duration:
                ease_factor = min(1, delta_time / ease_duration)
            elif ease_out and duration - delta_time < ease_duration:
                ease_factor = max(0, (duration - delta_time) / ease_duration)

            # Apply correction if enabled
            if correction:
                error = self.right_motor.angle() - self.left_motor.angle()
                correction_val = k * error
            else:
                correction_val = 0

            speed_left = (self.BASE_SPEED - correction_val) * ease_factor
            speed_right = (self.BASE_SPEED + correction_val) * ease_factor

            if reverse:
                self.left_motor.run(-speed_left)
                self.right_motor.run(-speed_right)
            else:
                self.left_motor.run(speed_left)
                self.right_motor.run(speed_right)

            wait(polling_rate)
            delta_time = self.watch.time() - start

        # Stop behavior
        if then == "HOLD":
            self.left_motor.hold()
            self.right_motor.hold()
        elif then == "STOP":
            self.left_motor.stop()
            self.right_motor.stop()
        elif then == "BRAKE":
            self.left_motor.brake()
            self.right_motor.brake()
    
    def bump_align(self, debounce_duration: int = 100, ease_in: bool = False, polling_rate: int = 10, correction: bool = True, then: str = "HOLD"):
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
        if self.debug_mode: print(f"Bump align with debounce_duration={debounce_duration}, ease_in={ease_in}, correction={correction}, then='{then}'")
        k = 0.5  # correction strength
        ease_duration = 400 if ease_in else 0
        stall_threshold = 100

        start = self.watch.time()
        last_movement_time = start
        speed_factor = 0 if ease_in else 1

        self.left_motor.reset_angle(0)
        self.right_motor.reset_angle(0)

        while True:
            now = self.watch.time()
            delta_time = now - start

            # Ease-in speed factor
            if ease_in and delta_time < ease_duration:
                speed_factor = min(1, delta_time / ease_duration)
            else:
                speed_factor = 1

            # Correction to keep straight path
            if correction:
                error = self.right_motor.angle() - self.left_motor.angle()
                correction_val = k * error
            else:
                correction_val = 0

            speed_left = (self.BASE_SPEED - correction_val) * speed_factor
            speed_right = (self.BASE_SPEED + correction_val) * speed_factor

            self.left_motor.run(speed_left)
            self.right_motor.run(speed_right)

            # Detect stall by checking if motors have moved significantly
            print(self.left_motor.speed())
            left_moved = abs(self.left_motor.speed()) > stall_threshold
            right_moved = abs(self.right_motor.speed()) > stall_threshold

            if left_moved or right_moved:
                last_movement_time = now

            # If no movement detected for debounce_duration, assume hit wall
            if now - last_movement_time > debounce_duration:
                break

            wait(polling_rate)

        # Stop motors based on 'then' parameter
        if then == "HOLD":
            self.left_motor.hold()
            self.right_motor.hold()
        elif then == "STOP":
            self.left_motor.stop()
            self.right_motor.stop()
        elif then == "BRAKE":
            self.left_motor.brake()
            self.right_motor.brake()

    def move_aux_angle(self, angle, motor_number : int):
        """
        Moves the auxiliary motor to a specific angle.

        Parameters:
        - angle: float, target angle in degrees.
        - motor_number: int, 1 or 2 to select auxiliary motor.
        """
        if self.debug_mode: print(f"Moving auxiliary motor {motor_number} to angle {angle}")
        if motor_number == 1:
            if not self.aux_motor_1:
                raise ValueError("Auxiliary motor 1 is not initialized.")
            aux_motor = self.aux_motor_1
        elif motor_number == 2:
            if not self.aux_motor_2:
                raise ValueError("Auxiliary motor 2 is not initialized.")
            aux_motor = self.aux_motor_2
        else:
            raise ValueError("Invalid motor number. Use 1 or 2.")
        
        aux_motor.run_angle(self.BASE_SPEED, angle, then=Stop.HOLD)
    
    def move_aux_stall(self, motor_number : int, stall_threshold=10, polling_rate=10):
        """
        Moves the auxiliary motor until it stalls.

        Parameters:
        - motor_number: int, 1 or 2 to select auxiliary motor.
        - stall_threshold: int, angle threshold to detect stall.
        - polling_rate: int, ms between control updates.
        """
        if self.debug_mode: print(f"Moving auxiliary motor {motor_number} until stall with threshold {stall_threshold}")
        if motor_number == 1:
            if not self.aux_motor_1:
                raise ValueError("Auxiliary motor 1 is not initialized.")
            aux_motor = self.aux_motor_1
        elif motor_number == 2:
            if not self.aux_motor_2:
                raise ValueError("Auxiliary motor 2 is not initialized.")
            aux_motor = self.aux_motor_2
        else:
            raise ValueError("Invalid motor number. Use 1 or 2.")
        
        aux_motor.run(self.BASE_SPEED)

        while abs(self.aux_motor.angle()) < stall_threshold:
            wait(polling_rate)

        aux_motor.stop()
