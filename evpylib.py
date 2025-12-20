from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor
from pybricks.parameters import Port, Stop
from pybricks.tools import wait, StopWatch
from random import choice as rand_choice

class Robot:
    def __init__(self, devices : dict, base_speed=1000, trace_speed=700, max_speed=1200, aux_speed = 200,
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
        self.gyro_sensor = devices["gyro_sensor"]
        self.line_sensor = devices["line_sensor"]

        self.watch = StopWatch()
        self.BASE_SPEED = base_speed
        self.TRACE_SPEED = trace_speed
        self.MAX_SPEED = max_speed
        self.AUX_SPEED = aux_speed
        self.TURN_CONST = turning_const

    def line_trace_time(
        self,
        duration : int,
        Kp, Kd,
        ease_duration : int = 2000,
        mode="balance",
        polling_rate : int = 5,
        then="HOLD",
        TRACE_TARGET=50,
        speed = None):
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

        if self.debug_mode: print("Starting line trace for {} ms with mode '{}'".format(duration, mode))
        last_error = 0
        start = self.watch.time()  # Start stopwatch
        if not speed: speed = self.TRACE_SPEED

        while (self.watch.time() - start) < duration:
            left_val = self.left_sensor.reflection()
            right_val = self.right_sensor.reflection()

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

                speed_left = min(max(speed * ease_factor + turn, 0), self.MAX_SPEED)
                speed_right = min(max(speed * ease_factor - turn, 0), self.MAX_SPEED)

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

    # def line_trace_junction(
    #     self,
    #     Kp, Kd,
    #     junction_count: int = 1,
    #     mode="balance",
    #     ease_duration: int = 400,
    #     polling_rate: int = 10,
    #     then="HOLD",
    #     TRACE_TARGET=50,
    #     junction_threshold: int = 30,
    #     speed = None
    # ):
    #     """
    #     PD line tracing with multiple modes until a specified number of junctions.

    #     Parameters:
    #     - ease_duration: time in ms to gradually increase speed at start
    #     - Kp, Kd: PD constants
    #     - mode: string, one of "balance", "left_only", "right_only", "left_minus_right", "right_minus_left"
    #     - polling_rate: wait time between updates in ms
    #     - then: what to do after target junctions: "HOLD", "STOP", "BRAKE"
    #     - junction_count: how many junctions to detect before stopping
    #     """

    #     if self.debug_mode:
    #         print("Starting line trace until {} junction(s) with mode '{}'".format(junction_count, mode))

    #     last_error = 0
    #     last_junction_time = 0
    #     junctions_detected = 0
    #     start_time = self.watch.time()
    #     last_update = start_time
    #     stability = 0
    #     if not speed: speed = self.TRACE_SPEED

    #     while True:
    #         left_val = self.left_sensor.reflection()
    #         right_val = self.right_sensor.reflection()

    #         if left_val is not None and right_val is not None:
    #             # Calculate error based on mode
    #             if mode == "balance":
    #                 error = (TRACE_TARGET - left_val) - (TRACE_TARGET - right_val)
    #             elif mode == "left_only":
    #                 error = TRACE_TARGET - left_val
    #             elif mode == "right_only":
    #                 error = TRACE_TARGET - right_val
    #             elif mode == "left_minus_right":
    #                 error = left_val - right_val
    #             elif mode == "right_minus_left":
    #                 error = right_val - left_val
    #             else:
    #                 error = 0

    #             delta_time = self.watch.time() - last_update
    #             derivative = (error - last_error) / delta_time if delta_time else 0
    #             turn = Kp * error + Kd * derivative

    #             elapsed = self.watch.time() - start_time
    #             ease_factor = min(1.0, elapsed / ease_duration) if ease_duration > 0 else 1.0

    #             # Only check junctions after easing
    #             if ease_factor >= 1.0:
    #                 detect_junction = abs(derivative) > junction_threshold
    #                 junction_cooldown = self.watch.time() - last_junction_time > 500

    #                 if detect_junction and (last_junction_time == 0 or junction_cooldown):
    #                     junctions_detected += 1
    #                     last_junction_time = self.watch.time()
    #                     if junctions_detected >= junction_count:
    #                         if self.debug_mode == 2: print("Left: {}, Right: {}, Error: {}, Derivative: {} Delta_time: {} Stability: {}".format(left_val, right_val, error, derivative, delta_time, stability))
    #                         break
    #                 else:
    #                     stability = stability * 0.9 + abs(derivative) * 0.1
                
    #             if self.debug_mode: 
    #                 if self.debug_mode == 2: print("Left: {}, Right: {}, Error: {}, Derivative: {} Delta_time: {} Stability: {}".format(left_val, right_val, error, derivative, self.watch.time() - last_update, stability))
    #                 last_update = self.watch.time()

    #             speed_left = min(max(speed * ease_factor + turn, 0), self.MAX_SPEED)
    #             speed_right = min(max(speed * ease_factor - turn, 0), self.MAX_SPEED)

    #             self.left_motor.run(speed_left)
    #             self.right_motor.run(speed_right)

    #             last_error = error

    #     # After loop ends
    #     if then == "HOLD":
    #         self.left_motor.hold()
    #         self.right_motor.hold()
    #     elif then == "STOP":
    #         self.left_motor.stop()
    #         self.right_motor.stop()
    #     elif then == "BRAKE":
    #         self.left_motor.brake()
    #         self.right_motor.brake()
        
    #     return stability

    # def line_trace_rotations(
    #     self,
    #     Kp, Kd,
    #     rotations: float = 1.0,
    #     mode="balance",
    #     ease_duration: int = 400,
    #     polling_rate: int = 10,
    #     then="HOLD",
    #     TRACE_TARGET=50,
    #     speed = None,
    # ):
    #     """
    #     PD line tracing for a specified number of wheel rotations.

    #     Parameters:
    #     - rotations: target number of wheel rotations to trace
    #     - ease_duration: time in ms to gradually increase speed at start
    #     - Kp, Kd: PD constants
    #     - mode: string, one of "balance", "left_only", "right_only", "left_minus_right", "right_minus_left"
    #     - polling_rate: wait time between updates in ms
    #     - then: what to do after tracing: "HOLD", "STOP", "BRAKE"
    #     """

    #     if self.debug_mode:
    #         print("Starting line trace for {} rotations in mode '{}'".format(rotations, mode))

    #     self.left_motor.reset_angle(0)
    #     self.right_motor.reset_angle(0)
    #     last_error = 0
    #     start_time = self.watch.time()
    #     last_update = start_time
    #     if not speed: speed = self.TRACE_SPEED

    #     while True:
    #         left_val = self.left_sensor.reflection()
    #         right_val = self.right_sensor.reflection()

    #         if left_val is not None and right_val is not None:
    #             # Compute error based on mode
    #             if mode == "balance":
    #                 error = (TRACE_TARGET - left_val) - (TRACE_TARGET - right_val)
    #             elif mode == "left_only":
    #                 error = TRACE_TARGET - left_val
    #             elif mode == "right_only":
    #                 error = TRACE_TARGET - right_val
    #             elif mode == "left_minus_right":
    #                 error = left_val - right_val
    #             elif mode == "right_minus_left":
    #                 error = right_val - left_val
    #             else:
    #                 error = 0

    #             derivative = error - last_error
    #             turn = Kp * error + Kd * derivative

    #             elapsed = self.watch.time() - start_time
    #             ease_factor = min(1.0, elapsed / ease_duration) if ease_duration > 0 else 1.0

    #             speed_left = min(max(speed * ease_factor + turn, 0), self.MAX_SPEED)
    #             speed_right = min(max(speed * ease_factor - turn, 0), self.MAX_SPEED)

    #             self.left_motor.run(speed_left)
    #             self.right_motor.run(speed_right)

    #             last_error = error
    #             if self.debug_mode == 2:
    #                 print("L: {}, R: {}, Err: {}, Deriv: {}, Δt: {}".format(
    #                 left_val, right_val, error, derivative, self.watch.time() - last_update))
    #                 last_update = self.watch.time()

    #             # Check if rotations reached
    #             avg_angle = (abs(self.left_motor.angle()) + abs(self.right_motor.angle())) / 2
    #             if avg_angle >= rotations * 360:
    #                 break

    #         wait(polling_rate)

    #     # Stop behavior
    #     if then == "HOLD":
    #         self.left_motor.hold()
    #         self.right_motor.hold()
    #     elif then == "STOP":
    #         self.left_motor.stop()
    #         self.right_motor.stop()
    #     elif then == "BRAKE":
    #         self.left_motor.brake()
    #         self.right_motor.brake()

    def turn_arc_legacy(self, angle: float, radius_factor=0.0, then="HOLD", speed=None, waiting = True):
        """
        Turns the robot along an arc using differential wheel speeds.

        Parameters:
        - angle: float, turn amount in degrees. Positive for right, negative for left.
        - radius_factor: float, adjusts arc radius (0 = in-place turn, higher = wider arc).
        - then: action after turn ends. One of "HOLD", "STOP", or "BRAKE".
        """
        if self.debug_mode: print("Turning arc with angle {} and radius factor {}".format(angle, radius_factor))
        if not speed: speed = self.BASE_SPEED

        left_angle = angle * self.TURN_CONST + radius_factor
        right_angle = -angle * self.TURN_CONST + radius_factor

        abs_left = abs(left_angle)
        abs_right = abs(right_angle)

        if abs_left > abs_right:
            ratio = abs_left / abs_right
            speed_left = speed
            speed_right = speed / ratio
        else:
            ratio = abs_right / abs_left
            speed_right = speed
            speed_left = speed / ratio

        self.left_motor.run_angle(speed_left if left_angle >= 0 else -speed_left, abs_left, wait=False, then=Stop.HOLD)
        self.right_motor.run_angle(speed_right if right_angle >= 0 else -speed_right, abs_right, wait=waiting, then=Stop.HOLD)

    def move_time(self, duration: float, reverse: bool = False, ease_in: bool = False, ease_out: bool = False, polling_rate: int = 10, then: str = "HOLD", correction: bool = True, speed = None):
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
        if self.debug_mode: print("Moving for {} ms, reverse={}, ease_in={}, ease_out={}, correction={}, then='{}'".format(duration, reverse, ease_in, ease_out, correction, then))
        if not speed: speed = self.BASE_SPEED
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

            speed_left = (speed - correction_val) * ease_factor
            speed_right = (speed + correction_val) * ease_factor

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
    
    def wall_align(self, reversed = True, speed = None, wall_sensitivity = 70):
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

        self.left_motor.run(-speed if reversed else speed)
        self.right_motor.run(-speed if reversed else speed)
        wait(200)
        previous_current = self.ev3.battery.current()
        left_start_angle = self.left_motor.angle()
        right_start_angle = self.right_motor.angle()
        while True:
            current_current = self.ev3.battery.current()
            if(current_current - previous_current > wall_sensitivity): break
            previous_current = current_current

        self.main_dc(-100 if reversed else 100)
        wait(200)
        self.main_dc(-50 if reversed else 50)
        wait(200)
        self.main_dc(0)
        wait(100)
        corrected_angle = ((self.left_motor.angle() - left_start_angle) - (self.right_motor.angle() - right_start_angle)) / self.TURN_CONST

        return corrected_angle

    def move_aux_angle(self, angle, motor_number : int, speed = None, waiting = True, backlash_adjust = 0):
        """
        Moves the auxiliary motor to a specific angle.

        Parameters:
        - angle: float, target angle in degrees.
        - motor_number: int, 1 or 2 to select auxiliary motor.
        """
        if self.debug_mode: print("Moving auxiliary motor {} to angle {}".format(motor_number, angle))
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
        
        aux_motor.run_angle(speed if speed else self.AUX_SPEED, angle + backlash_adjust if angle > 0 else angle - backlash_adjust, then=Stop.HOLD, wait = waiting)
        if waiting: aux_motor.run_angle(speed if speed else self.AUX_SPEED, -backlash_adjust if angle > 0 else backlash_adjust, then=Stop.HOLD, wait = False)
    
    def move_aux_stall(self, motor_number : int, reversed = False, speed = None, power = None, min_time = 200, stall_threshold=5, polling_rate=100):
        """
        Moves the auxiliary motor until it stalls.

        Parameters:
        - motor_number: int, 1 or 2 to select auxiliary motor.
        - stall_threshold: int, angle threshold to detect stall.
        - polling_rate: int, ms between control updates.
        """
        start = self.watch.time()
        if self.debug_mode: print("Moving auxiliary motor {} until stall with threshold {}".format(motor_number, stall_threshold))
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
        
        prev_angle = aux_motor.angle()
        if power: 
            for i in range(10) : aux_motor.dc(-power if reversed else power)
        elif speed: aux_motor.run(-speed if reversed else speed)
        else: aux_motor.run(-self.AUX_SPEED if reversed else self.AUX_SPEED)

        while abs(aux_motor.angle() - prev_angle) > stall_threshold or (self.watch.time() - start) < min_time:
            prev_angle = aux_motor.angle()
            wait(polling_rate)

        aux_motor.brake()

    def get_colour(self, sensor_num):
        """
        Returns the closest color name detected by the specified auxiliary sensor,
        using normalized RGB for brightness-invariant comparison.

        Parameters:
         - sensor_num: int, 1 or 2 to select auxiliary sensor.
        """
        if sensor_num == 1:
            if isinstance(self.aux_sensor_1, ColorSensor): r, g, b = self.aux_sensor_1.rgb()
            else: r, g, b = self.aux_sensor_1.read("RGB")[:3]
        elif sensor_num == 2:
            if isinstance(self.aux_sensor_2, ColorSensor): r, g, b = self.aux_sensor_2.rgb()
            else: r, g, b = self.aux_sensor_2.read("RGB")[:3]
        else:
            raise ValueError("Invalid sensor_num, must be 1 or 2")

        total = r + g + b
        if total == 0: return "unknown"
        r, g, b = r / total, g / total, b / total

        color_refs = {
            "red":     (0.8, 0.2, 0.0),
            "green":   (0.20, 0.55, 0.25),
            "blue":    (0.1, 0.2, 0.7),
            "yellow":  (0.55, 0.35, 0.1),
        }

        def distance(c1, c2):
            return (c1[0]-c2[0])**2 + (c1[1]-c2[1])**2 + (c1[2]-c2[2])**2

        closest = min(color_refs, key=lambda name: distance((r, g, b), color_refs[name]))
        return closest

    def assign_colour(self, values):
        color_refs = {
            "red":     (0.8, 0.2, 0.0),
            "green":   (0.20, 0.55, 0.25),
            "blue":    (0.1, 0.2, 0.7),
            "yellow":  (0.55, 0.35, 0.1),
        }

        def normalize(rgb):
            r, g, b = rgb
            total = r + g + b
            if total == 0:
                return None
            return (r / total, g / total, b / total)

        def distance(c1, c2):
            return (c1[0]-c2[0])**2 + (c1[1]-c2[1])**2 + (c1[2]-c2[2])**2

        def permutations(seq, length):
            if length == 0:
                yield []
            else:
                for i in range(len(seq)):
                    rest = seq[:i] + seq[i+1:]
                    for p in permutations(rest, length-1):
                        yield [seq[i]] + p

        norm_values = [normalize(rgb) for rgb in values]
        color_names = list(color_refs.keys())

        valid_indices = [i for i,v in enumerate(norm_values) if v is not None]
        valid_values  = [norm_values[i] for i in valid_indices]

        best_perm = None
        best_score = 1e9

        for perm in permutations(color_names, len(valid_values)):
            score = 0
            for i, color in enumerate(perm):
                score += distance(valid_values[i], color_refs[color])
            if score < best_score:
                best_score = score
                best_perm = perm

        result = [None]*len(values)
        used = set()

        for idx, color in zip(valid_indices, best_perm):
            result[idx] = color
            used.add(color)

        remaining = [c for c in color_names if c not in used]

        # Assign missing (black triplets) using random.choice
        for i in range(len(values)):
            if result[i] is None and remaining:
                choice = rand_choice(remaining)
                result[i] = choice
                used.add(choice)
                remaining.remove(choice)

        # Leftover = the one unused color
        leftover = [c for c in color_names if c not in used][0]

        return result + [leftover]

    def line_align(self, speed = None, Kp: float = -2, then: str = "HOLD", polling_rate: int = 5):
        """
        Aligns the robot to a line.

        The robot moves forward until a significant change in the sum of both sensor readings
        is detected, then adjusts its orientation until the difference between left and right
        sensor readings is within a target threshold.

        Parameters:
        - speed: float or None, initial forward speed. If None, uses self.BASE_SPEED.
        - change_treshold: float, minimum change in combined sensor readings to start alignment.
        - Kp: float, proportional gain for alignment correction.
        - then: str, action after alignment: "HOLD", "STOP", or "BRAKE".
        - polling_rate: int, ms between control updates.
        """

        if not speed: speed = self.BASE_SPEED
        self.left_motor.run(speed)
        self.right_motor.run(speed)
        black_threshold = 8
        values = (self.left_sensor.reflection(), self.right_sensor.reflection())
        print(values)
        while values[0] > black_threshold and values[1] > black_threshold:
            wait(polling_rate)
            values = (self.left_sensor.reflection(), self.right_sensor.reflection())
            print(values)
        self.move_rotations(-0.04, speed=speed)
        wait(100)
        # values = (self.left_sensor.reflection(), self.right_sensor.reflection())
        # print(values)
        # return
        sensor_difference = self.left_sensor.reflection() - self.right_sensor.reflection()
        if self.debug_mode >= 2: print(sensor_difference, self.left_sensor.reflection(), self.right_sensor.reflection())
        initial_difference = sensor_difference
        while abs(sensor_difference) != 0:
            self.left_motor.run(sensor_difference * Kp)
            self.right_motor.run(sensor_difference * Kp * -1)
            sensor_difference = self.left_sensor.reflection() - self.right_sensor.reflection()
            if self.debug_mode >= 2: print(sensor_difference, self.left_sensor.reflection(), self.right_sensor.reflection())
            wait(polling_rate)

        # if initial_difference > 0: self.turn_arc(-3)
        # elif initial_difference < 0: self.turn_arc(3)

        if then == "HOLD":
            self.left_motor.hold()
            self.right_motor.hold()
        elif then == "STOP":
            self.left_motor.stop()
            self.right_motor.stop()
        elif then == "BRAKE":
            self.left_motor.brake()
            self.right_motor.brake()

    def aux_dc(self, power, motor_num):
        if motor_num == 1:
            for i in range(10): self.aux_motor_1.dc(power)
        elif motor_num == 2:
            for i in range(10): self.aux_motor_2.dc(power)
        else:
            raise ValueError("Invalid motor number. Use 1 or 2.")
    
    def main_dc(self, power):
        for i in range(10): 
            self.right_motor.dc(power)
            self.left_motor.dc(power)

    def await_rotation(self, rotation, timeout = 0):
        if not timeout:
            rotation *= 720
            start_sum = self.right_motor.angle() + self.left_motor.angle()
            current_sum = start_sum
            while current_sum < start_sum + rotation if rotation > 0 else current_sum > start_sum + rotation:
                current_sum = self.right_motor.angle() + self.left_motor.angle()
            return (current_sum - start_sum - rotation) / 360
        else:
            rotation *= 720
            start_sum = self.right_motor.angle() + self.left_motor.angle()
            start_time = self.watch.time()
            current_sum = start_sum
            while current_sum < start_sum + rotation if rotation > 0 else current_sum > start_sum + rotation and duration < timeout:
                current_sum = self.right_motor.angle() + self.left_motor.angle()
                duration = self.watch.time() - start_time
            return (current_sum - start_sum - rotation) / 360
    
    def await_stop(self):
        last_sum = self.right_motor.angle() + self.left_motor.angle()
        
        while True:
            wait(20)
            current_sum = self.right_motor.angle() + self.left_motor.angle()
            if current_sum == last_sum:  # no change → stopped
                break
            last_sum = current_sum
        return last_sum

    def move_until_line(self, threshold, speed = None, polling_rate = 10):
        self.move_rotations(100, speed=speed if speed else self.BASE_SPEED)
        last_readings = self.left_sensor.reflection() + self.right_sensor.reflection()
        difference = 0
        while abs(difference) < threshold:
            current_readings = self.left_sensor.reflection() + self.right_sensor.reflection()
            difference = current_readings - last_readings
            print(difference)
            last_readings = current_readings
            wait(polling_rate)
        self.move_rotations(0)
        return

    def move_rotations(self, rotations: float, speed=None, acceleration_factor=0.5, tolerance=0.1, aKp=0.002, aKd=0.05):
        if not speed: 
            speed = self.BASE_SPEED
        if rotations < 0: 
            speed *= -1

        # convert speed from degrees/s to rotations/ms
        speed /= 360000

        # Record start rotations for each motor
        left_start_rotations = self.left_motor.angle() / 360
        right_start_rotations = self.right_motor.angle() / 360

        # Initialize PD variables
        previous_left_error = previous_right_error = 0
        previous_time = self.watch.time()

        # Compute accel/decel based on factor
        accel_rotations = abs(rotations) * 0.5 * acceleration_factor

        acceleration_time = 2 * accel_rotations / abs(speed) if speed != 0 else 0
        deceleration_time = acceleration_time
        cruise_rotations = abs(rotations) - 2 * accel_rotations
        cruise_time = cruise_rotations / abs(speed) if cruise_rotations > 0 else 0
        total_time = acceleration_time + cruise_time + deceleration_time

        start_time = self.watch.time()

        while True:
            current_time = self.watch.time()
            elapsed_time = current_time - start_time

            if elapsed_time >= total_time:
                break

            # Determine target speed based on phase
            if elapsed_time < acceleration_time:
                time_in_phase = elapsed_time
                target_speed = speed * (time_in_phase / acceleration_time)  # ramp up
            elif elapsed_time < acceleration_time + cruise_time:
                target_speed = speed  # cruise
            else:
                time_in_phase = elapsed_time - acceleration_time - cruise_time
                target_speed = speed * (1 - time_in_phase / deceleration_time)  # ramp down

            # print("{:.0f},{:.5f},{:.5f},{:.5f}".format(
            #     elapsed_time, target_speed * 360000, self.left_motor.speed(), self.right_motor.speed()
            # ))

            self.left_motor.run(target_speed * 360000)
            self.right_motor.run(target_speed * 360000)

        if tolerance > 0:
            while True:
                current_time = self.watch.time()
                delta_time = current_time - previous_time

                target_rotations = rotations
                left_error = target_rotations - (self.left_motor.angle() / 360 - left_start_rotations)
                right_error = target_rotations - (self.right_motor.angle() / 360 - right_start_rotations)

                if abs(left_error) <= tolerance and abs(right_error) <= tolerance:
                    break

                left_derivative = (left_error - previous_left_error) / delta_time if delta_time > 0 else 0
                right_derivative = (right_error - previous_right_error) / delta_time if delta_time > 0 else 0

                left_motor_speed = (aKp * left_error + aKd * left_derivative) * 360000
                right_motor_speed = (aKp * right_error + aKd * right_derivative) * 360000
                # print(left_motor_speed, right_motor_speed)

                self.left_motor.run(left_motor_speed)
                self.right_motor.run(right_motor_speed)

                previous_left_error, previous_right_error = left_error, right_error
                previous_time = current_time

        # Stop motors
        self.left_motor.hold()
        self.right_motor.hold()

    def turn_arc(self, angle: float, radius_factor=0.0, acceleration_factor=0.5, tolerance = 0.0, speed=None, aKp=0.002, aKd=0.05):
        """
        Turns the robot along an arc using differential wheel speeds.

        Parameters:
        - angle: float, turn amount in degrees. Positive for right, negative for left.
        - radius_factor: float, adjusts arc radius (0 = in-place turn, higher = wider arc).
        - then: action after turn ends. One of "HOLD", "STOP", or "BRAKE".
        """
        if self.debug_mode: print("Turning arc with angle {} and radius factor {}".format(angle, radius_factor))
        if not speed: speed = self.BASE_SPEED


        left_angle = angle * self.TURN_CONST + radius_factor
        right_angle = -angle * self.TURN_CONST + radius_factor

        abs_left = abs(left_angle)
        abs_right = abs(right_angle)

        if abs_left > abs_right:
            ratio = abs_left / abs_right
            speed_left = speed
            speed_right = speed / ratio
        else:
            ratio = abs_right / abs_left
            speed_right = speed
            speed_left = speed / ratio
        
        if left_angle < 0: speed_left *= -1
        if right_angle < 0: speed_right *= -1

        left_rotations = left_angle / 360
        right_rotations = right_angle / 360


        speed_left /= 360000
        speed_right /= 360000

        # Record start rotations for each motor
        left_start_rotations = self.left_motor.angle() / 360
        right_start_rotations = self.right_motor.angle() / 360

        # Initialize PD variables
        previous_left_error = previous_right_error = 0
        previous_time = self.watch.time()

        # Compute accel/decel based on factor
        left_accel_rotations = abs(left_rotations) * 0.5 * acceleration_factor
        right_accel_rotations = abs(right_rotations) * 0.5 * acceleration_factor

        acceleration_time = 2 * left_accel_rotations / abs(speed_left) if speed_left != 0 else 0
        deceleration_time = acceleration_time

        left_cruise_rotations = abs(left_rotations) - 2 * left_accel_rotations
        right_cruise_rotations = abs(right_rotations) - 2 * right_accel_rotations

        cruise_time = left_cruise_rotations / abs(speed_left) if left_cruise_rotations > 0 else 0
        total_time = acceleration_time + cruise_time + deceleration_time

        start_time = self.watch.time()

        while True:
            current_time = self.watch.time()
            elapsed_time = current_time - start_time

            if elapsed_time >= total_time:
                break

            # Determine target speed based on phase
            if elapsed_time < acceleration_time:
                time_in_phase = elapsed_time
                left_target_speed = speed_left * (time_in_phase / acceleration_time)  # ramp up
                right_target_speed = speed_right * (time_in_phase / acceleration_time)

            elif elapsed_time < acceleration_time + cruise_time:
                left_target_speed = speed_left  # cruise
                right_target_speed = speed_right
            else:
                time_in_phase = elapsed_time - acceleration_time - cruise_time
                left_target_speed = speed_left * (1 - time_in_phase / deceleration_time)  # ramp down
                right_target_speed = speed_right * (1 - time_in_phase / deceleration_time)  # ramp down

            self.left_motor.run(left_target_speed * 360000)
            self.right_motor.run(right_target_speed * 360000)

        if tolerance > 0:
            while True:
                current_time = self.watch.time()
                delta_time = current_time - previous_time

                left_error = left_rotations - (self.left_motor.angle() / 360 - left_start_rotations)
                right_error = right_rotations - (self.right_motor.angle() / 360 - right_start_rotations)

                if abs(left_error) <= tolerance and abs(right_error) <= tolerance:
                    break

                left_derivative = (left_error - previous_left_error) / delta_time if delta_time > 0 else 0
                right_derivative = (right_error - previous_right_error) / delta_time if delta_time > 0 else 0

                left_motor_speed = (aKp * left_error + aKd * left_derivative) * 360000
                right_motor_speed = (aKp * right_error + aKd * right_derivative) * 360000

                self.left_motor.run(left_motor_speed)
                self.right_motor.run(right_motor_speed)

                previous_left_error, previous_right_error = left_error, right_error
                previous_time = current_time
    
        self.left_motor.hold()
        self.right_motor.hold()
        

    def test_max_speed(self, side : str, test_duration = 60000, warmup_duration = 60000, sampling_interval = 1000):
        if side == "left": test_motor = self.left_motor
        elif side == "right": test_motor= self.right_motor

        print("Warming up motor...")
        test_motor.dc(100)
        wait(warmup_duration)
        print("Measuring motor speed...")
        start_time = self.watch.time()
        speeds = []
        while self.watch.time() - start_time < test_duration:
            speeds.append(test_motor.speed())
            wait(sampling_interval)
        
        mean_speed = sum(speeds) / len(speeds)
        deviation = (sum((speed - mean_speed) ** 2 for speed in speeds) / len(speeds)) ** 0.5
        print("Mean speed: {:.2f} degrees/s".format(mean_speed))
        print("Standard Deviation: {:.2f}".format(deviation))
        test_motor.stop()

    def line_trace_junction(
        self,
        Kp, Kd,
        junction_count: int = 1,
        ease_duration: int = 1000,
        stop = True,
        TRACE_TARGET=50,
        junction_threshold: int = 50,
        speed = None
    ):
        """
        PD line tracing with multiple modes until a specified number of junctions.

        Parameters:
        - ease_duration: time in ms to gradually increase speed at start
        - Kp, Kd: PD constants
        - then: what to do after target junctions: "HOLD", "STOP", "BRAKE"
        - junction_count: how many junctions to detect before stopping
        """

        if self.debug_mode:
            print("Starting line trace until {} junction".format(junction_count))
        if not speed: speed = self.TRACE_SPEED
        
        if self.line_sensor:
            last_error = 0
            last_junction_time = 0
            junctions_detected = 0
            start_time = self.watch.time()
            last_update = start_time
            val = None
            

            while True:
                if val == self.line_sensor.read("CAL"): continue
                val = self.line_sensor.read("CAL")
                left_val = val[4] * 0.4 + val[5] * 0.6 + val [6] * 0.8 + val[7] * 1
                right_val = val[0] * 1 + val[1] * 0.8 + val[2] * 0.6 + val[3] * 0.4

                if left_val is not None and right_val is not None:
                    
                    error = (TRACE_TARGET - left_val) - (TRACE_TARGET - right_val)
                    derivative = (error - last_error) / delta_time if delta_time else 0
                    turn = Kp * error + Kd * derivative

                    elapsed = self.watch.time() - start_time
                    ease_factor = min(1.0, elapsed / ease_duration) if ease_duration > 0 else 1.0

                    # Only check junctions after easing
                    if ease_factor >= 1.0:
                        detect_junction =  val[0] < junction_threshold or val[7] < junction_threshold
                        junction_cooldown = self.watch.time() - last_junction_time > 500

                        if detect_junction and (last_junction_time == 0 or junction_cooldown):
                            junctions_detected += 1
                            last_junction_time = self.watch.time()
                            if junctions_detected >= junction_count:
                                if self.debug_mode == 2: print("Left: {:.1f}, Right: {:.1f}, Error: {:.1f}, Derivative: {:.3f} Delta_time: {:.0f}".format(left_val, right_val, error, derivative, delta_time))
                                break
                    
                    if self.debug_mode: 
                        if self.debug_mode == 2: print("Left: {:.1f}, Right: {:.1f}, Error: {:.1f}, Derivative: {:.3f} Delta_time: {:.0f}".format(left_val, right_val, error, derivative, delta_time))
                        last_update = self.watch.time()

                    speed_left = min(max(speed * ease_factor + turn, 0), self.MAX_SPEED)
                    speed_right = min(max(speed * ease_factor - turn, 0), self.MAX_SPEED)

                    self.left_motor.run(speed_left)
                    self.right_motor.run(speed_right)

                    last_error = error
        elif self.left_sensor and self.right_sensor:
            last_error = 0
            last_junction_time = 0
            junctions_detected = 0
            start_time = self.watch.time()
            last_update = start_time

            while True:
                left_val = self.left_sensor.reflection()
                right_val = self.right_sensor.reflection()

                if left_val is not None and right_val is not None:
                    
                    error = (TRACE_TARGET - left_val) - (TRACE_TARGET - right_val)

                    delta_time = self.watch.time() - last_update
                    derivative = (error - last_error) / delta_time if delta_time else 0
                    turn = Kp * error + Kd * derivative

                    elapsed = self.watch.time() - start_time
                    ease_factor = min(1.0, elapsed / ease_duration) if ease_duration > 0 else 1.0

                    # Only check junctions after easing
                    if ease_factor >= 1.0:
                        detect_junction =  left_val < junction_threshold or right_val < junction_threshold
                        junction_cooldown = self.watch.time() - last_junction_time > 500

                        if detect_junction and (last_junction_time == 0 or junction_cooldown):
                            junctions_detected += 1
                            last_junction_time = self.watch.time()
                            if junctions_detected >= junction_count:
                                if self.debug_mode == 2: print("Left: {}, Right: {}, Error: {}, Derivative: {} Delta_time: {}".format(left_val, right_val, error, derivative, delta_time))
                                break
                    
                    if self.debug_mode: 
                        if self.debug_mode == 2: print("Left: {}, Right: {}, Error: {}, Derivative: {} Delta_time: {}".format(left_val, right_val, error, derivative, self.watch.time() - last_update))
                        last_update = self.watch.time()

                    speed_left = min(max(speed * ease_factor + turn, 0), self.MAX_SPEED)
                    speed_right = min(max(speed * ease_factor - turn, 0), self.MAX_SPEED)

                    self.left_motor.run(speed_left)
                    self.right_motor.run(speed_right)

                    last_error = error
        else:
            raise ValueError("No line sensor or left/right sensors available for line tracing.")

        # After loop ends
        if stop:
            self.left_motor.stop()
            self.right_motor.stop()
    
    def line_trace_rotations(
        self,
        Kp, Kd,
        rotations: float,
        ease_duration: int = 1000,
        stop = True,
        TRACE_TARGET=50,
        speed = None,
        polling_rate = 10,
    ):
        """
        PD line tracing with multiple modes until a specified number of junctions.

        Parameters:
        - ease_duration: time in ms to gradually increase speed at start
        - Kp, Kd: PD constants
        - then: what to do after target junctions: "HOLD", "STOP", "BRAKE"
        - junction_count: how many junctions to detect before stopping
        """

        if self.debug_mode:
            print("Starting line trace until {} rotations".format(rotations))
        if not speed: speed = self.TRACE_SPEED
        
        if self.line_sensor:
            last_error = None
            start_time = self.watch.time()
            start_rotations = (self.left_motor.angle() + self.right_motor.angle()) / 720
            last_update = start_time
            val = None
            
            while True:
                if val == self.line_sensor.read("CAL"): continue
                delta_time = self.watch.time() - last_update
                wait(max(0, polling_rate - delta_time))  # Ensure at least 10ms between updates
                delta_time = self.watch.time() - last_update

                val = self.line_sensor.read("CAL")
                left_val = val[4] * 0.5 + val[5] * 1 + val [6] * 0.5 + val[7] * 1
                right_val = val[3] * 0.5 + val[1] * 1 + val[1] * 0.5 + val[3] * 1
                print(val)
                
                error = (TRACE_TARGET - left_val) - (TRACE_TARGET - right_val)
                derivative = error - last_error if last_error != None else 0
                turn = Kp * error + Kd * derivative

                elapsed = self.watch.time() - start_time
                ease_factor = min(1.0, elapsed / ease_duration) if ease_duration > 0 else 1.0

                # Check if rotations reached
                if (self.left_motor.angle() + self.right_motor.angle()) / 720 - start_rotations >= rotations:
                    if self.debug_mode == 2: print("Left: {:.1f}, Right: {:.1f}, Error: {:.1f}, Derivative: {:.3f}, Turn: {:.3f} Delta_time: {:.0f}".format(left_val, right_val, error, derivative, turn, delta_time))
                    break
                
                if self.debug_mode: 
                    if self.debug_mode == 2: print("Left: {:.1f}, Right: {:.1f}, Error: {:.1f}, Derivative: {:.3f}, Turn: {:.3f} Delta_time: {:.0f}".format(left_val, right_val, error, derivative, turn, delta_time))
                    last_update = self.watch.time()

                speed_left = speed * ease_factor + turn
                speed_right = speed * ease_factor - turn
                print(speed_left, speed_right)

                self.left_motor.run(speed_left)
                self.right_motor.run(speed_right)

                last_error = error
        elif self.left_sensor and self.right_sensor:
            last_error = None
            start_time = self.watch.time()
            start_rotations = (self.left_motor.angle() + self.right_motor.angle()) / 720
            last_update = start_time

            while True:
                delta_time = self.watch.time() - last_update
                wait(max(0, polling_rate - delta_time))  # Ensure at least 10ms between updates
                delta_time = self.watch.time() - last_update
                left_val = self.left_sensor.reflection()
                right_val = self.right_sensor.reflection()

                if left_val is not None and right_val is not None:
                    
                    error = (TRACE_TARGET - left_val) - (TRACE_TARGET - right_val)
                    derivative = (error - last_error) / delta_time if delta_time and last_error != None else 0
                    turn = Kp * error + Kd * derivative

                    elapsed = self.watch.time() - start_time
                    ease_factor = min(1.0, elapsed / ease_duration) if ease_duration > 0 else 1.0

                    # Check if rotations reached
                    if (self.left_motor.angle() + self.right_motor.angle()) / 720 - start_rotations >= rotations:
                        if self.debug_mode == 2: print("Left: {:.1f}, Right: {:.1f}, Error: {:.1f}, Derivative: {:.3f} Delta_time: {:.0f}".format(left_val, right_val, error, derivative, delta_time))
                        break
                    
                    if self.debug_mode: 
                        if self.debug_mode == 2: print("Left: {}, Right: {}, Error: {}, Derivative: {} Delta_time: {}".format(left_val, right_val, error, derivative, self.watch.time() - last_update))
                        last_update = self.watch.time()

                    speed_left = speed * ease_factor + turn
                    speed_right = speed * ease_factor - turn

                    self.left_motor.run(speed_left)
                    self.right_motor.run(speed_right)

                    last_error = error
        else:
            raise ValueError("No line sensor or left/right sensors available for line tracing.")

        # After loop ends
        if stop:
            self.left_motor.stop()
            self.right_motor.stop()

    def calibrate_turning(self, depth : int, start = 360, repeats = 3):

        total_angle = start


        self.wall_align(reversed=False, wall_sensitivity=50)


        for i in range(depth):
            errors = []
            for j in range(repeats):
                self.move_rotations(-0.6, speed=400, tolerance=0.01)
                self.turn_arc(total_angle, speed=600, acceleration_factor=360/total_angle, tolerance=0.01)
                left_start_angle = self.left_motor.angle()
                right_start_angle = self.right_motor.angle()
                self.main_dc(40)
                wait(2000)
                error = ((self.left_motor.angle() - left_start_angle) - (self.right_motor.angle() - right_start_angle)) / self.TURN_CONST * 0.5
                errors.append(error)
                print("Repeat {} Error: {}".format(j + 1, error))

            average_error = sum(errors) / len(errors)
            new_turning_const = self.TURN_CONST * (total_angle + average_error) / total_angle
            print("Depth {} Average Error: {} New Turn Const: {} Δ {}".format(i + 1, average_error, new_turning_const, new_turning_const - self.TURN_CONST))
            self.TURN_CONST = new_turning_const
            total_angle *= 2
