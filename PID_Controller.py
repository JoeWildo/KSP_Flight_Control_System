import time
import math


class PID_Controller:
    """
    Generic reusable PID controller.

    Intended to be subclassed or configured for:
    - inner-loop aircraft control
    - outer-loop guidance control
    - runway steering
    - speed/throttle control
    """

    def __init__(
        self,
        kp,
        ki,
        kd,
        setpoint=0.0,
        output_limits=(-1.0, 1.0),
        integral_limit=1.0,
        is_angle=False,
        deadband=0.0,
        derivative_filter_alpha=0.2,
        max_output_rate=None,
        name="PID_Controller"
    ):
        self.name = name

        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint

        self.output_min, self.output_max = output_limits
        self.integral_limit = abs(integral_limit)

        self.is_angle = is_angle
        self.deadband = abs(deadband)

        self.derivative_filter_alpha = max(0.0, min(1.0, derivative_filter_alpha))
        self.max_output_rate = max_output_rate

        self.integral = 0.0
        self.last_error = 0.0
        self.last_time = None
        self.last_output = 0.0
        self.filtered_derivative = 0.0

    def reset(self, measurement=None):
        self.integral = 0.0
        self.filtered_derivative = 0.0
        self.last_output = 0.0
        self.last_time = None

        if measurement is None:
            self.last_error = 0.0
        else:
            self.last_error = self._compute_error(measurement)

    def set_target(self, setpoint):
        self.setpoint = setpoint

    def _wrap_angle_error(self, error):
        return (error + 180.0) % 360.0 - 180.0

    def _compute_error(self, measurement):
        error = self.setpoint - measurement

        if self.is_angle:
            error = self._wrap_angle_error(error)

        if abs(error) < self.deadband:
            error = 0.0

        return error

    def update(self, measurement, rate=None, dt=None):
        now = time.time()

        if self.last_time is None:
            self.last_time = now
            self.last_error = self._compute_error(measurement)
            return 0.0

        if dt is None:
            dt = now - self.last_time

        if dt <= 1e-6:
            dt = 1e-6

        error = self._compute_error(measurement)

        # Proportional
        p = self.kp * error

        # Derivative
        if rate is None:
            raw_derivative = (error - self.last_error) / dt
        else:
            raw_derivative = -rate

        alpha = self.derivative_filter_alpha
        self.filtered_derivative = (
            alpha * raw_derivative +
            (1.0 - alpha) * self.filtered_derivative
        )

        d = self.kd * self.filtered_derivative

        # Integral with limiting
        proposed_integral = self.integral + error * dt
        proposed_integral = max(
            -self.integral_limit,
            min(proposed_integral, self.integral_limit)
        )

        i = self.ki * proposed_integral

        unsaturated_output = p + i + d
        clamped_output = max(
            self.output_min,
            min(unsaturated_output, self.output_max)
        )

        # Anti-windup
        if clamped_output == unsaturated_output:
            self.integral = proposed_integral
        else:
            if clamped_output >= self.output_max and error < 0.0:
                self.integral = proposed_integral
            elif clamped_output <= self.output_min and error > 0.0:
                self.integral = proposed_integral

        output = p + self.ki * self.integral + d

        # Output clamp
        output = max(self.output_min, min(output, self.output_max))

        # Slew-rate limiting
        if self.max_output_rate is not None:
            max_delta = abs(self.max_output_rate) * dt
            delta = output - self.last_output

            if delta > max_delta:
                output = self.last_output + max_delta
            elif delta < -max_delta:
                output = self.last_output - max_delta

        self.last_error = error
        self.last_time = now
        self.last_output = output

        return output
    
    
    
    
    
"""
INNER-LOOP CHILD CONTROLLERS
"""
class RollPID(PID_Controller):
    """
    Inner loop:
    target roll angle -> aileron input
    """

    def __init__(self, kp, ki, kd, setpoint=0.0):
        super().__init__(
            kp=kp,
            ki=ki,
            kd=kd,
            setpoint=setpoint,
            output_limits=(-1.0, 1.0),
            integral_limit=0.5,
            is_angle=True,
            deadband=0.2,
            derivative_filter_alpha=0.25,
            max_output_rate=3.0,
            name="RollPID"
        )


class PitchPID(PID_Controller):
    """
    Inner loop:
    target pitch angle -> elevator input
    """

    def __init__(self, kp, ki, kd, setpoint=0.0):
        super().__init__(
            kp=kp,
            ki=ki,
            kd=kd,
            setpoint=setpoint,
            output_limits=(-1.0, 1.0),
            integral_limit=0.5,
            is_angle=True,
            deadband=0.1,
            derivative_filter_alpha=0.25,
            max_output_rate=2.5,
            name="PitchPID"
        )


class YawPID(PID_Controller):
    """
    Inner loop:
    target yaw / sideslip / runway heading error -> rudder input
    """

    def __init__(self, kp, ki, kd, setpoint=0.0):
        super().__init__(
            kp=kp,
            ki=ki,
            kd=kd,
            setpoint=setpoint,
            output_limits=(-1.0, 1.0),
            integral_limit=10.0,
            is_angle=True,
            deadband=0.3,
            derivative_filter_alpha=0.4,
            max_output_rate=10.0,
            name="YawPID"
        )


class FPAPID(PID_Controller):
    """
    Inner loop:
    target flight path angle -> elevator input
    """

    def __init__(self, kp, ki, kd, setpoint=0.0):
        super().__init__(
            kp=kp,
            ki=ki,
            kd=kd,
            setpoint=setpoint,
            output_limits=(-1.0, 1.0),
            integral_limit=0.5,
            is_angle=False,
            deadband=0.05,
            derivative_filter_alpha=0.2,
            max_output_rate=2.5,
            name="FPAPID"
        )


class SpeedPID(PID_Controller):
    """
    Inner loop:
    target speed -> throttle command
    """

    def __init__(self, kp, ki, kd, setpoint=0.0):
        super().__init__(
            kp=kp,
            ki=ki,
            kd=kd,
            setpoint=setpoint,
            output_limits=(0.0, 1.0),
            integral_limit=1.0,
            is_angle=False,
            deadband=0.5,
            derivative_filter_alpha=0.15,
            max_output_rate=0.5,
            name="SpeedPID"
        )

        
        
        
        
"""
OUTER LOOP CHILD CONTROLLER
"""
class HeadingPID(PID_Controller):
    """
    Outer loop:
    target heading -> target roll angle
    """

    def __init__(self, kp, ki, kd, setpoint=0.0, max_roll_cmd=30.0):
        super().__init__(
            kp=kp,
            ki=ki,
            kd=kd,
            setpoint=setpoint,
            output_limits=(-max_roll_cmd, max_roll_cmd),
            integral_limit=10.0,
            is_angle=True,
            deadband=0.5,
            derivative_filter_alpha=0.2,
            max_output_rate=20.0,
            name="HeadingPID"
        )


class AltitudePID(PID_Controller):
    """
    Outer loop:
    target altitude -> target FPA
    """

    def __init__(self, kp, ki, kd, setpoint=0.0, max_fpa_cmd=10.0):
        super().__init__(
            kp=kp,
            ki=ki,
            kd=kd,
            setpoint=setpoint,
            output_limits=(-max_fpa_cmd, max_fpa_cmd),
            integral_limit=500.0,
            is_angle=False,
            deadband=2.0,
            derivative_filter_alpha=0.15,
            max_output_rate=5.0,
            name="AltitudePID"
        )


class GlideSlopePID(PID_Controller):
    """
    Outer loop:
    glide slope error -> target FPA correction
    """

    def __init__(self, kp, ki, kd, setpoint=0.0, max_fpa_correction=5.0):
        super().__init__(
            kp=kp,
            ki=ki,
            kd=kd,
            setpoint=setpoint,
            output_limits=(-max_fpa_correction, max_fpa_correction),
            integral_limit=100.0,
            is_angle=False,
            deadband=0.5,
            derivative_filter_alpha=0.2,
            max_output_rate=3.0,
            name="GlideSlopePID"
        )


class CrossTrackPID(PID_Controller):
    """
    Outer loop:
    cross-track error -> heading correction
    """

    def __init__(self, kp, ki, kd, setpoint=0.0, max_heading_correction=30.0):
        super().__init__(
            kp=kp,
            ki=ki,
            kd=kd,
            setpoint=setpoint,
            output_limits=(-max_heading_correction, max_heading_correction),
            integral_limit=1000.0,
            is_angle=False,
            deadband=5.0,
            derivative_filter_alpha=0.15,
            max_output_rate=15.0,
            name="CrossTrackPID"
        )