import time
import math

from shared_data_classes import AircraftState


# ============================================================
# Internal Rate Calculator
# ============================================================

class AngleRateCalculator:
    """
    Calculates rate of change for angular values.

    Use wrap=True for heading/yaw angles that wrap around 0/360.
    """

    def __init__(self, wrap=False):
        self.wrap = wrap
        self.last_value = None
        self.last_time = None

    def reset(self):
        self.last_value = None
        self.last_time = None

    def update(self, value, timestamp=None):
        if timestamp is None:
            timestamp = time.time()

        if self.last_value is None:
            self.last_value = value
            self.last_time = timestamp
            return 0.0

        dt = timestamp - self.last_time

        if dt <= 1e-6:
            return 0.0

        delta = value - self.last_value

        if self.wrap:
            delta = (delta + 180.0) % 360.0 - 180.0

        rate = delta / dt

        self.last_value = value
        self.last_time = timestamp

        return rate


# ============================================================
# Sensor Input Manager
# ============================================================

class SensorInputManager:
    """
    Creates and manages KrPC streams for the active vessel.

    The ModeManager should call:

        aircraft_state = sensor_input_manager.get_snapshot()

    once per update cycle.

    That same AircraftState snapshot should then be passed to:
    - GuidanceMethod
    - FlightState
    - Autopilot
    """

    def __init__(self, conn, vessel):
        if conn is None:
            raise ValueError("conn must not be None")

        if vessel is None:
            raise ValueError("vessel must not be None")

        self.conn = conn
        self.vessel = vessel

        self.body_reference_frame = vessel.orbit.body.reference_frame
        self.surface_reference_frame = vessel.surface_reference_frame

        self.flight_body = vessel.flight(self.body_reference_frame)
        self.flight_surface = vessel.flight(self.surface_reference_frame)

        self.streams = {}

        self.pitch_rate_calculator = AngleRateCalculator(wrap=False)
        self.yaw_rate_calculator = AngleRateCalculator(wrap=True)
        self.roll_rate_calculator = AngleRateCalculator(wrap=False)
        self.fpa_rate_calculator = AngleRateCalculator(wrap=False)
        
        self.mode_manager_receiver = None
        self.guidance_receiver = None
        self.flight_state_receiver = None

        self._build_streams()
        
        
    # --------------------------------------------------------
    # Connections
    # --------------------------------------------------------

    def set_mode_manager_receiver(self, receiver):
        self.mode_manager_receiver = receiver


    def set_guidance_receiver(self, receiver):
        self.guidance_receiver = receiver


    def set_flight_state_receiver(self, receiver):
        self.flight_state_receiver = receiver
        
        
    # --------------------------------------------------------
    # Distributions
    # --------------------------------------------------------

    def _distribute_snapshot(self, aircraft_state):
        if self.mode_manager_receiver is not None:
            if hasattr(self.mode_manager_receiver, "receive_aircraft_state"):
                self.mode_manager_receiver.receive_aircraft_state(aircraft_state)

        if self.guidance_receiver is not None:
            if hasattr(self.guidance_receiver, "receive_aircraft_state"):
                self.guidance_receiver.receive_aircraft_state(aircraft_state)

        if self.flight_state_receiver is not None:
            if hasattr(self.flight_state_receiver, "receive_aircraft_state"):
                self.flight_state_receiver.receive_aircraft_state(aircraft_state)


    # --------------------------------------------------------
    # Stream creation
    # --------------------------------------------------------

    def _build_streams(self):
        self.streams["latitude"] = self.conn.add_stream(
            getattr,
            self.flight_body,
            "latitude"
        )

        self.streams["longitude"] = self.conn.add_stream(
            getattr,
            self.flight_body,
            "longitude"
        )

        self.streams["altitude"] = self.conn.add_stream(
            getattr,
            self.flight_body,
            "mean_altitude"
        )

        self.streams["speed"] = self.conn.add_stream(
            getattr,
            self.flight_body,
            "speed"
        )

        self.streams["vertical_speed"] = self.conn.add_stream(
            getattr,
            self.flight_surface,
            "vertical_speed"
        )

        self.streams["pitch"] = self.conn.add_stream(
            getattr,
            self.flight_surface,
            "pitch"
        )

        self.streams["heading"] = self.conn.add_stream(
            getattr,
            self.flight_surface,
            "heading"
        )

        self.streams["roll"] = self.conn.add_stream(
            getattr,
            self.flight_surface,
            "roll"
        )

        self.streams["angle_of_attack"] = self.conn.add_stream(
            getattr,
            self.flight_surface,
            "angle_of_attack"
        )

        self.streams["sideslip_angle"] = self.conn.add_stream(
            getattr,
            self.flight_surface,
            "sideslip_angle"
        )

    # --------------------------------------------------------
    # Stream reading helpers
    # --------------------------------------------------------

    def _read(self, name, default=0.0):
        stream = self.streams.get(name)

        if stream is None:
            return default

        try:
            value = stream()
        except Exception:
            return default

        if value is None:
            return default

        return value

    def _get_velocity_surface(self):
        try:
            return self.vessel.velocity(self.surface_reference_frame)
        except Exception:
            return (0.0, 0.0, 0.0)

    # --------------------------------------------------------
    # Derived calculations
    # --------------------------------------------------------

    def _calculate_horizontal_speed(self, velocity_surface):
        vx, vy, vz = velocity_surface
        return math.hypot(vx, vz)

    def _calculate_flight_path_angle(self, velocity_surface):
        vx, vy, vz = velocity_surface

        vertical_speed = vy
        horizontal_speed = math.hypot(vx, vz)

        if horizontal_speed < 1e-6:
            return 0.0

        return math.degrees(
            math.atan2(vertical_speed, horizontal_speed)
        )

    # --------------------------------------------------------
    # Public snapshot method
    # --------------------------------------------------------

    def get_snapshot(self):
        timestamp = time.time()

        velocity_surface = self._get_velocity_surface()

        horizontal_speed = self._calculate_horizontal_speed(
            velocity_surface
        )

        flight_path_angle = self._calculate_flight_path_angle(
            velocity_surface
        )

        latitude = self._read("latitude")
        longitude = self._read("longitude")
        altitude = self._read("altitude")

        speed = self._read("speed")
        vertical_speed = self._read("vertical_speed")

        pitch = self._read("pitch")
        heading = self._read("heading") % 360.0
        roll = self._read("roll")

        angle_of_attack = self._read("angle_of_attack")
        sideslip_angle = self._read("sideslip_angle")

        pitch_rate = self.pitch_rate_calculator.update(
            pitch,
            timestamp
        )

        yaw_rate = self.yaw_rate_calculator.update(
            heading,
            timestamp
        )

        roll_rate = self.roll_rate_calculator.update(
            roll,
            timestamp
        )

        flight_path_angle_rate = self.fpa_rate_calculator.update(
            flight_path_angle,
            timestamp
        )

        aircraft_state = AircraftState(
            timestamp=timestamp,
            latitude=latitude,
            longitude=longitude,
            altitude=altitude,
            speed=speed,
            vertical_speed=vertical_speed,
            horizontal_speed=horizontal_speed,
            pitch=pitch,
            heading=heading,
            roll=roll,
            pitch_rate=pitch_rate,
            yaw_rate=yaw_rate,
            roll_rate=roll_rate,
            angle_of_attack=angle_of_attack,
            sideslip_angle=sideslip_angle,
            flight_path_angle=flight_path_angle,
            flight_path_angle_rate=flight_path_angle_rate,
            velocity_surface=velocity_surface,
        )
        
        self._distribute_snapshot(aircraft_state)
        
        return aircraft_state


    # --------------------------------------------------------
    # Utility
    # --------------------------------------------------------

    def reset_rate_calculators(self):
        self.pitch_rate_calculator.reset()
        self.yaw_rate_calculator.reset()
        self.roll_rate_calculator.reset()
        self.fpa_rate_calculator.reset()