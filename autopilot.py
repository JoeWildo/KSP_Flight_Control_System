import time
import threading
from dataclasses import dataclass, field
from enum import Enum

from shared_data_classes import AircraftState


# ============================================================
# Autopilot Modes
# ============================================================

class AutopilotMode(Enum):
    OFF = "OFF"
    FPA_HEADING_HOLD = "FPA_HEADING_HOLD"
    CLIMB = "CLIMB"
    CRUISE = "CRUISE"
    DESCENT = "DESCENT"
    LANDING_APPROACH = "LANDING_APPROACH"
    TAKEOFF = "TAKEOFF"
    TAXI = "TAXI"


# ============================================================
# Data Objects
# ============================================================

@dataclass
class TargetSnapshot:
    mode: AutopilotMode = AutopilotMode.OFF

    target_fpa: float | None = None
    target_heading: float | None = None
    target_roll: float | None = None
    target_pitch: float | None = None
    target_speed: float | None = None
    target_altitude: float | None = None

    cross_track_error: float | None = None
    path_heading: float | None = None
    glide_slope_error: float | None = None

    limits: dict = field(default_factory=dict)
    active_controllers: set = field(default_factory=set)


@dataclass
class RawCommand:
    pitch: float = 0.0
    roll: float = 0.0
    yaw: float = 0.0
    throttle: float | None = None
    brakes: float | None = None


@dataclass
class SafeCommand:
    pitch: float = 0.0
    roll: float = 0.0
    yaw: float = 0.0
    throttle: float | None = None
    brakes: float | None = None


# ============================================================
# Sensor Input Manager
# ============================================================

class SensorInputManager:
    def __init__(self, streams: dict):
        self.streams = streams

    def _read(self, name, default=0.0):
        stream = self.streams.get(name)
        if stream is None:
            return default
        return stream()

    def get_state(self) -> AircraftState:
        return AircraftState(
            timestamp=time.time(),

            latitude=self._read("latitude"),
            longitude=self._read("longitude"),
            altitude=self._read("altitude"),

            speed=self._read("speed"),
            vertical_speed=self._read("vertical_speed"),
            horizontal_speed=self._read("horizontal_speed"),

            pitch=self._read("pitch"),
            heading=self._read("heading") % 360.0,
            roll=self._read("roll"),

            pitch_rate=self._read("pitch_rate"),
            yaw_rate=self._read("yaw_rate"),
            roll_rate=self._read("roll_rate"),

            angle_of_attack=self._read("angle_of_attack"),
            sideslip_angle=self._read("sideslip_angle"),

            flight_path_angle=self._read("flight_path_angle"),
            flight_path_angle_rate=self._read("flight_path_angle_rate"),

            velocity_surface=self._read("velocity_surface", (0.0, 0.0, 0.0)),
        )


# ============================================================
# Target Manager
# ============================================================

class TargetManager:
    def __init__(self):
        self._lock = threading.Lock()
        self._targets = TargetSnapshot()

    def set_mode(self, mode: AutopilotMode):
        with self._lock:
            self._targets.mode = mode

    def update_targets(self, **kwargs):
        with self._lock:
            for key, value in kwargs.items():
                if not hasattr(self._targets, key):
                    raise AttributeError(f"Unknown target field: {key}")

                if key == "target_fpa" and value is not None:
                    value = max(min(value, 89.0), -89.0)

                if key == "target_heading" and value is not None:
                    value = value % 360.0

                if key == "target_roll" and value is not None:
                    value = max(min(value, 180.0), -180.0)

                setattr(self._targets, key, value)

    def set_limits(self, **limits):
        with self._lock:
            self._targets.limits.update(limits)

    def set_active_controllers(self, controllers: set[str]):
        with self._lock:
            self._targets.active_controllers = controllers

    def get_targets(self) -> TargetSnapshot:
        with self._lock:
            return TargetSnapshot(
                mode=self._targets.mode,
                target_fpa=self._targets.target_fpa,
                target_heading=self._targets.target_heading,
                target_roll=self._targets.target_roll,
                target_pitch=self._targets.target_pitch,
                target_speed=self._targets.target_speed,
                target_altitude=self._targets.target_altitude,
                cross_track_error=self._targets.cross_track_error,
                path_heading=self._targets.path_heading,
                glide_slope_error=self._targets.glide_slope_error,
                limits=dict(self._targets.limits),
                active_controllers=set(self._targets.active_controllers),
            )


# ============================================================
# Controller Registry
# ============================================================

class ControllerRegistry:
    def __init__(self, controllers: dict):
        self.controllers = controllers

    def get(self, name: str):
        controller = self.controllers.get(name)
        if controller is None:
            raise KeyError(f"PID controller '{name}' not found.")
        return controller

    def reset_all(self, state: AircraftState | None = None):
        for controller in self.controllers.values():
            controller.reset()

    def reset(self, name: str, measurement=None):
        self.get(name).reset(measurement=measurement)


# ============================================================
# Controller Chain Manager
# ============================================================

class ControllerChainManager:
    def __init__(self, registry: ControllerRegistry):
        self.registry = registry

    def update(self, state: AircraftState, targets: TargetSnapshot) -> RawCommand:
        if targets.mode == AutopilotMode.OFF:
            return RawCommand()

        if targets.mode == AutopilotMode.FPA_HEADING_HOLD:
            return self._fpa_heading_hold(state, targets)

        if targets.mode == AutopilotMode.CLIMB:
            return self._climb(state, targets)

        if targets.mode == AutopilotMode.CRUISE:
            return self._cruise(state, targets)

        if targets.mode == AutopilotMode.DESCENT:
            return self._descent(state, targets)

        if targets.mode == AutopilotMode.LANDING_APPROACH:
            return self._landing_approach(state, targets)

        if targets.mode == AutopilotMode.TAKEOFF:
            return self._takeoff(state, targets)

        if targets.mode == AutopilotMode.TAXI:
            return self._taxi(state, targets)

        return RawCommand()

    def _run_fpa_loop(self, state, target_fpa):
        fpa_pid = self.registry.get("fpa")
        fpa_pid.set_target(target_fpa)
        return fpa_pid.update(
            state.flight_path_angle,
            rate=state.flight_path_angle_rate
        )

    def _run_roll_loop(self, state, target_roll):
        roll_pid = self.registry.get("roll")
        roll_pid.set_target(target_roll)
        return roll_pid.update(state.roll, rate=state.roll_rate)

    def _run_heading_to_roll_loop(self, state, target_heading):
        heading_pid = self.registry.get("heading")
        heading_pid.set_target(target_heading)

        target_roll = heading_pid.update(
            state.heading,
            rate=state.yaw_rate
        )

        return self._run_roll_loop(state, target_roll)

    def _run_speed_loop(self, state, target_speed):
        speed_pid = self.registry.get("speed")
        speed_pid.set_target(target_speed)
        return speed_pid.update(state.speed)

    def _fpa_heading_hold(self, state, targets):
        cmd = RawCommand()

        if targets.target_fpa is not None:
            cmd.pitch = self._run_fpa_loop(state, targets.target_fpa)

        if targets.target_heading is not None:
            cmd.roll = self._run_heading_to_roll_loop(
                state,
                targets.target_heading
            )

        if targets.target_roll is not None:
            cmd.roll = self._run_roll_loop(state, targets.target_roll)

        return cmd

    def _climb(self, state, targets):
        cmd = RawCommand()

        if targets.target_heading is not None:
            cmd.roll = self._run_heading_to_roll_loop(
                state,
                targets.target_heading
            )

        if targets.target_fpa is not None:
            cmd.pitch = self._run_fpa_loop(state, targets.target_fpa)

        if targets.target_speed is not None:
            cmd.throttle = self._run_speed_loop(state, targets.target_speed)

        return cmd

    def _cruise(self, state, targets):
        cmd = RawCommand()

        heading_target = targets.target_heading

        if targets.cross_track_error is not None and targets.path_heading is not None:
            cross_track_pid = self.registry.get("cross_track")
            cross_track_pid.set_target(0.0)

            heading_correction = cross_track_pid.update(
                targets.cross_track_error
            )

            heading_target = (targets.path_heading + heading_correction) % 360.0

        if heading_target is not None:
            cmd.roll = self._run_heading_to_roll_loop(state, heading_target)

        if targets.target_altitude is not None:
            altitude_pid = self.registry.get("altitude")
            altitude_pid.set_target(targets.target_altitude)

            target_fpa = altitude_pid.update(
                state.altitude,
                rate=state.vertical_speed
            )

            cmd.pitch = self._run_fpa_loop(state, target_fpa)

        elif targets.target_fpa is not None:
            cmd.pitch = self._run_fpa_loop(state, targets.target_fpa)

        if targets.target_speed is not None:
            cmd.throttle = self._run_speed_loop(state, targets.target_speed)

        return cmd

    def _descent(self, state, targets):
        cmd = RawCommand()

        if targets.target_heading is not None:
            cmd.roll = self._run_heading_to_roll_loop(
                state,
                targets.target_heading
            )

        if targets.target_fpa is not None:
            cmd.pitch = self._run_fpa_loop(state, targets.target_fpa)

        if targets.target_speed is not None:
            cmd.throttle = self._run_speed_loop(state, targets.target_speed)

        return cmd

    def _landing_approach(self, state, targets):
        cmd = RawCommand()

        heading_target = targets.target_heading

        if targets.cross_track_error is not None and targets.path_heading is not None:
            cross_track_pid = self.registry.get("cross_track")
            cross_track_pid.set_target(0.0)

            heading_correction = cross_track_pid.update(
                targets.cross_track_error
            )

            heading_target = (targets.path_heading + heading_correction) % 360.0

        if heading_target is not None:
            cmd.roll = self._run_heading_to_roll_loop(state, heading_target)

        if targets.glide_slope_error is not None:
            glide_pid = self.registry.get("glide_slope")
            glide_pid.set_target(0.0)

            target_fpa_correction = glide_pid.update(
                targets.glide_slope_error
            )

            base_fpa = targets.target_fpa if targets.target_fpa is not None else -3.0
            target_fpa = base_fpa + target_fpa_correction

            cmd.pitch = self._run_fpa_loop(state, target_fpa)

        elif targets.target_fpa is not None:
            cmd.pitch = self._run_fpa_loop(state, targets.target_fpa)

        if targets.target_speed is not None:
            cmd.throttle = self._run_speed_loop(state, targets.target_speed)

        return cmd

    def _takeoff(self, state, targets):
        cmd = RawCommand()

        if targets.target_heading is not None:
            yaw_pid = self.registry.get("yaw")
            yaw_pid.set_target(targets.target_heading)
            cmd.yaw = yaw_pid.update(state.heading, rate=state.yaw_rate)

        if targets.target_fpa is not None:
            cmd.pitch = self._run_fpa_loop(state, targets.target_fpa)

        if targets.target_roll is not None:
            cmd.roll = self._run_roll_loop(state, targets.target_roll)
        else:
            cmd.roll = self._run_roll_loop(state, 0.0)

        if targets.target_speed is not None:
            cmd.throttle = self._run_speed_loop(state, targets.target_speed)
        else:
            cmd.throttle = 1.0

        return cmd

    def _taxi(self, state, targets):
        cmd = RawCommand()

        if targets.target_heading is not None:
            yaw_pid = self.registry.get("yaw")
            yaw_pid.set_target(targets.target_heading)
            cmd.yaw = yaw_pid.update(state.heading, rate=state.yaw_rate)

        if targets.target_speed is not None:
            cmd.throttle = self._run_speed_loop(state, targets.target_speed)
            
        # Taxi mode must release brakes unless braking is commanded
        cmd.brakes = 0.0

        return cmd


# ============================================================
# Command Output Manager
# ============================================================

class CommandOutputManager:
    def __init__(self, vessel):
        self.vessel = vessel
        
        # Debug print limiter
        self._last_debug_print = 0.0
        self._debug_print_period = 2.0      # seconds = 0.5 Hz

    def _clamp(self, value, low, high):
        return max(low, min(value, high))

    def process(self, raw: RawCommand, targets: TargetSnapshot) -> SafeCommand:
        limits = targets.limits

        pitch_min, pitch_max = limits.get("pitch_limits", (-1.0, 1.0))
        roll_min, roll_max = limits.get("roll_limits", (-1.0, 1.0))
        yaw_min, yaw_max = limits.get("yaw_limits", (-1.0, 1.0))
        throttle_min, throttle_max = limits.get("throttle_limits", (0.0, 1.0))
        brake_min, brake_max = limits.get("brake_limits", (0.0, 1.0))

        safe = SafeCommand(
            pitch=self._clamp(raw.pitch, pitch_min, pitch_max),
            roll=self._clamp(raw.roll, roll_min, roll_max),
            yaw=self._clamp(raw.yaw, yaw_min, yaw_max),
        )

        if raw.throttle is not None:
            safe.throttle = self._clamp(
                raw.throttle,
                throttle_min,
                throttle_max
            )

        if raw.brakes is not None:
            safe.brakes = self._clamp(
                raw.brakes,
                brake_min,
                brake_max
            )
            
        if targets.mode == AutopilotMode.TAXI and safe.brakes is None:
            safe.brakes = 0.0

        return safe

    def write_to_vessel(self, cmd: SafeCommand):
        ctrl = self.vessel.control

        ctrl.pitch = cmd.pitch
        ctrl.roll = cmd.roll
        ctrl.yaw = cmd.yaw

        if cmd.throttle is not None:
            ctrl.throttle = cmd.throttle

        if cmd.brakes is not None:
            ctrl.brakes = cmd.brakes > 0.5
            
        # --------------------------------------------------------
        # Rate-limited debug output (0.5 Hz)
        # --------------------------------------------------------
        now = time.time()
        if now - self._last_debug_print >= self._debug_print_period:
            self._last_debug_print = now

            print("[Autopilot][VesselControl Written]")
            print({
                "pitch": ctrl.pitch,
                "roll": ctrl.roll,
                "yaw": ctrl.yaw,
                "throttle": ctrl.throttle,
                "brakes": ctrl.brakes,
            })
            
    def zero_flight_controls(self):
        ctrl = self.vessel.control
        ctrl.pitch = 0.0
        ctrl.roll = 0.0
        ctrl.yaw = 0.0


# ============================================================
# Main Autopilot Class
# ============================================================

class Autopilot:
    def __init__(
        self,
        vessel,
        streams: dict,
        controller_registry: ControllerRegistry,
        update_rate_hz=20.0
    ):
        self.vessel = vessel
        self.update_rate_hz = update_rate_hz
        self.update_period = 1.0 / update_rate_hz

        self.sensor_input_manager = SensorInputManager(streams)
        self.target_manager = TargetManager()
        self.controller_registry = controller_registry
        self.controller_chain_manager = ControllerChainManager(
            controller_registry
        )
        self.command_output_manager = CommandOutputManager(vessel)
        
        self.latest_aircraft_state = None

        self._thread = None
        self._stop_event = threading.Event()
        self._running = False

    def set_mode(self, mode: AutopilotMode):
        self.target_manager.set_mode(mode)

    def set_targets(self, **targets):
        self.target_manager.update_targets(**targets)

    def set_limits(self, **limits):
        self.target_manager.set_limits(**limits)

    def set_active_controllers(self, controllers: set[str]):
        self.target_manager.set_active_controllers(controllers)

    def configure_from_flight_state(
        self,
        mode: AutopilotMode,
        targets: dict,
        limits: dict | None = None,
        active_controllers: set[str] | None = None,
        reset_controllers=False
    ):
        self.set_mode(mode)
        self.set_targets(**targets)

        if limits is not None:
            self.set_limits(**limits)

        if active_controllers is not None:
            self.set_active_controllers(active_controllers)

        if reset_controllers:
            state = self.sensor_input_manager.get_state()
            self.reset_controllers_for_state(state)
            
    # ---------------- Added 11/06/26 ----------------
    def receive_aircraft_state(self, aircraft_state: AircraftState):
        self.latest_aircraft_state = aircraft_state
    # -------------------------------------------------

    def reset_controllers_for_state(self, state: AircraftState | None = None):
        if state is None:
            state = self.sensor_input_manager.get_state()

        resets = {
            "fpa": state.flight_path_angle,
            "heading": state.heading,
            "roll": state.roll,
            "pitch": state.pitch,
            "yaw": state.heading,
            "speed": state.speed,
            "altitude": state.altitude,
            "cross_track": 0.0,
            "glide_slope": 0.0,
        }

        for name, measurement in resets.items():
            try:
                self.controller_registry.reset(name, measurement)
            except KeyError:
                pass

    def update_once(self):
        # state: AircraftState = self.sensor_input_manager.get_state()
        if self.latest_aircraft_state is None:
            state: AircraftState = self.sensor_input_manager.get_state()
        else:
            state: AircraftState = self.latest_aircraft_state
        targets = self.target_manager.get_targets()

        # print("[Autopilot][State]")
        # print({
        #     "heading": state.heading,
        #     "speed": state.speed,
        #     "pitch": state.pitch,
        #     "roll": state.roll,
        #     "yaw_rate": state.yaw_rate,
        # })
        # print("[Autopilot][Targets]")
        # print(targets)

        raw_cmd = self.controller_chain_manager.update(state, targets)
        
        # print("[Autopilot][RawCommand]")
        # print(raw_cmd)
        
        safe_cmd = self.command_output_manager.process(raw_cmd, targets)
        
        # print("[Autopilot][SafeCommand]")
        # print(safe_cmd)

        self.command_output_manager.write_to_vessel(safe_cmd)

    def start(self):
        if self._running:
            return

        self._stop_event.clear()
        self._running = True

        self._thread = threading.Thread(
            target=self._loop,
            daemon=True
        )
        self._thread.start()

    def stop(self):
        if not self._running:
            return

        self._stop_event.set()

        if self._thread is not None:
            self._thread.join(timeout=2.0)

        self.command_output_manager.zero_flight_controls()

        self._running = False
        self._thread = None

    def _loop(self):
        print("[Autopilot] Engaged.")

        while not self._stop_event.is_set():
            try:
                self.update_once()
            except Exception as e:
                print(f"[Autopilot] Update error: {e}")

            time.sleep(self.update_period)

        print("[Autopilot] Disengaged.")