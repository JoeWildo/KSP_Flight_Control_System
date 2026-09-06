from abc import ABC, abstractmethod
from enum import Enum

from autopilot import AutopilotMode
from shared_data_classes import (
    AircraftState,
    GuidanceDemands,
)


# ============================================================
# Flight State Names
# ============================================================

class FlightStateName(Enum):
    TAXI = "TAXI"
    TAKEOFF = "TAKEOFF"
    CLIMB = "CLIMB"
    CRUISE = "CRUISE"
    DESCENT = "DESCENT"
    LANDING = "LANDING"


# ============================================================
# Base FlightState
# ============================================================

class FlightState(ABC):
    """
    Base class for all flight states.

    FlightState:
    - receives AircraftState from SensorInputManager
    - receives GuidanceDemands from GuidanceMethod
    - converts GuidanceDemands into Autopilot configuration
    - updates/configures Autopilot
    - does not select GuidanceMethod
    - does not read ActiveGuidanceTarget directly
    """

    def __init__(self, name: FlightStateName, autopilot=None):
        self.name = name
        self.autopilot = autopilot

        self.has_entered = False
        self.complete = False
        self.next_state = None

        self.latest_aircraft_state: AircraftState | None = None
        self.latest_guidance_demands: GuidanceDemands = GuidanceDemands()

        self.diagnostics = {}

    # --------------------------------------------------------
    # Connections
    # --------------------------------------------------------

    def set_autopilot(self, autopilot):
        self.autopilot = autopilot

    def receive_aircraft_state(self, aircraft_state: AircraftState):
        self.latest_aircraft_state = aircraft_state

    def receive_guidance_demands(self, guidance_demands: GuidanceDemands):
        self.latest_guidance_demands = guidance_demands

    # --------------------------------------------------------
    # Lifecycle
    # --------------------------------------------------------

    def enter(self, context=None, **kwargs):
        self.has_entered = True
        self.complete = False
        self.next_state = None

        if self.latest_aircraft_state is not None and self.autopilot is not None:
            self._on_enter(
                self.latest_aircraft_state,
                self.latest_guidance_demands,
                self.autopilot,
                context,
            )

    def update(self, context=None, **kwargs):
        if "mission_context" in kwargs and context is None:
            context = kwargs["mission_context"]

        if self.latest_aircraft_state is None:
            raise RuntimeError(f"{self.name.value} FlightState has no AircraftState.")

        if self.autopilot is None:
            raise RuntimeError(f"{self.name.value} FlightState has no Autopilot.")

        if not self.has_entered:
            self.enter(context=context)

        autopilot_config = self.build_autopilot_config(
            self.latest_aircraft_state,
            self.latest_guidance_demands,
            context,
        )
        
        # ---------- Added 11/06/26 ----------
        if hasattr(self.autopilot, "receive_aircraft_state"):
            self.autopilot.receive_aircraft_state(self.latest_aircraft_state)
        # ------------------------------------

        self.autopilot.configure_from_flight_state(
            mode=autopilot_config["mode"],
            targets=autopilot_config["targets"],
            limits=autopilot_config["limits"],
            active_controllers=autopilot_config["active_controllers"],
            reset_controllers=autopilot_config.get("reset_controllers", False),
        )

        self.complete = self.is_complete(
            self.latest_aircraft_state,
            self.latest_guidance_demands,
            context,
        )

        if self.complete:
            self.next_state = self.get_next_state(
                self.latest_aircraft_state,
                self.latest_guidance_demands,
                context,
            )

    def exit(self, context=None, **kwargs):
        if "mission_context" in kwargs and context is None:
            context = kwargs["mission_context"]

        if self.latest_aircraft_state is not None and self.autopilot is not None:
            self._on_exit(
                self.latest_aircraft_state,
                self.latest_guidance_demands,
                self.autopilot,
                context,
            )

        self.has_entered = False

    def get_diagnostics(self):
        return dict(self.diagnostics)

    # --------------------------------------------------------
    # Methods child classes override
    # --------------------------------------------------------

    def _on_enter(self, aircraft_state, guidance_demands, autopilot, context):
        pass

    def _on_exit(self, aircraft_state, guidance_demands, autopilot, context):
        pass

    @abstractmethod
    def build_autopilot_config(
        self,
        aircraft_state: AircraftState,
        guidance_demands: GuidanceDemands,
        context
    ) -> dict:
        pass

    @abstractmethod
    def is_complete(
        self,
        aircraft_state: AircraftState,
        guidance_demands: GuidanceDemands,
        context
    ) -> bool:
        pass

    @abstractmethod
    def get_next_state(
        self,
        aircraft_state: AircraftState,
        guidance_demands: GuidanceDemands,
        context
    ):
        pass


# ============================================================
# Taxi State
# ============================================================

class TaxiState(FlightState):
    def __init__(self, autopilot=None):
        super().__init__(FlightStateName.TAXI, autopilot=autopilot)

    def _on_enter(self, aircraft_state, guidance_demands, autopilot, context):
        autopilot.reset_controllers_for_state(aircraft_state)

    def build_autopilot_config(self, aircraft_state, guidance_demands, context):
        target_heading = (
            guidance_demands.desired_heading
            if guidance_demands.desired_heading is not None
            else aircraft_state.heading
        )

        target_speed = (
            guidance_demands.desired_speed
            if guidance_demands.desired_speed is not None
            else 10.0
        )

        return {
            "mode": AutopilotMode.TAXI,
            "targets": {
                "target_heading": target_heading,
                "target_speed": target_speed,
            },
            "active_controllers": {
                "yaw",
                "speed",
            },
            "limits": {
                "yaw_limits": (-1.0, 1.0),
                "throttle_limits": (0.0, 1.0),
                "brake_limits": (0.0, 1.0),
            },
            "reset_controllers": False,
        }
        
    def is_complete(self, aircraft_state, guidance_demands, context):
        return False

    def get_next_state(self, aircraft_state, guidance_demands, context):
        return None


# ============================================================
# Takeoff State
# ============================================================

class TakeoffState(FlightState):
    def __init__(
        self,
        rotation_speed=70.0,
        liftoff_altitude=10.0,
        initial_climb_fpa=8.0,
        autopilot=None,
    ):
        super().__init__(FlightStateName.TAKEOFF, autopilot=autopilot)
        self.rotation_speed = rotation_speed
        self.liftoff_altitude = liftoff_altitude
        self.initial_climb_fpa = initial_climb_fpa

    def _on_enter(self, aircraft_state, guidance_demands, autopilot, context):
        autopilot.reset_controllers_for_state(aircraft_state)

    def build_autopilot_config(self, aircraft_state, guidance_demands, context):
        runway_heading = (
            guidance_demands.runway_heading
            or guidance_demands.desired_heading
            or aircraft_state.heading
        )

        rotate = aircraft_state.speed >= self.rotation_speed

        if not rotate:
            target_fpa = 0.0
        else:
            target_fpa = (
                guidance_demands.desired_fpa
                if guidance_demands.desired_fpa is not None
                else self.initial_climb_fpa
            )

        return {
            "mode": AutopilotMode.TAKEOFF,
            "targets": {
                "target_heading": runway_heading,
                "target_roll": 0.0,
                "target_fpa": target_fpa,
                "target_speed": guidance_demands.desired_speed,
            },
            "active_controllers": {
                "yaw",
                "roll",
                "fpa",
                "speed",
            },
            "limits": {
                "pitch_limits": (-0.2, 0.8) if rotate else (-0.1, 0.1),
                "roll_limits": (-0.4, 0.4),
                "yaw_limits": (-0.6, 0.6),
                "throttle_limits": (0.8, 1.0),
            },
            "reset_controllers": False,
        }
        
    def is_complete(self, aircraft_state, guidance_demands, context):
        return False

    def get_next_state(self, aircraft_state, guidance_demands, context):
        return None


# ============================================================
# Climb State
# ============================================================

class ClimbState(FlightState):
    def __init__(self, cruise_altitude=5000.0, autopilot=None):
        super().__init__(FlightStateName.CLIMB, autopilot=autopilot)
        self.cruise_altitude = cruise_altitude

    def _on_enter(self, aircraft_state, guidance_demands, autopilot, context):
        autopilot.reset_controllers_for_state(aircraft_state)

    def build_autopilot_config(self, aircraft_state, guidance_demands, context):
        target_fpa = guidance_demands.desired_fpa

        if target_fpa is None:
            target_fpa = 5.0

        return {
            "mode": AutopilotMode.CLIMB,
            "targets": {
                "target_heading": guidance_demands.desired_heading,
                "target_fpa": target_fpa,
                "target_speed": guidance_demands.desired_speed,
            },
            "active_controllers": {
                "heading",
                "roll",
                "fpa",
                "speed",
            },
            "limits": {
                "pitch_limits": (-0.4, 0.7),
                "roll_limits": (-0.8, 0.8),
                "yaw_limits": (-0.25, 0.25),
                "throttle_limits": (0.5, 1.0),
            },
            "reset_controllers": False,
        }
        
    def is_complete(self, aircraft_state, guidance_demands, context):
        return False

    def get_next_state(self, aircraft_state, guidance_demands, context):
        return None


# ============================================================
# Cruise State
# ============================================================

class CruiseState(FlightState):
    def __init__(self, autopilot=None):
        super().__init__(FlightStateName.CRUISE, autopilot=autopilot)

    def _on_enter(self, aircraft_state, guidance_demands, autopilot, context):
        autopilot.reset_controllers_for_state(aircraft_state)

    def build_autopilot_config(self, aircraft_state, guidance_demands, context):
        return {
            "mode": AutopilotMode.CRUISE,
            "targets": {
                "target_heading": guidance_demands.desired_heading,
                "path_heading": guidance_demands.path_heading,
                "cross_track_error": guidance_demands.cross_track_error,
                "target_altitude": guidance_demands.desired_altitude,
                "target_speed": guidance_demands.desired_speed,
            },
            "active_controllers": {
                "cross_track",
                "heading",
                "roll",
                "altitude",
                "fpa",
                "speed",
            },
            "limits": {
                "pitch_limits": (-0.4, 0.4),
                "roll_limits": (-0.7, 0.7),
                "yaw_limits": (-0.2, 0.2),
                "throttle_limits": (0.3, 0.9),
            },
            "reset_controllers": False,
        }
        
    def is_complete(self, aircraft_state, guidance_demands, context):
        return False

    def get_next_state(self, aircraft_state, guidance_demands, context):
        return None


# ============================================================
# Descent State
# ============================================================

class DescentState(FlightState):
    def __init__(self, autopilot=None):
        super().__init__(FlightStateName.DESCENT, autopilot=autopilot)

    def _on_enter(self, aircraft_state, guidance_demands, autopilot, context):
        autopilot.reset_controllers_for_state(aircraft_state)

    def build_autopilot_config(self, aircraft_state, guidance_demands, context):
        target_fpa = guidance_demands.desired_fpa

        if target_fpa is None:
            target_fpa = -3.0

        return {
            "mode": AutopilotMode.DESCENT,
            "targets": {
                "target_heading": guidance_demands.desired_heading,
                "target_fpa": target_fpa,
                "target_speed": guidance_demands.desired_speed,
            },
            "active_controllers": {
                "heading",
                "roll",
                "fpa",
                "speed",
            },
            "limits": {
                "pitch_limits": (-0.6, 0.3),
                "roll_limits": (-0.7, 0.7),
                "yaw_limits": (-0.2, 0.2),
                "throttle_limits": (0.0, 0.8),
            },
            "reset_controllers": False,
        }
        
    def is_complete(self, aircraft_state, guidance_demands, context):
        return False

    def get_next_state(self, aircraft_state, guidance_demands, context):
        return None


# ============================================================
# Landing State
# ============================================================

class LandingState(FlightState):
    def __init__(
        self,
        flare_altitude=20.0,
        autopilot=None,
    ):
        super().__init__(FlightStateName.LANDING, autopilot=autopilot)
        self.flare_altitude = flare_altitude

    def _on_enter(self, aircraft_state, guidance_demands, autopilot, context):
        autopilot.reset_controllers_for_state(aircraft_state)

    def build_autopilot_config(self, aircraft_state, guidance_demands, context):
        runway_heading = (
            guidance_demands.runway_heading
            or guidance_demands.desired_heading
        )

        in_flare = aircraft_state.altitude <= self.flare_altitude

        if in_flare:
            target_fpa = -0.5
            throttle_limits = (0.0, 0.2)
        else:
            target_fpa = (
                guidance_demands.desired_fpa
                if guidance_demands.desired_fpa is not None
                else -3.0
            )
            throttle_limits = (0.0, 0.7)

        return {
            "mode": AutopilotMode.LANDING_APPROACH,
            "targets": {
                "target_heading": runway_heading,
                "target_fpa": target_fpa,
                "target_speed": guidance_demands.desired_speed,
                "glide_slope_error": guidance_demands.glide_slope_error,
                "cross_track_error": guidance_demands.cross_track_error,
                "path_heading": guidance_demands.path_heading,
            },
            "active_controllers": {
                "cross_track",
                "heading",
                "roll",
                "glide_slope",
                "fpa",
                "speed",
            },
            "limits": {
                "pitch_limits": (-0.5, 0.5),
                "roll_limits": (-0.5, 0.5),
                "yaw_limits": (-0.3, 0.3),
                "throttle_limits": throttle_limits,
                "brake_limits": (0.0, 1.0),
            },
            "reset_controllers": False,
        }  
        
    def is_complete(self, aircraft_state, guidance_demands, context):
        return False

    def get_next_state(self, aircraft_state, guidance_demands, context):
        return None
    
    
    