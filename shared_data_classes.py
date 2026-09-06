
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional


# ============================================================
# Aircraft State Snapshot
# ============================================================

@dataclass(frozen=True)
class AircraftState:
    timestamp: float

    latitude: float
    longitude: float
    altitude: float

    speed: float
    vertical_speed: float
    horizontal_speed: float

    pitch: float
    heading: float
    roll: float

    pitch_rate: float
    yaw_rate: float
    roll_rate: float

    angle_of_attack: float
    sideslip_angle: float

    flight_path_angle: float
    flight_path_angle_rate: float

    velocity_surface: tuple


# ============================================================
# Mission Type Definitions
# ============================================================

class MissionType(Enum):
    SINGLE_AERIAL_WAYPOINT = "SINGLE_AERIAL_WAYPOINT"
    SINGLE_GROUND_WAYPOINT = "SINGLE_GROUND_WAYPOINT"
    FULL_WAYPOINT_MISSION = "FULL_WAYPOINT_MISSION"


class FinalActionType(Enum):
    NONE = "NONE"
    ORBIT = "ORBIT"
    LAND_AND_TAXI = "LAND_AND_TAXI"
    STOP_AUTOMATION = "STOP_AUTOMATION"


class WaypointType(Enum):
    AIRBORNE = "AIRBORNE"
    GROUND = "GROUND"
    APPROACH = "APPROACH"
    TAXI = "TAXI"
    HOLD_CENTER = "HOLD_CENTER"


# ============================================================
# Core Navigation Data
# ============================================================

@dataclass
class Waypoint:
    name: str

    latitude: float
    longitude: float
    altitude: float

    speed: Optional[float] = None

    waypoint_type: WaypointType = WaypointType.AIRBORNE

    extra: dict = field(default_factory=dict)


@dataclass
class RunwayData:
    name: str

    latitude: float
    longitude: float
    altitude: float

    heading: float

    approach_fpa: float = -3.0
    approach_speed: Optional[float] = None

    runway_length: Optional[float] = None
    touchdown_offset: float = 0.0

    extra: dict = field(default_factory=dict)


# ============================================================
# Mission Context Support Classes
# ============================================================

@dataclass
class FlightProfile:
    cruise_altitude: Optional[float] = None

    cruise_speed: Optional[float] = None
    climb_speed: Optional[float] = None
    descent_speed: Optional[float] = None
    approach_speed: Optional[float] = None
    taxi_speed: Optional[float] = None

    minimum_safe_speed: Optional[float] = None
    maximum_safe_speed: Optional[float] = None

    maximum_bank_angle: Optional[float] = None
    maximum_pitch_angle: Optional[float] = None

    maximum_descent_rate: Optional[float] = None


@dataclass
class RouteRules:
    waypoint_capture_radius: float = 250.0

    max_cross_track_error: float = 500.0
    warning_cross_track_error: float = 250.0

    path_rejoin_allowed: bool = True
    path_rejoin_distance: float = 1000.0

    max_intercept_angle: float = 45.0

    altitude_tolerance: float = 50.0
    speed_tolerance: float = 10.0
    heading_tolerance: float = 5.0


@dataclass
class LandingRules:
    approach_capture_distance: float = 6000.0
    touchdown_capture_distance: float = 80.0

    go_around_allowed: bool = True
    go_around_altitude: Optional[float] = None

    flare_altitude: Optional[float] = None

    taxi_after_landing: bool = True


@dataclass
class HoldRules:
    hold_enabled: bool = False

    hold_radius: float = 1000.0
    hold_direction_clockwise: bool = True

    hold_altitude: Optional[float] = None
    hold_speed: Optional[float] = None


@dataclass
class SafetyRules:
    minimum_safe_altitude: Optional[float] = None

    stall_recovery_speed: Optional[float] = None
    overspeed_limit: Optional[float] = None

    lost_path_recovery_allowed: bool = True


# ============================================================
# Mission Context
# ============================================================

@dataclass
class MissionContext:
    mission_name: str
    mission_type: MissionType

    final_action: FinalActionType = FinalActionType.NONE

    waypoints: list[Waypoint] = field(default_factory=list)
    runway: Optional[RunwayData] = None

    flight_profile: FlightProfile = field(default_factory=FlightProfile)
    route_rules: RouteRules = field(default_factory=RouteRules)
    landing_rules: LandingRules = field(default_factory=LandingRules)
    hold_rules: HoldRules = field(default_factory=HoldRules)
    safety_rules: SafetyRules = field(default_factory=SafetyRules)

    extra: dict = field(default_factory=dict)


# ============================================================
# Mission Progress
# ============================================================

@dataclass
class MissionProgress:
    active_waypoint_index: int = 0
    previous_waypoint_index: Optional[int] = None

    completed_waypoint_indices: list[int] = field(default_factory=list)

    final_action_active: bool = False
    landing_started: bool = False
    landing_complete: bool = False
    taxi_started: bool = False
    taxi_complete: bool = False

    mission_complete: bool = False

    last_event: Optional[str] = None
    extra: dict = field(default_factory=dict)


# ============================================================
# Guidance Demands
# ============================================================

@dataclass
class GuidanceDemands:
    desired_heading: Optional[float] = None
    desired_fpa: Optional[float] = None
    desired_roll: Optional[float] = None
    desired_pitch: Optional[float] = None
    desired_speed: Optional[float] = None
    desired_altitude: Optional[float] = None

    path_heading: Optional[float] = None
    cross_track_error: Optional[float] = None
    glide_slope_error: Optional[float] = None

    runway_heading: Optional[float] = None
    distance_to_waypoint: Optional[float] = None
    distance_to_runway: Optional[float] = None

    waypoint_index: Optional[int] = None

    extra: dict = field(default_factory=dict)


# ============================================================
# Active Guidance Target
# ============================================================

class ActiveGuidanceTargetType(Enum):
    TAXI = "TAXI"
    RUNWAY = "RUNWAY"
    POINT_TO_POINT = "POINT_TO_POINT"
    PATH_FOLLOWING = "PATH_FOLLOWING"
    DESCENT_PATH = "DESCENT_PATH"
    RUNWAY_APPROACH = "RUNWAY_APPROACH"


@dataclass
class ActiveGuidanceTarget:
    target_type: ActiveGuidanceTargetType

    target_waypoint: Optional[Waypoint] = None
    previous_waypoint: Optional[Waypoint] = None
    next_waypoint: Optional[Waypoint] = None

    runway: Optional[RunwayData] = None

    desired_altitude: Optional[float] = None
    desired_speed: Optional[float] = None
    desired_fpa: Optional[float] = None
    desired_heading: Optional[float] = None

    capture_radius: Optional[float] = None

    max_fpa: Optional[float] = None
    max_descent_fpa: Optional[float] = None

    glide_slope_angle: Optional[float] = None
    approach_capture_distance: Optional[float] = None
    touchdown_capture_distance: Optional[float] = None

    taxi_speed: Optional[float] = None
    rotation_speed: Optional[float] = None
    initial_climb_fpa: Optional[float] = None

    extra: dict = field(default_factory=dict)


# ============================================================
# Transition Request
# ============================================================

@dataclass
class TransitionRequest:
    next_flight_state: object | None = None
    next_guidance_method: object | None = None
    reason: str = ""
    reset_autopilot: bool = True
    mission_complete: bool = False