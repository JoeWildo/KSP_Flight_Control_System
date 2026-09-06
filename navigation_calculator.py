import math
from abc import ABC, abstractmethod
from dataclasses import dataclass, field

from shared_data_classes import (
    AircraftState,
    ActiveGuidanceTarget,
)


# ============================================================
# Navigation Maths
# ============================================================

EARTH_RADIUS_M = 600_000.0
# Kerbin radius is roughly 600 km. Adjust if using another body.


def wrap_360(angle: float) -> float:
    return angle % 360.0


def wrap_180(angle: float) -> float:
    return (angle + 180.0) % 360.0 - 180.0


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(value, high))


def first_valid(*values):
    """
    Returns the first value that is not None.

    This avoids the common issue where a valid command of 0.0 is treated
    as False by Python's normal `or` fallback logic.
    """
    for value in values:
        if value is not None:
            return value
    return None


def latlon_to_local_xy(
    lat: float,
    lon: float,
    ref_lat: float,
    ref_lon: float,
    radius: float = EARTH_RADIUS_M,
):
    """
    Simple local tangent-plane approximation.

    Good enough for short-range waypoint guidance in KSP.
    x = east
    y = north
    """
    lat_rad = math.radians(lat)
    lon_rad = math.radians(lon)
    ref_lat_rad = math.radians(ref_lat)
    ref_lon_rad = math.radians(ref_lon)

    x = radius * (lon_rad - ref_lon_rad) * math.cos(ref_lat_rad)
    y = radius * (lat_rad - ref_lat_rad)

    return x, y


def local_xy_to_latlon(
    x: float,
    y: float,
    ref_lat: float,
    ref_lon: float,
    radius: float = EARTH_RADIUS_M,
):
    ref_lat_rad = math.radians(ref_lat)

    lat = ref_lat + math.degrees(y / radius)

    if abs(math.cos(ref_lat_rad)) < 1e-9:
        lon = ref_lon
    else:
        lon = ref_lon + math.degrees(x / (radius * math.cos(ref_lat_rad)))

    return lat, lon


def bearing_between(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    x, y = latlon_to_local_xy(lat2, lon2, lat1, lon1)
    bearing = math.degrees(math.atan2(x, y))
    return wrap_360(bearing)


def distance_between(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    x, y = latlon_to_local_xy(lat2, lon2, lat1, lon1)
    return math.hypot(x, y)


def required_fpa_deg(
    current_alt: float,
    target_alt: float,
    horizontal_distance: float,
) -> float:
    if horizontal_distance < 1.0:
        return 0.0

    altitude_error = target_alt - current_alt
    return math.degrees(math.atan2(altitude_error, horizontal_distance))


def cross_track_error_m(
    aircraft_lat: float,
    aircraft_lon: float,
    start_lat: float,
    start_lon: float,
    end_lat: float,
    end_lon: float,
) -> float:
    """
    Returns signed cross-track error in metres.

    Positive/negative sign depends on path orientation.
    """
    ax, ay = latlon_to_local_xy(aircraft_lat, aircraft_lon, start_lat, start_lon)
    bx, by = latlon_to_local_xy(end_lat, end_lon, start_lat, start_lon)

    path_length = math.hypot(bx, by)

    if path_length < 1e-6:
        return 0.0

    return (bx * ay - by * ax) / path_length


def along_track_distance_m(
    aircraft_lat: float,
    aircraft_lon: float,
    start_lat: float,
    start_lon: float,
    end_lat: float,
    end_lon: float,
) -> float:
    ax, ay = latlon_to_local_xy(aircraft_lat, aircraft_lon, start_lat, start_lon)
    bx, by = latlon_to_local_xy(end_lat, end_lon, start_lat, start_lon)

    path_length = math.hypot(bx, by)

    if path_length < 1e-6:
        return 0.0

    return (ax * bx + ay * by) / path_length


def waypoint_latlon_at_bearing_distance(
    start_lat: float,
    start_lon: float,
    bearing_deg: float,
    distance_m: float,
):
    bearing_rad = math.radians(bearing_deg)
    x = distance_m * math.sin(bearing_rad)
    y = distance_m * math.cos(bearing_rad)

    return local_xy_to_latlon(x, y, start_lat, start_lon)


def glide_slope_error_m(
    current_altitude: float,
    runway_altitude: float,
    glide_slope_angle_deg: float,
    distance_to_runway: float,
) -> tuple[float, float]:
    """
    Returns:
    - glide_slope_error
    - desired_altitude_on_slope

    Positive glide_slope_error means aircraft is above the desired glide path.
    Negative glide_slope_error means aircraft is below the desired glide path.
    """
    desired_altitude_on_slope = runway_altitude + (
        math.tan(math.radians(abs(glide_slope_angle_deg))) * distance_to_runway
    )

    glide_slope_error = current_altitude - desired_altitude_on_slope

    return glide_slope_error, desired_altitude_on_slope


# ============================================================
# Navigation Solution
# ============================================================

@dataclass
class NavigationSolution:
    """
    Generic container for navigation geometry and navigation errors.

    Not every guidance mode will populate every field.
    """
    bearing_to_target: float | None = None
    distance_to_target: float | None = None

    bearing_to_waypoint: float | None = None
    distance_to_waypoint: float | None = None

    runway_heading: float | None = None
    runway_centreline_error: float | None = None
    distance_along_runway: float | None = None
    runway_remaining_distance: float | None = None
    distance_to_runway: float | None = None

    path_heading: float | None = None
    cross_track_error: float | None = None
    along_track_distance: float | None = None
    segment_length: float | None = None
    segment_progress: float | None = None

    altitude_error: float | None = None
    required_fpa: float | None = None
    geometric_fpa: float | None = None

    speed_error_to_rotation: float | None = None
    rotation_ready: bool | None = None

    approach_path_heading: float | None = None
    desired_altitude_on_slope: float | None = None
    glide_slope_error: float | None = None

    target: object | None = None
    extra: dict = field(default_factory=dict)


# ============================================================
# Parent Navigation Calculator
# ============================================================

class NavigationCalculator(ABC):
    """
    Parent class for all navigation calculators.

    Navigation calculators:
    - receive AircraftState
    - receive ActiveGuidanceTarget
    - calculate geometry and navigation errors
    - do not create GuidanceDemands
    - do not send data to FlightState
    - do not update Autopilot
    """

    def __init__(self, name: str):
        self.name = name
        self.diagnostics = {}

    def reset(self):
        self.diagnostics.clear()

    def get_diagnostics(self):
        return dict(self.diagnostics)

    @abstractmethod
    def calculate(
        self,
        aircraft_state: AircraftState,
        active_guidance_target: ActiveGuidanceTarget,
    ) -> NavigationSolution:
        pass


# ============================================================
# Taxi Navigation Calculator
# ============================================================

class TaxiNavigationCalculator(NavigationCalculator):
    def __init__(self):
        super().__init__("TaxiNavigationCalculator")

    def calculate(self, aircraft_state, active_guidance_target):
        waypoint = active_guidance_target.target_waypoint

        if waypoint is None:
            raise ValueError("TaxiNavigationCalculator requires target_waypoint in AGT.")

        bearing = bearing_between(
            aircraft_state.latitude,
            aircraft_state.longitude,
            waypoint.latitude,
            waypoint.longitude,
        )

        distance = distance_between(
            aircraft_state.latitude,
            aircraft_state.longitude,
            waypoint.latitude,
            waypoint.longitude,
        )

        solution = NavigationSolution(
            bearing_to_target=bearing,
            distance_to_target=distance,
            bearing_to_waypoint=bearing,
            distance_to_waypoint=distance,
            target=waypoint,
            extra={
                "waypoint_index": active_guidance_target.extra.get("waypoint_index"),
            },
        )

        self.diagnostics.update({
            "bearing_to_taxi_waypoint": bearing,
            "distance_to_taxi_waypoint": distance,
        })

        return solution


# ============================================================
# Runway Navigation Calculator
# ============================================================

class RunwayNavigationCalculator(NavigationCalculator):
    def __init__(self):
        super().__init__("RunwayNavigationCalculator")

    def calculate(self, aircraft_state, active_guidance_target):
        runway = active_guidance_target.runway

        if runway is None:
            raise ValueError("RunwayNavigationCalculator requires runway in AGT.")

        runway_heading = wrap_360(runway.heading)

        rotation_speed = first_valid(
            active_guidance_target.rotation_speed,
            70.0,
        )

        speed_error_to_rotation = rotation_speed - aircraft_state.speed
        rotation_ready = aircraft_state.speed >= rotation_speed

        # Approximate centreline and along-track geometry using a synthetic runway path.
        # This is limited because RunwayData may not yet contain explicit runway start/end points.
        runway_start_lat, runway_start_lon = waypoint_latlon_at_bearing_distance(
            runway.latitude,
            runway.longitude,
            wrap_360(runway_heading + 180.0),
            5_000.0,
        )

        runway_end_lat, runway_end_lon = waypoint_latlon_at_bearing_distance(
            runway.latitude,
            runway.longitude,
            runway_heading,
            5_000.0,
        )

        centreline_error = cross_track_error_m(
            aircraft_state.latitude,
            aircraft_state.longitude,
            runway_start_lat,
            runway_start_lon,
            runway_end_lat,
            runway_end_lon,
        )

        distance_along_runway = along_track_distance_m(
            aircraft_state.latitude,
            aircraft_state.longitude,
            runway_start_lat,
            runway_start_lon,
            runway_end_lat,
            runway_end_lon,
        )

        runway_remaining = max(0.0, 10_000.0 - distance_along_runway)

        solution = NavigationSolution(
            runway_heading=runway_heading,
            runway_centreline_error=centreline_error,
            distance_along_runway=distance_along_runway,
            runway_remaining_distance=runway_remaining,
            speed_error_to_rotation=speed_error_to_rotation,
            rotation_ready=rotation_ready,
            target=runway,
            extra={
                "rotation_speed": rotation_speed,
            },
        )

        self.diagnostics.update({
            "runway_heading": runway_heading,
            "runway_centreline_error": centreline_error,
            "distance_along_runway": distance_along_runway,
            "runway_remaining_distance": runway_remaining,
            "speed_error_to_rotation": speed_error_to_rotation,
            "rotation_ready": rotation_ready,
        })

        return solution


# ============================================================
# Point-to-Point Navigation Calculator
# ============================================================

class PointToPointNavigationCalculator(NavigationCalculator):
    def __init__(self):
        super().__init__("PointToPointNavigationCalculator")

    def calculate(self, aircraft_state, active_guidance_target):
        waypoint = active_guidance_target.target_waypoint

        if waypoint is None:
            raise ValueError(
                "PointToPointNavigationCalculator requires target_waypoint in AGT."
            )

        bearing = bearing_between(
            aircraft_state.latitude,
            aircraft_state.longitude,
            waypoint.latitude,
            waypoint.longitude,
        )

        distance = distance_between(
            aircraft_state.latitude,
            aircraft_state.longitude,
            waypoint.latitude,
            waypoint.longitude,
        )

        altitude_error = waypoint.altitude - aircraft_state.altitude

        required_fpa = required_fpa_deg(
            aircraft_state.altitude,
            waypoint.altitude,
            distance,
        )

        solution = NavigationSolution(
            bearing_to_target=bearing,
            distance_to_target=distance,
            bearing_to_waypoint=bearing,
            distance_to_waypoint=distance,
            altitude_error=altitude_error,
            required_fpa=required_fpa,
            target=waypoint,
            extra={
                "waypoint_index": active_guidance_target.extra.get("waypoint_index"),
            },
        )

        self.diagnostics.update({
            "bearing_to_waypoint": bearing,
            "distance_to_waypoint": distance,
            "altitude_error": altitude_error,
            "required_fpa": required_fpa,
        })

        return solution


# ============================================================
# Path-Following Navigation Calculator
# ============================================================

class PathFollowingNavigationCalculator(NavigationCalculator):
    def __init__(self):
        super().__init__("PathFollowingNavigationCalculator")

    def calculate(self, aircraft_state, active_guidance_target):
        previous_wp = active_guidance_target.previous_waypoint
        active_wp = active_guidance_target.target_waypoint

        if previous_wp is None or active_wp is None:
            raise ValueError(
                "PathFollowingNavigationCalculator requires previous_waypoint "
                "and target_waypoint in AGT."
            )

        path_heading = bearing_between(
            previous_wp.latitude,
            previous_wp.longitude,
            active_wp.latitude,
            active_wp.longitude,
        )

        xtrack = cross_track_error_m(
            aircraft_state.latitude,
            aircraft_state.longitude,
            previous_wp.latitude,
            previous_wp.longitude,
            active_wp.latitude,
            active_wp.longitude,
        )

        along_track = along_track_distance_m(
            aircraft_state.latitude,
            aircraft_state.longitude,
            previous_wp.latitude,
            previous_wp.longitude,
            active_wp.latitude,
            active_wp.longitude,
        )

        distance_to_wp = distance_between(
            aircraft_state.latitude,
            aircraft_state.longitude,
            active_wp.latitude,
            active_wp.longitude,
        )

        segment_length = distance_between(
            previous_wp.latitude,
            previous_wp.longitude,
            active_wp.latitude,
            active_wp.longitude,
        )

        if segment_length > 1e-6:
            segment_progress = clamp(along_track / segment_length, 0.0, 1.0)
        else:
            segment_progress = 0.0

        solution = NavigationSolution(
            path_heading=path_heading,
            cross_track_error=xtrack,
            along_track_distance=along_track,
            distance_to_waypoint=distance_to_wp,
            segment_length=segment_length,
            segment_progress=segment_progress,
            target=active_wp,
            extra={
                "waypoint_index": active_guidance_target.extra.get("waypoint_index"),
            },
        )

        self.diagnostics.update({
            "path_heading": path_heading,
            "cross_track_error": xtrack,
            "along_track_distance": along_track,
            "distance_to_waypoint": distance_to_wp,
            "segment_length": segment_length,
            "segment_progress": segment_progress,
        })

        return solution


# ============================================================
# Descent Path Navigation Calculator
# ============================================================

class DescentPathNavigationCalculator(NavigationCalculator):
    def __init__(self):
        super().__init__("DescentPathNavigationCalculator")

    def calculate(self, aircraft_state, active_guidance_target):
        if active_guidance_target.runway is not None:
            target = active_guidance_target.runway
            distance_to_runway = distance_between(
                aircraft_state.latitude,
                aircraft_state.longitude,
                target.latitude,
                target.longitude,
            )
        elif active_guidance_target.target_waypoint is not None:
            target = active_guidance_target.target_waypoint
            distance_to_runway = None
        else:
            raise ValueError(
                "DescentPathNavigationCalculator requires runway or target_waypoint in AGT."
            )

        bearing = bearing_between(
            aircraft_state.latitude,
            aircraft_state.longitude,
            target.latitude,
            target.longitude,
        )

        distance_to_target = distance_between(
            aircraft_state.latitude,
            aircraft_state.longitude,
            target.latitude,
            target.longitude,
        )

        altitude_error = target.altitude - aircraft_state.altitude

        geometric_fpa = required_fpa_deg(
            aircraft_state.altitude,
            target.altitude,
            distance_to_target,
        )

        solution = NavigationSolution(
            bearing_to_target=bearing,
            distance_to_target=distance_to_target,
            distance_to_waypoint=distance_to_target,
            distance_to_runway=distance_to_runway,
            altitude_error=altitude_error,
            geometric_fpa=geometric_fpa,
            required_fpa=geometric_fpa,
            target=target,
        )

        self.diagnostics.update({
            "bearing_to_target": bearing,
            "distance_to_target": distance_to_target,
            "distance_to_runway": distance_to_runway,
            "altitude_error": altitude_error,
            "geometric_fpa": geometric_fpa,
        })

        return solution


# ============================================================
# Runway Approach Navigation Calculator
# ============================================================

class RunwayApproachNavigationCalculator(NavigationCalculator):
    def __init__(self, fake_start_distance=10_000.0):
        super().__init__("RunwayApproachNavigationCalculator")
        self.fake_start_distance = fake_start_distance

    def calculate(self, aircraft_state, active_guidance_target):
        runway = active_guidance_target.runway

        if runway is None:
            raise ValueError(
                "RunwayApproachNavigationCalculator requires runway in AGT."
            )

        distance_to_runway = distance_between(
            aircraft_state.latitude,
            aircraft_state.longitude,
            runway.latitude,
            runway.longitude,
        )

        path_heading = wrap_360(runway.heading)
        approach_course = wrap_360(runway.heading + 180.0)

        start_lat, start_lon = waypoint_latlon_at_bearing_distance(
            runway.latitude,
            runway.longitude,
            approach_course,
            self.fake_start_distance,
        )

        xtrack = cross_track_error_m(
            aircraft_state.latitude,
            aircraft_state.longitude,
            start_lat,
            start_lon,
            runway.latitude,
            runway.longitude,
        )

        along_track = along_track_distance_m(
            aircraft_state.latitude,
            aircraft_state.longitude,
            start_lat,
            start_lon,
            runway.latitude,
            runway.longitude,
        )

        glide_slope_angle = first_valid(
            active_guidance_target.glide_slope_angle,
            runway.approach_fpa,
            -3.0,
        )

        gs_error, desired_altitude_on_slope = glide_slope_error_m(
            aircraft_state.altitude,
            runway.altitude,
            glide_slope_angle,
            distance_to_runway,
        )

        solution = NavigationSolution(
            runway_heading=path_heading,
            path_heading=path_heading,
            approach_path_heading=path_heading,
            cross_track_error=xtrack,
            along_track_distance=along_track,
            segment_length=self.fake_start_distance,
            segment_progress=clamp(along_track / self.fake_start_distance, 0.0, 1.0),
            distance_to_runway=distance_to_runway,
            desired_altitude_on_slope=desired_altitude_on_slope,
            glide_slope_error=gs_error,
            target=runway,
            extra={
                "approach_course": approach_course,
                "approach_start_latitude": start_lat,
                "approach_start_longitude": start_lon,
                "glide_slope_angle": glide_slope_angle,
            },
        )

        self.diagnostics.update({
            "runway_heading": path_heading,
            "approach_course": approach_course,
            "distance_to_runway": distance_to_runway,
            "cross_track_error": xtrack,
            "along_track_distance": along_track,
            "desired_altitude_on_slope": desired_altitude_on_slope,
            "glide_slope_error": gs_error,
            "glide_slope_angle": glide_slope_angle,
        })

        return solution
