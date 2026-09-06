from abc import ABC, abstractmethod

from shared_data_classes import (
    AircraftState,
    ActiveGuidanceTarget,
    GuidanceDemands,
)

from navigation_calculator import (
    NavigationSolution,
    wrap_360,
    clamp,
    first_valid,
)

# NOTE:
# The current shared_data_classes.GuidanceDemands dataclass does not define
# a `complete` constructor field. Completion should either be added to that
# dataclass later, or stored in `extra` and reflected through GuidanceMethod.


# ============================================================
# Parent Guidance Law
# ============================================================

class GuidanceLaw(ABC):
    """
    Parent class for all guidance laws.

    Guidance laws:
    - receive AircraftState
    - receive ActiveGuidanceTarget
    - receive NavigationSolution
    - convert navigation geometry/errors into GuidanceDemands
    - do not receive AircraftState directly from SensorInputManager
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
        navigation_solution: NavigationSolution,
    ) -> GuidanceDemands:
        pass


# ============================================================
# Taxi Guidance Law
# ============================================================

class TaxiGuidanceLaw(GuidanceLaw):
    def __init__(self, default_taxi_speed=8.0):
        super().__init__("TaxiGuidanceLaw")
        self.default_taxi_speed = default_taxi_speed

    def calculate(self, aircraft_state, active_guidance_target, navigation_solution):
        waypoint = navigation_solution.target

        desired_heading = navigation_solution.bearing_to_waypoint

        desired_speed = first_valid(
            getattr(active_guidance_target, "taxi_speed", None),
            active_guidance_target.desired_speed,
            getattr(waypoint, "speed", None),
            self.default_taxi_speed,
        )

        capture_radius = first_valid(
            active_guidance_target.capture_radius,
            10.0,
        )

        waypoint_captured = (
            navigation_solution.distance_to_waypoint is not None
            and navigation_solution.distance_to_waypoint <= capture_radius
        )

        self.diagnostics.update({
            "desired_heading": desired_heading,
            "desired_speed": desired_speed,
            "waypoint_captured": waypoint_captured,
            "capture_radius": capture_radius,
        })

        return GuidanceDemands(
            desired_heading=desired_heading,
            desired_speed=desired_speed,
            distance_to_waypoint=navigation_solution.distance_to_waypoint,
            waypoint_index=navigation_solution.extra.get("waypoint_index"),
            extra={
                "target_type": active_guidance_target.target_type.value,
                "waypoint_captured": waypoint_captured,
            },
        )


# ============================================================
# Runway Guidance Law
# ============================================================

class RunwayGuidanceLaw(GuidanceLaw):
    def __init__(
        self,
        default_rotation_speed=70.0,
        default_initial_climb_fpa=8.0,
    ):
        super().__init__("RunwayGuidanceLaw")
        self.default_rotation_speed = default_rotation_speed
        self.default_initial_climb_fpa = default_initial_climb_fpa

    def calculate(self, aircraft_state, active_guidance_target, navigation_solution):
        runway = navigation_solution.target

        rotation_speed = first_valid(
            active_guidance_target.rotation_speed,
            navigation_solution.extra.get("rotation_speed"),
            self.default_rotation_speed,
        )

        rotation_ready = bool(navigation_solution.rotation_ready)

        desired_fpa = (
            first_valid(
                active_guidance_target.initial_climb_fpa,
                self.default_initial_climb_fpa,
            )
            if rotation_ready
            else 0.0
        )

        desired_speed = first_valid(
            active_guidance_target.desired_speed,
            getattr(runway, "approach_speed", None),
            rotation_speed,
        )

        runway_heading = wrap_360(navigation_solution.runway_heading)

        self.diagnostics.update({
            "runway_heading": runway_heading,
            "rotation_ready": rotation_ready,
            "rotation_speed": rotation_speed,
            "desired_fpa": desired_fpa,
            "desired_speed": desired_speed,
        })

        return GuidanceDemands(
            desired_heading=runway_heading,
            runway_heading=runway_heading,
            desired_fpa=desired_fpa,
            desired_speed=desired_speed,
            cross_track_error=navigation_solution.runway_centreline_error,
            extra={
                "rotation_ready": rotation_ready,
                "rotation_speed": rotation_speed,
                "runway_centreline_error": navigation_solution.runway_centreline_error,
                "runway_remaining_distance": navigation_solution.runway_remaining_distance,
                "target_type": active_guidance_target.target_type.value,
            },
        )


# ============================================================
# Point-to-Point Guidance Law
# ============================================================

class PointToPointGuidanceLaw(GuidanceLaw):
    def __init__(
        self,
        default_max_fpa=10.0,
        default_speed=140.0,
    ):
        super().__init__("PointToPointGuidanceLaw")
        self.default_max_fpa = default_max_fpa
        self.default_speed = default_speed

    def calculate(self, aircraft_state, active_guidance_target, navigation_solution):
        waypoint = navigation_solution.target

        max_fpa = first_valid(
            active_guidance_target.max_fpa,
            self.default_max_fpa,
        )

        desired_fpa = clamp(
            navigation_solution.required_fpa,
            -max_fpa,
            max_fpa,
        )

        desired_speed = first_valid(
            active_guidance_target.desired_speed,
            getattr(waypoint, "speed", None),
            self.default_speed,
        )

        desired_altitude = first_valid(
            active_guidance_target.desired_altitude,
            getattr(waypoint, "altitude", None),
        )

        capture_radius = first_valid(
            active_guidance_target.capture_radius,
            100.0,
        )

        waypoint_captured = (
            navigation_solution.distance_to_waypoint is not None
            and navigation_solution.distance_to_waypoint <= capture_radius
        )

        self.diagnostics.update({
            "desired_heading": navigation_solution.bearing_to_waypoint,
            "desired_fpa": desired_fpa,
            "desired_speed": desired_speed,
            "desired_altitude": desired_altitude,
            "waypoint_captured": waypoint_captured,
        })

        return GuidanceDemands(
            desired_heading=navigation_solution.bearing_to_waypoint,
            desired_fpa=desired_fpa,
            desired_speed=desired_speed,
            desired_altitude=desired_altitude,
            distance_to_waypoint=navigation_solution.distance_to_waypoint,
            waypoint_index=navigation_solution.extra.get("waypoint_index"),
            extra={
                "target_type": active_guidance_target.target_type.value,
                "waypoint_captured": waypoint_captured,
                "altitude_error": navigation_solution.altitude_error,
            },
        )


# ============================================================
# Path-Following Guidance Law
# ============================================================

class PathFollowingGuidanceLaw(GuidanceLaw):
    def __init__(
        self,
        default_speed=160.0,
        default_cross_track_gain=0.02,
        default_max_intercept_angle=30.0,
    ):
        super().__init__("PathFollowingGuidanceLaw")
        self.default_speed = default_speed
        self.default_cross_track_gain = default_cross_track_gain
        self.default_max_intercept_angle = default_max_intercept_angle

    def calculate(self, aircraft_state, active_guidance_target, navigation_solution):
        active_wp = navigation_solution.target

        cross_track_gain = first_valid(
            active_guidance_target.extra.get("cross_track_gain"),
            self.default_cross_track_gain,
        )

        max_intercept_angle = first_valid(
            active_guidance_target.extra.get("max_intercept_angle"),
            self.default_max_intercept_angle,
        )

        correction_angle = clamp(
            cross_track_gain * navigation_solution.cross_track_error,
            -max_intercept_angle,
            max_intercept_angle,
        )

        desired_heading = wrap_360(
            navigation_solution.path_heading + correction_angle
        )

        desired_speed = first_valid(
            active_guidance_target.desired_speed,
            getattr(active_wp, "speed", None),
            self.default_speed,
        )

        desired_altitude = first_valid(
            active_guidance_target.desired_altitude,
            getattr(active_wp, "altitude", None),
        )

        capture_radius = first_valid(
            active_guidance_target.capture_radius,
            100.0,
        )

        waypoint_captured = (
            navigation_solution.distance_to_waypoint is not None
            and navigation_solution.distance_to_waypoint <= capture_radius
        )

        self.diagnostics.update({
            "path_heading": navigation_solution.path_heading,
            "cross_track_error": navigation_solution.cross_track_error,
            "cross_track_gain": cross_track_gain,
            "correction_angle": correction_angle,
            "desired_heading": desired_heading,
            "desired_speed": desired_speed,
            "desired_altitude": desired_altitude,
            "waypoint_captured": waypoint_captured,
        })

        return GuidanceDemands(
            desired_heading=desired_heading,
            path_heading=navigation_solution.path_heading,
            cross_track_error=navigation_solution.cross_track_error,
            desired_altitude=desired_altitude,
            desired_speed=desired_speed,
            distance_to_waypoint=navigation_solution.distance_to_waypoint,
            waypoint_index=navigation_solution.extra.get("waypoint_index"),
            extra={
                "target_type": active_guidance_target.target_type.value,
                "along_track_distance": navigation_solution.along_track_distance,
                "segment_length": navigation_solution.segment_length,
                "segment_progress": navigation_solution.segment_progress,
                "cross_track_correction_angle": correction_angle,
                "waypoint_captured": waypoint_captured,
            },
        )


# ============================================================
# Descent Path Guidance Law
# ============================================================

class DescentPathGuidanceLaw(GuidanceLaw):
    def __init__(
        self,
        default_descent_fpa=-3.0,
        default_max_descent_fpa=-8.0,
        default_descent_speed=130.0,
    ):
        super().__init__("DescentPathGuidanceLaw")
        self.default_descent_fpa = default_descent_fpa
        self.default_max_descent_fpa = default_max_descent_fpa
        self.default_descent_speed = default_descent_speed

    def calculate(self, aircraft_state, active_guidance_target, navigation_solution):
        target = navigation_solution.target

        max_descent_fpa = first_valid(
            active_guidance_target.max_descent_fpa,
            self.default_max_descent_fpa,
        )

        geometric_fpa = navigation_solution.geometric_fpa

        desired_fpa = clamp(
            geometric_fpa,
            max_descent_fpa,
            0.0,
        )

        nominal_descent_fpa = first_valid(
            active_guidance_target.desired_fpa,
            self.default_descent_fpa,
        )

        if desired_fpa > nominal_descent_fpa:
            desired_fpa = nominal_descent_fpa

        desired_speed = first_valid(
            active_guidance_target.desired_speed,
            self.default_descent_speed,
        )

        desired_altitude = getattr(target, "altitude", None)

        capture_radius = first_valid(
            active_guidance_target.capture_radius,
            500.0,
        )

        descent_path_captured = (
            navigation_solution.distance_to_target is not None
            and navigation_solution.distance_to_target <= capture_radius
        )

        self.diagnostics.update({
            "bearing_to_target": navigation_solution.bearing_to_target,
            "geometric_fpa": geometric_fpa,
            "desired_fpa": desired_fpa,
            "desired_speed": desired_speed,
            "desired_altitude": desired_altitude,
            "descent_path_captured": descent_path_captured,
        })

        return GuidanceDemands(
            desired_heading=navigation_solution.bearing_to_target,
            desired_fpa=desired_fpa,
            desired_speed=desired_speed,
            desired_altitude=desired_altitude,
            distance_to_waypoint=navigation_solution.distance_to_target,
            distance_to_runway=navigation_solution.distance_to_runway,
            extra={
                "target_type": active_guidance_target.target_type.value,
                "altitude_error": navigation_solution.altitude_error,
                "geometric_fpa": geometric_fpa,
                "descent_path_captured": descent_path_captured,
            },
        )


# ============================================================
# Runway Approach Guidance Law
# ============================================================

class RunwayApproachGuidanceLaw(GuidanceLaw):
    def __init__(
        self,
        default_glide_slope_angle=-3.0,
        default_approach_speed=75.0,
        default_cross_track_gain=0.02,
        default_max_intercept_angle=25.0,
        default_glide_slope_gain=0.01,
        default_max_glide_slope_correction=2.0,
    ):
        super().__init__("RunwayApproachGuidanceLaw")
        self.default_glide_slope_angle = default_glide_slope_angle
        self.default_approach_speed = default_approach_speed
        self.default_cross_track_gain = default_cross_track_gain
        self.default_max_intercept_angle = default_max_intercept_angle
        self.default_glide_slope_gain = default_glide_slope_gain
        self.default_max_glide_slope_correction = default_max_glide_slope_correction

    def calculate(self, aircraft_state, active_guidance_target, navigation_solution):
        runway = navigation_solution.target

        cross_track_gain = first_valid(
            active_guidance_target.extra.get("cross_track_gain"),
            self.default_cross_track_gain,
        )

        max_intercept_angle = first_valid(
            active_guidance_target.extra.get("max_intercept_angle"),
            self.default_max_intercept_angle,
        )

        correction_angle = clamp(
            cross_track_gain * navigation_solution.cross_track_error,
            -max_intercept_angle,
            max_intercept_angle,
        )

        desired_heading = wrap_360(
            navigation_solution.approach_path_heading + correction_angle
        )

        glide_slope_angle = first_valid(
            navigation_solution.extra.get("glide_slope_angle"),
            active_guidance_target.glide_slope_angle,
            getattr(runway, "approach_fpa", None),
            self.default_glide_slope_angle,
        )

        glide_slope_gain = first_valid(
            active_guidance_target.extra.get("glide_slope_gain"),
            self.default_glide_slope_gain,
        )

        max_glide_slope_correction = first_valid(
            active_guidance_target.extra.get("max_glide_slope_correction"),
            self.default_max_glide_slope_correction,
        )

        # Positive glide_slope_error means above the slope, so command slightly
        # more negative FPA. Negative error means below the slope, so command
        # slightly less negative FPA.
        fpa_correction = clamp(
            -glide_slope_gain * navigation_solution.glide_slope_error,
            -max_glide_slope_correction,
            max_glide_slope_correction,
        )

        desired_fpa = glide_slope_angle + fpa_correction

        desired_speed = first_valid(
            active_guidance_target.desired_speed,
            getattr(runway, "approach_speed", None),
            self.default_approach_speed,
        )

        capture_radius = first_valid(
            active_guidance_target.capture_radius,
            50.0,
        )

        runway_captured = (
            navigation_solution.distance_to_runway is not None
            and navigation_solution.distance_to_runway <= capture_radius
        )

        self.diagnostics.update({
            "runway_heading": navigation_solution.runway_heading,
            "desired_heading": desired_heading,
            "cross_track_error": navigation_solution.cross_track_error,
            "correction_angle": correction_angle,
            "glide_slope_angle": glide_slope_angle,
            "glide_slope_error": navigation_solution.glide_slope_error,
            "fpa_correction": fpa_correction,
            "desired_fpa": desired_fpa,
            "desired_speed": desired_speed,
            "runway_captured": runway_captured,
        })

        return GuidanceDemands(
            desired_heading=desired_heading,
            runway_heading=navigation_solution.runway_heading,
            path_heading=navigation_solution.approach_path_heading,
            cross_track_error=navigation_solution.cross_track_error,
            glide_slope_error=navigation_solution.glide_slope_error,
            desired_fpa=desired_fpa,
            desired_speed=desired_speed,
            distance_to_runway=navigation_solution.distance_to_runway,
            extra={
                "target_type": active_guidance_target.target_type.value,
                "along_track_distance": navigation_solution.along_track_distance,
                "segment_progress": navigation_solution.segment_progress,
                "desired_altitude_on_slope": navigation_solution.desired_altitude_on_slope,
                "localizer_correction_angle": correction_angle,
                "glide_slope_fpa_correction": fpa_correction,
                "runway_captured": runway_captured,
            },
        )
