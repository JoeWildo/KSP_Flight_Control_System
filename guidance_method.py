from abc import ABC
from enum import Enum

from shared_data_classes import (
    AircraftState,
    GuidanceDemands,
    ActiveGuidanceTarget,
)

from navigation_calculator import (
    NavigationCalculator,
    TaxiNavigationCalculator,
    RunwayNavigationCalculator,
    PointToPointNavigationCalculator,
    PathFollowingNavigationCalculator,
    DescentPathNavigationCalculator,
    RunwayApproachNavigationCalculator,
)

from guidance_law import (
    GuidanceLaw,
    TaxiGuidanceLaw,
    RunwayGuidanceLaw,
    PointToPointGuidanceLaw,
    PathFollowingGuidanceLaw,
    DescentPathGuidanceLaw,
    RunwayApproachGuidanceLaw,
)


# ============================================================
# Shared Data Objects
# ============================================================

class GuidanceMethodName(Enum):
    TAXI = "TAXI"
    RUNWAY = "RUNWAY"
    POINT_TO_POINT = "POINT_TO_POINT"
    PATH_FOLLOWING = "PATH_FOLLOWING"
    DESCENT_PATH = "DESCENT_PATH"
    RUNWAY_APPROACH = "RUNWAY_APPROACH"


# ============================================================
# Parent Guidance Method
# ============================================================

class GuidanceMethod(ABC):
    """
    Parent class for all guidance methods.

    Guidance methods:
    - receive AircraftState from SensorInputManager
    - receive ActiveGuidanceTarget from ModeManager
    - coordinate a NavigationCalculator
    - coordinate a GuidanceLaw
    - send GuidanceDemands directly to FlightState
    - do not read MissionContext directly
    - do not update Autopilot directly
    """

    def __init__(
        self,
        name: GuidanceMethodName,
        navigation_calculator: NavigationCalculator,
        guidance_law: GuidanceLaw,
    ):
        self.name = name
        self.has_entered = False

        self.latest_aircraft_state: AircraftState | None = None
        self.active_guidance_target: ActiveGuidanceTarget | None = None

        self.navigation_calculator = navigation_calculator
        self.guidance_law = guidance_law

        self.output_receiver = None

        self.last_navigation_solution = None
        self.last_demands = GuidanceDemands()
        self.diagnostics = {}
        self.complete = False

    # --------------------------------------------------------
    # Connections
    # --------------------------------------------------------

    def receive_aircraft_state(self, aircraft_state: AircraftState):
        self.latest_aircraft_state = aircraft_state

    def set_output_receiver(self, receiver):
        self.output_receiver = receiver

    def set_active_guidance_target(
        self,
        active_guidance_target: ActiveGuidanceTarget,
    ):
        self.active_guidance_target = active_guidance_target

    # --------------------------------------------------------
    # Lifecycle
    # --------------------------------------------------------

    def enter(
        self,
        active_guidance_target: ActiveGuidanceTarget | None = None,
        **kwargs,
    ):
        self.has_entered = True
        self.complete = False
        self.diagnostics.clear()

        if self.navigation_calculator is not None:
            self.navigation_calculator.reset()

        if self.guidance_law is not None:
            self.guidance_law.reset()

        if active_guidance_target is not None:
            self.active_guidance_target = active_guidance_target

        self._on_enter()

    def update(
        self,
        active_guidance_target: ActiveGuidanceTarget | None = None,
        **kwargs,
    ) -> GuidanceDemands:
        if active_guidance_target is not None:
            self.active_guidance_target = active_guidance_target

        if not self.has_entered:
            self.enter(active_guidance_target=self.active_guidance_target)

        if self.latest_aircraft_state is None:
            raise RuntimeError(
                f"{self.name.value} GuidanceMethod has no AircraftState."
            )

        if self.active_guidance_target is None:
            raise RuntimeError(
                f"{self.name.value} GuidanceMethod has no ActiveGuidanceTarget."
            )

        navigation_solution = self.navigation_calculator.calculate(
            self.latest_aircraft_state,
            self.active_guidance_target,
        )

        demands = self.guidance_law.calculate(
            self.latest_aircraft_state,
            self.active_guidance_target,
            navigation_solution,
        )

        self.last_navigation_solution = navigation_solution
        self.last_demands = demands
        self.complete = bool(getattr(demands, "complete", False))

        self.diagnostics.update({
            "guidance_method": self.name.value,
            "complete": self.complete,
            "navigation_calculator": self.navigation_calculator.get_diagnostics(),
            "guidance_law": self.guidance_law.get_diagnostics(),
        })

        if self.output_receiver is not None:
            if hasattr(self.output_receiver, "receive_guidance_demands"):
                self.output_receiver.receive_guidance_demands(demands)

        return demands

    def exit(self, **kwargs):
        self._on_exit()
        self.has_entered = False

    def get_diagnostics(self):
        return dict(self.diagnostics)

    def get_transition_request(self):
        """
        Placeholder for future transition logic.

        ModeManager can call this method, but guidance completion/transition
        policy should be developed separately once mission sequencing is stable.
        """
        return None

    def _on_enter(self):
        pass

    def _on_exit(self):
        pass


# ============================================================
# Taxi Guidance
# ============================================================

class TaxiGuidance(GuidanceMethod):
    def __init__(self, default_taxi_speed=8.0):
        super().__init__(
            name=GuidanceMethodName.TAXI,
            navigation_calculator=TaxiNavigationCalculator(),
            guidance_law=TaxiGuidanceLaw(
                default_taxi_speed=default_taxi_speed,
            ),
        )


# ============================================================
# Runway / Takeoff Guidance
# ============================================================

class RunwayGuidance(GuidanceMethod):
    def __init__(
        self,
        default_rotation_speed=70.0,
        default_initial_climb_fpa=8.0,
    ):
        super().__init__(
            name=GuidanceMethodName.RUNWAY,
            navigation_calculator=RunwayNavigationCalculator(),
            guidance_law=RunwayGuidanceLaw(
                default_rotation_speed=default_rotation_speed,
                default_initial_climb_fpa=default_initial_climb_fpa,
            ),
        )


# ============================================================
# Point-to-Point Guidance
# ============================================================

class PointToPointGuidance(GuidanceMethod):
    def __init__(
        self,
        default_max_fpa=10.0,
        default_speed=140.0,
    ):
        super().__init__(
            name=GuidanceMethodName.POINT_TO_POINT,
            navigation_calculator=PointToPointNavigationCalculator(),
            guidance_law=PointToPointGuidanceLaw(
                default_max_fpa=default_max_fpa,
                default_speed=default_speed,
            ),
        )


# ============================================================
# Path-Following Guidance
# ============================================================

class PathFollowingGuidance(GuidanceMethod):
    def __init__(
        self,
        default_speed=160.0,
        default_cross_track_gain=0.02,
        default_max_intercept_angle=30.0,
    ):
        super().__init__(
            name=GuidanceMethodName.PATH_FOLLOWING,
            navigation_calculator=PathFollowingNavigationCalculator(),
            guidance_law=PathFollowingGuidanceLaw(
                default_speed=default_speed,
                default_cross_track_gain=default_cross_track_gain,
                default_max_intercept_angle=default_max_intercept_angle,
            ),
        )


# ============================================================
# Descent Path Guidance
# ============================================================

class DescentPathGuidance(GuidanceMethod):
    def __init__(
        self,
        default_descent_fpa=-3.0,
        default_max_descent_fpa=-8.0,
        default_descent_speed=130.0,
    ):
        super().__init__(
            name=GuidanceMethodName.DESCENT_PATH,
            navigation_calculator=DescentPathNavigationCalculator(),
            guidance_law=DescentPathGuidanceLaw(
                default_descent_fpa=default_descent_fpa,
                default_max_descent_fpa=default_max_descent_fpa,
                default_descent_speed=default_descent_speed,
            ),
        )


# ============================================================
# Runway Approach Guidance
# ============================================================

class RunwayApproachGuidance(GuidanceMethod):
    def __init__(
        self,
        default_glide_slope_angle=-3.0,
        default_approach_speed=75.0,
        default_cross_track_gain=0.02,
        default_max_intercept_angle=25.0,
        default_glide_slope_gain=0.01,
        default_max_glide_slope_correction=2.0,
    ):
        super().__init__(
            name=GuidanceMethodName.RUNWAY_APPROACH,
            navigation_calculator=RunwayApproachNavigationCalculator(),
            guidance_law=RunwayApproachGuidanceLaw(
                default_glide_slope_angle=default_glide_slope_angle,
                default_approach_speed=default_approach_speed,
                default_cross_track_gain=default_cross_track_gain,
                default_max_intercept_angle=default_max_intercept_angle,
                default_glide_slope_gain=default_glide_slope_gain,
                default_max_glide_slope_correction=default_max_glide_slope_correction,
            ),
        )
