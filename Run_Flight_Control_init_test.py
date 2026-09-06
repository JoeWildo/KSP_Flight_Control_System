import time
import krpc

from sensor_input_manager import SensorInputManager
from mode_manager import ModeManager

from guidance_method import (
    TaxiGuidance,
    RunwayGuidance,
    PointToPointGuidance,
    PathFollowingGuidance,
    DescentPathGuidance,
    RunwayApproachGuidance,
)

from flight_state import (
    TaxiState,
    TakeoffState,
    ClimbState,
    CruiseState,
    DescentState,
    LandingState,
)

from autopilot import (
    Autopilot,
    ControllerRegistry,
)

from PID_Controller import (
    RollPID,
    PitchPID,
    YawPID,
    FPAPID,
    SpeedPID,
    HeadingPID,
    AltitudePID,
    GlideSlopePID,
    CrossTrackPID,
)

from shared_data_classes import (
    MissionContext,
    MissionType,
    FinalActionType,
    FlightProfile,
    RouteRules,
    Waypoint,
    WaypointType,
    GuidanceDemands,
)


def build_controller_registry():
    controllers = {
        "roll": RollPID(0.04, 0.0, 0.02),
        "pitch": PitchPID(0.04, 0.0, 0.02),
        "yaw": YawPID(1.2, 0.5, 0.5),
        "fpa": FPAPID(0.08, 0.0, 0.03),
        "speed": SpeedPID(0.03, 0.0, 0.01),
        "heading": HeadingPID(0.8, 0.0, 0.2),
        "altitude": AltitudePID(0.01, 0.0, 0.003),
        "glide_slope": GlideSlopePID(0.05, 0.0, 0.02),
        "cross_track": CrossTrackPID(0.01, 0.0, 0.002),
    }

    return ControllerRegistry(controllers)


def build_guidance_methods():
    return {
        "TAXI": TaxiGuidance(),
        "RUNWAY": RunwayGuidance(),
        "POINT_TO_POINT": PointToPointGuidance(),
        "PATH_FOLLOWING": PathFollowingGuidance(),
        "DESCENT_PATH": DescentPathGuidance(),
        "RUNWAY_APPROACH": RunwayApproachGuidance(),
    }


def build_flight_states(autopilot):
    return {
        "TAXI": TaxiState(autopilot=autopilot),
        "TAKEOFF": TakeoffState(autopilot=autopilot),
        "CLIMB": ClimbState(autopilot=autopilot),
        "CRUISE": CruiseState(autopilot=autopilot),
        "DESCENT": DescentState(autopilot=autopilot),
        "LANDING": LandingState(autopilot=autopilot),
    }


def build_mission_context():
    return MissionContext(
        mission_name="Taxi guidance integration test",
        mission_type=MissionType.SINGLE_GROUND_WAYPOINT,
        final_action=FinalActionType.NONE,
        waypoints=[
            Waypoint(
                name='Taxi waypoint - End of Runway',
                latitude=-0.0502,
                longitude=-74.6974,
                altitude=71.0,
                speed=10.0,
                waypoint_type=WaypointType.TAXI,
                extra={
                    "commence_takeoff": False,
                },
            )
        ],
        flight_profile=FlightProfile(
            taxi_speed=8.0
        ),
        route_rules=RouteRules(
            waypoint_capture_radius=10.0,
            heading_tolerance=20.0,
            speed_tolerance=100.0,
        ),
        extra={
            'active_waypoint_index': 0,
            'initial_guidance_method': 'TAXI',
            'initial_flight_state': 'TAXI',
        }
    )


def main():
    print("[Main] Connecting to kRPC...")

    conn = krpc.connect(name="KSP Flight Control System Init Test")
    vessel = conn.space_center.active_vessel

    print(f"[Main] Connected to vessel: {vessel.name}")

    print("[Main] Creating SensorInputManager...")
    sensor_input_manager = SensorInputManager(conn, vessel)

    print("[Main] Creating PID ControllerRegistry...")
    controller_registry = build_controller_registry()

    print("[Main] Creating Autopilot...")
    autopilot = Autopilot(
        vessel=vessel,
        streams={},  # Not used in this startup test.
        controller_registry=controller_registry,
        update_rate_hz=20.0,
    )

    print("[Main] Creating GuidanceDemands placeholder...")
    guidance_demands = GuidanceDemands()

    print("[Main] Creating GuidanceMethods...")
    guidance_methods = build_guidance_methods()

    print("[Main] Creating FlightStates...")
    flight_states = build_flight_states(autopilot)

    print("[Main] Creating MissionContext...")
    mission_context = build_mission_context()

    print("[Main] Creating ModeManager...")
    mode_manager = ModeManager(
        sensor_input_manager=sensor_input_manager,
        flight_states=flight_states,
        guidance_methods=guidance_methods,
        initial_flight_state="TAXI",
        initial_guidance_method="TAXI",
        mission_context=mission_context,
        update_rate_hz=2.0,
    )

    print("[Main] Starting ModeManager connection test...")
    
    time.sleep(2)
    
    mode_manager.start()
    autopilot.start()

    try:
        while True:
            time.sleep(1.0)
            # print("[Main] Diagnostics:")
            # print(mode_manager.get_diagnostics())

    except KeyboardInterrupt:
        print("[Main] Ctrl+C received. Stopping FCS...")


    # time.sleep(2.0)

    # print("[Main] Diagnostics:")
    # print(mode_manager.get_diagnostics())

    # print("[Main] Stopping ModeManager...")
    # mode_manager.stop()

    # print("[Main] Startup/connectivity test complete.")
    # print("[Main] Autopilot control loop was not started.")


if __name__ == "__main__":
    main()