import time
import threading
from enum import Enum

from shared_data_classes import (
    TransitionRequest,
    ActiveGuidanceTarget,
    ActiveGuidanceTargetType,
    Waypoint,
    WaypointType
)


class ModeManagerStatus(Enum):
    STOPPED = "STOPPED"
    RUNNING = "RUNNING"
    PAUSED = "PAUSED"
    COMPLETE = "COMPLETE"
    ABORTED = "ABORTED"


class ModeManager:
    """
    Corrected ModeManager.

    The ModeManager coordinates update order and transitions.

    It does not pass AircraftState to GuidanceMethod or FlightState.
    It does not receive GuidanceDemands.
    It does not update Autopilot directly.
    """

    def __init__(
        self,
        sensor_input_manager,
        flight_states: dict,
        guidance_methods: dict,
        initial_flight_state,
        initial_guidance_method,
        mission_context=None,
        update_rate_hz=20.0,
    ):
        self.sensor_input_manager = sensor_input_manager
        self.flight_states = flight_states
        self.guidance_methods = guidance_methods
        self.mission_context = mission_context

        self.current_flight_state = self._resolve_flight_state(
            initial_flight_state
        )
        self.current_guidance_method = self._resolve_guidance_method(
            initial_guidance_method
        )

        self.previous_flight_state = None
        self.previous_guidance_method = None

        self.latest_aircraft_state = None

        self.status = ModeManagerStatus.STOPPED
        self.update_rate_hz = update_rate_hz
        self.update_period = 1.0 / update_rate_hz

        self._thread = None
        self._stop_event = threading.Event()

        self.last_transition_reason = None
        self.diagnostics = {}

    # --------------------------------------------------------
    # Resolve helpers
    # --------------------------------------------------------

    def _resolve_flight_state(self, state):
        if isinstance(state, str):
            return self.flight_states[state]
        return state

    def _resolve_guidance_method(self, guidance):
        if isinstance(guidance, str):
            return self.guidance_methods[guidance]
        return guidance
    
    def _build_initial_active_guidance_target(self):
        if self.latest_aircraft_state is None:
            return ActiveGuidanceTarget(
                target_type=ActiveGuidanceTargetType.POINT_TO_POINT,
                desired_speed=0.0,
                desired_fpa=0.0,
                desired_heading=0.0,
                capture_radius=999999.0,
                extra={
                    "initialization_target": True,
                    "reason": "No aircraft snapshot available yet",
                },
            )

        hold_position = Waypoint(
            name="INITIALIZATION_HOLD_POSITION",
            latitude=self.latest_aircraft_state.latitude,
            longitude=self.latest_aircraft_state.longitude,
            altitude=self.latest_aircraft_state.altitude,
            speed=self.latest_aircraft_state.speed,
            waypoint_type=WaypointType.AIRBORNE,
        )

        return ActiveGuidanceTarget(
            target_type=ActiveGuidanceTargetType.POINT_TO_POINT,
            target_waypoint=hold_position,
            desired_altitude=self.latest_aircraft_state.altitude,
            desired_speed=self.latest_aircraft_state.speed,
            desired_fpa=0.0,
            desired_heading=self.latest_aircraft_state.heading,
            capture_radius=999999.0,
            max_fpa=0.0,
            extra={
                "initialization_target": True,
                "reason": "Neutral AGT for system startup connection test",
                "waypoint_index": None,
            },
        )


    def _build_active_guidance_target_from_mission_context(self):
        if self.mission_context is None:
            return self._build_initial_active_guidance_target()

        if not self.mission_context.waypoints:
            return self._build_initial_active_guidance_target()

        active_index = self.mission_context.extra.get("active_waypoint_index", 0)
        active_waypoint = self.mission_context.waypoints[active_index]

        if active_waypoint.waypoint_type == WaypointType.TAXI:
            agt = ActiveGuidanceTarget(
                target_type=ActiveGuidanceTargetType.TAXI,
                target_waypoint=active_waypoint,
                desired_speed=active_waypoint.speed,
                taxi_speed=self.mission_context.flight_profile.taxi_speed,
                capture_radius=self.mission_context.route_rules.waypoint_capture_radius,
                extra={
                    "waypoint_index": active_index,
                    "source": "mission_context",
                    "builder": "_build_active_guidance_target_from_mission_context",
                },
            )

            # print("[ModeManager][AGT BUILD]")
            # print(self._describe_active_guidance_target(agt))

            return agt

        return self._build_initial_active_guidance_target()
        
    # ActiveGuidanceTarget Debug formatter
    def _describe_active_guidance_target(self, agt):
        if agt is None:
            return None

        waypoint = agt.target_waypoint

        return {
            "target_type": agt.target_type.value if agt.target_type is not None else None,

            "target_waypoint": None if waypoint is None else {
                "name": waypoint.name,
                "latitude": waypoint.latitude,
                "longitude": waypoint.longitude,
                "altitude": waypoint.altitude,
                "speed": waypoint.speed,
                "waypoint_type": waypoint.waypoint_type.value,
                "extra": dict(waypoint.extra),
            },

            "desired_altitude": agt.desired_altitude,
            "desired_speed": agt.desired_speed,
            "desired_fpa": agt.desired_fpa,
            "desired_heading": agt.desired_heading,
            "capture_radius": agt.capture_radius,
            "taxi_speed": agt.taxi_speed,
            "rotation_speed": agt.rotation_speed,
            "extra": dict(agt.extra),
        }
        

    # --------------------------------------------------------
    # Lifecycle
    # --------------------------------------------------------

    def start(self):
        if self.status == ModeManagerStatus.RUNNING:
            return

        self.status = ModeManagerStatus.RUNNING
        self._stop_event.clear()

        self._connect_active_modules()
        
        self.latest_aircraft_state = self.sensor_input_manager.get_snapshot()
        # self.active_guidance_target = self._build_initial_active_guidance_target()a
        self.active_guidance_target = self._build_active_guidance_target_from_mission_context()
        
        self._enter_active_modules()

        self._thread = threading.Thread(
            target=self._loop,
            daemon=True
        )
        self._thread.start()

        print("[ModeManager] Started.")

    def stop(self):
        self._stop_event.set()

        if self._thread is not None:
            self._thread.join(timeout=2.0)

        self._exit_active_modules()

        self.status = ModeManagerStatus.STOPPED
        self._thread = None

        print("[ModeManager] Stopped.")

    def pause(self):
        if self.status == ModeManagerStatus.RUNNING:
            self.status = ModeManagerStatus.PAUSED

    def resume(self):
        if self.status == ModeManagerStatus.PAUSED:
            self.status = ModeManagerStatus.RUNNING

    def abort(self, reason="Abort requested"):
        self.status = ModeManagerStatus.ABORTED
        self._stop_event.set()
        print(f"[ModeManager] Aborted: {reason}")

    # --------------------------------------------------------
    # Main update loop
    # --------------------------------------------------------

    def _loop(self):
        while not self._stop_event.is_set():
            if self.status == ModeManagerStatus.RUNNING:
                try:
                    self.update_once()
                except Exception as exc:
                    self.abort(f"ModeManager update error: {exc}")

            time.sleep(self.update_period)

    def update_once(self):
        """
        One deterministic update cycle.

        Correct order:
        1. ModeManager requests aircraft snapshot.
        2. SensorInputManager distributes snapshot to subscribed modules.
        3. ModeManager triggers GuidanceMethod update.
        4. GuidanceMethod sends GuidanceDemands to FlightState.
        5. ModeManager triggers FlightState update.
        6. FlightState updates Autopilot.
        7. ModeManager checks transitions.
        """

        # 1. Request snapshot.
        # SensorInputManager is responsible for distributing this snapshot
        # to ModeManager, GuidanceMethod, and FlightState.
        self.latest_aircraft_state = self.sensor_input_manager.get_snapshot()

        # 2. Trigger guidance update.
        # GuidanceMethod uses:
        #     latest AircraftState received from SensorInputManager
        #     ActiveGuidanceTarget received from ModeManager
        self.current_guidance_method.update(
            active_guidance_target=self.active_guidance_target
        )

        # 3. Trigger flight state update.
        # FlightState uses:
        #     latest AircraftState received from SensorInputManager
        #     latest GuidanceDemands received from GuidanceMethod
        #     autopilot reference stored inside FlightState
        self.current_flight_state.update(
            context=self.mission_context
        )

        # 4. Check transitions.
        transition = self._check_transition()

        if transition is not None:
            self._apply_transition(transition)

        self._update_diagnostics()

    # --------------------------------------------------------
    # Module connection
    # --------------------------------------------------------

    def _connect_active_modules(self):
        """
        Connects the active GuidanceMethod and FlightState together.

        GuidanceMethod sends GuidanceDemands directly to FlightState.
        SensorInputManager sends AircraftState to both.
        """

        if hasattr(self.current_guidance_method, "set_output_receiver"):
            self.current_guidance_method.set_output_receiver(
                self.current_flight_state
            )

        if hasattr(self.sensor_input_manager, "set_mode_manager_receiver"):
            self.sensor_input_manager.set_mode_manager_receiver(self)

        if hasattr(self.sensor_input_manager, "set_guidance_receiver"):
            self.sensor_input_manager.set_guidance_receiver(
                self.current_guidance_method
            )

        if hasattr(self.sensor_input_manager, "set_flight_state_receiver"):
            self.sensor_input_manager.set_flight_state_receiver(
                self.current_flight_state
            )

    def receive_aircraft_state(self, aircraft_state):
        """
        Optional receiver method.

        SensorInputManager can call this when it creates a new snapshot.
        """
        self.latest_aircraft_state = aircraft_state

    # --------------------------------------------------------
    # Enter / Exit
    # --------------------------------------------------------

    def _enter_active_modules(self):
        if hasattr(self.current_guidance_method, "enter"):
            self.current_guidance_method.enter(
                active_guidance_target=self.active_guidance_target
            )

        if hasattr(self.current_flight_state, "enter"):
            self.current_flight_state.enter(
                context=self.mission_context
            )

    def _exit_active_modules(self):
        if hasattr(self.current_flight_state, "exit"):
            self.current_flight_state.exit(
                context=self.mission_context
            )

        if hasattr(self.current_guidance_method, "exit"):
            self.current_guidance_method.exit()

    # --------------------------------------------------------
    # Transition logic
    # --------------------------------------------------------

    def _check_transition(self):
        """
        Transition logic remains centralised in ModeManager.

        The FlightState or GuidanceMethod may expose completion flags
        or transition requests, but the ModeManager decides whether
        to apply them.
        """

        if hasattr(self.current_flight_state, "get_transition_request"):
            request = self.current_flight_state.get_transition_request()
            if request is not None:
                return request

        if hasattr(self.current_guidance_method, "get_transition_request"):
            request = self.current_guidance_method.get_transition_request()
            if request is not None:
                return request

        if getattr(self.current_flight_state, "complete", False):
            next_state = getattr(self.current_flight_state, "next_state", None)

            if next_state is not None:
                next_guidance = self._select_guidance_for_state(next_state)

                return TransitionRequest(
                    next_flight_state=next_state,
                    next_guidance_method=next_guidance,
                    reason=f"{self.current_flight_state.name} complete",
                    reset_autopilot=True,
                )

        return None

    def _apply_transition(self, transition: TransitionRequest):
        if transition.mission_complete:
            self.status = ModeManagerStatus.COMPLETE
            self._stop_event.set()
            print("[ModeManager] Mission complete.")
            return

        next_state = self._resolve_flight_state(
            transition.next_flight_state
        )

        next_guidance = self._resolve_guidance_method(
            transition.next_guidance_method
        )

        print(
            f"[ModeManager] Transition: "
            f"{self.current_flight_state.name} -> {next_state.name} | "
            f"{self.current_guidance_method.name} -> {next_guidance.name} | "
            f"Reason: {transition.reason}"
        )

        self.last_transition_reason = transition.reason

        # Exit current modules.
        if hasattr(self.current_flight_state, "exit"):
            self.current_flight_state.exit(
                context=self.mission_context
            )

        if hasattr(self.current_guidance_method, "exit"):
            self.current_guidance_method.exit()

        self.previous_flight_state = self.current_flight_state
        self.previous_guidance_method = self.current_guidance_method

        self.current_flight_state = next_state
        self.current_guidance_method = next_guidance

        # Reconnect distribution paths.
        self._connect_active_modules()

        # Reset autopilot if the FlightState owns the autopilot reference.
        if transition.reset_autopilot:
            if hasattr(self.current_flight_state, "reset_autopilot"):
                self.current_flight_state.reset_autopilot()

        # Enter new modules.
        if hasattr(self.current_guidance_method, "enter"):
            self.current_guidance_method.enter(
                active_guidance_target=self.active_guidance_target
            )

        if hasattr(self.current_flight_state, "enter"):
            self.current_flight_state.enter(
                context=self.mission_context
            )

    def _select_guidance_for_state(self, next_state):
        state_name = (
            next_state.name.value
            if hasattr(next_state, "name") and hasattr(next_state.name, "value")
            else str(next_state)
        )

        mapping = {
            "TAXI": "TAXI",
            "TAKEOFF": "RUNWAY",
            "CLIMB": "POINT_TO_POINT",
            "CRUISE": "PATH_FOLLOWING",
            "DESCENT": "DESCENT_PATH",
            "LANDING": "RUNWAY_APPROACH",
        }

        guidance_key = mapping.get(state_name)

        if guidance_key is None:
            return self.current_guidance_method

        return self._resolve_guidance_method(guidance_key)

    # --------------------------------------------------------
    # Diagnostics
    # --------------------------------------------------------

    def _update_diagnostics(self):
        self.diagnostics = {
            "status": self.status.value,
            "flight_state": str(getattr(self.current_flight_state, "name", None)),
            "guidance_method": str(getattr(self.current_guidance_method, "name", None)),
            "last_transition_reason": self.last_transition_reason,
            "aircraft_state_available": self.latest_aircraft_state is not None,
            "flight_state_complete": getattr(
                self.current_flight_state,
                "complete",
                None
            ),
            "guidance_complete": getattr(
                self.current_guidance_method,
                "complete",
                None
            ),
        }

    def get_diagnostics(self):
        return dict(self.diagnostics)