#!/usr/bin/env python3
"""UWB mission node.

Mission modes:
  - mock_full: pure software flow test.
  - bench_velocity: real hardware preflight + ARM + short Z velocity profile + DISARM.
  - takeoff_loiter_land: climb gently in GUIDED, hover in LOITER, switch back to GUIDED, land.
  - takeoff_forward_land: low GUIDED takeoff, body-forward local-position move, hover, land.
  - takeoff_waypoint_return_land: low GUIDED takeoff, local-position waypoint, return, land.
  - uwb_approach_land: low GUIDED takeoff, UWB approach to tag, hover, land.
  - real_full: take off, UWB approach, descend, fake/real grasp, climb, return, drop, land.
"""

import math
import threading
from enum import Enum

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import OpticalFlow, State as MavrosState
from mavros_msgs.srv import SetMode
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Range
from std_msgs.msg import String

from uav_delta_msgs.msg import FcuState, UwbAoa
from uav_delta_msgs.srv import FlightCommand


SENSOR_QOS = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)


class Phase(Enum):
    INIT = 0
    ARM = 1
    BENCH_VELOCITY = 2
    TAKEOFF = 3
    HOVER_TAKEOFF = 4
    MOVE_ABOVE = 5
    HOVER_ABOVE = 6
    DESCEND = 7
    HOVER_FINAL = 8
    WAIT_GRASP = 9
    CLIMB = 10
    HOVER_CLIMB = 11
    RETURN = 12
    HOVER_RETURN = 13
    WAIT_DROP = 14
    LAND = 15
    DONE = 16
    PAUSED_MANUAL = 17
    RECOVERING = 18
    FAILSAFE = 19
    HOVER_LOITER = 20
    LAND_WAIT = 21
    FORWARD = 22
    HOVER_FORWARD = 23
    WAYPOINT_OUTBOUND = 24
    HOVER_WAYPOINT = 25
    WAYPOINT_RETURN = 26
    HOVER_RETURN_HOME = 27
    WAYPOINT_DESCEND = 28
    HOVER_WAYPOINT_LOW = 29
    WAYPOINT_RECLIMB = 30
    HOVER_WAYPOINT_RECLIMB = 31


PHASE_NAMES = {phase: phase.name for phase in Phase}


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


def string_is_done(value: str) -> bool:
    return value.strip().upper() in {"1", "TRUE", "OK", "DONE", "COMPLETE", "SUCCESS"}


class TestMissionNode(Node):
    def __init__(self):
        super().__init__("test_mission_node")

        self.uwb_aoa_topic = self.declare_parameter("uwb_aoa_topic", "uwb_aoa/data").value
        self.fcu_state_topic = self.declare_parameter("fcu_state_topic", "fcu_state").value
        self.local_pose_topic = self.declare_parameter(
            "local_pose_topic", "/mavros/local_position/pose"
        ).value
        self.mavros_state_topic = self.declare_parameter(
            "mavros_state_topic", "/mavros/state"
        ).value
        self.rangefinder_topic = self.declare_parameter(
            "rangefinder_topic", "/mavros/rangefinder_pub"
        ).value
        self.optical_flow_topic = self.declare_parameter(
            "optical_flow_topic", "/mavros/optical_flow/raw/optical_flow"
        ).value
        self.fcu_link_topic = self.declare_parameter("fcu_link_topic", "fcu_link/status").value
        self.cmd_vel_topic = self.declare_parameter("cmd_vel_topic", "cmd_vel").value
        self.mission_state_topic = self.declare_parameter(
            "mission_state_topic", "test_mission/state"
        ).value
        self.mission_event_topic = self.declare_parameter(
            "mission_event_topic", "test_mission/event"
        ).value
        self.flight_command_service = self.declare_parameter(
            "flight_command_service", "flight_command"
        ).value

        self.mission_mode = self.declare_parameter("mission_mode", "").value
        self.use_mock = self.declare_parameter("use_mock", False).value
        self.desktop_test = self.declare_parameter("desktop_test", False).value
        if not self.mission_mode:
            if self.use_mock:
                self.mission_mode = "mock_full"
            elif self.desktop_test:
                self.mission_mode = "bench_velocity"
            else:
                self.mission_mode = "real_full"

        self.fake_grasp = self.declare_parameter("fake_grasp", True).value
        self.fake_grasp_delay_sec = self.declare_parameter("fake_grasp_delay_sec", 10.0).value
        self.grasp_timeout_sec = self.declare_parameter("grasp_timeout_sec", 15.0).value
        self.fake_drop = self.declare_parameter("fake_drop", True).value
        self.fake_drop_delay_sec = self.declare_parameter("fake_drop_delay_sec", 2.0).value
        self.drop_timeout_sec = self.declare_parameter("drop_timeout_sec", 10.0).value
        self.grasp_done_topic = self.declare_parameter("grasp_done_topic", "grasp_done").value
        self.drop_done_topic = self.declare_parameter("drop_done_topic", "drop_done").value

        self.takeoff_altitude = self.declare_parameter("takeoff_altitude", 1.5).value
        self.descend_altitude = self.declare_parameter("descend_altitude", 0.5).value
        self.takeoff_method = self.declare_parameter("takeoff_method", "mavros").value
        self.takeoff_climb_velocity = self.declare_parameter("takeoff_climb_velocity", 0.12).value
        self.takeoff_command_delay_sec = self.declare_parameter(
            "takeoff_command_delay_sec", 1.5
        ).value
        self.takeoff_height_timeout_sec = self.declare_parameter(
            "takeoff_height_timeout_sec", 8.0
        ).value
        self.loiter_min_rel_alt = self.declare_parameter("loiter_min_rel_alt", 0.25).value
        self.loiter_alt_loss_timeout_sec = self.declare_parameter(
            "loiter_alt_loss_timeout_sec", 0.7
        ).value
        self.guided_pre_loiter_stable_time_sec = self.declare_parameter(
            "guided_pre_loiter_stable_time_sec", 1.5
        ).value
        self.land_ground_rel_alt_threshold = self.declare_parameter(
            "land_ground_rel_alt_threshold", 0.12
        ).value
        self.land_ground_stable_sec = self.declare_parameter(
            "land_ground_stable_sec", 1.5
        ).value
        self.land_wait_timeout_sec = self.declare_parameter(
            "land_wait_timeout_sec", 12.0
        ).value

        self.kp_horizontal = self.declare_parameter("kp_horizontal", 0.4).value
        self.kp_vertical = self.declare_parameter("kp_vertical", 0.3).value
        self.kp_return = self.declare_parameter("kp_return", 0.5).value
        self.max_vel_xy = self.declare_parameter("max_vel_xy", 0.5).value
        self.max_vel_z = self.declare_parameter("max_vel_z", 0.3).value
        self.velocity_slew_rate = self.declare_parameter("velocity_slew_rate", 0.4).value

        self.azimuth_deadband = self.declare_parameter("azimuth_deadband", 3.0).value
        self.horizontal_deadband = self.declare_parameter("horizontal_deadband", 0.15).value
        self.uwb_azimuth_offset_deg = self.declare_parameter("uwb_azimuth_offset_deg", 0.0).value
        self.uwb_forward_sign = self.declare_parameter("uwb_forward_sign", 1.0).value
        self.uwb_lateral_sign = self.declare_parameter("uwb_lateral_sign", 1.0).value
        self.uwb_capture_radius_m = self.declare_parameter("uwb_capture_radius_m", 0.55).value
        self.uwb_slow_radius_m = self.declare_parameter("uwb_slow_radius_m", 1.0).value
        self.uwb_slow_max_vel_xy = self.declare_parameter("uwb_slow_max_vel_xy", 0.08).value
        self.uwb_target_hover_time_sec = self.declare_parameter(
            "uwb_target_hover_time_sec", 0.8
        ).value
        self.uwb_approach_front_sector_deg = self.declare_parameter(
            "uwb_approach_front_sector_deg", 65.0
        ).value
        self.uwb_capture_front_sector_deg = self.declare_parameter(
            "uwb_capture_front_sector_deg", 55.0
        ).value
        self.uwb_front_sector_timeout_sec = self.declare_parameter(
            "uwb_front_sector_timeout_sec", 2.0
        ).value
        self.uwb_capture_stable_sec = self.declare_parameter(
            "uwb_capture_stable_sec", 0.3
        ).value
        self.uwb_out_of_front_action = self.declare_parameter(
            "uwb_out_of_front_action", "LAND"
        ).value
        self.mission_soft_radius_m = self.declare_parameter("mission_soft_radius_m", 2.0).value
        self.mission_hard_radius_m = self.declare_parameter("mission_hard_radius_m", 2.5).value
        self.altitude_tolerance = self.declare_parameter("altitude_tolerance", 0.15).value
        self.return_xy_tolerance = self.declare_parameter("return_xy_tolerance", 0.3).value

        self.hover_stable_time = self.declare_parameter("hover_stable_time", 2.0).value
        self.control_rate_hz = self.declare_parameter("control_rate_hz", 20.0).value
        self.move_above_timeout_sec = self.declare_parameter(
            "move_above_timeout_sec", 20.0
        ).value
        self.uwb_missing_timeout_sec = self.declare_parameter(
            "uwb_missing_timeout_sec", 3.0
        ).value
        self.forward_target_distance = self.declare_parameter(
            "forward_target_distance", 0.3
        ).value
        self.forward_velocity = self.declare_parameter("forward_velocity", 0.12).value
        self.forward_timeout_sec = self.declare_parameter(
            "forward_timeout_sec", 8.0
        ).value
        self.waypoint_dx = self.declare_parameter("waypoint_dx", 1.0).value
        self.waypoint_dy = self.declare_parameter("waypoint_dy", 0.0).value
        self.kp_waypoint = self.declare_parameter("kp_waypoint", 0.45).value
        self.waypoint_max_velocity = self.declare_parameter(
            "waypoint_max_velocity", 0.3
        ).value
        self.waypoint_xy_tolerance = self.declare_parameter(
            "waypoint_xy_tolerance", 0.2
        ).value
        self.waypoint_timeout_sec = self.declare_parameter(
            "waypoint_timeout_sec", 12.0
        ).value
        self.return_hover_time = self.declare_parameter("return_hover_time", 2.0).value
        self.low_hover_time = self.declare_parameter("low_hover_time", 4.0).value
        self.target_hover_time = self.declare_parameter("target_hover_time", 2.0).value

        self.require_uwb_ready = self.declare_parameter("require_uwb_ready", True).value
        self.require_local_pose_ready = self.declare_parameter("require_local_pose_ready", True).value
        self.uwb_signal_timeout = self.declare_parameter("uwb_signal_timeout", 3.0).value
        self.local_pose_timeout = self.declare_parameter("local_pose_timeout", 1.0).value
        self.low_battery_pct = self.declare_parameter("low_battery_pct", 20.0).value
        self.recovery_timeout = self.declare_parameter("recovery_timeout", 3.0).value
        self.set_mode_service_timeout_sec = self.declare_parameter(
            "set_mode_service_timeout_sec", 5.0
        ).value
        self.mode_confirm_timeout_sec = self.declare_parameter(
            "mode_confirm_timeout_sec", 5.0
        ).value
        self.auto_modes = self._parse_modes(
            self.declare_parameter("auto_modes", "GUIDED").value
        )
        self.arm_mode = self.declare_parameter("arm_mode", "ALT_HOLD").value

        self.bench_velocity_z = self.declare_parameter("bench_velocity_z", 0.15).value
        self.bench_climb_sec = self.declare_parameter("bench_climb_sec", 2.0).value
        self.bench_hold_sec = self.declare_parameter("bench_hold_sec", 2.0).value
        self.bench_descend_sec = self.declare_parameter("bench_descend_sec", 2.0).value
        self.bench_zero_sec = self.declare_parameter("bench_zero_sec", 1.0).value
        self.bench_sensor_timeout = self.declare_parameter("bench_sensor_timeout", 2.0).value
        self.bench_exit_on_complete = self.declare_parameter(
            "bench_exit_on_complete", True
        ).value

        self.takeoff_altitude = max(0.5, self.takeoff_altitude)
        self.descend_altitude = max(0.2, self.descend_altitude)
        self.takeoff_method = str(self.takeoff_method).strip().lower()
        if self.takeoff_method not in ("mavros", "velocity"):
            self.takeoff_method = "mavros"
        self.takeoff_climb_velocity = clamp(
            abs(float(self.takeoff_climb_velocity)), 0.03, 0.25
        )
        self.takeoff_command_delay_sec = max(0.0, float(self.takeoff_command_delay_sec))
        self.takeoff_height_timeout_sec = max(1.0, float(self.takeoff_height_timeout_sec))
        self.loiter_min_rel_alt = max(0.05, float(self.loiter_min_rel_alt))
        self.loiter_alt_loss_timeout_sec = max(0.1, float(self.loiter_alt_loss_timeout_sec))
        self.guided_pre_loiter_stable_time_sec = max(
            0.0, float(self.guided_pre_loiter_stable_time_sec)
        )
        self.land_ground_rel_alt_threshold = max(0.03, float(self.land_ground_rel_alt_threshold))
        self.land_ground_stable_sec = max(0.2, float(self.land_ground_stable_sec))
        self.land_wait_timeout_sec = max(2.0, float(self.land_wait_timeout_sec))
        self.kp_horizontal = max(0.01, self.kp_horizontal)
        self.kp_vertical = max(0.01, self.kp_vertical)
        self.kp_return = max(0.01, self.kp_return)
        self.max_vel_xy = max(0.1, self.max_vel_xy)
        self.max_vel_z = max(0.05, self.max_vel_z)
        self.velocity_slew_rate = max(0.01, self.velocity_slew_rate)
        self.bench_velocity_z = clamp(abs(self.bench_velocity_z), 0.02, self.max_vel_z)
        self.grasp_timeout_sec = max(0.1, float(self.grasp_timeout_sec))
        self.drop_timeout_sec = max(0.1, float(self.drop_timeout_sec))
        self.set_mode_service_timeout_sec = max(
            0.5, float(self.set_mode_service_timeout_sec)
        )
        self.mode_confirm_timeout_sec = max(0.5, float(self.mode_confirm_timeout_sec))
        self.move_above_timeout_sec = max(1.0, float(self.move_above_timeout_sec))
        self.uwb_missing_timeout_sec = max(0.2, float(self.uwb_missing_timeout_sec))
        self.uwb_capture_radius_m = max(0.1, float(self.uwb_capture_radius_m))
        self.uwb_slow_radius_m = max(self.uwb_capture_radius_m, float(self.uwb_slow_radius_m))
        self.uwb_slow_max_vel_xy = clamp(
            abs(float(self.uwb_slow_max_vel_xy)), 0.02, self.max_vel_xy
        )
        self.uwb_target_hover_time_sec = max(0.1, float(self.uwb_target_hover_time_sec))
        self.uwb_approach_front_sector_deg = clamp(
            abs(float(self.uwb_approach_front_sector_deg)), 5.0, 179.0
        )
        self.uwb_capture_front_sector_deg = clamp(
            abs(float(self.uwb_capture_front_sector_deg)), 3.0, self.uwb_approach_front_sector_deg
        )
        self.uwb_front_sector_timeout_sec = max(0.2, float(self.uwb_front_sector_timeout_sec))
        self.uwb_capture_stable_sec = max(0.0, float(self.uwb_capture_stable_sec))
        self.uwb_out_of_front_action = str(self.uwb_out_of_front_action).strip().upper()
        if self.uwb_out_of_front_action not in ("LAND", "HOVER"):
            self.uwb_out_of_front_action = "LAND"
        self.mission_soft_radius_m = max(0.1, float(self.mission_soft_radius_m))
        self.mission_hard_radius_m = max(self.mission_soft_radius_m, float(self.mission_hard_radius_m))
        self.forward_target_distance = max(0.05, float(self.forward_target_distance))
        self.forward_velocity = clamp(abs(float(self.forward_velocity)), 0.03, self.max_vel_xy)
        self.forward_timeout_sec = max(1.0, float(self.forward_timeout_sec))
        self.waypoint_dx = float(self.waypoint_dx)
        self.waypoint_dy = float(self.waypoint_dy)
        self.kp_waypoint = max(0.01, float(self.kp_waypoint))
        self.waypoint_max_velocity = clamp(
            abs(float(self.waypoint_max_velocity)), 0.03, self.max_vel_xy
        )
        self.waypoint_xy_tolerance = max(0.05, float(self.waypoint_xy_tolerance))
        self.waypoint_timeout_sec = max(1.0, float(self.waypoint_timeout_sec))
        self.return_hover_time = max(0.2, float(self.return_hover_time))
        self.low_hover_time = max(0.2, float(self.low_hover_time))
        self.target_hover_time = max(0.2, float(self.target_hover_time))

        self.phase = Phase.INIT
        self.previous_flight_phase = Phase.INIT
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.origin_z = 0.0
        self.origin_yaw = None
        self.origin_range = None
        self.origin_recorded = False

        self.hover_start_time = None
        self.grasp_start_time = None
        self.drop_start_time = None
        self.bench_start_time = None
        self._pending_command = False
        self._command_retry_count = 0
        self._arm_mode_set = False
        self._desktop_disarm_done = False
        self._last_velocity = (0.0, 0.0, 0.0)
        self._last_velocity_time = self.get_clock().now()
        self._recovery_start_time = None
        self._last_preflight_event = ""
        self._last_preflight_event_time = self.get_clock().now()
        self._pending_mode_target = None
        self._pending_mode_next_phase = None
        self._pending_mode_future = None
        self._pending_mode_started = None
        self._pending_mode_request_sent = False
        self._pending_mode_warn_only = False
        self._pending_mav_frame = None
        self._pending_mav_frame_next_phase = None
        self._pending_mav_frame_future = None
        self._pending_mav_frame_started = None
        self._pending_mav_frame_request_sent = False

        self._data_lock = threading.Lock()
        self._last_uwb = None
        self._last_fcu_state = None
        self._last_mavros_state = None
        self._last_local_pose = None
        self._last_rangefinder = None
        self._last_optical_flow = None
        self._last_link_status = "OK"
        self._has_uwb = False
        self._has_fcu = False
        self._has_mavros_state = False
        self._has_pose = False
        self._has_rangefinder = False
        self._has_optical_flow = False
        self._last_uwb_time = self.get_clock().now()
        self._last_mavros_state_time = self.get_clock().now()
        self._last_pose_time = self.get_clock().now()
        self._last_rangefinder_time = self.get_clock().now()
        self._last_optical_flow_time = self.get_clock().now()
        self._grasp_done = False
        self._drop_done = False
        self._bench_arm_ok = False
        self._bench_velocity_started = False
        self._bench_velocity_done = False
        self._bench_disarm_ok = False
        self._bench_guided_ok = None
        self._bench_result_reported = False
        self._takeoff_land_takeoff_ok = False
        self._takeoff_land_hover_ok = False
        self._takeoff_land_land_ok = False
        self._takeoff_land_loiter_ok = False
        self._takeoff_land_guided_return_ok = False
        self._takeoff_land_forward_ok = False
        self._uwb_approach_ok = False
        self._takeoff_land_result_reported = False
        self._takeoff_land_abort_reason = None
        self._takeoff_wait_start_time = None
        self._takeoff_delay_start_time = None
        self._takeoff_climb_start_time = None
        self._loiter_alt_loss_start_time = None
        self._move_above_start_time = None
        self._uwb_missing_start_time = None
        self._uwb_out_of_front_start_time = None
        self._uwb_capture_start_time = None
        self._uwb_target_captured = False
        self._forward_start_time = None
        self._forward_start_xy = None
        self._waypoint_start_time = None
        self._waypoint_target_xy = None
        self._waypoint_return_start_time = None
        self._land_wait_start_time = None
        self._land_ground_start_time = None
        self._land_disarm_sent = False
        self._land_disarm_retry_count = 0
        self._land_disarm_retry_time = None
        self._shutdown_requested = False

        cb_group = ReentrantCallbackGroup()

        self.uwb_sub = self.create_subscription(UwbAoa, self.uwb_aoa_topic, self._uwb_callback, 10)
        self.fcu_sub = self.create_subscription(FcuState, self.fcu_state_topic, self._fcu_callback, 10)
        self.mavros_state_sub = self.create_subscription(
            MavrosState, self.mavros_state_topic, self._mavros_state_callback, 10
        )
        self.pose_sub = self.create_subscription(
            PoseStamped, self.local_pose_topic, self._pose_callback, SENSOR_QOS
        )
        self.rangefinder_sub = self.create_subscription(
            Range, self.rangefinder_topic, self._rangefinder_callback, SENSOR_QOS
        )
        self.optical_flow_sub = self.create_subscription(
            OpticalFlow, self.optical_flow_topic, self._optical_flow_callback, SENSOR_QOS
        )
        self.link_sub = self.create_subscription(String, self.fcu_link_topic, self._link_callback, 10)
        self.grasp_sub = self.create_subscription(String, self.grasp_done_topic, self._grasp_callback, 10)
        self.drop_sub = self.create_subscription(String, self.drop_done_topic, self._drop_callback, 10)

        self.vel_pub = self.create_publisher(TwistStamped, self.cmd_vel_topic, 20)
        self.state_pub = self.create_publisher(String, self.mission_state_topic, 10)
        self.event_pub = self.create_publisher(String, self.mission_event_topic, 10)
        self.flight_reset_pub = self.create_publisher(String, "uav_bridge/flight_reset", 10)

        self.flight_cmd_client = self.create_client(
            FlightCommand, self.flight_command_service, callback_group=cb_group
        )
        self.mavros_set_mode_client = self.create_client(
            SetMode, "/mavros/set_mode", callback_group=cb_group
        )
        self.mavros_setpoint_velocity_param_client = self.create_client(
            SetParameters,
            "/mavros/setpoint_velocity/set_parameters",
            callback_group=cb_group,
        )

        ctrl_period = 1.0 / max(1.0, self.control_rate_hz)
        self.ctrl_timer = self.create_timer(ctrl_period, self._control_loop)

        self.get_logger().info(
            f"test_mission_node started: mode={self.mission_mode} "
            f"mock={str(self.use_mock).lower()} "
            f"takeoff={self.takeoff_altitude:.1f}m descend={self.descend_altitude:.1f}m "
            f"takeoff_method={self.takeoff_method} "
            f"takeoff_delay={self.takeoff_command_delay_sec:.1f}s"
        )

    def _parse_modes(self, modes_text):
        return {mode.strip().upper() for mode in modes_text.split(",") if mode.strip()}

    def _uwb_callback(self, msg: UwbAoa):
        with self._data_lock:
            self._last_uwb = msg
            self._has_uwb = True
            self._last_uwb_time = self.get_clock().now()

    def _fcu_callback(self, msg: FcuState):
        with self._data_lock:
            self._last_fcu_state = msg
            self._has_fcu = True

    def _mavros_state_callback(self, msg: MavrosState):
        with self._data_lock:
            self._last_mavros_state = msg
            self._has_mavros_state = True
            self._last_mavros_state_time = self.get_clock().now()

    def _pose_callback(self, msg: PoseStamped):
        with self._data_lock:
            self._last_local_pose = msg
            self._has_pose = True
            self._last_pose_time = self.get_clock().now()

    def _rangefinder_callback(self, msg: Range):
        with self._data_lock:
            self._last_rangefinder = msg
            self._has_rangefinder = True
            self._last_rangefinder_time = self.get_clock().now()

    def _optical_flow_callback(self, msg: OpticalFlow):
        with self._data_lock:
            self._last_optical_flow = msg
            self._has_optical_flow = True
            self._last_optical_flow_time = self.get_clock().now()

    def _link_callback(self, msg: String):
        with self._data_lock:
            self._last_link_status = msg.data

    def _grasp_callback(self, msg: String):
        if string_is_done(msg.data):
            self._grasp_done = True

    def _drop_callback(self, msg: String):
        if string_is_done(msg.data):
            self._drop_done = True

    def _control_loop(self):
        if self._check_critical():
            self._publish_state()
            return

        if self._pending_mode_target is not None:
            self._tick_pending_mode_switch()
            self._publish_state()
            return

        if self._pending_mav_frame is not None:
            self._tick_pending_mav_frame_switch()
            self._publish_state()
            return

        tick_map = {
            Phase.INIT: self._tick_init,
            Phase.ARM: self._tick_arm,
            Phase.BENCH_VELOCITY: self._tick_bench_velocity,
            Phase.TAKEOFF: self._tick_takeoff,
            Phase.HOVER_TAKEOFF: self._tick_hover_takeoff,
            Phase.MOVE_ABOVE: self._tick_move_above,
            Phase.HOVER_ABOVE: self._tick_hover_above,
            Phase.DESCEND: self._tick_descend,
            Phase.HOVER_FINAL: self._tick_hover_final,
            Phase.WAIT_GRASP: self._tick_wait_grasp,
            Phase.CLIMB: self._tick_climb,
            Phase.HOVER_CLIMB: self._tick_hover_climb,
            Phase.RETURN: self._tick_return,
            Phase.HOVER_RETURN: self._tick_hover_return,
            Phase.WAIT_DROP: self._tick_wait_drop,
            Phase.LAND: self._tick_land,
            Phase.LAND_WAIT: self._tick_land_wait,
            Phase.DONE: self._tick_done,
            Phase.PAUSED_MANUAL: self._tick_paused_manual,
            Phase.RECOVERING: self._tick_recovering,
            Phase.FAILSAFE: self._tick_failsafe,
            Phase.HOVER_LOITER: self._tick_hover_loiter,
            Phase.FORWARD: self._tick_forward,
            Phase.HOVER_FORWARD: self._tick_hover_forward,
            Phase.WAYPOINT_OUTBOUND: self._tick_waypoint_outbound,
            Phase.HOVER_WAYPOINT: self._tick_hover_waypoint,
            Phase.WAYPOINT_RETURN: self._tick_waypoint_return,
            Phase.HOVER_RETURN_HOME: self._tick_hover_return_home,
            Phase.WAYPOINT_DESCEND: self._tick_waypoint_descend,
            Phase.HOVER_WAYPOINT_LOW: self._tick_hover_waypoint_low,
            Phase.WAYPOINT_RECLIMB: self._tick_waypoint_reclimb,
            Phase.HOVER_WAYPOINT_RECLIMB: self._tick_hover_waypoint_reclimb,
        }
        tick_map[self.phase]()
        self._publish_state()

    def _check_critical(self) -> bool:
        if self.phase in (Phase.INIT, Phase.ARM, Phase.DONE, Phase.LAND, Phase.FAILSAFE):
            return False

        link = self._last_link_status
        if link == "LOST" and self.phase != Phase.BENCH_VELOCITY:
            if self.phase != Phase.RECOVERING:
                self._publish_event("link_recovering")
                self._transition(Phase.RECOVERING)
                return True
            return False

        if self.use_mock:
            return False

        fcu = self._last_fcu_state
        if not fcu or not fcu.connected:
            self._publish_event("failsafe_fcu_disconnected")
            self._transition(Phase.FAILSAFE)
            return True

        if fcu.voltage > 0.1 and fcu.remaining < self.low_battery_pct / 100.0:
            self._publish_event("failsafe_low_battery")
            self._transition(Phase.FAILSAFE)
            return True

        if self.phase not in (Phase.PAUSED_MANUAL, Phase.BENCH_VELOCITY):
            mode = (fcu.mode or "").upper()
            if mode and mode not in self._allowed_auto_modes():
                self.previous_flight_phase = self.phase
                self._publish_velocity(0.0, 0.0, 0.0, immediate=True)
                self._publish_event(f"manual_takeover:{mode}")
                self._transition(Phase.PAUSED_MANUAL)
                return True

        radius = self._mission_xy_distance_from_origin()
        if (
            self._is_uwb_approach_land_mode()
            and radius is not None
            and radius >= self.mission_hard_radius_m
            and self.phase not in (Phase.LAND_WAIT, Phase.PAUSED_MANUAL)
        ):
            reason = (
                f"Mission hard radius exceeded: r={radius:.2f}/"
                f"{self.mission_hard_radius_m:.2f}m"
            )
            self._publish_velocity(0.0, 0.0, 0.0, immediate=True)
            self._publish_event("failsafe_mission_radius")
            self.get_logger().error(reason)
            self._takeoff_land_abort_reason = reason
            self._transition(Phase.FAILSAFE)
            return True

        return False

    def _allowed_auto_modes(self):
        modes = set(self.auto_modes)
        if self._is_takeoff_loiter_land_mode():
            if self.phase == Phase.HOVER_LOITER or self._pending_mode_target == "LOITER":
                modes.add("LOITER")
            else:
                modes.discard("LOITER")
        if self.phase == Phase.LAND_WAIT:
            modes.add("LAND")
        return modes

    def _uwb_valid_and_fresh(self) -> bool:
        if self.use_mock:
            return True
        with self._data_lock:
            if not self._has_uwb or self._last_uwb is None:
                return False
            if not self._last_uwb.signal_valid:
                return False
            age = (self.get_clock().now() - self._last_uwb_time).nanoseconds / 1e9
            return age <= self.uwb_signal_timeout

    def _local_pose_ready(self) -> bool:
        if self.use_mock:
            return True
        with self._data_lock:
            if not self._has_pose or self._last_local_pose is None:
                return False
            age = (self.get_clock().now() - self._last_pose_time).nanoseconds / 1e9
            return age <= self.local_pose_timeout

    def _message_fresh(self, stamp_time, timeout_sec=None) -> bool:
        timeout = self.bench_sensor_timeout if timeout_sec is None else timeout_sec
        age = (self.get_clock().now() - stamp_time).nanoseconds / 1e9
        return age <= timeout

    def _mavros_state_ready(self) -> bool:
        with self._data_lock:
            return (
                self._has_mavros_state
                and self._last_mavros_state is not None
                and self._message_fresh(self._last_mavros_state_time)
            )

    def _rangefinder_ready(self) -> bool:
        with self._data_lock:
            if not self._has_rangefinder or self._last_rangefinder is None:
                return False
            msg = self._last_rangefinder
            if not math.isfinite(msg.range):
                return False
            if msg.range < msg.min_range or msg.range > msg.max_range:
                return False
            return self._message_fresh(self._last_rangefinder_time)

    def _optical_flow_ready(self) -> bool:
        with self._data_lock:
            if not self._has_optical_flow or self._last_optical_flow is None:
                return False
            return self._last_optical_flow.quality > 0 and self._message_fresh(
                self._last_optical_flow_time
            )

    def _fcu_connected(self) -> bool:
        if self.use_mock:
            return True
        with self._data_lock:
            return self._has_fcu and self._last_fcu_state and self._last_fcu_state.connected

    def _get_uwb(self):
        with self._data_lock:
            uwb = self._last_uwb
            if uwb is None:
                return None
            return (uwb.azimuth_deg, uwb.distance_m, uwb.elevation_deg)

    def _get_fcu_altitude(self) -> float:
        with self._data_lock:
            if self._last_fcu_state is None:
                return 0.0
            return abs(self._last_fcu_state.local_z)

    def _get_local_xy(self):
        with self._data_lock:
            if self._last_local_pose is None:
                return None
            p = self._last_local_pose.pose.position
            return (p.x, p.y)

    def _mission_xy_distance_from_origin(self):
        if not self.origin_recorded:
            return None
        pos = self._get_local_xy()
        if pos is None:
            return None
        dx = pos[0] - self.origin_x
        dy = pos[1] - self.origin_y
        return math.sqrt(dx * dx + dy * dy)

    def _get_local_yaw(self):
        with self._data_lock:
            if self._last_local_pose is None:
                return None
            q = self._last_local_pose.pose.orientation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            return math.atan2(siny_cosp, cosy_cosp)

    def _get_local_z(self):
        with self._data_lock:
            if self._last_local_pose is None:
                return None
            return self._last_local_pose.pose.position.z

    def _get_rangefinder_m(self):
        with self._data_lock:
            if self._last_rangefinder is None:
                return None
            value = self._last_rangefinder.range
            if not math.isfinite(value):
                return None
            return value

    def _get_takeoff_land_relative_altitude(self):
        range_m = self._get_rangefinder_m()
        if range_m is not None and self.origin_range is not None:
            return max(0.0, range_m - self.origin_range), "rangefinder_rel"

        local_z = self._get_local_z()
        if local_z is not None:
            return abs(local_z - self.origin_z), "local_z_rel"

        return None, "missing"

    def _get_fcu_mode(self) -> str:
        with self._data_lock:
            if self._last_fcu_state is None:
                return ""
            return self._last_fcu_state.mode or ""

    def _armed_confirmed(self) -> bool:
        with self._data_lock:
            fcu_armed = bool(self._last_fcu_state and self._last_fcu_state.armed)
            mavros_armed = bool(self._last_mavros_state and self._last_mavros_state.armed)
            return fcu_armed or mavros_armed

    def _bench_snapshot(self):
        with self._data_lock:
            fcu = self._last_fcu_state
            mavros_state = self._last_mavros_state
            uwb = self._last_uwb
            rangefinder = self._last_rangefinder
            flow = self._last_optical_flow

        fcu_ok = self._fcu_connected()
        mavros_ok = self._mavros_state_ready()
        rc_ok = bool(mavros_ok and mavros_state.manual_input)
        uwb_ok = self._uwb_valid_and_fresh()
        pose_ok = self._local_pose_ready()
        range_ok = self._rangefinder_ready()
        flow_ok = self._optical_flow_ready()
        set_mode_ok = self.mavros_set_mode_client.service_is_ready()

        warnings = []
        if not rc_ok:
            warnings.append("RC/manual_input not confirmed")
        if not uwb_ok:
            warnings.append("UWB not fresh")
        if not range_ok:
            warnings.append("rangefinder not fresh")
        if not flow_ok:
            warnings.append("optical_flow not fresh")
        if not pose_ok:
            warnings.append("local_position missing: real_full not ready")
        if not set_mode_ok:
            warnings.append("MAVROS set_mode service not ready")
        if self._bench_guided_ok is False:
            warnings.append("GUIDED mode switch rejected/unavailable")

        return {
            "fcu_ok": fcu_ok,
            "fcu_mode": (fcu.mode if fcu else ""),
            "fcu_armed": bool(fcu.armed) if fcu else False,
            "estimator_ok": bool(fcu.estimator_ok) if fcu else False,
            "attitude_ok": bool(fcu.attitude_status) if fcu else False,
            "vel_horiz_ok": bool(fcu.vel_horiz_status) if fcu else False,
            "vel_vert_ok": bool(fcu.vel_vert_status) if fcu else False,
            "pos_horiz_ok": bool(fcu.pos_horiz_status) if fcu else False,
            "mavros_ok": mavros_ok,
            "rc_ok": rc_ok,
            "uwb_ok": uwb_ok,
            "uwb_distance": uwb.distance_m if uwb else None,
            "uwb_azimuth": uwb.azimuth_deg if uwb else None,
            "pose_ok": pose_ok,
            "range_ok": range_ok,
            "range_m": rangefinder.range if rangefinder else None,
            "flow_ok": flow_ok,
            "flow_quality": flow.quality if flow else None,
            "set_mode_ok": set_mode_ok,
            "warnings": warnings,
        }

    def _format_bench_snapshot(self, snapshot):
        def status(ok):
            return "OK" if ok else "WAIT"

        uwb_text = status(snapshot["uwb_ok"])
        if snapshot["uwb_ok"] and snapshot["uwb_distance"] is not None:
            uwb_text += f" d={snapshot['uwb_distance']:.2f}m az={snapshot['uwb_azimuth']:.1f}deg"

        range_text = status(snapshot["range_ok"])
        if snapshot["range_ok"] and snapshot["range_m"] is not None:
            range_text += f" {snapshot['range_m']:.2f}m"

        flow_text = status(snapshot["flow_ok"])
        if snapshot["flow_quality"] is not None:
            flow_text += f" q={snapshot['flow_quality']}"

        return (
            f"FCU={status(snapshot['fcu_ok'])} "
            f"mode={snapshot['fcu_mode'] or '-'} armed={str(snapshot['fcu_armed']).lower()} "
            f"RC={status(snapshot['rc_ok'])} "
            f"UWB={uwb_text} "
            f"rangefinder={range_text} "
            f"optical_flow={flow_text} "
            f"local_pose={status(snapshot['pose_ok'])} "
            f"set_mode_srv={status(snapshot['set_mode_ok'])}"
        )

    def _format_real_preflight_snapshot(self, snapshot):
        def status(ok):
            return "OK" if ok else "WAIT"

        estimator_text = (
            f"estimator={status(snapshot['estimator_ok'])} "
            f"att={status(snapshot['attitude_ok'])} "
            f"vel_h={status(snapshot['vel_horiz_ok'])} "
            f"vel_v={status(snapshot['vel_vert_ok'])} "
            f"pos_abs={status(snapshot['pos_horiz_ok'])}"
        )
        return f"{self._format_bench_snapshot(snapshot)} {estimator_text}"

    def _report_bench_result(self, force_fail_reason=None):
        if self._bench_result_reported:
            return
        self._bench_result_reported = True

        snapshot = self._bench_snapshot()
        core_ok = (
            self._bench_arm_ok
            and self._bench_velocity_started
            and self._bench_velocity_done
            and self._bench_disarm_ok
        )
        warnings = list(snapshot["warnings"])
        if self._bench_guided_ok is None:
            warnings.append("GUIDED mode switch result unknown")
        if force_fail_reason:
            warnings.append(force_fail_reason)

        if not core_ok or force_fail_reason:
            result = "FAIL"
        elif warnings:
            result = "PASS_WITH_WARNINGS"
        else:
            result = "PASS"

        self.get_logger().info("========== BENCH RESULT ==========")
        self.get_logger().info(f"BENCH RESULT: {result}")
        self.get_logger().info(
            "Core links: "
            f"ARM={'OK' if self._bench_arm_ok else 'FAIL'} "
            f"velocity_profile={'OK' if self._bench_velocity_done else 'FAIL'} "
            f"DISARM={'OK' if self._bench_disarm_ok else 'FAIL'}"
        )
        self.get_logger().info(f"Sensor links: {self._format_bench_snapshot(snapshot)}")
        if warnings:
            self.get_logger().warn("Bench warnings: " + "; ".join(warnings))
        if result == "PASS":
            self.get_logger().info("Bench next step: desktop bench is clean; real_full still needs field safety checks.")
        elif result == "PASS_WITH_WARNINGS":
            self.get_logger().warn(
                "Bench next step: core desktop test passed, but resolve warnings before real_full."
            )
        else:
            self.get_logger().error("Bench next step: stop and fix core links before any further flight test.")
        self.get_logger().info("==================================")
        if self.mission_mode == "bench_velocity" and self.bench_exit_on_complete:
            self._shutdown_requested = True

    def _is_takeoff_land_mode(self) -> bool:
        return self.mission_mode in (
            "takeoff_hover_land",
            "takeoff_loiter_land",
            "takeoff_forward_land",
            "takeoff_waypoint_return_land",
            "uwb_approach_land",
        )

    def _is_takeoff_loiter_land_mode(self) -> bool:
        return self.mission_mode == "takeoff_loiter_land"

    def _is_takeoff_forward_land_mode(self) -> bool:
        return self.mission_mode == "takeoff_forward_land"

    def _is_takeoff_waypoint_return_land_mode(self) -> bool:
        return self.mission_mode == "takeoff_waypoint_return_land"

    def _is_uwb_approach_land_mode(self) -> bool:
        return self.mission_mode == "uwb_approach_land"

    def _takeoff_land_label(self) -> str:
        if self._is_takeoff_loiter_land_mode():
            return "TAKEOFF_LOITER_LAND"
        if self._is_takeoff_forward_land_mode():
            return "TAKEOFF_FORWARD_LAND"
        if self._is_takeoff_waypoint_return_land_mode():
            return "TAKEOFF_WAYPOINT_RETURN_LAND"
        if self._is_uwb_approach_land_mode():
            return "UWB_APPROACH_LAND"
        return "TAKEOFF_LAND"

    def _takeoff_land_text(self) -> str:
        if self._is_takeoff_loiter_land_mode():
            return "Takeoff-loiter-land"
        if self._is_takeoff_forward_land_mode():
            return "Takeoff-forward-land"
        if self._is_takeoff_waypoint_return_land_mode():
            return "Takeoff-waypoint-return-land"
        if self._is_uwb_approach_land_mode():
            return "UWB approach-land"
        return "Takeoff-land"

    def _report_takeoff_land_result(self, force_fail_reason=None):
        if self._takeoff_land_result_reported:
            return
        self._takeoff_land_result_reported = True

        snapshot = self._bench_snapshot()
        core_ok = (
            self._bench_arm_ok
            and self._takeoff_land_takeoff_ok
            and self._takeoff_land_hover_ok
            and self._takeoff_land_land_ok
        )
        if self._is_takeoff_loiter_land_mode():
            core_ok = (
                core_ok
                and self._takeoff_land_loiter_ok
                and self._takeoff_land_guided_return_ok
            )
        if self._is_takeoff_forward_land_mode():
            core_ok = core_ok and self._takeoff_land_forward_ok
        if self._is_takeoff_waypoint_return_land_mode():
            core_ok = core_ok and self._takeoff_land_forward_ok and self._takeoff_land_guided_return_ok
        if self._is_uwb_approach_land_mode():
            core_ok = core_ok and self._uwb_approach_ok
        sensor_ok = (
            snapshot["fcu_ok"]
            and snapshot["rc_ok"]
            and ((not self._is_uwb_approach_land_mode()) or snapshot["uwb_ok"])
            and snapshot["pose_ok"]
            and snapshot["range_ok"]
            and snapshot["flow_ok"]
            and snapshot["set_mode_ok"]
        )
        warnings = []
        if not sensor_ok:
            warnings.append("required sensor link not OK")
        if force_fail_reason:
            warnings.append(force_fail_reason)

        result = "PASS" if core_ok and sensor_ok and not force_fail_reason else "FAIL"
        label = self._takeoff_land_label()
        text = self._takeoff_land_text()
        self.get_logger().info(f"========== {label} RESULT ==========")
        self.get_logger().info(f"{label} RESULT: {result}")
        if self._is_takeoff_loiter_land_mode():
            core_text = (
                "Core links: "
                f"ARM={'OK' if self._bench_arm_ok else 'FAIL'} "
                f"TAKEOFF={'OK' if self._takeoff_land_takeoff_ok else 'FAIL'} "
                f"LOITER={'OK' if self._takeoff_land_loiter_ok else 'FAIL'} "
                f"HOVER={'OK' if self._takeoff_land_hover_ok else 'FAIL'} "
                f"GUIDED_BACK={'OK' if self._takeoff_land_guided_return_ok else 'FAIL'} "
                f"LAND={'OK' if self._takeoff_land_land_ok else 'FAIL'}"
            )
        elif self._is_uwb_approach_land_mode():
            core_text = (
                "Core links: "
                f"ARM={'OK' if self._bench_arm_ok else 'FAIL'} "
                f"TAKEOFF={'OK' if self._takeoff_land_takeoff_ok else 'FAIL'} "
                f"HOVER={'OK' if self._takeoff_land_hover_ok else 'FAIL'} "
                f"UWB_APPROACH={'OK' if self._uwb_approach_ok else 'FAIL'} "
                f"LAND={'OK' if self._takeoff_land_land_ok else 'FAIL'}"
            )
        elif self._is_takeoff_forward_land_mode():
            core_text = (
                "Core links: "
                f"ARM={'OK' if self._bench_arm_ok else 'FAIL'} "
                f"TAKEOFF={'OK' if self._takeoff_land_takeoff_ok else 'FAIL'} "
                f"HOVER={'OK' if self._takeoff_land_hover_ok else 'FAIL'} "
                f"FORWARD={'OK' if self._takeoff_land_forward_ok else 'FAIL'} "
                f"LAND={'OK' if self._takeoff_land_land_ok else 'FAIL'}"
            )
        elif self._is_takeoff_waypoint_return_land_mode():
            core_text = (
                "Core links: "
                f"ARM={'OK' if self._bench_arm_ok else 'FAIL'} "
                f"TAKEOFF={'OK' if self._takeoff_land_takeoff_ok else 'FAIL'} "
                f"HOVER={'OK' if self._takeoff_land_hover_ok else 'FAIL'} "
                f"WAYPOINT={'OK' if self._takeoff_land_forward_ok else 'FAIL'} "
                f"RETURN={'OK' if self._takeoff_land_guided_return_ok else 'FAIL'} "
                f"LAND={'OK' if self._takeoff_land_land_ok else 'FAIL'}"
            )
        else:
            core_text = (
                "Core links: "
                f"ARM={'OK' if self._bench_arm_ok else 'FAIL'} "
                f"TAKEOFF={'OK' if self._takeoff_land_takeoff_ok else 'FAIL'} "
                f"HOVER={'OK' if self._takeoff_land_hover_ok else 'FAIL'} "
                f"LAND={'OK' if self._takeoff_land_land_ok else 'FAIL'}"
            )
        self.get_logger().info(core_text)
        self.get_logger().info(f"Sensor links: {self._format_bench_snapshot(snapshot)}")
        if warnings:
            self.get_logger().warn(f"{text} warnings: " + "; ".join(warnings))
        self.get_logger().info("=========================================")
        self._shutdown_requested = True

    def should_exit(self):
        return self._shutdown_requested

    def _tick_init(self):
        fcu_ok = self._fcu_connected()
        uwb_ready = self._uwb_valid_and_fresh()
        pose_ready = self._local_pose_ready()
        uwb_ok = (not self.require_uwb_ready) or uwb_ready
        pose_ok = (not self.require_local_pose_ready) or pose_ready

        if self.mission_mode == "bench_velocity":
            snapshot = self._bench_snapshot()
            self.get_logger().info(
                "Bench preflight: " + self._format_bench_snapshot(snapshot),
                throttle_duration_sec=5.0,
            )
        elif self._is_takeoff_land_mode():
            snapshot = self._bench_snapshot()
            required_ok = (
                snapshot["fcu_ok"]
                and snapshot["rc_ok"]
                and ((not self._is_uwb_approach_land_mode()) or snapshot["uwb_ok"])
                and snapshot["pose_ok"]
                and snapshot["range_ok"]
                and snapshot["flow_ok"]
                and snapshot["set_mode_ok"]
            )
            self.get_logger().info(
                f"{self._takeoff_land_text()} preflight: "
                + self._format_bench_snapshot(snapshot),
                throttle_duration_sec=5.0,
            )
            if not required_ok:
                wait_event = self._takeoff_land_preflight_wait_event(snapshot)
                if wait_event:
                    self._publish_preflight_event(wait_event)
                if not snapshot["rc_ok"]:
                    self.get_logger().warn("Waiting for RC/manual_input...", throttle_duration_sec=5.0)
                if self._is_uwb_approach_land_mode() and not snapshot["uwb_ok"]:
                    self.get_logger().warn("Waiting for UWB...", throttle_duration_sec=5.0)
                if not snapshot["pose_ok"]:
                    self.get_logger().warn("Waiting for local pose...", throttle_duration_sec=5.0)
                if not snapshot["range_ok"]:
                    self.get_logger().warn("Waiting for rangefinder...", throttle_duration_sec=5.0)
                if not snapshot["flow_ok"]:
                    self.get_logger().warn("Waiting for optical_flow...", throttle_duration_sec=5.0)
                if not snapshot["set_mode_ok"]:
                    self.get_logger().warn("Waiting for MAVROS set_mode service...", throttle_duration_sec=5.0)
                return
        elif self.mission_mode == "real_full":
            snapshot = self._bench_snapshot()
            self.get_logger().info(
                "Real preflight: " + self._format_real_preflight_snapshot(snapshot),
                throttle_duration_sec=5.0,
            )

        if fcu_ok and uwb_ok and pose_ok:
            self._publish_preflight_event("preflight_ready")
            self._record_origin()
            self.get_logger().info("Preflight links ready, starting mission")
            self._transition(Phase.ARM)
            return

        wait_event = self._core_preflight_wait_event(fcu_ok, uwb_ok, pose_ok)
        if wait_event:
            self._publish_preflight_event(wait_event)

        if not fcu_ok:
            self.get_logger().warn("Waiting for FCU...", throttle_duration_sec=5.0)
        if not uwb_ok:
            self.get_logger().warn("Waiting for UWB...", throttle_duration_sec=5.0)
        if not pose_ok:
            if self.mission_mode == "real_full":
                self.get_logger().warn(
                    "Waiting for local pose: check optical-flow/rangefinder EKF fusion and EKF origin before real_full.",
                    throttle_duration_sec=5.0,
                )
            else:
                self.get_logger().warn("Waiting for local pose...", throttle_duration_sec=5.0)

    def _tick_arm(self):
        if self._pending_command:
            return

        if not self._arm_mode_set:
            cur_mode = self._get_fcu_mode()
            if cur_mode == "LAND":
                self.get_logger().info(f"FCU in LAND, switching to {self.arm_mode} before ARM")
                self._call_mavros_set_mode(self.arm_mode)
                self._arm_mode_set = True
                return
            self._arm_mode_set = True

        if self._command_retry_count > 20:
            self.get_logger().error("ARM retry limit exceeded")
            if self.mission_mode == "bench_velocity":
                self._report_bench_result("ARM retry limit exceeded")
            self._transition(Phase.FAILSAFE)
            return

        self._pending_command = True
        self._call_flight_cmd(FlightCommand.Request.CMD_ARM, 0.0, self._on_arm_result)

    def _on_arm_result(self, success: bool, msg: str):
        self._pending_command = False
        if not success:
            self._command_retry_count += 1
            self.get_logger().warn(f"Arm failed ({self._command_retry_count}): {msg}")
            return

        self.get_logger().info(f"Arm OK: {msg}")
        self._publish_event("armed")
        self._command_retry_count = 0
        if self.mission_mode == "bench_velocity" or self._is_takeoff_land_mode():
            self._bench_arm_ok = True

        if self.mission_mode == "bench_velocity":
            self._start_mode_switch("GUIDED", Phase.BENCH_VELOCITY, warn_only=False)
            return

        self._start_mode_switch("GUIDED", Phase.TAKEOFF, warn_only=False)

    def _tick_bench_velocity(self):
        if self.bench_start_time is None:
            self.bench_start_time = self.get_clock().now()

        elapsed = (self.get_clock().now() - self.bench_start_time).nanoseconds / 1e9
        climb_end = self.bench_climb_sec
        hold_end = climb_end + self.bench_hold_sec
        descend_end = hold_end + self.bench_descend_sec
        zero_end = descend_end + self.bench_zero_sec

        if elapsed < climb_end:
            vz = self.bench_velocity_z
            label = "bench_climb"
        elif elapsed < hold_end:
            vz = 0.0
            label = "bench_hold"
        elif elapsed < descend_end:
            vz = -self.bench_velocity_z
            label = "bench_descend"
        elif elapsed < zero_end:
            vz = 0.0
            label = "bench_zero"
        else:
            self._publish_velocity(0.0, 0.0, 0.0)
            self._bench_velocity_done = True
            if not self._desktop_disarm_done and not self._pending_command:
                self._pending_command = True
                self.get_logger().info("Bench velocity profile complete, disarming")
                self._call_flight_cmd(FlightCommand.Request.CMD_DISARM, 0.0, self._on_bench_disarm)
            return

        self.get_logger().info(f"{label}: vz={vz:.2f} m/s", throttle_duration_sec=1.0)
        self._publish_velocity(0.0, 0.0, vz)
        self._bench_velocity_started = True

    def _on_bench_disarm(self, success: bool, msg: str):
        self._pending_command = False
        self._desktop_disarm_done = True
        self._bench_disarm_ok = success
        self.get_logger().info(f"Bench disarm: {'OK' if success else 'FAIL'} - {msg}")
        self._publish_event("bench_velocity_done")
        self._transition(Phase.DONE)
        self._report_bench_result(None if success else "DISARM failed")

    def _tick_takeoff(self):
        if self._pending_command:
            return

        if not self._armed_confirmed():
            now = self.get_clock().now()
            if self._takeoff_wait_start_time is None:
                self._takeoff_wait_start_time = now
                self.get_logger().info("Waiting for armed state before TAKEOFF...")
                return
            elapsed = (now - self._takeoff_wait_start_time).nanoseconds / 1e9
            if elapsed < 5.0:
                self.get_logger().info(
                    f"Waiting for armed state before TAKEOFF ({elapsed:.1f}s)...",
                    throttle_duration_sec=1.0,
                )
                return
            self.get_logger().error("TAKEOFF blocked: armed state was not confirmed after ARM")
            if self._is_takeoff_land_mode():
                self._report_takeoff_land_result("ARM state not confirmed before TAKEOFF")
            self._transition(Phase.FAILSAFE)
            return

        now = self.get_clock().now()
        self._takeoff_wait_start_time = None
        if self.takeoff_command_delay_sec > 0.0:
            if self._takeoff_delay_start_time is None:
                self._takeoff_delay_start_time = now
                self.get_logger().info(
                    f"Waiting {self.takeoff_command_delay_sec:.1f}s before TAKEOFF command..."
                )
                return
            delay_elapsed = (now - self._takeoff_delay_start_time).nanoseconds / 1e9
            if delay_elapsed < self.takeoff_command_delay_sec:
                self.get_logger().info(
                    f"Waiting before TAKEOFF command ({delay_elapsed:.1f}/"
                    f"{self.takeoff_command_delay_sec:.1f}s)...",
                    throttle_duration_sec=1.0,
                )
                return

        self._takeoff_delay_start_time = None
        self._record_origin()
        if self._is_takeoff_land_mode() and self.takeoff_method == "velocity":
            self._takeoff_climb_start_time = now
            self.get_logger().info(
                f"Starting velocity takeoff: vz={self.takeoff_climb_velocity:.2f}m/s "
                f"target_rel_alt={self.takeoff_altitude:.2f}m"
            )
            self._publish_event("takeoff_velocity_started")
            self._transition(Phase.HOVER_TAKEOFF)
            return

        self._pending_command = True
        self._call_flight_cmd(
            FlightCommand.Request.CMD_TAKEOFF,
            float(self.takeoff_altitude),
            self._on_takeoff_result,
        )

    def _on_takeoff_result(self, success: bool, msg: str):
        self._pending_command = False
        if success:
            self.get_logger().info(f"Takeoff OK: {msg}")
            self._takeoff_land_takeoff_ok = True
            self._takeoff_climb_start_time = self.get_clock().now()
            self._publish_event("takeoff_accepted")
            self._transition(Phase.HOVER_TAKEOFF)
        else:
            self.get_logger().error(f"Takeoff FAILED: {msg}")
            if self._is_takeoff_land_mode():
                self._takeoff_land_abort_reason = "TAKEOFF failed"
            self._transition(Phase.FAILSAFE)

    def _tick_hover_takeoff(self):
        if self.use_mock:
            self._publish_velocity(0.0, 0.0, 0.0)
            self._check_stable_and_transition(
                Phase.MOVE_ABOVE, "Mock takeoff hover stable", "hover_takeoff_stable"
            )
            return

        if self._is_takeoff_land_mode():
            rel_alt, source = self._get_takeoff_land_relative_altitude()
            now = self.get_clock().now()
            if rel_alt is None:
                self.hover_start_time = None
                if self.takeoff_method == "velocity":
                    self._publish_velocity(0.0, 0.0, 0.0)
                self.get_logger().warn(
                    "Waiting for relative takeoff altitude source...",
                    throttle_duration_sec=1.0,
                )
                return

            target_reached = rel_alt >= max(0.0, self.takeoff_altitude - self.altitude_tolerance)
            if target_reached:
                self._publish_velocity(0.0, 0.0, 0.0)
                self._takeoff_land_takeoff_ok = True
                self._takeoff_climb_start_time = None
                if self._is_takeoff_loiter_land_mode():
                    if self.hover_start_time is None:
                        self.hover_start_time = now
                        self.get_logger().info(
                            f"Takeoff height reached at rel_alt={rel_alt:.2f}m ({source}), "
                            "holding GUIDED before LOITER"
                        )
                        return
                    pre_loiter_elapsed = (now - self.hover_start_time).nanoseconds / 1e9
                    if pre_loiter_elapsed < self.guided_pre_loiter_stable_time_sec:
                        self.get_logger().info(
                            f"GUIDED pre-LOITER hold {pre_loiter_elapsed:.1f}/"
                            f"{self.guided_pre_loiter_stable_time_sec:.1f}s at "
                            f"rel_alt={rel_alt:.2f}m ({source})",
                            throttle_duration_sec=1.0,
                        )
                        return
                    self.get_logger().info(
                        f"GUIDED pre-LOITER hold stable at rel_alt={rel_alt:.2f}m "
                        f"({source}), switching to LOITER"
                    )
                    self.hover_start_time = None
                    self._start_mode_switch("LOITER", Phase.HOVER_LOITER, warn_only=False)
                    return
                if self._is_uwb_approach_land_mode():
                    self._check_stable_and_transition(
                        Phase.MOVE_ABOVE,
                        f"Takeoff hover stable at rel_alt={rel_alt:.2f}m ({source}), starting UWB approach",
                        "takeoff_hover_stable",
                    )
                    if self.phase == Phase.MOVE_ABOVE:
                        self._takeoff_land_hover_ok = True
                    return
                if self._is_takeoff_forward_land_mode():
                    self._check_stable_and_transition(
                        Phase.FORWARD,
                        f"Takeoff-forward hover stable at rel_alt={rel_alt:.2f}m ({source}), starting forward move",
                        "takeoff_forward_hover_stable",
                    )
                    if self.phase == Phase.FORWARD:
                        self._takeoff_land_hover_ok = True
                    return
                if self._is_takeoff_waypoint_return_land_mode():
                    self._check_stable_and_transition(
                        Phase.WAYPOINT_OUTBOUND,
                        f"Takeoff waypoint hover stable at rel_alt={rel_alt:.2f}m ({source}), starting waypoint move",
                        "takeoff_waypoint_hover_stable",
                    )
                    if self.phase == Phase.WAYPOINT_OUTBOUND:
                        self._takeoff_land_hover_ok = True
                    return
                self._check_stable_and_transition(
                    Phase.LAND,
                    f"Takeoff-land hover stable at rel_alt={rel_alt:.2f}m ({source})",
                    "takeoff_land_hover_stable",
                )
                if self.phase == Phase.LAND:
                    self._takeoff_land_hover_ok = True
                return

            self.hover_start_time = None
            if self._takeoff_climb_start_time is None:
                self._takeoff_climb_start_time = now
            climb_elapsed = (now - self._takeoff_climb_start_time).nanoseconds / 1e9
            if climb_elapsed >= self.takeoff_height_timeout_sec:
                self._publish_velocity(0.0, 0.0, 0.0)
                reason = (
                    f"TAKEOFF height timeout: rel_alt={rel_alt:.2f}/"
                    f"{self.takeoff_altitude:.2f}m ({source})"
                )
                self.get_logger().error(reason)
                self._takeoff_land_abort_reason = reason
                self._transition(Phase.FAILSAFE)
                return

            if self.takeoff_method == "velocity":
                self._publish_velocity(0.0, 0.0, self.takeoff_climb_velocity)
                self.get_logger().info(
                    f"Velocity takeoff climbing: rel_alt={rel_alt:.2f}/"
                    f"{self.takeoff_altitude:.2f}m ({source}) "
                    f"vz={self.takeoff_climb_velocity:.2f}m/s",
                    throttle_duration_sec=1.0,
                )
                return

            self.get_logger().info(
                f"Waiting for MAVROS takeoff height: rel_alt={rel_alt:.2f}/"
                f"{self.takeoff_altitude:.2f}m ({source})",
                throttle_duration_sec=1.0,
            )
            return

        self._publish_velocity(0.0, 0.0, 0.0)
        alt = self._get_fcu_altitude()
        if abs(alt - self.takeoff_altitude) <= self.altitude_tolerance:
            self._check_stable_and_transition(
                Phase.MOVE_ABOVE, f"Takeoff altitude stable at {alt:.2f}m", "hover_takeoff_stable"
            )
        else:
            self.hover_start_time = None

    def _tick_forward(self):
        if self.use_mock:
            self.get_logger().info("Mock FORWARD complete")
            self._takeoff_land_forward_ok = True
            self._transition(Phase.HOVER_FORWARD)
            return

        now = self.get_clock().now()
        pos = self._get_local_xy()
        if pos is None:
            self._publish_velocity(0.0, 0.0, 0.0)
            reason = "No local pose during forward move"
            self.get_logger().error(reason)
            self._takeoff_land_abort_reason = reason
            self._transition(Phase.FAILSAFE)
            return

        if self._forward_start_time is None or self._forward_start_xy is None:
            self._forward_start_time = now
            self._forward_start_xy = pos
            self.get_logger().info(
                f"Forward move started from ({pos[0]:.2f}, {pos[1]:.2f}); "
                f"target={self.forward_target_distance:.2f}m "
                f"cmd_body_x={self.forward_velocity:.2f}m/s"
            )

        dx = pos[0] - self._forward_start_xy[0]
        dy = pos[1] - self._forward_start_xy[1]
        dist = math.sqrt(dx * dx + dy * dy)
        elapsed = (now - self._forward_start_time).nanoseconds / 1e9

        if dist >= self.forward_target_distance:
            self._publish_velocity(0.0, 0.0, 0.0)
            self._takeoff_land_forward_ok = True
            self.get_logger().info(
                f"Forward target reached: d={dist:.2f}/"
                f"{self.forward_target_distance:.2f}m, hovering before LAND"
            )
            self._publish_event("forward_target_reached")
            self._transition(Phase.HOVER_FORWARD)
            return

        if elapsed >= self.forward_timeout_sec:
            self._publish_velocity(0.0, 0.0, 0.0)
            reason = (
                f"FORWARD timeout: d={dist:.2f}/"
                f"{self.forward_target_distance:.2f}m after {elapsed:.1f}s"
            )
            self.get_logger().error(reason)
            self._takeoff_land_abort_reason = reason
            self._transition(Phase.FAILSAFE)
            return

        rel_alt, source = self._get_takeoff_land_relative_altitude()
        vz = 0.0
        if rel_alt is not None:
            alt_err = self.takeoff_altitude - rel_alt
            vz = clamp(self.kp_vertical * alt_err, -self.max_vel_z, self.max_vel_z)
        self._publish_velocity(self.forward_velocity, 0.0, vz, frame_id="body")
        alt_text = "missing" if rel_alt is None else f"{rel_alt:.2f}m ({source})"
        self.get_logger().info(
            f"Forward moving: d={dist:.2f}/{self.forward_target_distance:.2f}m "
            f"elapsed={elapsed:.1f}/{self.forward_timeout_sec:.1f}s "
            f"rel_alt={alt_text} cmd_body=({self.forward_velocity:.2f},0.00,{vz:.2f})",
            throttle_duration_sec=1.0,
        )

    def _tick_hover_forward(self):
        self._publish_velocity(0.0, 0.0, 0.0)
        now = self.get_clock().now()
        if self.hover_start_time is None:
            self.hover_start_time = now
            self.get_logger().info(
                f"Forward target hover started, holding {self.target_hover_time:.1f}s before LAND"
            )
            return
        elapsed = (now - self.hover_start_time).nanoseconds / 1e9
        if elapsed >= self.target_hover_time:
            self.get_logger().info("Forward target hover stable, landing")
            self._publish_event("forward_hover_done")
            self._transition(Phase.LAND)
            return
        self.get_logger().info(
            f"Forward target hover holding {elapsed:.1f}/{self.target_hover_time:.1f}s",
            throttle_duration_sec=1.0,
        )

    def _tick_waypoint_outbound(self):
        if self.use_mock:
            self.get_logger().info("Mock WAYPOINT_OUTBOUND complete")
            self._takeoff_land_forward_ok = True
            self._transition(Phase.HOVER_WAYPOINT)
            return

        now = self.get_clock().now()
        pos = self._get_local_xy()
        if pos is None:
            self._publish_velocity(0.0, 0.0, 0.0)
            reason = "No local pose during waypoint outbound"
            self.get_logger().error(reason)
            self._takeoff_land_abort_reason = reason
            self._transition(Phase.FAILSAFE)
            return

        if self._waypoint_start_time is None:
            self._waypoint_start_time = now
            self._forward_start_xy = pos
            self.get_logger().info(
                f"Waypoint outbound BODY_NED started: origin=({self.origin_x:.2f}, {self.origin_y:.2f}) "
                f"current=({pos[0]:.2f}, {pos[1]:.2f}) body_offset=({self.waypoint_dx:.2f}, {self.waypoint_dy:.2f})"
            )

        dx = pos[0] - self._forward_start_xy[0]
        dy = pos[1] - self._forward_start_xy[1]
        dist = math.sqrt(dx * dx + dy * dy)
        target_dist = math.sqrt(self.waypoint_dx * self.waypoint_dx + self.waypoint_dy * self.waypoint_dy)
        elapsed = (now - self._waypoint_start_time).nanoseconds / 1e9

        if dist >= max(self.waypoint_xy_tolerance, target_dist):
            self._publish_velocity(0.0, 0.0, 0.0)
            self.get_logger().info(
                f"Waypoint outbound reached by displacement: d={dist:.2f}/{target_dist:.2f}m"
            )
            self._takeoff_land_forward_ok = True
            self._publish_event("waypoint_reached")
            self._transition(Phase.HOVER_WAYPOINT)
            return

        if elapsed >= self.waypoint_timeout_sec:
            self._publish_velocity(0.0, 0.0, 0.0)
            reason = (
                f"WAYPOINT outbound timeout: d={dist:.2f}/{target_dist:.2f}m "
                f"after {elapsed:.1f}s"
            )
            self.get_logger().error(reason)
            self._takeoff_land_abort_reason = reason
            self._transition(Phase.FAILSAFE)
            return

        rel_alt, source = self._get_takeoff_land_relative_altitude()
        vz = 0.0
        if rel_alt is not None:
            alt_err = self.takeoff_altitude - rel_alt
            vz = clamp(self.kp_vertical * alt_err, -self.max_vel_z, self.max_vel_z)

        vx_body = clamp(self.kp_waypoint * self.waypoint_dx, -self.waypoint_max_velocity, self.waypoint_max_velocity)
        vy_body = clamp(self.kp_waypoint * self.waypoint_dy, -self.waypoint_max_velocity, self.waypoint_max_velocity)
        speed = math.sqrt(vx_body * vx_body + vy_body * vy_body)
        if speed > self.waypoint_max_velocity:
            scale = self.waypoint_max_velocity / speed
            vx_body *= scale
            vy_body *= scale

        self._publish_velocity(vx_body, vy_body, vz, frame_id="body")
        alt_text = "missing" if rel_alt is None else f"{rel_alt:.2f}m ({source})"
        self.get_logger().info(
            f"Waypoint outbound BODY_NED moving: d={dist:.2f}/{target_dist:.2f}m "
            f"elapsed={elapsed:.1f}/{self.waypoint_timeout_sec:.1f}s "
            f"rel_alt={alt_text} cmd_body=({vx_body:.2f},{vy_body:.2f},{vz:.2f})",
            throttle_duration_sec=1.0,
        )

    def _tick_hover_waypoint(self):
        self._publish_velocity(0.0, 0.0, 0.0)
        now = self.get_clock().now()
        if self.hover_start_time is None:
            self.hover_start_time = now
            self.get_logger().info(
                f"Waypoint hover started, holding {self.target_hover_time:.1f}s before descend"
            )
            return
        elapsed = (now - self.hover_start_time).nanoseconds / 1e9
        if elapsed >= self.target_hover_time:
            self.get_logger().info("Waypoint hover stable, descending at target")
            self._publish_event("waypoint_hover_done")
            self._transition(Phase.WAYPOINT_DESCEND)
            return
        self.get_logger().info(
            f"Waypoint hover holding {elapsed:.1f}/{self.target_hover_time:.1f}s",
            throttle_duration_sec=1.0,
        )

    def _tick_waypoint_descend(self):
        if self.use_mock:
            self.get_logger().info("Mock WAYPOINT_DESCEND complete")
            self._transition(Phase.HOVER_WAYPOINT_LOW)
            return

        rel_alt, source = self._get_takeoff_land_relative_altitude()
        if rel_alt is None:
            self._publish_velocity(0.0, 0.0, 0.0, frame_id="body")
            self.hover_start_time = None
            self.get_logger().warn(
                "Waypoint descend waiting for relative altitude source...",
                throttle_duration_sec=1.0,
            )
            return

        alt_err = self.descend_altitude - rel_alt
        vz = clamp(self.kp_vertical * alt_err, -self.max_vel_z, self.max_vel_z)
        self._publish_velocity(0.0, 0.0, vz, frame_id="body")

        if abs(alt_err) <= self.altitude_tolerance:
            self._check_stable_and_transition(
                Phase.HOVER_WAYPOINT_LOW,
                f"Waypoint low altitude stable at rel_alt={rel_alt:.2f}m ({source})",
                "waypoint_low_altitude_reached",
            )
            return

        self.hover_start_time = None
        self.get_logger().info(
            f"Waypoint descending: rel_alt={rel_alt:.2f}/{self.descend_altitude:.2f}m "
            f"({source}) cmd_body=(0.00,0.00,{vz:.2f})",
            throttle_duration_sec=1.0,
        )

    def _tick_hover_waypoint_low(self):
        self._publish_velocity(0.0, 0.0, 0.0, frame_id="body")
        now = self.get_clock().now()
        if self.hover_start_time is None:
            self.hover_start_time = now
            self.get_logger().info(
                f"Waypoint low hover started, holding {self.low_hover_time:.1f}s before reclimb"
            )
            return
        elapsed = (now - self.hover_start_time).nanoseconds / 1e9
        if elapsed >= self.low_hover_time:
            self.get_logger().info("Waypoint low hover stable, reclimbing")
            self._publish_event("waypoint_low_hover_done")
            self._transition(Phase.WAYPOINT_RECLIMB)
            return
        self.get_logger().info(
            f"Waypoint low hover holding {elapsed:.1f}/{self.low_hover_time:.1f}s",
            throttle_duration_sec=1.0,
        )

    def _tick_waypoint_reclimb(self):
        if self.use_mock:
            self.get_logger().info("Mock WAYPOINT_RECLIMB complete")
            self._transition(Phase.HOVER_WAYPOINT_RECLIMB)
            return

        rel_alt, source = self._get_takeoff_land_relative_altitude()
        if rel_alt is None:
            self._publish_velocity(0.0, 0.0, 0.0, frame_id="body")
            self.hover_start_time = None
            self.get_logger().warn(
                "Waypoint reclimb waiting for relative altitude source...",
                throttle_duration_sec=1.0,
            )
            return

        alt_err = self.takeoff_altitude - rel_alt
        vz = clamp(self.kp_vertical * alt_err, -self.max_vel_z, self.max_vel_z)
        self._publish_velocity(0.0, 0.0, vz, frame_id="body")

        if abs(alt_err) <= self.altitude_tolerance:
            self._check_stable_and_transition(
                Phase.HOVER_WAYPOINT_RECLIMB,
                f"Waypoint reclimb stable at rel_alt={rel_alt:.2f}m ({source})",
                "waypoint_reclimb_done",
            )
            return

        self.hover_start_time = None
        self.get_logger().info(
            f"Waypoint reclimbing: rel_alt={rel_alt:.2f}/{self.takeoff_altitude:.2f}m "
            f"({source}) cmd_body=(0.00,0.00,{vz:.2f})",
            throttle_duration_sec=1.0,
        )

    def _tick_hover_waypoint_reclimb(self):
        self._publish_velocity(0.0, 0.0, 0.0, frame_id="body")
        now = self.get_clock().now()
        if self.hover_start_time is None:
            self.hover_start_time = now
            self.get_logger().info(
                f"Waypoint reclimb hover started, holding {self.target_hover_time:.1f}s before return"
            )
            return
        elapsed = (now - self.hover_start_time).nanoseconds / 1e9
        if elapsed >= self.target_hover_time:
            self.get_logger().info("Waypoint reclimb hover stable, switching MAVROS velocity frame to LOCAL_NED for return")
            self._publish_event("waypoint_reclimb_hover_done")
            self._start_mav_frame_switch("LOCAL_NED", Phase.WAYPOINT_RETURN)
            return
        self.get_logger().info(
            f"Waypoint reclimb hover holding {elapsed:.1f}/{self.target_hover_time:.1f}s",
            throttle_duration_sec=1.0,
        )

    def _tick_waypoint_return(self):
        if self.use_mock:
            self.get_logger().info("Mock WAYPOINT_RETURN complete")
            self._takeoff_land_guided_return_ok = True
            self._transition(Phase.HOVER_RETURN_HOME)
            return

        reached = self._tick_local_waypoint(
            (self.origin_x, self.origin_y),
            "Waypoint return",
            "WAYPOINT return timeout",
            "_waypoint_return_start_time",
        )
        if reached:
            self._takeoff_land_guided_return_ok = True
            self._publish_event("waypoint_returned_home")
            self._transition(Phase.HOVER_RETURN_HOME)

    def _tick_hover_return_home(self):
        self._publish_velocity(0.0, 0.0, 0.0)
        now = self.get_clock().now()
        if self.hover_start_time is None:
            self.hover_start_time = now
            self.get_logger().info(
                f"Home hover started, holding {self.return_hover_time:.1f}s before LAND"
            )
            return
        elapsed = (now - self.hover_start_time).nanoseconds / 1e9
        if elapsed >= self.return_hover_time:
            self.get_logger().info("Home hover stable, landing")
            self._publish_event("waypoint_home_hover_done")
            self._transition(Phase.LAND)
            return
        self.get_logger().info(
            f"Home hover holding {elapsed:.1f}/{self.return_hover_time:.1f}s",
            throttle_duration_sec=1.0,
        )

    def _tick_local_waypoint(self, target_xy, label: str, timeout_label: str, start_attr: str) -> bool:
        now = self.get_clock().now()
        pos = self._get_local_xy()
        if pos is None:
            self._publish_velocity(0.0, 0.0, 0.0)
            reason = f"No local pose during {label.lower()}"
            self.get_logger().error(reason)
            self._takeoff_land_abort_reason = reason
            self._transition(Phase.FAILSAFE)
            return False

        if getattr(self, start_attr) is None:
            setattr(self, start_attr, now)
            self.get_logger().info(
                f"{label} started: current=({pos[0]:.2f}, {pos[1]:.2f}) "
                f"target=({target_xy[0]:.2f}, {target_xy[1]:.2f})"
            )

        err_x = target_xy[0] - pos[0]
        err_y = target_xy[1] - pos[1]
        dist = math.sqrt(err_x * err_x + err_y * err_y)
        elapsed = (now - getattr(self, start_attr)).nanoseconds / 1e9

        if dist <= self.waypoint_xy_tolerance:
            self._publish_velocity(0.0, 0.0, 0.0)
            self.get_logger().info(
                f"{label} reached: d={dist:.2f}/{self.waypoint_xy_tolerance:.2f}m"
            )
            return True

        if elapsed >= self.waypoint_timeout_sec:
            self._publish_velocity(0.0, 0.0, 0.0)
            reason = (
                f"{timeout_label}: d={dist:.2f}/{self.waypoint_xy_tolerance:.2f}m "
                f"after {elapsed:.1f}s"
            )
            self.get_logger().error(reason)
            self._takeoff_land_abort_reason = reason
            self._transition(Phase.FAILSAFE)
            return False

        vx = self.kp_waypoint * err_x
        vy = self.kp_waypoint * err_y
        speed = math.sqrt(vx * vx + vy * vy)
        if speed > self.waypoint_max_velocity:
            scale = self.waypoint_max_velocity / speed
            vx *= scale
            vy *= scale

        rel_alt, source = self._get_takeoff_land_relative_altitude()
        vz = 0.0
        if rel_alt is not None:
            alt_err = self.takeoff_altitude - rel_alt
            vz = clamp(self.kp_vertical * alt_err, -self.max_vel_z, self.max_vel_z)

        self._publish_velocity(vx, vy, vz, frame_id="local")
        alt_text = "missing" if rel_alt is None else f"{rel_alt:.2f}m ({source})"
        self.get_logger().info(
            f"{label} moving: d={dist:.2f}/{self.waypoint_xy_tolerance:.2f}m "
            f"elapsed={elapsed:.1f}/{self.waypoint_timeout_sec:.1f}s "
            f"err=({err_x:.2f},{err_y:.2f}) rel_alt={alt_text} "
            f"cmd_local=({vx:.2f},{vy:.2f},{vz:.2f})",
            throttle_duration_sec=1.0,
        )
        return False

    def _tick_hover_loiter(self):
        self._publish_velocity(0.0, 0.0, 0.0)
        rel_alt, source = self._get_takeoff_land_relative_altitude()
        alt_text = "rel_alt=missing"
        if rel_alt is not None:
            alt_text = f"rel_alt={rel_alt:.2f}m ({source})"

        now = self.get_clock().now()
        if rel_alt is None:
            self.hover_start_time = None
            self._loiter_alt_loss_start_time = None
            self.get_logger().warn(
                "LOITER hover waiting for relative altitude source...",
                throttle_duration_sec=1.0,
            )
            return

        if rel_alt < self.loiter_min_rel_alt:
            self.hover_start_time = None
            if self._loiter_alt_loss_start_time is None:
                self._loiter_alt_loss_start_time = now
                self.get_logger().warn(
                    f"LOITER hover altitude low at {alt_text}; waiting before failsafe"
                )
                return
            low_elapsed = (now - self._loiter_alt_loss_start_time).nanoseconds / 1e9
            if low_elapsed >= self.loiter_alt_loss_timeout_sec:
                reason = (
                    f"LOITER altitude lost: rel_alt={rel_alt:.2f}m below "
                    f"{self.loiter_min_rel_alt:.2f}m"
                )
                self.get_logger().error(reason)
                self._takeoff_land_abort_reason = reason
                self._transition(Phase.FAILSAFE)
                return
            self.get_logger().warn(
                f"LOITER hover altitude low {low_elapsed:.1f}/"
                f"{self.loiter_alt_loss_timeout_sec:.1f}s at {alt_text}",
                throttle_duration_sec=0.5,
            )
            return

        self._loiter_alt_loss_start_time = None
        if self.hover_start_time is None:
            self.hover_start_time = now
            self.get_logger().info(f"LOITER hover started at {alt_text}, stabilizing")
            return

        elapsed = (now - self.hover_start_time).nanoseconds / 1e9
        if elapsed >= self.hover_stable_time:
            self.get_logger().info(f"LOITER hover stable at {alt_text}")
            self._publish_event("takeoff_loiter_hover_stable")
            self._takeoff_land_hover_ok = True
            self._start_mode_switch("GUIDED", Phase.LAND, warn_only=False)
            return

        self.get_logger().info(
            f"LOITER hover holding {elapsed:.1f}/{self.hover_stable_time:.1f}s at {alt_text}",
            throttle_duration_sec=1.0,
        )

    def _tick_move_above(self):
        if self.use_mock:
            self.get_logger().info("Mock MOVE_ABOVE complete")
            self._transition(Phase.HOVER_ABOVE)
            return

        now = self.get_clock().now()
        if self._is_uwb_approach_land_mode() and self._move_above_start_time is None:
            self._move_above_start_time = now
            self._uwb_missing_start_time = None

        if self._is_uwb_approach_land_mode() and self._move_above_start_time is not None:
            move_elapsed = (now - self._move_above_start_time).nanoseconds / 1e9
            if move_elapsed >= self.move_above_timeout_sec:
                self._publish_velocity(0.0, 0.0, 0.0)
                reason = f"UWB approach timeout after {move_elapsed:.1f}s"
                self.get_logger().error(reason)
                self._takeoff_land_abort_reason = reason
                self._transition(Phase.FAILSAFE)
                return

        uwb = self._get_uwb()
        if uwb is None or not self._uwb_valid_and_fresh():
            self._publish_velocity(0.0, 0.0, 0.0)
            if self._is_uwb_approach_land_mode():
                if self._uwb_missing_start_time is None:
                    self._uwb_missing_start_time = now
                    self.get_logger().warn("No fresh UWB data during approach, hovering")
                    return
                missing_elapsed = (now - self._uwb_missing_start_time).nanoseconds / 1e9
                if missing_elapsed >= self.uwb_missing_timeout_sec:
                    reason = f"UWB data lost for {missing_elapsed:.1f}s during approach"
                    self.get_logger().error(reason)
                    self._takeoff_land_abort_reason = reason
                    self._transition(Phase.FAILSAFE)
                    return
                self.get_logger().warn(
                    f"No fresh UWB data during approach "
                    f"{missing_elapsed:.1f}/{self.uwb_missing_timeout_sec:.1f}s",
                    throttle_duration_sec=0.5,
                )
                return
            self.get_logger().warn("No fresh UWB data, hovering", throttle_duration_sec=2.0)
            return

        self._uwb_missing_start_time = None
        raw_azimuth, distance, elevation = uwb
        azimuth = raw_azimuth - self.uwb_azimuth_offset_deg
        az_rad = azimuth * math.pi / 180.0
        el_rad = elevation * math.pi / 180.0
        horizontal_dist = abs(distance * math.cos(el_rad))
        forward_dist = horizontal_dist * math.cos(az_rad)
        lateral_dist = horizontal_dist * math.sin(az_rad)

        vx = 0.0
        vy = 0.0
        stop_radius = self.uwb_capture_radius_m if self._is_uwb_approach_land_mode() else self.horizontal_deadband
        speed_limit = self.max_vel_xy
        if self._is_uwb_approach_land_mode() and horizontal_dist <= self.uwb_slow_radius_m:
            speed_limit = min(speed_limit, self.uwb_slow_max_vel_xy)
        if horizontal_dist > stop_radius:
            vx = clamp(
                self.uwb_forward_sign * self.kp_horizontal * forward_dist,
                -speed_limit,
                speed_limit,
            )
            vy = clamp(
                self.uwb_lateral_sign * self.kp_horizontal * lateral_dist,
                -speed_limit,
                speed_limit,
            )

        if self._is_uwb_approach_land_mode():
            rel_alt, alt_source = self._get_takeoff_land_relative_altitude()
            alt_err = 0.0 if rel_alt is None else self.takeoff_altitude - rel_alt
        else:
            rel_alt = self._get_fcu_altitude()
            alt_source = "local_z_abs"
            alt_err = self.takeoff_altitude - rel_alt
        vz = clamp(self.kp_vertical * alt_err, -self.max_vel_z, self.max_vel_z)

        radius = self._mission_xy_distance_from_origin()
        if self._is_uwb_approach_land_mode():
            front_ok = abs(azimuth) <= self.uwb_approach_front_sector_deg
            capture_geometry_ok = (
                horizontal_dist <= self.uwb_capture_radius_m
                and abs(azimuth) <= self.uwb_capture_front_sector_deg
            )
            if not front_ok:
                self._uwb_capture_start_time = None
                if self._uwb_out_of_front_start_time is None:
                    self._uwb_out_of_front_start_time = now
                out_elapsed = (now - self._uwb_out_of_front_start_time).nanoseconds / 1e9
                self._publish_velocity(0.0, 0.0, vz, frame_id="body", immediate=True)
                self.get_logger().warn(
                    f"UWB target outside front sector: az={azimuth:.1f}deg "
                    f"raw_az={raw_azimuth:.1f}deg limit={self.uwb_approach_front_sector_deg:.1f}deg "
                    f"hdist={horizontal_dist:.2f}m elapsed={out_elapsed:.1f}/"
                    f"{self.uwb_front_sector_timeout_sec:.1f}s action={self.uwb_out_of_front_action}",
                    throttle_duration_sec=0.5,
                )
                if (
                    out_elapsed >= self.uwb_front_sector_timeout_sec
                    and self.uwb_out_of_front_action == "LAND"
                ):
                    reason = (
                        f"UWB target outside front sector for {out_elapsed:.1f}s "
                        f"(az={azimuth:.1f}deg, limit={self.uwb_approach_front_sector_deg:.1f}deg)"
                    )
                    self.get_logger().warn(f"{reason}; landing")
                    self._publish_event("uwb_out_of_front_land")
                    self._takeoff_land_abort_reason = reason
                    self._transition(Phase.LAND)
                return

            self._uwb_out_of_front_start_time = None
            if capture_geometry_ok:
                if self._uwb_capture_start_time is None:
                    self._uwb_capture_start_time = now
                    capture_elapsed = 0.0
                else:
                    capture_elapsed = (now - self._uwb_capture_start_time).nanoseconds / 1e9
                self._publish_velocity(0.0, 0.0, vz, frame_id="body", immediate=True)
                if capture_elapsed < self.uwb_capture_stable_sec:
                    self.get_logger().info(
                        f"UWB target capture candidate: az={azimuth:.1f}deg "
                        f"raw_az={raw_azimuth:.1f}deg el={elevation:.1f}deg "
                        f"hdist={horizontal_dist:.2f}/{self.uwb_capture_radius_m:.2f}m "
                        f"stable={capture_elapsed:.1f}/{self.uwb_capture_stable_sec:.1f}s",
                        throttle_duration_sec=0.2,
                    )
                    return
                self._uwb_target_captured = True
                self._uwb_approach_ok = True
                self._publish_event("uwb_target_captured")
                self.get_logger().info(
                    f"UWB target captured: az={azimuth:.1f}deg raw_az={raw_azimuth:.1f}deg "
                    f"el={elevation:.1f}deg hdist={horizontal_dist:.2f}/"
                    f"{self.uwb_capture_radius_m:.2f}m body_dist=({forward_dist:.2f},{lateral_dist:.2f}) "
                    f"stable={capture_elapsed:.1f}s"
                )
                self._transition(Phase.HOVER_ABOVE)
                return
            self._uwb_capture_start_time = None

        if (
            self._is_uwb_approach_land_mode()
            and radius is not None
            and radius >= self.mission_soft_radius_m
        ):
            self._publish_velocity(0.0, 0.0, vz, frame_id="body", immediate=True)
            self.hover_start_time = None
            self.get_logger().warn(
                f"Mission soft radius reached: r={radius:.2f}/"
                f"{self.mission_soft_radius_m:.2f}m, holding position",
                throttle_duration_sec=0.5,
            )
            return

        self._publish_velocity(vx, vy, vz, frame_id="body")

        if abs(azimuth) < self.azimuth_deadband and horizontal_dist < self.horizontal_deadband:
            self._check_stable_and_transition(
                Phase.HOVER_ABOVE,
                f"Above target: az={azimuth:.1f}deg hdist={horizontal_dist:.2f}m",
                "above_target_reached",
            )
            if self.phase == Phase.HOVER_ABOVE and self._is_uwb_approach_land_mode():
                self._uwb_approach_ok = True
        else:
            self.hover_start_time = None
            alt_text = "missing" if rel_alt is None else f"{rel_alt:.2f}m"
            self.get_logger().info(
                f"UWB approach BODY_NED: az={azimuth:.1f}deg raw_az={raw_azimuth:.1f}deg "
                f"el={elevation:.1f}deg hdist={horizontal_dist:.2f}m "
                f"body_dist=({forward_dist:.2f},{lateral_dist:.2f}) "
                f"rel_alt={alt_text} ({alt_source}) "
                f"cmd_body=({vx:.2f},{vy:.2f},{vz:.2f}) speed_limit={speed_limit:.2f} "
                f"front_limit={self.uwb_approach_front_sector_deg:.1f}deg "
                f"capture_limit={self.uwb_capture_front_sector_deg:.1f}deg",
                throttle_duration_sec=1.0,
            )

    def _tick_hover_above(self):
        if self._is_uwb_approach_land_mode():
            self._publish_velocity(0.0, 0.0, 0.0, immediate=True)
            now = self.get_clock().now()
            if self.hover_start_time is None:
                self.hover_start_time = now
                self.get_logger().info(
                    f"UWB target hover started, holding "
                    f"{self.uwb_target_hover_time_sec:.1f}s before LAND"
                )
                return
            elapsed = (now - self.hover_start_time).nanoseconds / 1e9
            if elapsed >= self.uwb_target_hover_time_sec:
                self.get_logger().info("UWB target hover stable, landing")
                self._publish_event("uwb_target_hover_done")
                self._transition(Phase.LAND)
                return
            self.get_logger().info(
                f"UWB target hover holding {elapsed:.1f}/"
                f"{self.uwb_target_hover_time_sec:.1f}s before LAND",
                throttle_duration_sec=0.5,
            )
            return
        self._publish_velocity(0.0, 0.0, 0.0)
        self._check_stable_and_transition(
            Phase.DESCEND, "Hover above target stable", "hover_above_done"
        )

    def _tick_descend(self):
        if self.use_mock:
            self._check_stable_and_transition(
                Phase.HOVER_FINAL, "Mock descent complete", "final_altitude_reached"
            )
            return

        vx = 0.0
        vy = 0.0
        uwb = self._get_uwb()
        if uwb is not None and self._uwb_valid_and_fresh():
            azimuth, distance, _ = uwb
            az_rad = azimuth * math.pi / 180.0
            horizontal_dist = distance * math.cos(az_rad)
            if abs(azimuth) > self.azimuth_deadband:
                vx = clamp(
                    -self.kp_horizontal * az_rad * self.max_vel_xy * 0.5,
                    -self.max_vel_xy * 0.5,
                    self.max_vel_xy * 0.5,
                )
            if horizontal_dist > self.horizontal_deadband:
                vy = clamp(
                    -self.kp_horizontal * horizontal_dist * 0.5,
                    -self.max_vel_xy * 0.5,
                    self.max_vel_xy * 0.5,
                )

        alt = self._get_fcu_altitude()
        alt_err = self.descend_altitude - alt
        vz = 0.0
        if abs(alt_err) > self.altitude_tolerance:
            vz = clamp(-self.kp_vertical * abs(alt_err), -self.max_vel_z, 0.0)

        self._publish_velocity(vx, vy, vz)

        if abs(alt - self.descend_altitude) <= self.altitude_tolerance:
            self._check_stable_and_transition(
                Phase.HOVER_FINAL, f"Final altitude stable at {alt:.2f}m", "final_altitude_reached"
            )
        else:
            self.hover_start_time = None

    def _tick_hover_final(self):
        self._publish_velocity(0.0, 0.0, 0.0)
        self._check_stable_and_transition(
            Phase.WAIT_GRASP, "Final hover stable, waiting for grasp", "final_hover_reached"
        )

    def _tick_wait_grasp(self):
        self._publish_velocity(0.0, 0.0, 0.0)
        if self.fake_grasp:
            if self.grasp_start_time is None:
                self.grasp_start_time = self.get_clock().now()
                self.get_logger().info(f"Waiting for fake grasp ({self.fake_grasp_delay_sec:.1f}s)")
            elapsed = (self.get_clock().now() - self.grasp_start_time).nanoseconds / 1e9
            if elapsed >= self.fake_grasp_delay_sec:
                self._publish_event("grasp_complete")
                self._transition(Phase.CLIMB)
            return

        if self._grasp_done:
            self._publish_event("grasp_complete")
            self._transition(Phase.CLIMB)
            return

        if self.grasp_start_time is None:
            self.grasp_start_time = self.get_clock().now()
            self.get_logger().info(
                f"Waiting for grasp_done signal (timeout={self.grasp_timeout_sec:.1f}s)"
            )

        elapsed = (self.get_clock().now() - self.grasp_start_time).nanoseconds / 1e9
        if elapsed >= self.grasp_timeout_sec:
            self.get_logger().error(f"grasp_done timeout after {elapsed:.1f}s, entering FAILSAFE")
            self._publish_event("grasp_timeout")
            self._publish_velocity(0.0, 0.0, 0.0)
            self._transition(Phase.FAILSAFE)
            return

        self.get_logger().info("Waiting for grasp_done signal...", throttle_duration_sec=2.0)

    def _tick_climb(self):
        if self.use_mock:
            self._check_stable_and_transition(Phase.HOVER_CLIMB, "Mock climb complete", "climb_done")
            return

        alt = self._get_fcu_altitude()
        alt_err = self.takeoff_altitude - alt
        vz = 0.0
        if abs(alt_err) > self.altitude_tolerance:
            vz = clamp(self.kp_vertical * alt_err, -self.max_vel_z, self.max_vel_z)

        self._publish_velocity(0.0, 0.0, vz)

        if abs(alt - self.takeoff_altitude) <= self.altitude_tolerance:
            self._check_stable_and_transition(Phase.HOVER_CLIMB, f"Climbed to {alt:.2f}m", "climb_done")
        else:
            self.hover_start_time = None

    def _tick_hover_climb(self):
        self._publish_velocity(0.0, 0.0, 0.0)
        self._check_stable_and_transition(
            Phase.RETURN, "Climb hover stable, returning", "hover_climb_done"
        )

    def _tick_return(self):
        if self.use_mock:
            self.get_logger().info("Mock RETURN complete")
            self._transition(Phase.HOVER_RETURN)
            return

        pos = self._get_local_xy()
        if pos is None:
            self._publish_velocity(0.0, 0.0, 0.0)
            self.get_logger().warn("No local pose for return, hovering", throttle_duration_sec=2.0)
            return

        dx = pos[0] - self.origin_x
        dy = pos[1] - self.origin_y
        dist = math.sqrt(dx * dx + dy * dy)

        vx = 0.0
        vy = 0.0
        if dist > self.return_xy_tolerance:
            vx = -self.kp_return * dx
            vy = -self.kp_return * dy
            speed = math.sqrt(vx * vx + vy * vy)
            if speed > self.max_vel_xy:
                scale = self.max_vel_xy / speed
                vx *= scale
                vy *= scale

        alt_err = self.takeoff_altitude - self._get_fcu_altitude()
        vz = clamp(self.kp_vertical * alt_err, -self.max_vel_z, self.max_vel_z)
        self._publish_velocity(vx, vy, vz)

        if dist <= self.return_xy_tolerance:
            self._check_stable_and_transition(
                Phase.HOVER_RETURN, f"Returned to origin: d={dist:.2f}m", "return_arrived"
            )
        else:
            self.hover_start_time = None

    def _tick_hover_return(self):
        self._publish_velocity(0.0, 0.0, 0.0)
        self._check_stable_and_transition(
            Phase.WAIT_DROP, "Hover above origin stable, waiting for drop", "hover_return_done"
        )

    def _tick_wait_drop(self):
        self._publish_velocity(0.0, 0.0, 0.0)
        if self.fake_drop:
            if self.drop_start_time is None:
                self.drop_start_time = self.get_clock().now()
                self.get_logger().info(f"Waiting for fake drop ({self.fake_drop_delay_sec:.1f}s)")
            elapsed = (self.get_clock().now() - self.drop_start_time).nanoseconds / 1e9
            if elapsed >= self.fake_drop_delay_sec:
                self._publish_event("drop_complete")
                self._transition(Phase.LAND)
            return

        if self._drop_done:
            self._publish_event("drop_complete")
            self._transition(Phase.LAND)
            return

        if self.drop_start_time is None:
            self.drop_start_time = self.get_clock().now()
            self.get_logger().info(
                f"Waiting for drop_done signal (timeout={self.drop_timeout_sec:.1f}s)"
            )

        elapsed = (self.get_clock().now() - self.drop_start_time).nanoseconds / 1e9
        if elapsed >= self.drop_timeout_sec:
            self.get_logger().error(f"drop_done timeout after {elapsed:.1f}s, entering FAILSAFE")
            self._publish_event("drop_timeout")
            self._publish_velocity(0.0, 0.0, 0.0)
            self._transition(Phase.FAILSAFE)
            return

        self.get_logger().info("Waiting for drop_done signal...", throttle_duration_sec=2.0)

    def _tick_land(self):
        if self._pending_command:
            return
        self._publish_velocity(0.0, 0.0, 0.0, immediate=True)
        self._pending_command = True
        self._call_flight_cmd(FlightCommand.Request.CMD_MODE_LAND, 0.0, self._on_land_result)

    def _on_land_result(self, success: bool, msg: str):
        self._pending_command = False
        if success:
            self.get_logger().info(f"Land OK: {msg}")
            self._publish_event("landing")
            self._transition(Phase.LAND_WAIT)
        else:
            self.get_logger().error(f"Land FAILED: {msg}, retrying")
            if self._is_takeoff_land_mode():
                self._report_takeoff_land_result("LAND failed")

    def _tick_land_wait(self):
        self._publish_velocity(0.0, 0.0, 0.0, immediate=True)
        now = self.get_clock().now()
        if self._land_wait_start_time is None:
            self._land_wait_start_time = now
            self._land_ground_start_time = None
            self._land_disarm_sent = False
            self._land_disarm_retry_count = 0

        rel_alt, source = self._get_takeoff_land_relative_altitude()
        alt_text = "rel_alt=missing"
        if rel_alt is not None:
            alt_text = f"rel_alt={rel_alt:.2f}m ({source})"

        if not self._armed_confirmed():
            self.get_logger().info(f"Landing complete: disarmed with {alt_text}")
            self._finish_landing_success()
            return

        wait_elapsed = (now - self._land_wait_start_time).nanoseconds / 1e9
        near_ground = rel_alt is not None and rel_alt <= self.land_ground_rel_alt_threshold
        if near_ground:
            if self._land_ground_start_time is None:
                self._land_ground_start_time = now
                self.get_logger().info(
                    f"Landing near ground at {alt_text}, waiting stable before DISARM"
                )
                return
            ground_elapsed = (now - self._land_ground_start_time).nanoseconds / 1e9
            if ground_elapsed >= self.land_ground_stable_sec:
                if self._pending_command:
                    return
                if not self._land_disarm_sent:
                    if self._land_disarm_retry_time is not None:
                        retry_elapsed = (now - self._land_disarm_retry_time).nanoseconds / 1e9
                        if retry_elapsed < 1.0:
                            self.get_logger().info(
                                f"Landing DISARM retry wait {retry_elapsed:.1f}/1.0s at {alt_text}",
                                throttle_duration_sec=0.5,
                            )
                            return
                    self._land_disarm_sent = True
                    self._pending_command = True
                    self.get_logger().info(
                        f"Landing ground confirmed at {alt_text}; sending DISARM"
                    )
                    self._call_flight_cmd(
                        FlightCommand.Request.CMD_DISARM,
                        0.0,
                        self._on_land_disarm_result,
                    )
                return
            self.get_logger().info(
                f"Landing ground stable wait {ground_elapsed:.1f}/"
                f"{self.land_ground_stable_sec:.1f}s at {alt_text}",
                throttle_duration_sec=0.5,
            )
            return

        self._land_ground_start_time = None
        if wait_elapsed >= self.land_wait_timeout_sec:
            reason = f"LAND wait timeout after {wait_elapsed:.1f}s at {alt_text}"
            self.get_logger().error(reason)
            if self._is_takeoff_land_mode():
                self._report_takeoff_land_result(reason)
            self._transition(Phase.FAILSAFE)
            return

        self.get_logger().info(
            f"Landing wait {wait_elapsed:.1f}/{self.land_wait_timeout_sec:.1f}s at {alt_text}",
            throttle_duration_sec=1.0,
        )

    def _on_land_disarm_result(self, success: bool, msg: str):
        self._pending_command = False
        if success or not self._armed_confirmed():
            self.get_logger().info(f"Landing DISARM: {'OK' if success else 'already disarmed'} - {msg}")
            self._finish_landing_success()
            return
        self._land_disarm_retry_count += 1
        if self._land_disarm_retry_count <= 3:
            self._land_disarm_sent = False
            self._land_disarm_retry_time = self.get_clock().now()
            self.get_logger().warn(
                f"Landing DISARM rejected ({self._land_disarm_retry_count}/3): {msg}; "
                "waiting and retrying"
            )
            return
        reason = f"Landing DISARM failed after retries: {msg}"
        self.get_logger().error(reason)
        if self._is_takeoff_land_mode():
            self._report_takeoff_land_result(reason)
        self._transition(Phase.FAILSAFE)

    def _finish_landing_success(self):
        if self._is_takeoff_land_mode():
            self._takeoff_land_land_ok = True
        self._transition(Phase.DONE)
        reset_msg = String()
        reset_msg.data = "RESET"
        self.flight_reset_pub.publish(reset_msg)
        if self._is_takeoff_land_mode():
            self._report_takeoff_land_result(self._takeoff_land_abort_reason)

    def _tick_done(self):
        self._publish_velocity(0.0, 0.0, 0.0, immediate=True)

    def _tick_paused_manual(self):
        self._publish_velocity(0.0, 0.0, 0.0, immediate=True)
        self.get_logger().warn(
            "Mission paused by RC/mode takeover. Restart mission node to resume autonomy.",
            throttle_duration_sec=5.0,
        )

    def _tick_recovering(self):
        self._publish_velocity(0.0, 0.0, 0.0)

        if self._last_link_status == "OK":
            self.get_logger().info("Link recovered, resuming from hover-takeoff stage")
            self._publish_event("link_recovered")
            self._recovery_start_time = None
            self._transition(Phase.HOVER_TAKEOFF)
            return

        if self._recovery_start_time is None:
            self._recovery_start_time = self.get_clock().now()

        elapsed = (self.get_clock().now() - self._recovery_start_time).nanoseconds / 1e9
        if elapsed >= self.recovery_timeout:
            self.get_logger().error(f"Recovery timeout ({elapsed:.1f}s), entering FAILSAFE")
            self._publish_event("recovery_timeout")
            self._transition(Phase.FAILSAFE)

    def _tick_failsafe(self):
        self._publish_velocity(0.0, 0.0, 0.0, immediate=True)
        if self._pending_command:
            return

        command = FlightCommand.Request.CMD_MODE_LAND
        command_name = "LAND"
        if self._takeoff_land_abort_reason and not self._takeoff_land_takeoff_ok:
            rel_alt, _ = self._get_takeoff_land_relative_altitude()
            if rel_alt is not None and rel_alt < 0.25:
                command = FlightCommand.Request.CMD_DISARM
                command_name = "DISARM"

        self._pending_command = True

        def _on_failsafe_result(ok, msg):
            self._pending_command = False
            self.get_logger().info(
                f"Failsafe {command_name}: {'OK' if ok else 'FAILED'} - {msg}"
            )
            if ok and command == FlightCommand.Request.CMD_MODE_LAND:
                self._transition(Phase.LAND_WAIT)
                return
            if self._is_takeoff_land_mode():
                reason = self._takeoff_land_abort_reason
                if not reason and not ok:
                    reason = f"FAILSAFE {command_name} failed"
                self._report_takeoff_land_result(reason)
            if ok and command == FlightCommand.Request.CMD_DISARM:
                self._transition(Phase.DONE)

        self._call_flight_cmd(
            command,
            0.0,
            _on_failsafe_result,
        )

    def _record_origin(self):
        if self.origin_recorded:
            return
        pos = self._get_local_xy()
        if pos is not None:
            self.origin_x = pos[0]
            self.origin_y = pos[1]
        local_z = self._get_local_z()
        if local_z is not None:
            self.origin_z = local_z
        self.origin_yaw = self._get_local_yaw()
        self.origin_range = self._get_rangefinder_m()
        self.origin_recorded = True
        range_text = "none" if self.origin_range is None else f"{self.origin_range:.2f}"
        yaw_text = "none" if self.origin_yaw is None else f"{math.degrees(self.origin_yaw):.1f}deg"
        self.get_logger().info(
            f"Origin recorded: ({self.origin_x:.2f}, {self.origin_y:.2f}) "
            f"z={self.origin_z:.2f} yaw={yaw_text} range={range_text}"
        )

    def _check_stable_and_transition(self, next_phase: Phase, log_msg: str, event: str):
        now = self.get_clock().now()
        if self.hover_start_time is None:
            self.hover_start_time = now
            self.get_logger().info(f"{log_msg}, stabilizing")
            return

        elapsed = (now - self.hover_start_time).nanoseconds / 1e9
        if elapsed >= self.hover_stable_time:
            self.get_logger().info(log_msg)
            self._publish_event(event)
            self._transition(next_phase)

    def _transition(self, new_phase: Phase):
        old = PHASE_NAMES[self.phase]
        new = PHASE_NAMES[new_phase]
        self.get_logger().info(f"Phase: {old} -> {new}")
        self.phase = new_phase
        self.hover_start_time = None
        if new_phase != Phase.WAIT_GRASP:
            self.grasp_start_time = None
        if new_phase != Phase.WAIT_DROP:
            self.drop_start_time = None
        if new_phase != Phase.TAKEOFF:
            self._takeoff_wait_start_time = None
            self._takeoff_delay_start_time = None
        if new_phase != Phase.HOVER_TAKEOFF:
            self._takeoff_climb_start_time = None
        if new_phase != Phase.HOVER_LOITER:
            self._loiter_alt_loss_start_time = None
        if new_phase != Phase.MOVE_ABOVE:
            self._move_above_start_time = None
            self._uwb_missing_start_time = None
            self._uwb_out_of_front_start_time = None
            self._uwb_capture_start_time = None
            if new_phase not in (Phase.HOVER_ABOVE, Phase.LAND):
                self._uwb_target_captured = False
        if new_phase != Phase.FORWARD:
            self._forward_start_time = None
            self._forward_start_xy = None
        if new_phase != Phase.WAYPOINT_OUTBOUND:
            self._waypoint_start_time = None
            if new_phase != Phase.HOVER_WAYPOINT:
                self._waypoint_target_xy = None
        if new_phase != Phase.WAYPOINT_RETURN:
            self._waypoint_return_start_time = None
        if new_phase != Phase.LAND_WAIT:
            self._land_wait_start_time = None
            self._land_ground_start_time = None
            self._land_disarm_sent = False
            self._land_disarm_retry_count = 0
            self._land_disarm_retry_time = None

    def _publish_velocity(
        self,
        vx: float,
        vy: float,
        vz: float,
        frame_id: str = "body",
        immediate: bool = False,
    ):
        now = self.get_clock().now()
        if not immediate:
            dt = max((now - self._last_velocity_time).nanoseconds / 1e9, 1.0 / max(self.control_rate_hz, 1.0))
            max_delta = self.velocity_slew_rate * dt
            last_vx, last_vy, last_vz = self._last_velocity
            vx = clamp(vx, last_vx - max_delta, last_vx + max_delta)
            vy = clamp(vy, last_vy - max_delta, last_vy + max_delta)
            vz = clamp(vz, last_vz - max_delta, last_vz + max_delta)

        msg = TwistStamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = frame_id
        msg.twist.linear.x = vx
        msg.twist.linear.y = vy
        msg.twist.linear.z = vz
        self.vel_pub.publish(msg)

        self._last_velocity = (vx, vy, vz)
        self._last_velocity_time = now

    def _publish_state(self):
        msg = String()
        msg.data = PHASE_NAMES[self.phase]
        self.state_pub.publish(msg)

    def _publish_event(self, event: str):
        msg = String()
        msg.data = event
        self.event_pub.publish(msg)

    def _publish_preflight_event(self, event: str):
        now = self.get_clock().now()
        elapsed = (now - self._last_preflight_event_time).nanoseconds / 1e9
        if event == self._last_preflight_event and elapsed < 5.0:
            return
        self._last_preflight_event = event
        self._last_preflight_event_time = now
        self._publish_event(event)

    def _core_preflight_wait_event(self, fcu_ok: bool, uwb_ok: bool, pose_ok: bool):
        if not fcu_ok:
            return "preflight_wait:fcu"
        if not uwb_ok:
            return "preflight_wait:uwb"
        if not pose_ok:
            return "preflight_wait:local_pose"
        return None

    def _takeoff_land_preflight_wait_event(self, snapshot):
        if not snapshot["fcu_ok"]:
            return "preflight_wait:fcu"
        if not snapshot["rc_ok"]:
            return "preflight_wait:rc_manual_input"
        if self._is_uwb_approach_land_mode() and not snapshot["uwb_ok"]:
            return "preflight_wait:uwb"
        if not snapshot["pose_ok"]:
            return "preflight_wait:local_pose"
        if not snapshot["range_ok"]:
            return "preflight_wait:rangefinder"
        if not snapshot["flow_ok"]:
            return "preflight_wait:optical_flow"
        if not snapshot["set_mode_ok"]:
            return "preflight_wait:set_mode_service"
        return None

    def _call_flight_cmd(self, command: int, param: float, callback):
        if not self.flight_cmd_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Flight command service not ready")
            callback(False, "service not ready")
            return

        req = FlightCommand.Request()
        req.command = command
        req.param = param
        future = self.flight_cmd_client.call_async(req)

        def _done(fut):
            try:
                result = fut.result()
                callback(result.success, result.message)
            except Exception as exc:
                self.get_logger().error(f"Service call exception: {exc}")
                callback(False, str(exc))

        future.add_done_callback(_done)

    def _call_mavros_set_mode(self, mode_str: str, warn_only: bool = False):
        if self.use_mock:
            return
        if not self.mavros_set_mode_client.wait_for_service(
            timeout_sec=self.set_mode_service_timeout_sec
        ):
            if self.mission_mode == "bench_velocity" and mode_str == "GUIDED":
                self._bench_guided_ok = False
            log = self.get_logger().warn if warn_only else self.get_logger().error
            log("MAVROS set_mode service not available")
            return
        req = SetMode.Request()
        req.custom_mode = mode_str
        future = self.mavros_set_mode_client.call_async(req)

        def done(fut):
            try:
                result = fut.result()
                if self.mission_mode == "bench_velocity" and mode_str == "GUIDED":
                    self._bench_guided_ok = bool(result.mode_sent)
                self.get_logger().info(f"MAVROS SetMode({mode_str}): mode_sent={result.mode_sent}")
            except Exception as exc:
                if self.mission_mode == "bench_velocity" and mode_str == "GUIDED":
                    self._bench_guided_ok = False
                log = self.get_logger().warn if warn_only else self.get_logger().error
                log(f"SetMode failed: {exc}")

        future.add_done_callback(done)

    def _start_mode_switch(self, mode_str: str, next_phase: Phase, warn_only: bool = False):
        if self.use_mock:
            self._transition(next_phase)
            return

        self._pending_mode_target = mode_str
        self._pending_mode_next_phase = next_phase
        self._pending_mode_future = None
        self._pending_mode_started = self.get_clock().now()
        self._pending_mode_request_sent = False
        self._pending_mode_warn_only = warn_only
        self.get_logger().info(
            f"Requesting FCU mode {mode_str} before {PHASE_NAMES[next_phase]}"
        )

    def _tick_pending_mode_switch(self):
        target = self._pending_mode_target
        next_phase = self._pending_mode_next_phase
        now = self.get_clock().now()
        elapsed = (now - self._pending_mode_started).nanoseconds / 1e9

        if self._get_fcu_mode() == target:
            self.get_logger().info(f"FCU mode confirmed: {target}")
            if self.mission_mode == "bench_velocity" and target == "GUIDED":
                self._bench_guided_ok = True
            if self._is_takeoff_loiter_land_mode() and target == "LOITER":
                self._takeoff_land_loiter_ok = True
            if (
                self._is_takeoff_loiter_land_mode()
                and target == "GUIDED"
                and next_phase == Phase.LAND
            ):
                self._takeoff_land_guided_return_ok = True
            self._clear_pending_mode_switch()
            if next_phase == Phase.BENCH_VELOCITY:
                self.bench_start_time = self.get_clock().now()
            self._transition(next_phase)
            return

        if not self._pending_mode_request_sent:
            if not self.mavros_set_mode_client.service_is_ready():
                if elapsed >= self.set_mode_service_timeout_sec:
                    self._finish_mode_switch_failure(
                        f"MAVROS set_mode service not ready after {elapsed:.1f}s"
                    )
                else:
                    self.get_logger().warn(
                        f"Waiting for MAVROS set_mode service before {target}...",
                        throttle_duration_sec=1.0,
                    )
                return

            req = SetMode.Request()
            req.custom_mode = target
            self._pending_mode_future = self.mavros_set_mode_client.call_async(req)
            self._pending_mode_request_sent = True
            self.get_logger().info(f"MAVROS SetMode({target}) request sent")
            return

        if self._pending_mode_future is not None and self._pending_mode_future.done():
            try:
                result = self._pending_mode_future.result()
                if not result.mode_sent:
                    self._finish_mode_switch_failure(
                        f"MAVROS SetMode({target}) rejected: mode_sent=False"
                    )
                    return
                self.get_logger().info(
                    f"MAVROS SetMode({target}) accepted, waiting for FCU state"
                )
                self._pending_mode_future = None
            except Exception as exc:
                self._finish_mode_switch_failure(f"SetMode({target}) failed: {exc}")
                return

        if elapsed >= self.mode_confirm_timeout_sec:
            self._finish_mode_switch_failure(
                f"FCU mode did not become {target} within {elapsed:.1f}s "
                f"(current={self._get_fcu_mode() or '-'})"
            )

    def _clear_pending_mode_switch(self):
        self._pending_mode_target = None
        self._pending_mode_next_phase = None
        self._pending_mode_future = None
        self._pending_mode_started = None
        self._pending_mode_request_sent = False
        self._pending_mode_warn_only = False

    def _finish_mode_switch_failure(self, reason: str):
        target = self._pending_mode_target
        warn_only = self._pending_mode_warn_only
        self._clear_pending_mode_switch()
        if self.mission_mode == "bench_velocity" and target == "GUIDED":
            self._bench_guided_ok = False
        log = self.get_logger().warn if warn_only else self.get_logger().error
        log(reason)
        self._publish_event(f"mode_switch_failed:{target}")

        if self.mission_mode == "bench_velocity":
            self._publish_velocity(0.0, 0.0, 0.0)
            self._report_bench_result(reason)
            if not self._desktop_disarm_done and not self._pending_command:
                self._pending_command = True
                self._call_flight_cmd(
                    FlightCommand.Request.CMD_DISARM, 0.0, self._on_bench_disarm
                )
            else:
                self._transition(Phase.FAILSAFE)
            return

        self._publish_velocity(0.0, 0.0, 0.0)
        if self._is_takeoff_land_mode():
            self._report_takeoff_land_result(reason)
        self._transition(Phase.FAILSAFE)

    def _start_mav_frame_switch(self, frame: str, next_phase: Phase):
        if self.use_mock:
            self._transition(next_phase)
            return

        self._pending_mav_frame = frame
        self._pending_mav_frame_next_phase = next_phase
        self._pending_mav_frame_future = None
        self._pending_mav_frame_started = self.get_clock().now()
        self._pending_mav_frame_request_sent = False
        self.get_logger().info(
            f"Requesting MAVROS setpoint_velocity mav_frame={frame} before {PHASE_NAMES[next_phase]}"
        )

    def _tick_pending_mav_frame_switch(self):
        frame = self._pending_mav_frame
        next_phase = self._pending_mav_frame_next_phase
        now = self.get_clock().now()
        elapsed = (now - self._pending_mav_frame_started).nanoseconds / 1e9

        if not self._pending_mav_frame_request_sent:
            if not self.mavros_setpoint_velocity_param_client.service_is_ready():
                if elapsed >= self.set_mode_service_timeout_sec:
                    self._finish_mav_frame_switch_failure(
                        f"MAVROS setpoint_velocity parameter service not ready after {elapsed:.1f}s"
                    )
                else:
                    self.get_logger().warn(
                        "Waiting for MAVROS setpoint_velocity parameter service...",
                        throttle_duration_sec=1.0,
                    )
                return

            param = Parameter()
            param.name = "mav_frame"
            param.value = ParameterValue()
            param.value.type = ParameterType.PARAMETER_STRING
            param.value.string_value = frame
            req = SetParameters.Request()
            req.parameters = [param]
            self._pending_mav_frame_future = self.mavros_setpoint_velocity_param_client.call_async(req)
            self._pending_mav_frame_request_sent = True
            self.get_logger().info(f"MAVROS setpoint_velocity mav_frame={frame} request sent")
            return

        if self._pending_mav_frame_future is not None and self._pending_mav_frame_future.done():
            try:
                result = self._pending_mav_frame_future.result()
                if not result.results or not result.results[0].successful:
                    reason = "unknown"
                    if result.results:
                        reason = result.results[0].reason or "rejected"
                    self._finish_mav_frame_switch_failure(
                        f"MAVROS setpoint_velocity mav_frame={frame} rejected: {reason}"
                    )
                    return
                self.get_logger().info(f"MAVROS setpoint_velocity mav_frame confirmed: {frame}")
                self._clear_pending_mav_frame_switch()
                self._transition(next_phase)
            except Exception as exc:
                self._finish_mav_frame_switch_failure(
                    f"MAVROS setpoint_velocity mav_frame={frame} failed: {exc}"
                )
            return

        if elapsed >= self.mode_confirm_timeout_sec:
            self._finish_mav_frame_switch_failure(
                f"MAVROS setpoint_velocity mav_frame={frame} did not complete within {elapsed:.1f}s"
            )

    def _clear_pending_mav_frame_switch(self):
        self._pending_mav_frame = None
        self._pending_mav_frame_next_phase = None
        self._pending_mav_frame_future = None
        self._pending_mav_frame_started = None
        self._pending_mav_frame_request_sent = False

    def _finish_mav_frame_switch_failure(self, reason: str):
        self._clear_pending_mav_frame_switch()
        self.get_logger().error(reason)
        self._publish_event("mav_frame_switch_failed")
        self._publish_velocity(0.0, 0.0, 0.0)
        if self._is_takeoff_land_mode():
            self._report_takeoff_land_result(reason)
        self._transition(Phase.FAILSAFE)


def main(args=None):
    rclpy.init(args=args)
    node = TestMissionNode()
    try:
        while rclpy.ok() and not node.should_exit():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
