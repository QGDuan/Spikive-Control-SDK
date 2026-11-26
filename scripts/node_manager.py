#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
spikive_manager / node_manager.py
--------------------------------

Per-drone supervisor for the Spikive ROS1 stack.

Responsibilities
================
- Start and stop logical "launches" such as MavRos, Lidar_Driver, LIO, and Planner.
- Publish a 1 Hz AutoManager status that the Lichtblick frontend uses to
  drive the Robot card's Start / Stop button.
- Refuse start/shutdown while the drone is armed (/mavros/state.armed).
- Keep its own log stream isolated from rosout so SLAM/Planner spam does
  not drown out manager events.
- Manage child processes via Popen + setsid, allowing reliable group kill.

Topics
======
- /drone_{id}_command_topic         (sub)  spikive_control_sdk/Command
- /drone_{id}_auto_manager_status   (pub)  spikive_control_sdk/AutoManager
- /mavros/state                     (sub)  mavros_msgs/State

Logs (per-drone)
================
~/spikive_logs/drone_{id}/
  manager.log               -- this node's events (one file, no rosout)
  drivers/{LaunchName}.log  -- stdout+stderr of each driver child
  algo/{LaunchName}.log     -- stdout+stderr of each algorithm child
"""

from __future__ import annotations

import logging
import os
import queue
import signal
import subprocess
import sys
import threading
import time
from collections import deque
from typing import Dict, List, Optional, Set

import rospkg
import rospy
import yaml

from spikive_control_sdk.msg import AutoManager, Command
from spikive_control_sdk.srv import ManageLaunch, ManageLaunchRequest, ManageLaunchResponse

try:
    from mavros_msgs.msg import State as MavrosState
    _MAVROS_AVAILABLE = True
except ImportError:  # mavros_msgs may not be installed at build time
    MavrosState = None
    _MAVROS_AVAILABLE = False


# ----- constants -------------------------------------------------------------

STATUS_PUBLISH_HZ = 1.0
ROSNODE_LIST_INTERVAL = 1.0          # seconds between rosnode list polls during readiness wait
HEALTH_POLL_INTERVAL = 2.0           # background health update period
GRACE_SHUTDOWN_SEC = 5.0             # SIGINT -> SIGKILL grace period for child groups
INTER_LAUNCH_DELAY_SEC = 0.5         # small breathing room between sequential launches
MAVROS_STATE_TOPIC = "/mavros/state"  # mandatory aircraft-internal safety interlock source
MAVROS_STATE_MAX_AGE_SEC = 3.0       # command-time armed check freshness window

# Status integers for AutoManager subsystem fields.
ST_NOT_RUNNING = 0
ST_PARTIAL = 1
ST_READY = 2

# Mode strings broadcast in AutoManager.mode.
MODE_IDLE = "idle"
MODE_STARTING = "starting"
MODE_READY = "ready"
MODE_STOPPING = "stopping"
MODE_ERROR = "error"


# ----- color terminal formatter (manager's own console only) -----------------


class _ColorFormatter(logging.Formatter):
    _COLORS = {
        "DEBUG": "\033[36m",
        "INFO": "\033[32m",
        "WARNING": "\033[33m",
        "ERROR": "\033[31m",
        "CRITICAL": "\033[1;41m",
    }
    _RESET = "\033[0m"

    def format(self, record: logging.LogRecord) -> str:
        color = self._COLORS.get(record.levelname, "")
        msg = super().format(record)
        return f"{color}{msg}{self._RESET}"


def _setup_logger(drone_id: str, log_root: str) -> logging.Logger:
    """Manager logger that does NOT propagate to rosout, so SLAM/Planner spam
    via /rosout cannot drown out manager events. Writes a dedicated file plus
    color-coded stderr."""
    log_dir = os.path.join(os.path.expanduser(log_root), f"drone_{drone_id}")
    os.makedirs(log_dir, exist_ok=True)

    logger = logging.getLogger(f"spikive_manager.drone_{drone_id}")
    logger.setLevel(logging.DEBUG)
    logger.propagate = False  # critical: stay out of rosout

    # File handler -- everything.
    fh = logging.FileHandler(os.path.join(log_dir, "manager.log"))
    fh.setLevel(logging.DEBUG)
    fh.setFormatter(
        logging.Formatter("%(asctime)s [%(levelname)s] %(message)s")
    )
    logger.addHandler(fh)

    # Console handler -- INFO+, colored.
    ch = logging.StreamHandler(sys.stderr)
    ch.setLevel(logging.INFO)
    ch.setFormatter(_ColorFormatter("%(asctime)s [%(levelname)s] %(message)s"))
    logger.addHandler(ch)

    return logger


# ----- ManagedProcess --------------------------------------------------------


class ManagedProcess:
    """Wraps a single subprocess.Popen that runs a roslaunch (or any shell)
    command. Each instance owns its log fd so output never mixes with
    other launches or with the manager's own log."""

    def __init__(self, name: str, cmd: str, log_path: str, logger: logging.Logger):
        self.name = name
        self.cmd = cmd
        self.log_path = log_path
        self.log = logger
        self._popen: Optional[subprocess.Popen] = None
        self._log_fd = None
        self.started_at: Optional[float] = None

    def is_alive(self) -> bool:
        child = self._popen
        return child is not None and child.poll() is None

    def has_group(self) -> bool:
        child = self._popen
        if child is None:
            return False
        child.poll()  # Reap the leader before checking its process group.
        try:
            os.killpg(child.pid, 0)
            return True
        except ProcessLookupError:
            return False

    def start(self) -> None:
        if self.is_alive():
            self.log.warning("ManagedProcess[%s] already running, skip start", self.name)
            return
        self._cleanup_fd()
        os.makedirs(os.path.dirname(self.log_path), exist_ok=True)
        self._log_fd = open(self.log_path, "ab", buffering=0)
        self._log_fd.write(
            f"\n===== {time.strftime('%Y-%m-%d %H:%M:%S')} starting: {self.cmd} =====\n".encode()
        )
        self._popen = subprocess.Popen(
            self.cmd,
            shell=True,
            executable="/bin/bash",
            stdout=self._log_fd,
            stderr=subprocess.STDOUT,
            start_new_session=True,  # own process group -> killpg works cleanly
            close_fds=True,
        )
        self.started_at = time.time()
        self.log.info("Started [%s] pid=%d (log=%s)", self.name, self._popen.pid, self.log_path)

    def stop(self, grace_sec: float = GRACE_SHUTDOWN_SEC) -> None:
        if self._popen is None:
            return
        pgid = self._popen.pid  # start_new_session makes the child's PID its PGID.
        for sig, timeout in ((signal.SIGINT, grace_sec), (signal.SIGKILL, 2.0)):
            if not self.has_group():
                break
            self.log.info("Stopping [%s] pgid=%d signal=%s", self.name, pgid, sig)
            try:
                os.killpg(pgid, sig)
            except ProcessLookupError:
                break
            deadline = time.monotonic() + timeout
            while self.has_group() and time.monotonic() < deadline:
                time.sleep(0.1)
        if self.has_group():
            raise RuntimeError(f"Process group for '{self.name}' is still present")
        self._cleanup_fd()
        self._popen = None
        self.started_at = None

    def _cleanup_fd(self) -> None:
        if self._log_fd is not None:
            try:
                self._log_fd.write(
                    f"===== {time.strftime('%Y-%m-%d %H:%M:%S')} stopped =====\n\n".encode()
                )
                self._log_fd.close()
            except Exception:
                pass
            self._log_fd = None


# ----- NodeManager -----------------------------------------------------------


class NodeManager:
    def __init__(self) -> None:
        rospy.init_node("node_manager", anonymous=False)

        self.drone_id: str = str(rospy.get_param("~drone_id", "")).strip()
        if not self.drone_id:
            rospy.logfatal("spikive_manager: ~drone_id is required")
            raise SystemExit(2)

        log_root = rospy.get_param("~log_root", "~/spikive_logs")
        self.log_root = os.path.expanduser(str(log_root))
        self.log = _setup_logger(self.drone_id, self.log_root)

        # Topic names follow the existing /drone_{id}_<base> convention.
        self.cmd_topic = f"/drone_{self.drone_id}_command_topic"
        self.status_topic = f"/drone_{self.drone_id}_auto_manager_status"
        # MAVROS state is internal to this aircraft computer and is not
        # drone-id namespaced. The frontend consumes only AutoManager.armed.
        self.mavros_state_topic = MAVROS_STATE_TOPIC

        self.log.info(
            "spikive_manager booting drone_id=%s cmd=%s status=%s mavros_state=%s",
            self.drone_id, self.cmd_topic, self.status_topic, self.mavros_state_topic,
        )

        # ---- config ---------------------------------------------------------
        config_path = rospy.get_param(
            "~config_path",
            os.path.join(rospkg.RosPack().get_path("spikive_control_sdk"), "config", "manager", "launch_config.yaml"),
        )
        self.launches: Dict[str, dict] = self._load_config(config_path)
        # Topological order of launch names (raises if a cycle is detected).
        self.launch_order: List[str] = self._topo_sort(self.launches)
        self.log.info("Launch order: %s", " -> ".join(self.launch_order))

        # ---- process registry ----------------------------------------------
        self.processes: Dict[str, ManagedProcess] = {}
        for name, cfg in self.launches.items():
            log_path = os.path.join(
                self.log_root, f"drone_{self.drone_id}",
                cfg.get("group", "misc"), f"{name}.log",
            )
            cmd = cfg["launch_cmd"].replace("${DRONE_ID}", self.drone_id)
            cfg["nodes"] = [n.replace("${DRONE_ID}", self.drone_id) for n in cfg.get("nodes", [])]
            self.processes[name] = ManagedProcess(
                name=name, cmd=cmd, log_path=log_path, logger=self.log,
            )

        # ---- runtime state --------------------------------------------------
        self._lock = threading.RLock()
        self.mode: str = MODE_IDLE
        self.is_active: bool = False
        self.starting: bool = False
        self.stopping: bool = False
        self.armed: bool = False
        self.mavros_connected: bool = False
        self.mavros_guided: bool = False
        self.mavros_manual_input: bool = False
        self.mavros_mode: str = ""
        self.mavros_system_status: int = 0
        self.mavros_state_stamp: str = ""
        self.mavros_state_rx_time: Optional[float] = None
        self._mavros_state_seen: bool = False
        self._last_mavros_log_snapshot = None
        self.last_error: str = ""
        self.last_error_seq: int = 0

        # Cache of the most recent rosnode list output (refreshed by health thread).
        self._running_nodes: Set[str] = set()
        self._running_nodes_lock = threading.Lock()

        # Single-consumer command queue: ROS callback enqueues, worker thread executes.
        # This prevents Popen + sleep loops from blocking the rospy callback queue.
        self.cmd_queue: "queue.Queue[Optional[Command]]" = queue.Queue(maxsize=1)
        self._last_command: Command = Command()

        # Reserve the lifecycle slot before ACK so service calls cannot overtake queued commands.
        self._operation_lock = threading.Lock()
        self._stop_evt = threading.Event()
        self._refresh_lock = threading.Lock()
        self._refresh_running_nodes()
        self._recover_state()

        # ---- ROS I/O --------------------------------------------------------
        self._status_pub = rospy.Publisher(self.status_topic, AutoManager, queue_size=10)

        if _MAVROS_AVAILABLE:
            self._mavros_state_sub = rospy.Subscriber(
                self.mavros_state_topic, MavrosState, self._on_mavros_state, queue_size=4,
            )
        else:
            self.log.critical(
                "mavros_msgs not importable; mandatory %s armed interlock unavailable",
                self.mavros_state_topic,
            )
            raise SystemExit(4)

        # ---- threads --------------------------------------------------------
        self._worker = threading.Thread(target=self._worker_loop, name="cmd_worker", daemon=True)
        self._worker.start()
        self._health = threading.Thread(target=self._health_loop, name="health_loop", daemon=True)
        self._health.start()
        self._status_timer = rospy.Timer(
            rospy.Duration(1.0 / STATUS_PUBLISH_HZ), self._publish_status,
        )
        rospy.on_shutdown(self._shutdown_hook)

        self._cmd_sub = rospy.Subscriber(self.cmd_topic, Command, self._on_command, queue_size=4)
        self._service = rospy.Service(f"/drone_{self.drone_id}_manage_launch", ManageLaunch, self._manage_launch)

        self.log.info(
            "spikive_manager ready (mode=%s is_active=%s)", self.mode, self.is_active,
        )

    # ------------------------------------------------------------------ config

    def _load_config(self, path: str) -> Dict[str, dict]:
        if not os.path.exists(path):
            self.log.error("Config file not found: %s", path)
            raise SystemExit(3)
        with open(path, "r") as f:
            data = yaml.safe_load(f) or {}
        launches = data.get("launches", {}) or {}
        if not launches:
            self.log.error("Config has no 'launches' section: %s", path)
            raise SystemExit(3)
        for name, cfg in launches.items():
            if "launch_cmd" not in cfg:
                self.log.error("Launch '%s' missing 'launch_cmd'", name)
                raise SystemExit(3)
            cfg.setdefault("nodes", [])
            cfg.setdefault("group", "misc")
            cfg.setdefault("depends_on", [])
            cfg.setdefault("start_timeout", 8.0)
            cfg.setdefault("ready_delay", 0.0)
        self.log.info("Loaded %d launches from %s", len(launches), path)
        return launches

    @staticmethod
    def _topo_sort(launches: Dict[str, dict]) -> List[str]:
        """Kahn's algorithm. Stable order for deterministic startup."""
        in_deg: Dict[str, int] = {n: 0 for n in launches}
        edges: Dict[str, List[str]] = {n: [] for n in launches}
        for name, cfg in launches.items():
            for dep in cfg.get("depends_on", []):
                if dep not in launches:
                    raise ValueError(f"Launch '{name}' depends on unknown '{dep}'")
                edges[dep].append(name)
                in_deg[name] += 1
        ordered: List[str] = []
        ready = deque(sorted(n for n, d in in_deg.items() if d == 0))
        while ready:
            n = ready.popleft()
            ordered.append(n)
            for m in sorted(edges[n]):
                in_deg[m] -= 1
                if in_deg[m] == 0:
                    ready.append(m)
        if len(ordered) != len(launches):
            raise ValueError("Dependency cycle detected in launch_config.yaml")
        return ordered

    # ------------------------------------------------------------------ ROS callbacks

    def _on_command(self, msg: Command) -> None:
        # Admission is intentionally done in the ROS callback before queueing:
        # stale lifecycle commands must not wait behind a long startup and then
        # execute against a different backend state.
        with self._lock:
            self._last_command = msg
        if not self._operation_lock.acquire(blocking=False):
            self._set_error("REJECTED: Manager busy", msg.command_type)
            return
        queued = False
        try:
            reason = self._command_reject_reason(msg)
            if reason:
                self._set_error(reason, msg.command_type)
                return
            self._publish_status_immediate()
            self.cmd_queue.put_nowait(msg)
            queued = True
        finally:
            if not queued:
                self._operation_lock.release()

    def _on_mavros_state(self, msg) -> None:  # MavrosState (or None at type-check time)
        stamp = getattr(getattr(msg, "header", None), "stamp", None)
        stamp_text = "unset"
        if stamp is not None:
            stamp_text = f"{getattr(stamp, 'secs', 0)}.{getattr(stamp, 'nsecs', 0):09d}"

        snapshot = (
            bool(getattr(msg, "connected", False)),
            bool(getattr(msg, "armed", False)),
            bool(getattr(msg, "guided", False)),
            bool(getattr(msg, "manual_input", False)),
            str(getattr(msg, "mode", "")),
            int(getattr(msg, "system_status", 0)),
            stamp_text,
        )
        log_snapshot = snapshot[:-1]

        should_log = False
        armed_changed = False
        first_state = False
        with self._lock:
            first_state = not self._mavros_state_seen
            armed_changed = snapshot[1] != self.armed
            self.mavros_connected = snapshot[0]
            self.armed = snapshot[1]
            self.mavros_guided = snapshot[2]
            self.mavros_manual_input = snapshot[3]
            self.mavros_mode = snapshot[4]
            self.mavros_system_status = snapshot[5]
            self.mavros_state_stamp = snapshot[6]
            self.mavros_state_rx_time = time.time()
            if not self._mavros_state_seen or log_snapshot != self._last_mavros_log_snapshot:
                should_log = True
            self._mavros_state_seen = True
            self._last_mavros_log_snapshot = log_snapshot

        if armed_changed:
            self.log.warning("MAVROS armed state changed: %s", self._format_mavros_state())
            self._publish_status_immediate()
        elif should_log:
            self.log.info("MAVROS state: %s", self._format_mavros_state())
            if first_state:
                self._publish_status_immediate()

    # ------------------------------------------------------------------ worker

    def _worker_loop(self) -> None:
        while True:
            msg = self.cmd_queue.get()
            if msg is None:
                return
            try:
                self._handle_command(msg)
            except Exception as e:  # pylint: disable=broad-except
                self.log.exception("Command handler crashed: %s", e)
                self._set_error(f"Internal error: {e!r}", msg.command_type)
            finally:
                self._operation_lock.release()

    def _command_reject_reason(self, msg: Command) -> Optional[str]:
        ctype = (msg.command_type or "").strip()
        if ctype not in ("start_all", "shutdown_all"):
            return f"Unknown command_type: {ctype!r}"

        if ctype == "start_all" and self.is_active:
            return "REJECTED: already running"
        if ctype == "shutdown_all" and not self.is_active and not any(
            p.has_group() for p in self.processes.values()
        ):
            return "REJECTED: no owned launch to stop"
        return self._interlock(ctype)

    def _interlock(self, operation: str) -> Optional[str]:
        if self._stop_evt.is_set():
            return "Manager is shutting down"
        with self._lock:
            fresh = self._is_mavros_state_fresh()
            if fresh and self.armed:
                return f"REJECTED: drone armed, aborting {operation}"
            if not fresh and operation in ("shutdown_all", "stop"):
                return f"REJECTED: MAVROS state missing or stale, aborting {operation}"
        return None

    def _manage_launch(self, req):
        name = req.launch_name
        if name not in self.launches:
            return ManageLaunchResponse(False, f"Unknown launch: {name}", [])
        if req.operation not in (ManageLaunchRequest.START, ManageLaunchRequest.STOP):
            return ManageLaunchResponse(False, f"Unknown operation: {req.operation}", [])
        if not self._operation_lock.acquire(blocking=False):
            return ManageLaunchResponse(False, "Manager busy", [])
        starting = req.operation == ManageLaunchRequest.START
        try:
            if not self._refresh_running_nodes():
                return ManageLaunchResponse(False, "Cannot query ROS node registry", [])
            reason = self._interlock("start" if starting else "stop")
            if reason:
                return ManageLaunchResponse(False, reason, [])
            if starting:
                missing = [dep for dep in self.launches[name]["depends_on"]
                           if self._check_subsystem(dep) != ST_READY]
                if missing:
                    return ManageLaunchResponse(False, "Dependencies not ready", missing)
            self._set_transition(starting)
            error = self._operate_launch(name, starting)
            error = self._finish_transition(error)
            return ManageLaunchResponse(not error, error or f"{name} {'ready' if starting else 'stopped'}", [])
        except Exception as exc:
            error = f"Cannot manage '{name}': {exc}"
            self._finish_transition(error)
            return ManageLaunchResponse(False, error, [])
        finally:
            self._operation_lock.release()

    def _set_transition(self, starting):
        with self._lock:
            self.starting = starting
            self.stopping = not starting
            self.mode = MODE_STARTING if starting else MODE_STOPPING
        self._publish_status_immediate()

    def _finish_transition(self, error=None):
        known = self._refresh_running_nodes()
        with self._lock:
            self.starting = self.stopping = False
            self._update_observed_state()
            if not known and not error:
                error = "Cannot query ROS node registry"
            if error:
                self._set_error(error, "lifecycle")
                return error
            else:
                self.last_error = ""
        self._publish_status_immediate()
        return None

    def _operate_launch(self, name, starting):
        process = self.processes[name]
        status = self._check_subsystem(name)
        owned = process.has_group()
        if not owned and status != ST_NOT_RUNNING:
            return f"'{name}' has external nodes; Manager does not own them"
        if starting:
            if owned:
                if process.is_alive() and status == ST_READY:
                    return None
                return f"'{name}' is partially running; stop it before restarting"
            return self._start_launch(name)
        else:
            process.stop()
            # A separately owned roslaunch child may take time to unregister.
            deadline = time.monotonic() + GRACE_SHUTDOWN_SEC
            while True:
                if not self._refresh_running_nodes():
                    return "Cannot query ROS node registry"
                if self._check_subsystem(name) == ST_NOT_RUNNING:
                    return None
                if time.monotonic() >= deadline:
                    return f"'{name}' still has registered nodes after stopping"
                time.sleep(0.2)
        return None

    def _handle_command(self, msg: Command) -> None:
        ctype = (msg.command_type or "").strip()

        # Keep a second guard here because state may change after admission.
        reject_reason = self._command_reject_reason(msg)
        if reject_reason is not None:
            self._set_error(reject_reason, ctype)
            return

        if ctype == "start_all":
            self._start_all()
        elif ctype == "shutdown_all":
            self._shutdown_all()
        else:
            self._set_error(f"Unknown command_type: {ctype!r}", ctype)

    # ------------------------------------------------------------------ lifecycle

    def _start_all(self) -> None:
        """Start every launch in topological order with readiness checks."""
        self._set_transition(True)
        error = None
        try:
            for name in self.launch_order:
                if not self._refresh_running_nodes():
                    error = "Cannot query ROS node registry"
                    break
                error = self._interlock("start_all") or self._operate_launch(name, True)
                if error:
                    break
                self._stop_evt.wait(INTER_LAUNCH_DELAY_SEC)
        except Exception as exc:
            error = str(exc)
        finally:
            self._finish_transition(error)

    def _shutdown_all(self) -> None:
        self._set_transition(False)
        errors = []
        try:
            for name in reversed(self.launch_order):
                if not self._refresh_running_nodes():
                    errors.append("Cannot query ROS node registry")
                    break
                try:
                    error = self._operate_launch(name, False)
                except Exception as exc:
                    error = str(exc)
                if error:
                    errors.append(error)
        finally:
            self._finish_transition(", ".join(errors))

    def _start_launch(self, name: str) -> Optional[str]:
        config = self.launches[name]
        process = self.processes[name]
        process.start()
        timeout = float(config["start_timeout"])
        deadline = time.monotonic() + timeout
        while not self._stop_evt.is_set():
            if not process.is_alive():
                error = f"'{name}' exited during startup"
                break
            if self._refresh_running_nodes() and self._check_subsystem(name) == ST_READY:
                delay = float(config["ready_delay"])
                if self._stop_evt.wait(delay):
                    error = "Manager is shutting down"
                    break
                if process.is_alive() and self._refresh_running_nodes() and self._check_subsystem(name) == ST_READY:
                    self.log.info("'%s' READY", name)
                    return None
                error = f"'{name}' lost readiness during startup"
                break
            if time.monotonic() >= deadline:
                with self._running_nodes_lock:
                    missing = sorted(set(config["nodes"]) - self._running_nodes)
                error = f"'{name}' not READY after {timeout:.1f}s (missing={missing})"
                break
            self._stop_evt.wait(ROSNODE_LIST_INTERVAL)
        else:
            error = "Manager is shutting down"
        process.stop()
        return error

    # ------------------------------------------------------------------ health

    def _health_loop(self) -> None:
        while not self._stop_evt.is_set() and not rospy.is_shutdown():
            self._refresh_running_nodes()
            with self._lock:
                if not self.starting and not self.stopping:
                    previous_mode = self.mode
                    self._update_observed_state()
                    if previous_mode == MODE_ERROR:
                        self.mode = MODE_ERROR
            self._stop_evt.wait(HEALTH_POLL_INTERVAL)

    def _refresh_running_nodes(self) -> bool:
        with self._refresh_lock:
            return self._read_registry()

    def _read_registry(self) -> bool:
        known = True
        try:
            out = subprocess.check_output(
                ["rosnode", "list"], stderr=subprocess.DEVNULL, timeout=3.0,
            ).decode("utf-8", errors="replace")
            running = set(line.strip() for line in out.splitlines() if line.strip())
        except (subprocess.CalledProcessError, subprocess.TimeoutExpired, FileNotFoundError):
            known = False
            running = set()
        with self._running_nodes_lock:
            self._running_nodes = running
        return known

    def _recover_state(self) -> None:
        """If manager itself was restarted while children are still running,
        figure out current state from rosnode list."""
        with self._lock:
            self._update_observed_state()
            if self.mode == "partial":
                self.last_error = "Recovered with external nodes running"
                self.last_error_seq += 1

    def _update_observed_state(self):
        states = [self._check_subsystem(name) for name in self.launch_order]
        self.is_active = all(state == ST_READY for state in states)
        if self.is_active:
            self.mode = MODE_READY
        elif any(states) or any(p.has_group() for p in self.processes.values()):
            self.mode = "partial"
        else:
            self.mode = MODE_IDLE

    # ------------------------------------------------------------------ status

    def _check_subsystem(self, launch_name: str) -> int:
        nodes = self.launches[launch_name].get("nodes", [])
        if not nodes:
            return ST_READY if self.processes[launch_name].is_alive() else ST_NOT_RUNNING
        with self._running_nodes_lock:
            present = sum(1 for n in nodes if n in self._running_nodes)
        if present == 0:
            return ST_NOT_RUNNING
        if present < len(nodes):
            return ST_PARTIAL
        return ST_READY

    def _build_status_msg(self) -> AutoManager:
        m = AutoManager()
        m.header.stamp = rospy.Time.now()
        m.header.frame_id = f"drone_{self.drone_id}"
        if hasattr(m, "drone_id"):
            m.drone_id = self.drone_id
        with self._lock:
            m.mode = self.mode
            m.is_active = self.is_active
            m.starting = self.starting
            if hasattr(m, "stopping"):
                m.stopping = self.stopping
            m.armed = self.armed
            m.last_error = self.last_error
            m.last_error_seq = self.last_error_seq
        m.odometry_status = 0  # not tracked yet
        m.mavros_status = self._check_subsystem("MavRos") if "MavRos" in self.launches else 0
        m.cam_driver_status = (
            self._check_subsystem("Camera_Driver") if "Camera_Driver" in self.launches else 0
        )
        m.lidar_driver_status = (
            self._check_subsystem("Lidar_Driver") if "Lidar_Driver" in self.launches else 0
        )
        m.slam_status = self._check_subsystem("LIO") if "LIO" in self.launches else 0
        m.planner_status = self._check_subsystem("Planner") if "Planner" in self.launches else 0
        m.ctrl_status = self._check_subsystem("Ctrl") if "Ctrl" in self.launches else 0
        m.command = self._last_command
        return m

    def _publish_status(self, _evt) -> None:
        try:
            self._status_pub.publish(self._build_status_msg())
        except Exception as e:  # pylint: disable=broad-except
            self.log.error("status publish failed: %s", e)

    def _publish_status_immediate(self) -> None:
        try:
            self._status_pub.publish(self._build_status_msg())
        except Exception as e:  # pylint: disable=broad-except
            self.log.error("status publish (immediate) failed: %s", e)

    # ------------------------------------------------------------------ utility

    def _set_error(self, message: str, context: str = "") -> None:
        with self._lock:
            self.last_error_seq += 1
            self.last_error = message
            if not self.starting and not self.stopping:
                self.mode = MODE_ERROR
        self.log.error("[%s] %s (seq=%d)", context, message, self.last_error_seq)
        self._publish_status_immediate()

    def _format_mavros_state(self) -> str:
        with self._lock:
            if not self._mavros_state_seen:
                return (
                    f"mavros topic={self.mavros_state_topic} state=not_received "
                    "armed=unknown"
                )
            age_text = "unknown"
            if self.mavros_state_rx_time is not None:
                age_text = f"{max(0.0, time.time() - self.mavros_state_rx_time):.2f}s"
            return (
                f"mavros topic={self.mavros_state_topic} "
                f"connected={self.mavros_connected} armed={self.armed} "
                f"guided={self.mavros_guided} manual_input={self.mavros_manual_input} "
                f"mode={self.mavros_mode!r} system_status={self.mavros_system_status} "
                f"stamp={self.mavros_state_stamp} rx_age={age_text}"
            )

    def _is_mavros_state_fresh(self) -> bool:
        with self._lock:
            if not self._mavros_state_seen or self.mavros_state_rx_time is None:
                return False
            return time.time() - self.mavros_state_rx_time <= MAVROS_STATE_MAX_AGE_SEC

    def _shutdown_hook(self) -> None:
        self.log.info("ROS shutdown received; tearing down children")
        self._stop_evt.set()
        with self._operation_lock:
            self.cmd_queue.put_nowait(None)
            self._cleanup_children()

    def _cleanup_children(self):
        for name in reversed(self.launch_order):
            try:
                self.processes[name].stop(grace_sec=2.0)
            except Exception:
                pass


# ----- entrypoint ------------------------------------------------------------


def main() -> int:
    try:
        NodeManager()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except SystemExit:
        raise
    except Exception as e:  # pylint: disable=broad-except
        rospy.logfatal(f"spikive_manager crashed: {e!r}")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
