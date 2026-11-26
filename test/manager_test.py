#!/usr/bin/env python3
"""Exercise real process ownership and the public ROS interfaces on a rostest master."""
import os
from pathlib import Path
import shlex
import signal
import subprocess
import tempfile
import time
import unittest
from concurrent.futures import ThreadPoolExecutor

import rosnode
import rospy
import rostest
import yaml
from mavros_msgs.msg import State
from spikive_control_sdk.msg import AutoManager, Command
from spikive_control_sdk.srv import ManageLaunch, ManageLaunchRequest

START = ManageLaunchRequest.START
STOP = ManageLaunchRequest.STOP
PREFIX = "/drone_23_"


class ManagerLifecycle(unittest.TestCase):
    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.root = Path(directory.name)
        self.names = ["LIO", "Planner", "WaypointRecorder", "Localization"]
        dependencies = [[], ["LIO"], ["Planner"], ["LIO", "WaypointRecorder"]]
        launches = {}
        for name, deps in zip(self.names, dependencies):
            node = name.lower()
            script = (f"import os, rospy; from pathlib import Path; "
                      f"Path({str(self.root / node)!r}).write_text(str(os.getpid())); "
                      f"rospy.init_node('fixture_{node}'); rospy.spin()")
            launches[name] = dict(launch_cmd="exec python3 -c " + shlex.quote(script),
                                  nodes=["/fixture_" + node], depends_on=deps, start_timeout=5)
        # An additional criterion lets the test force a partial launch or a timeout.
        if self._testMethodName == "test_interlock_and_timeout":
            launches["LIO"]["nodes"].append("/missing_node")
        else:
            # Both ROS nodes belong to LIO's process group.
            child = launches["LIO"]["launch_cmd"][5:]
            probe = "python3 -c " + shlex.quote("import rospy; rospy.init_node('readiness_probe'); rospy.spin()")
            launches["LIO"]["launch_cmd"] = child + " & " + probe + " & wait"
            launches["LIO"]["nodes"].append("/readiness_probe")
        launches["Planner"]["ready_delay"] = 1.0
        config = self.root / "profile.yaml"
        config.write_text(yaml.safe_dump({"launches": launches}))
        self.manager = subprocess.Popen(["rosrun", "spikive_control_sdk", "node_manager.py",
                                         "_drone_id:=23", "_config_path:=" + str(config),
                                         "_log_root:=" + str(self.root)])
        self.addCleanup(self.stop_process, self.manager)
        self.service = PREFIX + "manage_launch"
        rospy.wait_for_service(self.service, timeout=15)
        self.cmd = rospy.Publisher(PREFIX + "command_topic", Command, queue_size=2)
        self.state = rospy.Publisher("/mavros/state", State, queue_size=1, latch=True)
        self.addCleanup(self.cmd.unregister)
        self.addCleanup(self.state.unregister)

    @staticmethod
    def stop_process(proc):
        if proc.poll() is None:
            proc.send_signal(signal.SIGINT)
            proc.wait(timeout=20)

    def probe(self):
        proc = subprocess.Popen(["python3", "-c", "import rospy; rospy.init_node('readiness_probe'); rospy.spin()"])
        self._probe_process = proc
        self.addCleanup(self.stop_process, proc)
        self.wait_until(lambda: "/readiness_probe" in rosnode.get_node_names())

    def call(self, name, operation):
        return rospy.ServiceProxy(self.service, ManageLaunch)(name, operation)

    def assert_call(self, name, operation):
        result = self.call(name, operation)
        self.assertTrue(result.success, result.message)

    def wait_until(self, condition, timeout=25):
        deadline = time.monotonic() + timeout
        while not condition():
            if time.monotonic() >= deadline:
                self.fail("Timed out waiting for fixture state")
            time.sleep(0.05)

    def status(self):
        return rospy.wait_for_message(PREFIX + "auto_manager_status", AutoManager, timeout=5)

    def keep_disarmed(self):
        timer = rospy.Timer(rospy.Duration(0.2), lambda _: self.state.publish(State(armed=False)))
        self.addCleanup(timer.shutdown)

    def pid(self, name):
        pid = int((self.root / name.lower()).read_text())
        os.kill(pid, 0)
        return pid

    def bulk(self, command, mode):
        request = Command(command_type=command, extra_data=command + str(time.monotonic()))
        self.wait_until(lambda: self.cmd.get_num_connections() > 0)
        self.cmd.publish(request)
        def done():
            status = self.status()
            return status.command.extra_data == request.extra_data and status.mode == mode
        self.wait_until(done)

    def test_dependencies_and_mixed_operations(self):
        self.keep_disarmed()
        # No member, then one of two members, of LIO is present.
        self.assertEqual(list(self.call("Planner", START).unready_dependencies), ["LIO"])
        self.assertFalse((self.root / "planner").exists())
        self.probe()
        self.assertEqual(list(self.call("Planner", START).unready_dependencies), ["LIO"])
        self.assertEqual(list(self.call("Localization", START).unready_dependencies), ["LIO", "WaypointRecorder"])
        self.assertIn("external", self.call("LIO", STOP).message)
        self.stop_process(self._probe_process)
        self.wait_until(lambda: "/readiness_probe" not in rosnode.get_node_names())
        self.assert_call("LIO", START)
        lio_pid = self.pid("LIO")
        with ThreadPoolExecutor(1) as executor:
            pending = executor.submit(self.call, "Planner", START)
            self.wait_until(lambda: (self.root / "planner").exists())
            self.assertIn("busy", self.call("LIO", STOP).message)
            self.assertTrue(pending.result(timeout=15).success)
        self.assert_call("Planner", START)
        planner_pid = self.pid("Planner")
        self.assert_call("WaypointRecorder", START)
        waypoint_pid = self.pid("WaypointRecorder")
        self.assert_call("Planner", STOP)
        self.assertEqual(self.pid("LIO"), lio_pid)
        self.assertEqual(self.pid("WaypointRecorder"), waypoint_pid)
        with self.assertRaises(ProcessLookupError):
            os.kill(planner_pid, 0)
        self.assert_call("Planner", STOP)
        self.bulk("start_backend", "ready")
        self.assertEqual(self.pid("WaypointRecorder"), waypoint_pid)
        planner_pid = self.pid("Planner")
        self.assert_call("LIO", STOP)
        self.assertEqual(self.pid("Planner"), planner_pid)
        self.assertEqual(self.pid("WaypointRecorder"), waypoint_pid)
        self.assertEqual(self.status().mode, "partial")
        self.bulk("stop_backend", "idle")
        self.assertFalse(set(rosnode.get_node_names()) & {"/fixture_" + n.lower() for n in self.names})

    def test_interlock_and_timeout(self):
        self.assertFalse(self.call("unknown", START).success)
        self.assertFalse(self.call("LIO", 99).success)
        self.assertIn("stale", self.call("Planner", STOP).message)
        result = self.call("LIO", START)
        self.assertIn("not READY", result.message)
        self.assertNotIn("/fixture_lio", rosnode.get_node_names())
        self.state.publish(State(armed=True))
        self.wait_until(lambda: self.status().armed)
        for operation in (START, STOP):
            self.assertIn("armed", self.call("LIO", operation).message)
        time.sleep(3.1)
        self.assertIn("stale", self.call("Planner", STOP).message)


if __name__ == "__main__":
    rospy.init_node("manager_lifecycle_test")
    rostest.rosrun("spikive_control_sdk", "manager_lifecycle", ManagerLifecycle)
