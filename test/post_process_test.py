#!/usr/bin/env python3
"""Real bag -> PCD post-process test on an isolated ROS master."""
import os
import shutil
import subprocess
import sys
import time
import unittest

import actionlib
import rosbag
import rosnode
import rospy
import rostest
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from spikive_control_sdk.msg import PostProcessAction, PostProcessActionFeedback, PostProcessGoal

ROOT = "/tmp/spikive_post_process_test"
OUT = os.path.join(ROOT, "out")
TOPIC = "/test/cloud"
FIELDS = [
    PointField("x", 0, PointField.FLOAT32, 1),
    PointField("y", 4, PointField.FLOAT32, 1),
    PointField("z", 8, PointField.FLOAT32, 1),
    PointField("intensity", 12, PointField.FLOAT32, 1),
]


def wait_for(predicate, seconds=5):
    until = time.monotonic() + seconds
    while time.monotonic() < until:
        if predicate():
            return True
        time.sleep(0.02)
    return False


def write_bag(name, points, topic=TOPIC):
    path = os.path.join(ROOT, name + ".bag")
    bag = rosbag.Bag(path, "w")
    header = Header()
    header.frame_id = "world"
    cloud = point_cloud2.create_cloud(header, FIELDS, points)
    bag.write(topic, cloud, rospy.Time(100, 0))
    bag.close()
    return path


def pcd_point_count(path):
    with open(path, "rb") as stream:
        data = stream.read()
    marker = b"DATA binary\n"
    offset = data.index(marker) + len(marker)
    header = data[:offset].decode("ascii", "replace")
    points = None
    for line in header.splitlines():
        if line.startswith("POINTS "):
            points = int(line.split()[1])
    assert points is not None, header
    assert len(data) > offset
    return points


class PostProcessTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        shutil.rmtree(ROOT, ignore_errors=True)
        os.makedirs(OUT, exist_ok=True)
        cls.client = actionlib.SimpleActionClient("/nav/map/post_process", PostProcessAction)
        assert cls.client.wait_for_server(rospy.Duration(10)), "post-process action server missing"
        cls.feedbacks = []
        cls.feedback_sub = rospy.Subscriber(
            "/nav/map/post_process/feedback", PostProcessActionFeedback, cls.on_feedback
        )
        assert wait_for(lambda: cls.feedback_sub.get_num_connections() > 0)

    @classmethod
    def on_feedback(cls, message):
        cls.feedbacks.append(message.feedback.state)

    def setUp(self):
        type(self).feedbacks = []

    def goal(self, bag_name, map_name, timeout=10):
        goal = PostProcessGoal()
        goal.cmd = 1
        goal.bag_path = ROOT
        goal.bag_name = bag_name
        goal.map_path = OUT
        goal.map_name = map_name
        goal.timeout = timeout
        return goal

    def run_goal(self, goal):
        self.client.send_goal(goal)
        self.assertTrue(self.client.wait_for_result(rospy.Duration(30)))
        return self.client.get_result()

    def test_01_concatenates_and_exports(self):
        write_bag("concat", [(1, 2, 3, 0.5), (4, 5, 6, 1.0)])
        result = self.run_goal(PostProcessGoal(cmd=1, bag_name="concat", map_name="map01", timeout=10))
        self.assertTrue(result.success, result.message)
        self.assertEqual(result.error_code, 0)
        self.assertTrue(os.path.isfile(os.path.join(OUT, "map01.pcd")))
        self.assertEqual(pcd_point_count(os.path.join(OUT, "map01.pcd")), 2)
        self.assertIn(1, self.feedbacks)
        self.assertIn(3, self.feedbacks)

    def test_02_rejects_running_lio(self):
        write_bag("lio", [(1, 2, 3, 0.5)])
        fake = subprocess.Popen(
            [sys.executable, os.path.join(os.path.dirname(__file__), "fake_lio_node.py"), "laserMapping"]
        )
        try:
            self.assertTrue(
                wait_for(lambda: "/laserMapping" in rosnode.get_node_names(), 5),
                "fake LIO node did not start",
            )
            result = self.run_goal(self.goal("lio", "lio"))
            self.assertFalse(result.success)
            self.assertEqual(result.error_code, 12, result.message)
        finally:
            fake.terminate()
            fake.wait(timeout=5)
            wait_for(lambda: "/laserMapping" not in rosnode.get_node_names(), 5)

    def test_03_rejects_missing_topic(self):
        write_bag("other", [(1, 2, 3, 0.5)], "/other/cloud")
        result = self.run_goal(self.goal("other", "other"))
        self.assertFalse(result.success)
        self.assertEqual(result.error_code, 12, result.message)

    def test_04_refuses_existing_output(self):
        write_bag("existing", [(1, 2, 3, 0.5)])
        target = os.path.join(OUT, "existing.pcd")
        with open(target, "w", encoding="utf-8") as stream:
            stream.write("existing")
        result = self.run_goal(self.goal("existing", "existing"))
        self.assertFalse(result.success)
        self.assertEqual(result.error_code, 19, result.message)
        with open(target, "r", encoding="utf-8") as stream:
            self.assertEqual(stream.read(), "existing")


if __name__ == "__main__":
    rospy.init_node("post_process_test")
    rostest.rosrun("spikive_control_sdk", "post_process", PostProcessTest)
