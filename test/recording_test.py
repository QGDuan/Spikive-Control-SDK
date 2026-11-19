#!/usr/bin/env python3
"""Real rosbag recording on an isolated master, with no recording backend."""
import io
import os
import shutil
import signal
import tempfile
import time
import unittest
import xmlrpc.client

import actionlib
import rosbag
import rosgraph
import rosnode
import rospy
import rostest
from actionlib_msgs.msg import GoalStatus
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from spikive_control_sdk.msg import RecordAction, RecordActionFeedback, RecordGoal


def wait_for(predicate, seconds=5):
    until = time.monotonic() + seconds
    while time.monotonic() < until:
        if predicate(): return True
        time.sleep(.02)
    return False


def wire(msg):
    stream = io.BytesIO(); msg.serialize(stream); return stream.getvalue()


class Recording(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.root = tempfile.mkdtemp(prefix='sdk bag test ')
        cls.client = actionlib.SimpleActionClient('/nav/map/record', RecordAction)
        assert cls.client.wait_for_server(rospy.Duration(10))
        cls.feedbacks = []
        cls.merge_progress = []
        cls.feedback_sub = rospy.Subscriber('/nav/map/record/feedback', RecordActionFeedback,
            cls.on_feedback)
        assert wait_for(lambda: cls.feedback_sub.get_num_connections() > 0)
        cls.odom = rospy.Publisher('/record_test/odom', Odometry, queue_size=10, latch=True)
        cls.text = rospy.Publisher('/record_test/text', String, queue_size=10, latch=True)
        cls.omitted = rospy.Publisher('/record_test/not_selected', String, queue_size=10, latch=True)
        cls.omitted.publish('must not appear')

    @classmethod
    def on_feedback(cls, message):
        feedback = message.feedback
        cls.feedbacks.append(feedback.state)
        if feedback.state == 2:
            cls.merge_progress.append((feedback.completed_parts, feedback.total_parts))

    @classmethod
    def tearDownClass(cls):
        shutil.rmtree(cls.root)

    def setUp(self):
        self.name = self._testMethodName
        self.target = os.path.join(self.root, self.name + '.bag')
        self.resumed_pid = None

    def tearDown(self):
        if self.resumed_pid:
            try: os.kill(self.resumed_pid, signal.SIGCONT)
            except ProcessLookupError: pass
        # Even after assertion failures, close only our SDK session before the next case.
        if self.worker_pid():
            self.client.cancel_goal()
            time.sleep(.1)
            self.client.send_goal(RecordGoal(cmd=2, timeout=5))
            self.client.wait_for_result(rospy.Duration(6))

    def command(self, goal, code=0):
        offset = len(self.feedbacks)
        self.client.send_goal(goal)
        self.assertTrue(self.client.wait_for_result(rospy.Duration(6)))
        result = self.client.get_result()
        self.assertEqual(result.error_code, code, result)
        self.assertEqual(result.success, code == 0)
        # feedback/result 属于两个 TCPROS 连接，result 可能先到；已 DONE 的
        # SimpleActionClient 会忽略晚到 feedback，用原话题核对实际发布内容。
        if code == 0:
            terminal = 1 if goal.cmd == 1 else 3
            self.assertTrue(wait_for(lambda: terminal in self.feedbacks[offset:]))
        return result, self.feedbacks[offset:]

    def start(self, topics=None, timeout=3, code=0):
        return self.command(RecordGoal(cmd=1, bag_path=self.root, bag_name=self.name,
            topics=topics if topics is not None else ['/record_test/odom'], timeout=timeout), code)

    def worker_pid(self):
        for node in rosnode.get_node_names():
            if node.startswith('/spikive_bag_'):
                try:
                    uri = rosgraph.Master(rospy.get_name()).lookupNode(node)
                    code, _, pid = xmlrpc.client.ServerProxy(uri).getPid(rospy.get_name())
                    if code == 1: return pid
                except Exception: pass  # master can temporarily retain a stopped child registration
        return None

    def test_01_exact_topics_and_original_message(self):
        self.odom.publish(Odometry())
        self.text.publish('before start (latched)')
        result, feedback = self.start(['/record_test/odom', '/record_test/text'])
        self.assertEqual(feedback[-1], 1)
        self.assertIn(self.target, result.message)
        self.assertTrue(wait_for(lambda: self.odom.get_num_connections() > 0 and self.text.get_num_connections() > 0))
        observed = []
        sub = rospy.Subscriber('/record_test/odom', Odometry, lambda m: observed.append(wire(m)))
        self.assertTrue(wait_for(lambda: self.odom.get_num_connections() > 1))
        message = Odometry(); message.header.frame_id = 'world'; message.child_frame_id = 'body'
        message.header.stamp = rospy.Time(123, 456); message.pose.pose.position.x = 7.5
        message.pose.covariance = [float(i) for i in range(36)]
        self.odom.publish(message); self.text.publish('after start')
        self.assertTrue(wait_for(lambda: len(observed) >= 2))
        time.sleep(.2)
        sub.unregister()
        _, feedback = self.command(RecordGoal(cmd=2, timeout=3))
        self.assertEqual(feedback[-1], 3)
        with rosbag.Bag(self.target) as bag:
            self.assertEqual(set(bag.get_type_and_topic_info().topics), {'/record_test/odom', '/record_test/text'})
            recorded = [wire(msg) for topic, msg, _ in bag.read_messages('/record_test/odom')]
            self.assertIn(observed[-1], recorded)  # Header, covariance and serialized bytes preserved
            self.assertIn('before start (latched)', [m.data for _, m, _ in bag.read_messages('/record_test/text')])
        self.assertFalse(any(name.startswith('.spikive-record-') for name in os.listdir(self.root)))
        self.start(code=19)  # Existing file cannot be overwritten.

    def test_02_empty_invalid_topics_and_stop_arguments(self):
        for topics in [['odom'], ['/record_test/.*'], ['/record_test/odom', '/record_test/odom']]:
            self.start(topics, code=19)
        self.command(RecordGoal(cmd=2, timeout=1), 12)
        self.start()
        self.start(code=21)
        self.command(RecordGoal(cmd=2, topics=['/record_test/other'], timeout=1), 19)
        self.command(RecordGoal(cmd=2, bag_name='other', timeout=1), 19)
        self.command(RecordGoal(cmd=2, timeout=3))

    def test_03_topic_can_appear_after_start(self):
        self.start(['/record_test/late'])
        publisher = rospy.Publisher('/record_test/late', String, queue_size=10)
        self.assertTrue(wait_for(lambda: publisher.get_num_connections() > 0))
        publisher.publish('late data'); time.sleep(.2)
        self.command(RecordGoal(cmd=2, timeout=3)); publisher.unregister()
        with rosbag.Bag(self.target) as bag:
            self.assertEqual([msg.data for _, msg, _ in bag.read_messages()], ['late data'])

    def test_04_start_timeout_can_still_stop(self):
        self.start(timeout=0.000001, code=2)
        self.command(RecordGoal(cmd=2, timeout=4))
        self.assertTrue(os.path.isfile(self.target))

    def test_05_stop_cancel_finishes_file_without_reviving_goal(self):
        self.start()
        pid = self.worker_pid(); self.assertIsNotNone(pid)
        os.kill(pid, signal.SIGSTOP); self.resumed_pid = pid
        self.client.send_goal(RecordGoal(cmd=2, timeout=4))
        self.assertTrue(wait_for(lambda: self.client.get_state() == GoalStatus.ACTIVE))
        self.client.cancel_goal()
        self.assertTrue(self.client.wait_for_result(rospy.Duration(2)))
        self.assertEqual(self.client.get_result().error_code, 3)
        os.kill(pid, signal.SIGCONT); self.resumed_pid = None
        self.assertTrue(wait_for(lambda: os.path.isfile(self.target)))
        time.sleep(.1)
        self.assertEqual(self.client.get_result().error_code, 3)

    def test_06_stop_timeout_does_not_leave_worker(self):
        self.start()
        pid = self.worker_pid(); self.assertIsNotNone(pid)
        os.kill(pid, signal.SIGSTOP); self.resumed_pid = pid
        self.command(RecordGoal(cmd=2, timeout=.1), 2)
        os.kill(pid, signal.SIGCONT); self.resumed_pid = None
        self.assertTrue(wait_for(lambda: os.path.isfile(self.target)))
        self.assertTrue(wait_for(lambda: self.worker_pid() is None))
        self.assertEqual(self.client.get_result().error_code, 2)

    def test_07_destination_created_during_recording_is_preserved(self):
        self.start()
        with open(self.target, 'w') as out: out.write('do not overwrite')
        result, _ = self.command(RecordGoal(cmd=2, timeout=3), 99)
        self.assertIn('data retained:', result.message)
        with open(self.target) as source: self.assertEqual(source.read(), 'do not overwrite')
        retained = result.message.split('data retained: ', 1)[1]
        with rosbag.Bag(os.path.join(retained, self.name + '_0.bag')): pass
        shutil.rmtree(retained)

    def test_08_unexpected_worker_death(self):
        self.start()
        pid = self.worker_pid(); self.assertIsNotNone(pid)
        os.kill(pid, signal.SIGKILL)
        time.sleep(.2)
        self.command(RecordGoal(cmd=2, timeout=3), 99)
        self.assertFalse(os.path.exists(self.target))

    def test_09_empty_goal_uses_the_configured_topics(self):
        self.text.publish('configured topic')
        self.start([])
        self.assertTrue(wait_for(lambda: self.text.get_num_connections() > 0))
        time.sleep(.2)
        self.command(RecordGoal(cmd=2, timeout=3))
        with rosbag.Bag(self.target) as bag:
            self.assertEqual([(topic, msg.data) for topic, msg, _ in bag.read_messages()],
                             [('/record_test/text', 'configured topic')])

    def test_10_split_and_merge_reports_progress_without_changing_messages(self):
        before = set(os.listdir(self.root))
        self.start([])
        self.assertTrue(wait_for(lambda: self.text.get_num_connections() > 0))
        staging, = set(os.listdir(self.root)) - before
        staging = os.path.join(self.root, staging)
        messages = [f'{i:03d}:' + 'x' * (128 * 1024) for i in range(25)]
        for message in messages:
            self.text.publish(message)
            time.sleep(.03)
        self.assertTrue(wait_for(lambda: os.path.isfile(os.path.join(staging, self.name + '_1.bag'))))
        self.assertFalse(os.path.exists(self.target))
        offset = len(self.merge_progress)
        self.command(RecordGoal(cmd=2, timeout=5))
        self.assertTrue(wait_for(lambda: any(done == total and total > 1 for done, total
                                            in self.merge_progress[offset:])))
        progress = self.merge_progress[offset:]
        self.assertEqual([done for done, _ in progress], sorted(done for done, _ in progress))
        with rosbag.Bag(self.target) as bag:
            recorded = list(bag.read_messages(return_connection_header=True))
        self.assertEqual([msg.data for _, msg, _, _ in recorded if msg.data in messages], messages)
        self.assertTrue(all(topic == '/record_test/text' and header.get('latching') == b'1'
                            for topic, _, _, header in recorded))
        self.assertFalse(os.path.exists(staging))

    def test_11_shutdown_closes_owned_recording(self):
        self.start()
        pid = self.worker_pid(); self.assertIsNotNone(pid)
        rosnode.kill_nodes(['/spikive_map_record_node'])
        self.assertTrue(wait_for(lambda: os.path.isfile(self.target)))
        self.assertTrue(wait_for(lambda: not os.path.exists('/proc/' + str(pid))))
        with rosbag.Bag(self.target): pass


if __name__ == '__main__':
    rospy.init_node('sdk_recording_test')
    rostest.rosrun('spikive_control_sdk', 'sdk_recording', Recording)
