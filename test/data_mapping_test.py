#!/usr/bin/env python3
"""Isolated ROS tests for native mapping and the visualization branch; no flight commands."""
import copy
import io
import subprocess
import time
import unittest

import rospy
import rostest
from sensor_msgs.msg import PointCloud2, PointField


def wire(message):
    buffer = io.BytesIO()
    message.serialize(buffer)
    return buffer.getvalue()


def wait_for(predicate, timeout=4):
    end = time.monotonic() + timeout
    while time.monotonic() < end and not rospy.is_shutdown():
        if predicate():
            return True
        time.sleep(.02)
    return False


def cloud():
    c = PointCloud2()
    c.header.seq = 55
    c.header.stamp = rospy.Time(123, 456)
    c.header.frame_id = 'world'
    c.width, c.height, c.point_step, c.row_step = 10, 2, 8, 84
    c.fields = [PointField('x', 0, PointField.FLOAT32, 1), PointField('extra', 4, PointField.UINT32, 1)]
    c.data = bytes(range(168))
    return c


class DataMapping(unittest.TestCase):
    def setUp(self):
        self.connections, self.processes = [], []

    def tearDown(self):
        for process in self.processes:
            if process.poll() is None:
                process.terminate()
            try:
                process.communicate(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.communicate()
        for connection in self.connections:
            connection.unregister()

    def start_node(self, suffix, params):
        name = 'data_' + suffix
        defaults = {'data/pointcloud_source': '/' + suffix + '/source',
                    'topics/navigation_odometry': '/' + suffix + '/odom_source',
                    'data/pointcloud': '/' + suffix + '/raw',
                    'data/odometry': '/' + suffix + '/odom',
                    'visual_pointcloud/output_topic': '/' + suffix + '/visual'}
        defaults.update(params)
        for key, value in defaults.items():
            rospy.set_param('/' + name + '/' + key, value)
        process = subprocess.Popen(['rosrun', 'spikive_control_sdk', 'spikive_data_mapping_node',
                                    '__name:=' + name], stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        self.processes.append(process)
        return process

    def subscribe(self, topic):
        messages = []
        self.connections.append(rospy.Subscriber(topic, PointCloud2, messages.append, queue_size=10))
        return messages

    def publisher(self, topic):
        pub = rospy.Publisher(topic, PointCloud2, queue_size=2)
        self.connections.append(pub)
        self.assertTrue(wait_for(lambda: pub.get_num_connections() > 0))
        # Allow output subscriptions to complete before the one-shot input.
        time.sleep(.2)
        return pub

    def test_default_raw_and_visual(self):
        raw = self.subscribe('/nav/data/pointcloud')
        visual = self.subscribe('/nav/data/visual_pointcloud')
        pub = self.publisher('/drone_42_cloud_registered')
        sent = self.subscribe('/drone_42_cloud_registered')
        time.sleep(.1)
        c = cloud()
        pub.publish(c)
        self.assertTrue(wait_for(lambda: raw and visual and sent))
        self.assertEqual(wire(raw[-1]), wire(sent[-1]))
        expected = copy.deepcopy(sent[-1])
        expected.width, expected.height, expected.row_step = 2, 1, 16
        expected.data = c.data[:8] + c.data[84:92]
        self.assertEqual(wire(visual[-1]), wire(expected))
        self.assertEqual(visual[-1].header.frame_id, 'world')

        # Bad visualization input is dropped; the full mapping remains untouched.
        bad = copy.deepcopy(c)
        bad.data = b'bad'
        before = len(visual)
        pub.publish(bad)
        self.assertTrue(wait_for(lambda: len(raw) == 2))
        time.sleep(.15)
        self.assertEqual(len(visual), before)
        pub.publish(c)
        self.assertTrue(wait_for(lambda: len(visual) == before + 1))

    def test_custom_input_ratio_and_passthrough(self):
        for suffix, ratio in [('half', .5), ('full', 1.)]:
            with self.subTest(ratio=ratio):
                self.start_node(suffix, {'visual_pointcloud/input_topic': '/' + suffix + '/separate',
                                         'visual_pointcloud/keep_ratio': ratio})
                out = self.subscribe('/' + suffix + '/visual')
                raw = self.subscribe('/' + suffix + '/raw')
                pub = self.publisher('/' + suffix + '/separate')
                sent = self.subscribe('/' + suffix + '/separate')
                time.sleep(.1)
                c = cloud()
                pub.publish(c)
                self.assertTrue(wait_for(lambda: out and sent))
                self.assertFalse(raw)
                if ratio == 1.:
                    self.assertEqual(wire(out[-1]), wire(sent[-1]))
                else:
                    self.assertEqual(out[-1].width, 10)
                    self.assertEqual(out[-1].header, sent[-1].header)
                    records = [c.data[(i // 10) * 84 + (i % 10) * 8: (i // 10) * 84 + (i % 10) * 8 + 8]
                               for i in range(0, 20, 2)]
                    self.assertEqual(bytes(out[-1].data), b''.join(records))

    def test_disabled_keeps_raw_mapping(self):
        self.start_node('disabled', {'visual_pointcloud/enabled': False})
        raw = self.subscribe('/disabled/raw')
        out = self.subscribe('/disabled/visual')
        pub = self.publisher('/disabled/source')
        pub.publish(cloud())
        self.assertTrue(wait_for(lambda: raw))
        self.assertNotIn('/disabled/visual', dict(rospy.get_published_topics()))
        self.assertFalse(out)

    def test_invalid_configuration_reports_error(self):
        for index, ratio in enumerate([0., -.1, 1.1, float('nan'), float('inf')]):
            with self.subTest(ratio=ratio):
                process = self.start_node('invalid' + str(index), {'visual_pointcloud/keep_ratio': ratio})
                output, _ = process.communicate(timeout=5)
                self.assertNotEqual(process.returncode, 0)
                self.assertIn(b'keep_ratio must be in (0, 1]', output)
        for suffix, topic in [('loop', '/loop/source'), ('collision', '/collision/raw')]:
            process = self.start_node(suffix, {'visual_pointcloud/output_topic': topic})
            output, _ = process.communicate(timeout=5)
            self.assertNotEqual(process.returncode, 0)
            self.assertIn(b'Data output topics must be', output)


if __name__ == '__main__':
    rospy.init_node('data_mapping_test')
    rostest.rosrun('spikive_control_sdk', 'data_mapping', DataMapping)
