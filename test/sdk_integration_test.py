#!/usr/bin/env python3
"""Only for an isolated rostest master: simulated backend, never real flight control."""
import copy
import io
import json
import os
import shutil
import struct
import tempfile
import threading
import time
import unittest

import actionlib
import rosbag
import rospy
import rostest
from actionlib_msgs.msg import GoalStatus
from controller_msgs.msg import DroneState, cmd
from geometry_msgs.msg import PoseWithCovarianceStamped
from mavros_msgs.msg import State, ExtendedState, Altitude
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import GoalSet, PositionCommand
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String, Float64
from spikive_control_sdk.msg import (takeoffAction, takeoffGoal, landAction, landGoal,
    gotoAction, gotoGoal, return_homeAction, return_homeGoal, RecordAction, RecordGoal,
    relocalizeAction, relocalizeGoal, FlightState, takeoffActionFeedback, landActionFeedback,
    gotoActionFeedback, return_homeActionFeedback)
from spikive_control_sdk.srv import EmergencyCmd, LoadMap


def wait_for(predicate, timeout=4):
    end = time.monotonic() + timeout
    while time.monotonic() < end and not rospy.is_shutdown():
        if predicate(): return True
        time.sleep(0.02)
    return False


def wire(message):
    out = io.BytesIO(); message.serialize(out); return out.getvalue()


class Backend:
    def __init__(self):
        self.root = tempfile.mkdtemp(prefix='sdk-v14-')
        self.commands, self.goals, self.initials, self.flight_states = [], [], [], []
        self.odom_out, self.cloud_out = [], []
        self.feedbacks = {name: [] for name in ["takeoff", "land", "goto", "return_home"]}
        self.s = dict(ground=1, landed=True, tookoff=False, reached=False, returned=False,
                      connected=True, armed=False, xyz=(0., 0., 0.))
        self.publishers = {}
        for name, cls in [('state', State), ('extended_state', ExtendedState),
                          ('local_position/odom', Odometry), ('altitude', Altitude)]:
            self.publishers[name] = rospy.Publisher('/mavros/' + name, cls, queue_size=10)
        for key, topic, cls in [
            ('drone', '/drone_42_state', DroneState), ('nav', '/drone_42_visual_slam/odom', Odometry),
            ('cloud', '/drone_42_cloud_registered', PointCloud2),
            ('trajectory', '/drone_42_planning/pos_cmd', PositionCommand),
            ('control', '/control', cmd), ('goal', '/goal_with_id', GoalSet),
            ('map', '/map', PointCloud2), ('diff', '/drone_42_diff_odom', Odometry)]:
            self.publishers[key] = rospy.Publisher(topic, cls, queue_size=10)
        self.status = rospy.Publisher('/drone_42_bag_record_status', String, queue_size=10, latch=True)
        self.match = rospy.Publisher('/drone_42_localization_match_status', String, queue_size=10, latch=True)
        self.bag, self.recording, self.drop_ack = None, False, False
        self.record_requests = []
        self.subs = [
            rospy.Subscriber('/control', cmd, self.on_control),
            rospy.Subscriber('/goal_with_id', GoalSet, lambda m: self.goals.append(m)),
            rospy.Subscriber('/nav/status/flight_state', FlightState, lambda m: self.flight_states.append(m)),
            rospy.Subscriber('/nav/data/odom', Odometry, lambda m: self.odom_out.append(m)),
            rospy.Subscriber('/nav/data/pointcloud', PointCloud2, lambda m: self.cloud_out.append(m)),
            rospy.Subscriber('/drone_42_bag_record_start', String, self.record_start),
            rospy.Subscriber('/drone_42_bag_record_stop', String, self.record_stop),
            rospy.Subscriber('/drone_42_localization_pcl', PointCloud2, self.on_map),
            rospy.Subscriber('/drone_42_initialpose', PoseWithCovarianceStamped, self.on_initial)]
        for name, typ in [('takeoff', takeoffActionFeedback), ('land', landActionFeedback),
                          ('goto', gotoActionFeedback), ('return_home', return_homeActionFeedback)]:
            self.subs.append(rospy.Subscriber('/nav/flight/' + name + '/feedback', typ,
                lambda m, key=name: self.feedbacks[key].append(m.feedback)))
        self.publish_record('', 'status')
        self.publish_match('idle')
        self.timer = rospy.Timer(rospy.Duration(.02), self.tick)

    def tick(self, _):
        s = self.s.copy(); stamp = rospy.Time.now()
        state = State(); state.header.stamp = stamp; state.connected = s['connected']; state.armed = s['armed']
        self.publishers['state'].publish(state)
        ext = ExtendedState(); ext.header.stamp = stamp; ext.landed_state = s['ground']
        self.publishers['extended_state'].publish(ext)
        drone = DroneState(); drone.header.stamp = stamp
        for key in ['landed', 'tookoff', 'reached', 'returned']: setattr(drone, key, s[key])
        self.publishers['drone'].publish(drone)
        odom = Odometry(); odom.header.stamp = stamp; odom.header.frame_id = 'world'; odom.child_frame_id = 'body'
        odom.pose.pose.orientation.w = 1
        odom.twist.twist.linear.x = 3; odom.twist.twist.linear.y = 4
        self.publishers['local_position/odom'].publish(odom)
        odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z = s['xyz']
        odom.pose.covariance = [float(i) for i in range(36)]
        self.publishers['nav'].publish(odom)
        alt = Altitude(); alt.bottom_clearance = .7; self.publishers['altitude'].publish(alt)
        trajectory = PositionCommand(); trajectory.header.stamp = stamp
        self.publishers['trajectory'].publish(trajectory)

    def on_control(self, message):
        self.commands.append(message.cmd)
        if message.cmd == 3:
            def publish():
                time.sleep(.05)
                self.publishers['goal'].publish(GoalSet(drone_id=42, goal=[9, 8, 2]))
            threading.Thread(target=publish, daemon=True).start()

    def publish_record(self, request_id, phase):
        self.status.publish(String(json.dumps(dict(schema_version=1, request_id=request_id, drone_id='42',
            phase=phase, ok=True, state='recording' if self.recording else 'idle',
            bag_path=getattr(self, 'source', ''), error=None))))

    def record_start(self, message):
        request = json.loads(message.data); assert type(request['schema_version']) is int
        self.record_requests.append(('start', request['request_id']))
        self.source = os.path.join(self.root, request['request_id'] + '.bag')
        self.bag = rosbag.Bag(self.source + '.active.bag', 'w')
        self.bag.write('/Odometry_trans', Odometry(), rospy.Time.now())
        self.recording = True
        if not self.drop_ack: self.publish_record(request['request_id'], 'record_start')

    def record_stop(self, message):
        request = json.loads(message.data); self.record_requests.append(('stop', request['request_id']))
        if self.bag:
            self.bag.close(); self.bag = None
            os.rename(self.source + '.active.bag', self.source)
        self.recording = False
        if not self.drop_ack: self.publish_record(request['request_id'], 'record_stop')

    def publish_match(self, state, fitness=-1., attempts=0):
        self.match.publish(String(json.dumps(dict(schema_version=1, drone_id='42', state=state,
            message=state, fitness_score=fitness, threshold=.3, attempt_count=attempts, elapsed_ms=20))))

    def on_map(self, message):
        cloud = copy.deepcopy(message); cloud.header.stamp = rospy.Time.now(); cloud.header.frame_id = 'map'
        self.publishers['map'].publish(cloud); self.publish_match('waiting_initialpose')

    def on_initial(self, message):
        self.initials.append(message)
        self.publish_match('matching')
        def finish():
            time.sleep(.1)
            odom = Odometry(); odom.header.stamp = rospy.Time.now(); odom.header.frame_id = 'map'
            odom.pose.pose.orientation.w = 1; odom.pose.pose.position.x = 2
            self.publishers['diff'].publish(odom)
            self.publish_match('localized', .1, 10)
        threading.Thread(target=finish, daemon=True).start()

    def close(self):
        self.timer.shutdown()
        if self.bag: self.bag.close()
        shutil.rmtree(self.root)


class Integration(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.b = Backend()
        cls.clients = {}
        for name, typ in [('takeoff', takeoffAction), ('land', landAction), ('goto', gotoAction),
                          ('return_home', return_homeAction), ('record', RecordAction), ('relocalize', relocalizeAction)]:
            topic = '/nav/map/record' if name == 'record' else '/nav/localization/relocalize' if name == 'relocalize' else '/nav/flight/' + name
            client = actionlib.SimpleActionClient(topic, typ)
            assert client.wait_for_server(rospy.Duration(10)), topic
            cls.clients[name] = client
        rospy.wait_for_service('/nav/safety/emergency', 5)
        rospy.wait_for_service('/nav/localization/load_map', 5)
        cls.emergency = rospy.ServiceProxy('/nav/safety/emergency', EmergencyCmd)
        cls.load_map = rospy.ServiceProxy('/nav/localization/load_map', LoadMap)
        time.sleep(.5)

    @classmethod
    def tearDownClass(cls): cls.b.close()

    def setUp(self):
        self.optional_connections = []
        self.b.s.update(ground=1, landed=True, tookoff=False, reached=False, returned=False,
                        connected=True, armed=False, xyz=(0., 0., 0.))
        time.sleep(.1)

    def tearDown(self):
        for name in ['takeoff', 'land', 'goto', 'return_home']:
            client = self.clients[name]
            if client.get_state() in (GoalStatus.PENDING, GoalStatus.ACTIVE):
                client.cancel_goal()
                client.wait_for_result(rospy.Duration(2))
        for connection in self.optional_connections:
            connection.unregister()
        if self.optional_connections: time.sleep(.15)

    def enable_extensions(self):
        self.speeds, self.yaws = [], []
        self.yaw_status = rospy.Publisher('/drone_42_control/yaw_override/status', String, queue_size=10, latch=True)
        self.optional_connections = [self.yaw_status,
            rospy.Subscriber('/drone_42_planning/max_vel', Float64, lambda m: self.speeds.append(m.data)),
            rospy.Subscriber('/drone_42_control/yaw_override', Float64, lambda m: self.yaws.append(m.data))]
        self.yaw_status.publish(String('done'))  # Simulate a latched completion from a previous request.
        self.assertTrue(wait_for(lambda: self.yaw_status.get_num_connections() > 0))
        time.sleep(.2)

    def arrive(self, xyz):
        self.b.s.update(xyz=xyz, reached=True)

    def wait_result(self, name, code=0):
        client = self.clients[name]
        self.assertTrue(client.wait_for_result(rospy.Duration(4)))
        self.assertEqual(client.get_result().error_code, code, client.get_result())
        return client.get_result()

    def airborne(self):
        self.b.s.update(ground=2, landed=False, armed=True)
        time.sleep(.1)

    def result(self, name, goal, code=0):
        c = self.clients[name]; c.send_goal(goal)
        self.assertTrue(c.wait_for_result(rospy.Duration(4)), name)
        self.assertEqual(c.get_result().error_code, code, c.get_result())
        return c.get_result()

    def test_01_native_data_mapping(self):
        self.assertTrue(wait_for(lambda: self.b.odom_out))
        odom = self.b.odom_out[-1]
        self.assertEqual((odom.header.frame_id, odom.child_frame_id), ('world', 'body'))
        self.assertEqual(list(odom.pose.covariance), list(map(float, range(36))))
        self.assertEqual(odom.twist.twist.linear.x, 3)
        cloud = PointCloud2(); cloud.header.stamp = rospy.Time.now(); cloud.header.frame_id = 'world'
        cloud.height = 1; cloud.width = 2; cloud.fields = [PointField('custom', 0, PointField.FLOAT32, 1)]
        cloud.point_step = 4; cloud.row_step = 8; cloud.is_dense = True; cloud.data = struct.pack('<ff', 1., 2.)
        self.b.publishers['cloud'].publish(cloud)
        self.assertTrue(wait_for(lambda: self.b.cloud_out))
        self.assertEqual(wire(self.b.cloud_out[-1]), wire(cloud))

    def test_02_unsupported_and_invalid_fields(self):
        before = len(self.b.commands)
        self.result('takeoff', takeoffGoal(target_altitude=1, timeout=1), 20)
        self.result('goto', gotoGoal(speed=1, timeout=1), 20)
        self.result('goto', gotoGoal(yaw_enable=True, timeout=1), 20)
        self.result('return_home', return_homeGoal(speed=1, timeout=1), 20)
        self.result('land', landGoal(timeout=-1), 19)
        self.result('takeoff', takeoffGoal(timeout=float('nan')), 19)
        self.assertEqual(len(self.b.commands), before)

    def test_03_takeoff_and_land_complete(self):
        feedback = []; c = self.clients['takeoff']
        c.send_goal(takeoffGoal(timeout=3), feedback_cb=lambda m: feedback.append(m.phase))
        self.assertTrue(wait_for(lambda: self.b.commands and self.b.commands[-1] == 1))
        self.b.s.update(ground=3, landed=False); time.sleep(.15)
        self.b.s.update(ground=2, tookoff=True)
        self.assertTrue(c.wait_for_result(rospy.Duration(3))); self.assertTrue(c.get_result().success)
        self.assertIn(2, feedback)
        self.assertTrue(wait_for(lambda: self.b.feedbacks['takeoff'][-1].phase == 3))
        self.b.s.update(tookoff=False)
        feedback = []; c = self.clients['land']
        c.send_goal(landGoal(timeout=3), feedback_cb=lambda m: feedback.append(m.phase))
        self.assertTrue(wait_for(lambda: self.b.commands[-1] == 2))
        self.b.s.update(ground=4, landed=True); time.sleep(.15)
        self.assertIsNone(c.get_result())  # LANDING must not count as touchdown.
        self.b.s.update(ground=1, armed=False)
        self.assertTrue(c.wait_for_result(rospy.Duration(3))); self.assertTrue(c.get_result().success)
        self.assertTrue(wait_for(lambda: self.b.feedbacks['land'][-1].phase in (2, 3)))

    def test_04_timeout_and_cancel_do_not_send_extra_control(self):
        self.result('takeoff', takeoffGoal(timeout=.15), 2)
        self.airborne()
        self.result('land', landGoal(timeout=.15), 2)
        self.result('return_home', return_homeGoal(timeout=.15), 2)
        goals = len(self.b.goals)
        self.result('goto', gotoGoal(position=[4, 3, 2], timeout=.05), 2)
        time.sleep(.25); self.assertEqual(len(self.b.goals), goals)
        c = self.clients['goto']; c.send_goal(gotoGoal(position=[4, 3, 2], timeout=2))
        self.assertTrue(wait_for(lambda: c.get_state() == GoalStatus.ACTIVE))
        n = len(self.b.commands); c.cancel_goal()
        self.assertTrue(c.wait_for_result(rospy.Duration(2))); self.assertEqual(c.get_result().error_code, 3)
        time.sleep(.3); self.assertEqual(len(self.b.commands), n)
        self.assertEqual(len(self.b.goals), goals)

    def test_05_goto_emergency_return_and_global_state(self):
        self.airborne(); feedback = []
        c = self.clients['goto']; c.send_goal(gotoGoal(position=[5, 3, 2], timeout=3), feedback_cb=feedback.append)
        self.assertTrue(wait_for(lambda: self.b.goals and list(self.b.goals[-1].goal) == [5, 3, 2]))
        self.b.s.update(reached=True)  # An old reached flag must not override this task's distance check.
        self.assertTrue(wait_for(lambda: self.b.flight_states and self.b.flight_states[-1].state == 4))
        self.b.s.update(xyz=(5., 3., 2.), reached=True)
        self.assertTrue(c.wait_for_result(rospy.Duration(3))); self.assertTrue(c.get_result().success)
        self.assertTrue(wait_for(lambda: self.b.feedbacks['goto'][-1].state == 3))
        self.assertAlmostEqual(self.b.feedbacks['goto'][-1].distance_total, 0)
        self.assertAlmostEqual(feedback[-1].current_speed, 5)
        self.b.s.update(reached=False)
        c.send_goal(gotoGoal(position=[6, 3, 2], timeout=3))
        self.assertTrue(wait_for(lambda: c.get_state() == GoalStatus.ACTIVE))
        for value, native in [(1, 5), (2, 5), (3, 4)]:
            response = self.emergency(value); self.assertTrue(response.success, response)
            self.assertTrue(wait_for(lambda: self.b.commands[-1] == native))
        self.assertTrue(c.wait_for_result(rospy.Duration(2))); self.assertEqual(c.get_result().error_code, 3)
        self.assertFalse(self.emergency(9).success)
        c = self.clients['return_home']; feedback = []
        c.send_goal(return_homeGoal(timeout=3), feedback_cb=feedback.append)
        self.assertTrue(wait_for(lambda: self.b.commands[-1] == 3))
        self.b.s.update(returned=False); time.sleep(.15)
        self.b.s.update(xyz=(9., 8., 2.)); time.sleep(.12)
        self.b.s.update(returned=True)
        self.assertTrue(c.wait_for_result(rospy.Duration(3))); self.assertTrue(c.get_result().success)
        self.assertTrue(wait_for(lambda: self.b.feedbacks['return_home'][-1].phase == 2))
        self.assertAlmostEqual(self.b.feedbacks['return_home'][-1].distance_to_home, 0)
        self.assertEqual(self.b.commands[-1], 3)
        self.b.s.update(returned=False, reached=False)
        external = cmd(); external.header.stamp = rospy.Time.now(); external.cmd = 3
        self.b.publishers['control'].publish(external)
        self.assertTrue(wait_for(lambda: self.b.flight_states and self.b.flight_states[-1].state == 6))
        self.b.s.update(ground=4)
        self.assertTrue(wait_for(lambda: self.b.flight_states[-1].state == 7))
        self.b.s.update(ground=1, landed=True)
        self.assertTrue(wait_for(lambda: self.b.flight_states[-1].state == 8))

    def test_06_record_ack_export_and_timeout(self):
        path = os.path.join(self.b.root, 'output')
        self.assertTrue(self.result('record', RecordGoal(cmd=1, bag_path=path, bag_name='map', timeout=2)).success)
        self.assertTrue(self.b.recording)
        self.result('record', RecordGoal(cmd=1, bag_path=path, bag_name='busy', timeout=2), 21)
        r = self.result('record', RecordGoal(cmd=2, timeout=2))
        self.assertTrue(r.success); self.assertTrue(os.path.isfile(os.path.join(path, 'map.bag')))
        with rosbag.Bag(os.path.join(path, 'map.bag')) as bag: self.assertEqual(bag.get_message_count(), 1)
        self.assertTrue(os.path.isfile(self.b.source))
        self.result('record', RecordGoal(cmd=1, bag_path=path, bag_name='map', timeout=2), 19)
        self.b.drop_ack = True
        self.result('record', RecordGoal(cmd=1, bag_path=path, bag_name='late', timeout=.15), 2)
        self.assertTrue(self.b.recording); self.assertEqual(self.b.record_requests[-1][0], 'start')
        self.b.drop_ack = False
        self.assertTrue(self.result('record', RecordGoal(cmd=2, timeout=2)).success)
        self.assertTrue(os.path.isfile(os.path.join(path, 'late.bag')))

    def test_07_load_map_and_one_time_matching(self):
        path = os.path.join(self.b.root, 'map.pcd')
        with open(path, 'w') as out:
            out.write('VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH 5\nHEIGHT 1\nPOINTS 5\nDATA ascii\n0 0 0\n1 0 0\n0 1 0\n1 1 0\n2 2 0\n')
        self.assertTrue(self.load_map(path, '').success)
        goal = relocalizeGoal(); goal.initial_pose.header.frame_id = 'world'
        goal.initial_pose.pose.pose.orientation.w = 1; goal.initial_pose.pose.pose.position.x = 3
        n = len(self.b.initials); r = self.result('relocalize', goal)
        self.assertTrue(r.success); self.assertAlmostEqual(r.fitness_score, .1)
        self.assertEqual((r.match_transform.header.frame_id, r.match_transform.child_frame_id), ('map', 'world'))
        time.sleep(.3); self.assertEqual(len(self.b.initials), n + 1)
        self.assertEqual(self.b.initials[-1].pose.pose.position.x, 3)

    def test_08_record_wrong_ack_cancel_and_late_ack(self):
        path = os.path.join(self.b.root, 'output')
        self.b.drop_ack = True
        c = self.clients['record']; n = len(self.b.record_requests)
        c.send_goal(RecordGoal(cmd=1, bag_path=path, bag_name='cancelled', timeout=3))
        self.assertTrue(wait_for(lambda: len(self.b.record_requests) > n))
        own_id = self.b.record_requests[-1][1]
        self.b.publish_record('wrong_request', 'record_start'); time.sleep(.1)
        self.assertIsNone(c.get_result())
        c.cancel_goal(); self.assertTrue(c.wait_for_result(rospy.Duration(2)))
        self.assertEqual(c.get_result().error_code, 3)
        self.assertTrue(self.b.recording); self.assertEqual(self.b.record_requests[-1][0], 'start')
        self.b.publish_record(own_id, 'record_start'); time.sleep(.1)
        self.assertEqual(c.get_result().error_code, 3)
        self.b.drop_ack = False
        self.assertTrue(self.result('record', RecordGoal(cmd=2, timeout=2)).success)

    def test_09_busy_task_and_cached_takeoff_success(self):
        c = self.clients['takeoff']; c.send_goal(takeoffGoal(timeout=3))
        self.assertTrue(wait_for(lambda: c.get_state() == GoalStatus.ACTIVE))
        self.result('land', landGoal(timeout=2), 21)
        c.cancel_goal(); self.assertTrue(c.wait_for_result(rospy.Duration(2)))
        self.assertEqual(c.get_result().error_code, 3)
        self.b.s.update(tookoff=True); time.sleep(.1)
        self.result('takeoff', takeoffGoal(timeout=2), 7)

    def test_10_own_control_observer_is_not_backend_readiness(self):
        self.b.subs[0].unregister(); time.sleep(.3)
        try: self.result('takeoff', takeoffGoal(timeout=.2), 12)
        finally: self.b.subs[0] = rospy.Subscriber('/control', cmd, self.b.on_control)

    def test_11_goto_return_speed_and_native_default(self):
        self.enable_extensions(); self.airborne()
        c = self.clients['goto']
        c.send_goal(gotoGoal(position=[5, 3, 2], speed=.7, timeout=3))
        self.assertTrue(wait_for(lambda: self.speeds and self.b.goals and list(self.b.goals[-1].goal) == [5, 3, 2]))
        self.assertAlmostEqual(self.speeds[-1], .7, places=6)
        self.result('return_home', return_homeGoal(speed=.4, timeout=2), 21)
        self.assertEqual(len(self.speeds), 1)  # Rejected requests cannot change persistent speed.
        self.arrive((5., 3., 2.)); self.assertTrue(self.wait_result('goto').success)
        self.b.s.update(reached=False)
        c.send_goal(gotoGoal(position=[6, 3, 2], speed=0, timeout=3))
        self.assertTrue(wait_for(lambda: list(self.b.goals[-1].goal) == [6, 3, 2]))
        self.arrive((6., 3., 2.)); self.wait_result('goto')
        self.assertEqual(len(self.speeds), 1)  # speed<=0 sends no reset/zero speed.
        self.b.s.update(returned=False)
        self.clients['return_home'].send_goal(return_homeGoal(speed=.8, timeout=3))
        self.assertTrue(wait_for(lambda: len(self.speeds) == 2 and list(self.b.goals[-1].goal) == [9, 8, 2]))
        self.assertAlmostEqual(self.speeds[-1], .8, places=6)
        self.b.s.update(xyz=(9., 8., 2.)); time.sleep(.12)
        self.b.s.update(returned=True); self.assertTrue(self.wait_result('return_home').success)
        self.assertEqual(len(self.speeds), 2)
        self.assertEqual(self.yaws, [])

    def test_12_yaw_requires_new_ordered_status_after_position(self):
        self.enable_extensions(); self.airborne()
        c = self.clients['goto']
        c.send_goal(gotoGoal(position=[5, 3, 2], speed=.6, yaw=4.5, yaw_enable=True, timeout=4))
        self.assertTrue(wait_for(lambda: self.b.goals and list(self.b.goals[-1].goal) == [5, 3, 2]))
        time.sleep(.25)
        self.assertFalse(self.yaws); self.assertIsNone(c.get_result())
        self.arrive((5., 3., 2.))
        self.assertTrue(wait_for(lambda: self.yaws))
        self.assertEqual(self.yaws, [4.5])  # No SDK axis/unit conversion.
        self.assertIsNone(c.get_result())
        self.yaw_status.publish(String('done')); time.sleep(.1)
        self.assertIsNone(c.get_result())  # A stale/unrelated done cannot complete this task.
        self.yaw_status.publish(String('pending')); time.sleep(.1)
        self.yaw_status.publish(String('done')); time.sleep(.1)
        self.assertIsNone(c.get_result())  # Must observe turning too.
        self.yaw_status.publish(String('turning')); time.sleep(.1)
        self.assertEqual(self.b.feedbacks['goto'][-1].state, 2)
        self.yaw_status.publish(String('done'))
        self.assertTrue(self.wait_result('goto').success)
        self.assertEqual(len(self.yaws), 1)
        # An identical next target with yaw disabled uses the existing positional contract.
        c.send_goal(gotoGoal(position=[5, 3, 2], yaw_enable=False, timeout=2))
        self.assertTrue(self.wait_result('goto').success)
        self.assertEqual(len(self.yaws), 1)

    def test_13_yaw_backend_reset_and_new_identical_goal_abort(self):
        self.enable_extensions(); self.airborne(); self.arrive((5., 3., 2.))
        c = self.clients['goto']
        for reset in ['idle', 'goal', 'pending']:
            before = len(self.yaws)
            c.send_goal(gotoGoal(position=[5, 3, 2], yaw=1, yaw_enable=True, timeout=3))
            self.assertTrue(wait_for(lambda: len(self.yaws) > before))
            self.yaw_status.publish(String('pending')); time.sleep(.1)
            if reset in ('idle', 'pending'): self.yaw_status.publish(String(reset))
            else: self.b.publishers['goal'].publish(GoalSet(drone_id=42, goal=[5, 3, 2]))
            self.assertFalse(self.wait_result('goto', 3).success)
            self.assertEqual(c.get_state(), GoalStatus.ABORTED)

    def test_14_yaw_timeout_cancel_and_late_done_do_not_control(self):
        self.enable_extensions(); self.airborne(); self.arrive((5., 3., 2.))
        c = self.clients['goto']
        c.send_goal(gotoGoal(position=[5, 3, 2], yaw=1, yaw_enable=True, timeout=.8))
        self.assertTrue(wait_for(lambda: self.yaws))
        controls = len(self.b.commands)
        self.yaw_status.publish(String('pending'))
        self.wait_result('goto', 2)
        self.assertEqual(len(self.b.commands), controls)
        self.assertEqual(len(self.yaws), 1)
        c.send_goal(gotoGoal(position=[5, 3, 2], yaw=2, yaw_enable=True, timeout=3))
        self.assertTrue(wait_for(lambda: len(self.yaws) == 2))
        self.yaw_status.publish(String('pending')); time.sleep(.1)
        controls = len(self.b.commands)
        c.cancel_goal(); self.wait_result('goto', 3)
        self.yaw_status.publish(String('turning')); time.sleep(.05)
        self.yaw_status.publish(String('done')); time.sleep(.15)
        self.assertEqual(c.get_state(), GoalStatus.PREEMPTED)
        self.assertEqual(len(self.b.commands), controls)
        self.assertEqual(len(self.yaws), 2)


if __name__ == '__main__':
    rospy.init_node('sdk_v14_test')
    rostest.rosrun('spikive_control_sdk', 'sdk_v14_integration', Integration)
