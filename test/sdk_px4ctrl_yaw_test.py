#!/usr/bin/env python3
"""Optional integration: real SDK + supplied px4ctrl, fake odometry/MAVROS, private master.

Source a workspace containing the new drone_control backend before running.
Never starts a MAVROS node or connects to a flight controller.
"""
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import tempfile
import time


def main():
    with socket.socket() as probe:
        probe.bind(('127.0.0.1', 11341))
    with tempfile.TemporaryDirectory(prefix='sdk-px4-yaw-') as directory:
        os.environ.update(ROS_MASTER_URI='http://127.0.0.1:11341', ROS_HOSTNAME='127.0.0.1',
                          ROS_IP='127.0.0.1', ROS_HOME=directory + '/ros', ROS_LOG_DIR=directory + '/log')
        import actionlib
        import rosgraph
        import rospy
        from mavros_msgs.msg import State, ExtendedState, RCIn, PositionTarget
        from nav_msgs.msg import Odometry
        from quadrotor_msgs.msg import PositionCommand
        from std_msgs.msg import String
        from spikive_control_sdk.msg import gotoAction, gotoGoal

        processes, statuses, outputs = [], [], []
        position, heading, automatic = [0., 0., 1.5], 0., True
        log_path = Path(directory) / 'process.log'
        with log_path.open('w') as log:
            try:
                processes.append(subprocess.Popen(['roscore', '-p', '11341'], stdout=log, stderr=log))
                for _ in range(100):
                    try:
                        rosgraph.Master('/sdk_yaw_driver').getPid()
                        break
                    except Exception:
                        time.sleep(.1)
                else:
                    raise AssertionError('isolated master failed to start')
                rospy.init_node('sdk_yaw_driver', disable_signals=True)
                pubs = {key: rospy.Publisher(topic, typ, queue_size=10) for key, topic, typ in [
                    ('state', '/mavros/state', State), ('ext', '/mavros/extended_state', ExtendedState),
                    ('rc', '/mavros/rc/in', RCIn), ('odom', '/mavros/local_position/odom', Odometry),
                    ('traj', '/position_cmd', PositionCommand)]}
                subscriptions = [
                    rospy.Subscriber('/yaw_backend/yaw_override/status', String, lambda m: statuses.append(m.data)),
                    rospy.Subscriber('/mavros/setpoint_raw/local', PositionTarget, outputs.append)]
                processes.append(subprocess.Popen(['rosrun', 'drone_control', 'px4ctrl',
                    '__name:=yaw_backend', '_drone_id:=93', '_yaw_override/settle_time:=0.15'], stdout=log, stderr=log))
                processes.append(subprocess.Popen(['rosrun', 'spikive_control_sdk', 'spikive_flight_node',
                    '__name:=sdk_yaw_bridge', '_drone_id:=93', '_topics/drone_state:=/drone_state',
                    '_topics/navigation_odometry:=/mavros/local_position/odom', '_topics/trajectory:=/position_cmd',
                    '_topics/yaw_override:=/yaw_backend/yaw_override',
                    '_topics/yaw_status:=/yaw_backend/yaw_override/status', '_goto/stable_time:=0.15'],
                    stdout=log, stderr=log))

                def tick():
                    stamp = rospy.Time.now()
                    state = State(connected=True, armed=True, mode='OFFBOARD'); state.header.stamp = stamp
                    ext = ExtendedState(landed_state=ExtendedState.LANDED_STATE_IN_AIR); ext.header.stamp = stamp
                    odom = Odometry(); odom.header.stamp = stamp; odom.header.frame_id = 'world'
                    odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z = position
                    odom.pose.pose.orientation.z = math.sin(heading / 2)
                    odom.pose.pose.orientation.w = math.cos(heading / 2)
                    traj = PositionCommand(); traj.header.stamp = stamp
                    traj.position.x, traj.position.y, traj.position.z = position
                    pubs['state'].publish(state); pubs['ext'].publish(ext); pubs['odom'].publish(odom)
                    pubs['rc'].publish(RCIn(channels=[1500] * 6 + [1500 if automatic else 1200] + [1500]))
                    pubs['traj'].publish(traj)

                def wait_for(predicate, seconds=6):
                    end = time.monotonic() + seconds
                    while time.monotonic() < end:
                        tick()
                        if predicate(): return
                        time.sleep(.02)
                    raise AssertionError('timed out; yaw states=' + str(statuses))

                def cruise(seconds):
                    end = time.monotonic() + seconds
                    while time.monotonic() < end:
                        tick(); time.sleep(.02)

                client = actionlib.SimpleActionClient('/nav/flight/goto', gotoAction)
                assert client.wait_for_server(rospy.Duration(10))
                wait_for(lambda: statuses and outputs)
                cruise(.3)
                client.send_goal(gotoGoal(position=[4, 0, 1.5], yaw=1.2, yaw_enable=True, timeout=8))
                cruise(.5)
                assert 'pending' not in statuses and client.get_result() is None
                position[0] = 4.
                wait_for(lambda: statuses[-1] == 'turning')
                assert client.get_result() is None, 'position reached must not finish yaw-enabled Goto'
                assert outputs[-1].yaw_rate > 0
                heading = 1.2
                wait_for(lambda: client.get_result() is not None)
                assert client.get_result().success and client.get_result().error_code == 0
                assert statuses.count('pending') == 1 and statuses.count('done') == 1
                print('PASS: real SDK -> real px4ctrl -> pending/turning/done -> Goto success', flush=True)

                # Latched done cannot complete the next task. RC mode change cancels the new turn.
                client.send_goal(gotoGoal(position=[5, 0, 1.5], yaw=-1.2, yaw_enable=True, timeout=8))
                cruise(.4)
                assert client.get_result() is None
                position[0] = 5.
                wait_for(lambda: statuses[-1] == 'turning')
                automatic = False
                wait_for(lambda: client.get_result() is not None)
                assert not client.get_result().success and client.get_result().error_code == 3
                assert statuses[-1] == 'idle' and statuses.count('done') == 1
                print('PASS: next task ignores old done; real backend cancellation aborts SDK tracking', flush=True)
                for subscription in subscriptions: subscription.unregister()
            except Exception:
                log.flush()
                print(log_path.read_text())
                raise
            finally:
                for process in reversed(processes):
                    if process.poll() is None:
                        process.send_signal(signal.SIGINT)
                        try: process.wait(timeout=5)
                        except subprocess.TimeoutExpired: process.kill(); process.wait()


if __name__ == '__main__':
    main()
