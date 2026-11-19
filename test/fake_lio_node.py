#!/usr/bin/env python3
"""Test-only node used to verify the post-process LIO precondition."""
import sys

import rospy

if __name__ == "__main__":
    name = sys.argv[1] if len(sys.argv) > 1 else "laserMapping"
    rospy.init_node(name)
    rospy.spin()
