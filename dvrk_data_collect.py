# derived from dvrk_bag_replay

import dvrk
import sys
import csv
import time

import numpy as np
import rospy
import rosbag
import numpy
import PyKDL
import argparse

if sys.version_info.major < 3:
    input = raw_input

# ros init node so we can use default ros arguments (e.g. __ns:= for namespace)
rospy.init_node('dvrk_data_collection', anonymous=True)
# strip ros arguments
argv = rospy.myargv(argv=sys.argv)

# parse arguments
parser = argparse.ArgumentParser()
parser.add_argument('-a', '--arm', type = str, required = True,
                    choices = ['ECM', 'MTML', 'MTMR', 'PSM1', 'PSM2', 'PSM3'],
                    help = 'arm name corresponding to ROS topics without namespace.  Use __ns:= to specify the namespace')
parser.add_argument('-i', '--interval', type = float, default = 0.001,
                    help = 'expected interval in seconds between messages sent by the device')
parser.add_argument('-c', '--csv', type = argparse.FileType('r'), required = True,
                    help = 'ros bag containing the trajectory to replay.  The script assumes the topic to use is /<arm>/setpoint_cp.  You can change the topic used with the -t option')

args = parser.parse_args(argv[1:]) # skip argv[0], script name

# read commanded joint position
poses = []
with open(args.csv.name) as csvfile:
    for pose in csv.reader(csvfile, delimiter=','):
        poses.append(np.array(pose).astype(float))

# info
print('-- This script will replay a trajectory defined in %s on arm %s' % (args.arm, args.csv.name))

# send trajectory to arm
arm = dvrk.psm(arm_name = args.arm,
               expected_interval = args.interval)

# make sure the arm is powered
print('-- Enabling arm')
if not arm.enable(10):
    sys.exit('-- Failed to enable within 10 seconds')

print('-- Homing arm')
if not arm.home(10):
    sys.exit('-- Failed to home within 10 seconds')

input('---> Make sure the arm is ready to move using cartesian positions.  For a PSM or ECM, you need to have a tool in place and the tool tip needs to be outside the cannula.  You might have to manually adjust your arm.  Press "\Enter" when the arm is ready.')

input('---> Press \"Enter\" to move to close gripper')
# Close gripper
jaw_jp = np.array([0])
arm.jaw.move_jp(jaw_jp).wait()

input('---> Press \"Enter\" to move to start position')

jp = poses[0]
arm.move_jp(jp).wait()

# Replay
input('---> Press \"Enter\" to replay trajectory')

counter = 0
total = len(poses)
td = 0.001
start_time = time.time()

for pose in poses:
    arm.servo_jp(pose)
    counter = counter + 1
    sys.stdout.write('\r-- Progress %02.1f%%' % (float(counter) / float(total) * 100.0))
    sys.stdout.flush()

print('\n--> Time to replay trajectory: %f seconds' % (time.time() - start_time))
print('--> Done!')
