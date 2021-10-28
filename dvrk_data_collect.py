# derived from dvrk_bag_replay

import dvrk
import os
import sys
import csv
import time
import signal

import numpy as np
import rospy
import rosbag
import numpy
import PyKDL
import argparse
import subprocess


# helper function to kill the rosbag process
def terminate_process_and_children(p):
    ps_command = subprocess.Popen("ps -o pid --ppid %d --noheaders" % p.pid, shell=True, stdout=subprocess.PIPE)
    ps_output = ps_command.stdout.read()
    retcode = ps_command.wait()
    assert retcode == 0, "ps command returned %d" % retcode
    for pid_str in ps_output.split("\n")[:-1]:
            os.kill(int(pid_str), signal.SIGINT)
    p.terminate()

if sys.version_info.major < 3:
    input = raw_input

# ---------------------------------------------
# ros setup
# ---------------------------------------------
# ros init node
rospy.init_node('dvrk_data_collection', anonymous=True)
# strip ros arguments
argv = rospy.myargv(argv=sys.argv)

# ---------------------------------------------
# parse arguments
# ---------------------------------------------
parser = argparse.ArgumentParser()
parser.add_argument('-a', '--arm', type=str, required=True,
                    choices=['ECM', 'MTML', 'MTMR', 'PSM1', 'PSM2', 'PSM3'],
                    help='arm name corresponding to ROS topics without namespace.')
parser.add_argument('-c', '--csv', type=argparse.FileType('r'), required=True,
                    help='csv file containing the trajectory to replay.')
parser.add_argument('-s', '--setup', type=str, required=True,
                    choices=['PSM', 'PSM-SUJ'],
                    help='description of your dvrk setup, necessary for collect shell script selection.')
parser.add_argument('-i', '--id', type=str, required=True,
                    choices=['1', '2', '3'],
                    help='your PSM ID, e.g., if you are using PSM2, then you should type -i 2')

args = parser.parse_args(argv[1:])  # skip argv[0], script name

# ---------------------------------------------
# prepare rosbag script
# ---------------------------------------------
# create data folder
if not os.path.exists(os.path.join(os.getcwd(), 'data')):
    os.mkdir(os.path.join(os.getcwd(), 'data'))

folder_name = os.path.join(os.getcwd(), 'data')

# determine script based on setup
if args.setup == 'PSM':
    command = "rosbag record -O {0}.bag /{1}/measured_cp /{1}/measured_js /{1}/jaw/measured_js /{1}/spatial/jacobian".format(os.path.join(folder_name, args.csv.name.split('.csv')[0]), 'PSM'+args.id)
else:
    command = "rosbag record -O {0}.bag /{1}/measured_cp /{1}/measured_js /{1}/jaw/measured_js /{1}/spatial/jacobian /SUJ/{1}/measured_cp /SUJ/{1}/measured_js".format(os.path.join(folder_name, args.csv.name.split('.csv')[0]), 'PSM'+args.id)

# sanity check command
sanity_cmd = "rosbag info {0}.bag".format(os.path.join(folder_name, args.csv.name.split('.csv')[0]))

# ---------------------------------------------
# read commanded joint position
# ---------------------------------------------
poses = []
with open(args.csv.name) as csvfile:
    for pose in csv.reader(csvfile, delimiter=','):
        poses.append(np.array(pose).astype(float))

# ---------------------------------------------
# prepare psm
# ---------------------------------------------
print('-- This script will replay a trajectory defined in %s on arm %s' % (args.arm, args.csv.name))

# create arm
arm = dvrk.psm(arm_name=args.arm, expected_interval=0.001)

# make sure the arm is powered
print('-- Enabling arm')
if not arm.enable(10):
    sys.exit('-- Failed to enable within 10 seconds')

print('-- Homing arm')
if not arm.home(10):
    sys.exit('-- Failed to home within 10 seconds')

input('---> Make sure the arm is ready to move using cartesian positions.  For a PSM or ECM, you need to have a tool in place and the tool tip needs to be outside the cannula.  You might have to manually adjust your arm.  Press "\Enter" when the arm is ready.')

# close gripper
input('---> Press \"Enter\" to move to close gripper')
jaw_jp = np.array([0])
arm.jaw.move_jp(jaw_jp).wait()

# go to initial position and wait
input('---> Press \"Enter\" to move to start position')
jp = poses[0]
arm.move_jp(jp).wait()

# ---------------------------------------------
# start playing trajectory and data collection
# ---------------------------------------------
# play trajectory
input('---> Press \"Enter\" to play trajectory and collect data')

# run shell script
rosbag_process = subprocess.Popen(command.split(' '))
time.sleep(2)

# main play process
counter = 0
total = len(poses)
start_time = time.time()

for pose in poses:
    arm.servo_jp(pose)
    counter = counter + 1
    sys.stdout.write('\r-- Progress %02.1f%%' % (float(counter) / float(total) * 100.0))
    sys.stdout.flush()

# stop bagging 
terminate_process_and_children(rosbag_process)

print('\n--> Time to replay trajectory: %f seconds' % (time.time() - start_time))
print('--> Done!')

# check if required topics are collected correctly
subprocess.Popen(sanity_cmd.split(' '))
