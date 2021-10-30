
dVRK System Identification
===

What is in the folder
===
- ``signals/dvrk_excitation_signal_*.csv``: CSV file that contains excitation path, specified as joint positions.
- ``dvrk_data_collect.py``: python script that plays the excitation path on the designated PSM and invokes the data collection process.

How to use
===

Step 1:

Get the code, anywhere you want on your computer:
```sh
git clone https://github.com/jtzhangLogan/dvrk_data_collection.git
```

Step 2:

Place the instrument **LARGE-NEEDLE-DRIVER** on the robot. If you don't have this instrument, you can use whatever instrument you have, but please leave a note about what instrument you used to collect data.

Step 3:

Run ``roscore`` in one terminal:
```sh
# run roscore
cd <PATH-TO-YOUR-CATKIN-WS>  # go to your catkin_ws
source devel/setup.bash      # set environment variables
roscore

Step 4:

Run ``dvrk_robot`` in a second terminal:
```sh
# in another terminal, run the dvrk application
cd <PATH-TO-YOUR-CATKIN-WS>  # go to your catkin_ws
source devel/setup.bash      # set environment variables
rosrun dvrk_robot dvrk_console_json -j <PATH-TO-YOUR-CONSOLE-CONFIG.JSON> -p 0.001 # run the dvrk console, it is IMPORTANT to set ROS rate to 1kHz, i.e., don't forget to add -p 0.001
```

Step 5:

Start playing excitation path and record data.

```sh
cd <PATH-TO-YOUR-CATKIN-WS>        # go to your catkin_ws
source devel/setup.bash            # set environment variables
cd <PATH-TO-DVRK-DATA-COLLECT.PY>  # go to folder that contains dvrk_data_collect.py

# python2 dvrk_data_collect.py -a <PSM-ID> -f <CSV-FILE-NAME> -s <DVRK-SETUP>
#
# two main cases:
# 1. If you are using PSM1 with SUJ and you want to play excitation path dvrk_excitation_signal_1.csv
#    python2 dvrk_data_collect.py -a PSM1 -f signals/dvrk_excitation_signal_1.csv -s PSM-SUJ
#
# 2. If you are using PSM2 only and you want to play excitation path dvrk_excitation_signal_3.csv
#    python2 dvrk_data_collect.py -a PSM2 -f signals/dvrk_excitation_signal_3.csv -s PSM
#

# example:
./dvrk_data_collect.py -a PSM1 -f signals/dvrk_excitation_signal_6.csv -s PSM

```

When the script is done collecting, it will display a report based on the ROS bag.  For the robot's state topics, you should see a bit less than 90,000 messages recorded.  For example:
```
topics:      /PSM1/jaw/measured_js    89364 msgs    : sensor_msgs/JointState
             /PSM1/measured_cp        89361 msgs    : geometry_msgs/TransformStamped
             /PSM1/measured_js        89356 msgs    : sensor_msgs/JointState
             /PSM1/spatial/jacobian   89358 msgs    : std_msgs/Float64MultiArray
```
If you have way less messages recorded, double check that you started the `dvrk_console_json` node with the option `-p 0.001` in **Step 4**.

Step 6:

Repeat **Step 5** for all CSV files (7 of them)

Step 7:

Archive and check the size of the collected data:
```sh
cd <PATH-TO-DVRK-DATA-COLLECT.PY>
tar zcvf collected-data.tgz data
ls -lh collected-data.tgz
```

If the size of the compress file is way under 250M, double check that you started the `dvrk_console_json` node with the option `-p 0.001` in **Step 4**

The end
===
**Congratulations!** You have reached the end of the data collection! Please compress your data folder ``data`` and send it to us. Please also note the version of dVRK software (e.g., Version 2.1.0) and firmware (e.g., Version 7) on your system. If you do not know the firmware version, you can type ``qladisp`` and it will report the firmware versions of all connected boards.

If you encounter any trouble during data collection, don't hesitate to contact me via email: ``jzhan247@jhu.edu``. I am also available for a zoom session to help you.

**Your effort and time are much appreciated! Thank you for your contributions!**
