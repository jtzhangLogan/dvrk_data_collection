dVRK System Identification 
===  
  
What is in the folder
===  
- ``dvrk_excitation_signal_*.csv``: CSV file that contains excitation path, which specifies joint positions. 
- ``dvrk_path_replay.py``: python script that plays the excitation path on designated PSM.

How to use:  
===
Step 1: 
Place ``dvrk_path_replay.py`` inside your ``catkin_ws``. It can be in any folder. 

Step 2:
Place ``dvrk_excitation_signal_*.csv`` files in the same folder where you place ``dvrk_path_replay.py``. Technically, these csv files could be in another folder, but you need to give a complete path to these files when you run the script.

Step 3:
Place the instrument **LARGE-NEEDLE-DRIVER** on the robot. If you don't have this instrument, you can use whatever instrument you have, but please leave a note about what instrument you used to collect data.

Step 4:
Run ``roscore`` and ``dvrk_robot``.

```
# run roscore
cd <PATH-TO-YOUR-CATKIN-WS>  # go to your catkin_ws
source devel/setup.bash      # set environment variables
roscore                      

# in another terminal, run the dvrk application
cd <PATH-TO-YOUR-CATKIN-WS>  # go to your catkin_ws
source devel/setup.bash      # set environment variables
rosrun dvrk_robot dvrk_console_json -j <PATH-TO-YOUR-CONSOLE-CONFIG.JSON> -p 0.001 # run the dvrk console
```
If you have an MTM, after you run the console, please enable teleoperation.

Step 5:
Prepare Robot for data collection.
**NOTE: When you are prompted to replay trajectory, stop there and go to step 6.**

```
cd <PATH-TO-YOUR-CATKIN-WS>       # go to your catkin_ws
source devel/setup.bash           # set environment variables 
cd <PATH-TO-DVRK-PATH-REPLAY.PY>  # go to folder that contains dvrk_path_replay.py
python2 dvrk_path_replay.py -a <YOUR-PSM-ID> -c <CSV-FILE-NAME> # run the script, follow the prompt until you about to replay trajectory
```

Step 6:
Prepare ``rosbag `` record command.

```
cd <PATH-TO-YOUR-CATKIN-WS>       # go to your catkin_ws
source devel/setup.bash           # set environment variables 
mkdir data                        # create a folder to store collected data
cd data

# depending on your dvrk setup, please type one of the following three commands, but do not run yet
# Your setup includes: SUJ, MTM, and PSM, type this in your terminal. <MTM-ID> is the MTM that controls the PSM with <YOUR-PSM-ID>. e.g., if you are using MTMR to control PSM1, then <MTM-ID> == MTMR, <YOUR-PSM-ID> == PSM1
rosbag record -O <CVS-FILE-NAME-W/O-EXT>.bag /<YOUR-PSM-ID>/measured_cp /<YOUR-PSM-ID>/measured_js /<YOUR-PSM-ID>/jaw/measured_js /<YOUR-PSM-ID>/spatial/jacobian /SUJ/<YOUR-PSM-ID>/measured_cp /SUJ/<YOUR-PSM-ID>/measured_js /<MTM-ID>/measured_cp /<MTM-ID>/measured_js /<MTM-ID>/spatial/jacobian

# Your setup includes: MTM, and PSM, type this in your terminal
rosbag record -O <CVS-FILE-NAME-W/O-EXT>.bag /<YOUR-PSM-ID>/measured_cp /<YOUR-PSM-ID>/measured_js /<YOUR-PSM-ID>/jaw/measured_js /<YOUR-PSM-ID>/spatial/jacobian /<MTM-ID>/measured_cp /<MTM-ID>/measured_js /<MTM-ID>/spatial/jacobian

# Your setup includes: PSM, type this in your terminal
rosbag record -O <CVS-FILE-NAME-W/O-EXT>.bag /<YOUR-PSM-ID>/measured_cp /<YOUR-PSM-ID>/measured_js /<YOUR-PSM-ID>/jaw/measured_js /<YOUR-PSM-ID>/spatial/jacobian
```

Step 7:
Once you have the ``rosbag record`` command ready, you are ready to collect data. Press ``Enter``  in the terminal for Step 6 to start bagging data. Immediately after you start the ``rosbag`` process, press ``Enter`` in the terminal for Step 5 to play the excitation path. After the path has been played, stop the ``rosbag`` process with ``Ctrl C``.

Step 8:
Repeat the **STEP 5** to **STEP7** for all CSV files. 

**Congratulations!** You have reached the end of the data collection! Please zip your collect data and send it to us. 

If you have met any trouble during data collection, don't hesitate to contact me via email: ``jzhan247@jhu.edu``. I am also available for a zoom session to help you. 

**Your effort and time are much appreciated! Thank you for your contributions!**

