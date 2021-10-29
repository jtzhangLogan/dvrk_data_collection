
dVRK System Identification 
===  
  
What is in the folder
===  
- ``signals/dvrk_excitation_signal_*.csv``: CSV file that contains excitation path, specified as joint positions. 
- ``dvrk_data_collect.py``: python script that plays the excitation path on the designated PSM and invokes the data collection process.

How to use
===
Step 1: 

Place ``dvrk_data_collect.py`` inside your ``catkin_ws``. It can be in any folder. 

Step 2:

Place the ``dvrk_excitation_signal_*.csv`` files in the same folder where you placed ``dvrk_data_collect.py``. Technically, these csv files could be in another folder, but then you would need to specify the complete path to these files when you run the script.

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
rosrun dvrk_robot dvrk_console_json -j <PATH-TO-YOUR-CONSOLE-CONFIG.JSON> -p 0.001 # run the dvrk console, it is IMPORTANT to set ROS rate to 1kHz, i.e., don't forget to add -p 0.001
```

Step 5:

Start playing excitation path and record data.

```
cd <PATH-TO-YOUR-CATKIN-WS>        # go to your catkin_ws
source devel/setup.bash            # set environment variables 
cd <PATH-TO-DVRK-DATA-COLLECT.PY>  # go to folder that contains dvrk_data_collect.py

# example usage: 
# 1. If you are using PSM1 with SUJ and you want to play excitation path dvrk_excitation_signal_1.csv
#    python2 dvrk_data_collect.py -a PSM1 -f dvrk_excitation_signal_1.csv -s PSM-SUJ
#
# 2. If you are using PSM2 only and you want to play excitation path dvrk_excitation_signal_3.csv
#    python2 dvrk_data_collect.py -a PSM2 -f dvrk_excitation_signal_3.csv -s PSM
#

python2 dvrk_data_collect.py -a <PSM-ID> -f <CSV-FILE-NAME> -s <DVRK-SETUP>
```

Step 6:

Repeat **STEP 5** for all CSV files. 

The end
===
**Congratulations!** You have reached the end of the data collection! Please zip your data folder ``data`` under your ``catkin_ws`` and send it to us. 

If you encounter any trouble during data collection, don't hesitate to contact me via email: ``jzhan247@jhu.edu``. I am also available for a zoom session to help you. 

**Your effort and time are much appreciated! Thank you for your contributions!**



