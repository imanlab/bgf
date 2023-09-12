# human trials data collection.

We performed trials with several different structures, below are the subject ID's alongside the type of trial they did:

### Final scoring system:
This is the scoring system we finally settled on, each participant had a 2 min break after every 5 trials.  
- 10 baseline familiarisation
- 10 baseline (discrete time score) 
- 10 control familiarisation (time, rotation and full score)
- 10 control (full score ONLY)

Participants who had this scoring system:
- 009 - 018

### Pre-Final scoring system:
This is the scoring system we finally settled on, each participant had a 2 min break after every 5 trials.  
- 10 baseline familiarisation
- 10 baseline (discrete time score) 
- 10 control familiarisation (time, rotation and full score)
- 10 control (time, rotation and full score)

Participants who had this scoring system:
- 

### Prelim  scoring system:
This is the scoring system we finally settled on, each participant had a 2 min break after every 5 trials.  
- 10 baseline familiarisation
- 10 baseline (audio beep motivation) 
- 10 control familiarisation (full score)
- 10 control (full score)

Participants who had this scoring system:
- 


# How to collect a trial:

### 1. Tactile Sensor:
Bring up the xela sensor with:

- sudo slcand -o -s8 -t hw -S 3000000 /dev/ttyUSB0
- sudo ifconfig slcan0 up
- kia@kia-PC-BX19280:/etc/xela$ ./xela_server

### 2. Upload the IMU programs to the Arduino:
With the arduino software, use the upload button to upload the programs, it should flash on the corresponding arduino when uploading.
- imu_arduino     --> Hand --> USB4
- imu_arduino_obj --> Book --> USB2

### 3. Launch the data collection files and the callibration scripts for the IMU.
This script launches all the scripts we will need. Including the callibration script for the IMU's
- kia@kia-PC-BX19280:~$ roslaunch data_collection_human_test collect_data.launch

### 4. Callibrate the IMU.
This process needs to be done for both the hand and the object IMU's
1. place the objects in their starting positions (hand placed palm down on the table, book upright in the start location)
2. Leave the objects there for 5-10 seconds
3. Move the objects randomly around for 5-10 seconds.
4. Move the objects to the 5-10 random orientations and leave stationary for 3-5 seconds.
5. place the objects back in their starting positions (hand placed palm down on the table, book upright in the start location)

### 4. Update the scripts for the new subjects name.
- data_sub.py --> line 19 --> name = "subject_XXX"
- trial_plot.py --> line 21 --> name = "subject_XXX"
- trial_score_baseline.py --> line 19 --> name = "subject_XXX"
- trial_score_controlled.py --> line 19 --> name = "subject_XXX"

### 5. For each stage in the stages, update the scripts:
- data_sub.py --> line 20 --> stage = "baseline_familirization" / "baseline" / "controlled_familirization" / "controlled"
- trial_plot.py --> line 22 --> stage = "baseline_familirization" / "baseline" / "controlled_familirization" / "controlled"
- trial_score_baseline.py --> line 20 --> familiarization = True / False
- data_sub.py --> line 20 --> stage = familiarization = True / False

### 6. Collection process:
- rosrun data_collection_human_test data_sub.py

To visualise the scores run one of these:
- ~/catkin_ws/src/data_collection_human_test/src$ python3 trial_score_baseline.py
- ~/catkin_ws/src/data_collection_human_test/src$ python3 trial_score_controlled.py

To evaluate the data for a trial run:
- ~/catkin_ws/src/data_collection_human_test/src$ python3 trial_plot.py


