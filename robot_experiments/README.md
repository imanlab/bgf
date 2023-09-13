# Real-time slip control using Tactile Forward Models and Trajectory Modulation


ROS publisher requirements:

- tactile sensor:

        cd /etc/xela
        ./xela_server
        rosrun robot_test xela_publisher.py
- Aruco marker pose publisher:

        roslaunch ralsense2_camera rs_camera.launch
        roslaunch aruco_ros double.launch
- robot state publusher

        rosrun robot_test test_/ref_name "robot_ip"

The /src directory includes the Cartesian velocity scripts with three reference trajectories (Trapezoid, Cubic, and Quintic). For instance the controller with trapezoid reference can be run as follows:

        rosrun robot_test test_trapezoid "robot_ip"

The /python_script directory includes the slip controller which calculates the deviations from the reference trajectory in a real-time optimisation loop. It can be run by:

        rosrun robot_test slip_control.py

This script subscribes to a topic which includes the (i) tactile, (ii) robot, and (iii) marker data being published by the synchronised publisher script:

        rosrun robot_test sync_publisher_node.py

The slip_control.py published the optimised trajectory values to the C++ script under '/optimal_traj' topic name. To record the experiments data in a desired directory, please change the "save_results_path" in the init function.

Slip_control.py loads two trained pytorch models namely ACTP (action condition tactile prediction) and ClassifierLSTM (slip classification model). The model architecturs are includes in the corresponding folder names ([ACTP](/robot_experiments/python%20scripts/ACTP/) and [ClassifierLSTM](/robot_experiments/python%20scripts/ClassifierLSTM/)).

