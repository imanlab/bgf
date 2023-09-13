# Tactile Forward Model

Our tactile forward model has two parts:

- ACTP: Action Conditioned Tactile Predictoin
- SCM: Slip Classification Model

The repository includes the scripts for:

- Collect real-world robot pick-and-move data
- Preprocessing data for trainig
- Train the ACTP and SCM models
- Evaluate the ACTP and SCM performormance.


For data collection you have to run:

        rosrun robot_test OL_data_collection "robot_ip"
        rosrun robot_test collect_data.py

Various data collection trials are performed by trying different test objects and reference trajectories.

For preprocessing the data, you need to run:

        python /tactile_forward_model/gen_dataset.py


This script saves the preprocessed data (after applyinmg Kalman filter and nomalisation) for each sequence as .npy file in the specified directory.

For trainig the ACTP model, you can run:

        python /tactile_forward_model/model_trainer.py

For training the classification model you can use the seq_classification.ipynb jupyter notebook.

Training resutls such as learning curve and best performing model will be saved in the specified directory.

To test the model on the hold-out set you can run:

        python /tactile_forward_model/model_tester.py

This script saves all the predictoin data for the test in the specified directory as .npy files.
