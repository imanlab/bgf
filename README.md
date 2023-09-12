**Beyond Grip Force: Investigating Trajectory Modulation for Slip Control in Human and Robot Manipulation Tasks**
============

This repository provides code for experimentation and analysis of the paper titled "Beyond ..."

We provde implementation of the following subset of methods described in the paper:

- Human physiology study on Slip Control
- Robotic experiments on slip control using trajectory modulation
- Tactile forward model offline training


**Human Study**

The /human_study folder includes the code for performing the experimentation with human participants. The main components of the code are:

- Data collectoin Script
- Visualise trial results
- Analyse subject scores

Detailed description for each part is provided in the readme file of the folder (provide hyperlink to the file here)

*Human sutdy data*

The raw data of the human participants study presented in the paper can be downloaded with this link.

**Robotic Experiments**

The /robot_experiments directory includes the code for regenerating the implementation of the slip controller introduced in the paper. This ros package depends on *libfranka* library which is the low-level control software of franka Emika robotic arm.

**Tactile Forward Model**

The /tactile_forward_model directory includes the code for trainig the forward model including Action Conditioned Tactile Prediction Model and Slip Classification Model.