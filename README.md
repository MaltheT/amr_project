# Autonomous Mobile Robots
This is a guide to run the controller, missionStateMachine and object detection model for the F250 Drone Racer in the Gazebo  

## Prerequisites
The simulation has been tested on Ubuntu 18.04 machine:

- airlab_gazebo environment: https://github.com/open-airlab/airlab_gazebo
- ROS Melodic
- PX4 with Mavros, Mavlink and Gazebo 


## Running the Drone
Before running:
- Clone the term_project folder into your catkin workspace and build it


For running the drone, open three terminals. 

First terminal:
- Start the airlab_gazebo environment in Gazebo

Second terminal: 
```bash
…\catkin_ws\src\term_project\scripts $ python drone_velocity_controller.py
```

Third terminal:
```bash
…\catkin_ws\src\term_project\scripts $ python mission_state.py
```

Note: Please run the controller first and wait for it to hover before running the mission_state



## Object Detection Model

The costume trained object detection model can be tested using the YOLOv5.ipynb notebook file in the 'object_detection' folder. 

Please run all the cells in the notebook, as this will install most dependencies. 

Additinal dependencies are required such as torch and cv2 libraries and more, but will not be installed automatically by the notebook.

Finally, test images can be put into the 'test_images' folder, or pathed to in the code.
