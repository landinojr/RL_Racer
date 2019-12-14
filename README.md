# Rl_racer

Ros impelmnation of deep learning tecniques for robot naviagation and facial recognition

### Prerequisites

`Have ROS meliodic enviroment installed` 


**Installation Guide(Facial Recognition):**

`pip install face_recognition` 

`pip install opencv-python` (takes ~20 minutes)

`pip install numpy` 

`chmod +x [Files name]` 

**Installation Guide(Deep learning navigation training):**


**Running Guide(Facial Recognition):**

1. be in outermost file

2. run bu shortcut on a robot

3. run roslaunch turtlebot3_bringup turtlebot3_rpicamera.launch framerate:=20 

4. `rqt_image_view` and select the compressed file you should see this

5. While in RL_racer folder type into terminal $rosrun RL_racer face_finder.py

