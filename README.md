# RL_Racer

Ros impelmnation of deep learning tecniques for robot naviagation and facial recognition

### Prerequisites

`Have ROS meliodic enviroment installed` 


**Installation Guide(Facial Recognition):**

`pip install face_recognition` 

`pip install opencv-python` (takes ~20 minutes)

`pip install numpy` 

`chmod +x [Files name]` 

**Installation Guide(Deep learning navigation training):**

I recommend creating a conda environment using python 2.7 for this project. 

Follow directions at https://www.anaconda.com/distribution/#download-section to install anaconda

Then:

'conda install numpy'

'conda install pytorch torchvision cudatoolkit=10.1 -c pytorch'    (change cudatoolkit version as necessary)



**Running Guide(Facial Recognition):**

1. be in outermost file

2. run bu shortcut on a robot

3. run roslaunch turtlebot3_bringup turtlebot3_rpicamera.launch framerate:=20 

4. `rqt_image_view` and select the compressed file you should see this

5. While in RL_racer folder type into terminal $rosrun RL_racer face_finder.py

