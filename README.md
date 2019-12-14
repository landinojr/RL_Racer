# Rl_racer




import face_recognition
import cv2

I will rewrite my startup script to launch a python file named arm commander, so the bash script doesn't have to be changed

**Installation Guide:**

`pip install face_recognition` 

`pip install opencv-python` (takes ~20 minutes)

**Running Guide**

1. be in outermost file

2. run bu shortcut on a robot

3. run roslaunch turtlebot3_bringup turtlebot3_rpicamera.launch framerate:=20 (didn't work on my computer)

4. `rqt_image_view` and select the compressed file you should see this

   This will show![Shows Camera view of lab](docs/rqtView.png)

   5) python face_finder.py, and the program will print out 

   ![Out put of Face finder Program](docs/Output.png)
