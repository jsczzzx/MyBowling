# My Bowling


## Demo Video Link
https://drive.google.com/file/d/1gJe52hrytvFFNjhquxXWRUoD4jVxFF2z/view?usp=share_link
## Screenshot
![screenshot](https://github.com/jsczzzx/my_bowling/assets/39892107/9e5cbe99-c108-4023-9832-3ff7d81ee288)


## About

This project uses a turtlebot robot in gazebo for a bowling simulation. The general process of the whole simulation is as follows:
1. With the help of aruco_detect, the robot locates and navigates itself by identifying fiducial markers, and moves to the hitting position.
2. With the help of OpenCV, the robot rotates itself so that the detected bowling ball is in the center of the field of view
3. The robot moves forward and hits the ball

## How to Run

1. Make sure ROS Noetic is properly installed.
2. Build aruco_detect, an important dependency in this project.
3. ``` roslaunch my_bowling start.launch ```
4. ``` rqt_image_view  ## if you want to see camera view ```
