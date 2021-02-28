# Structure_from_motion_project
This is my Perception project for my NYU Robot perception class FALL 2020 <br><br>
It is a Structure From Motion (SFM) project using a combination of six wide-angle cameras positioned on various parts of a simple car robot model.<br>
The car robot model moves through a city model and creates an SFM representation of the city model.<br>
This project was done entirely on ROS and Gazebo.<br><br>

The link to a more detailed video of the map creation BEFORE Bundle adjustment is below https://drive.google.com/file/d/1CaTkDDLY0U51aXv9jbMz3m_Sc3G4hOVP/view?usp=sharing
<br><br>


The link to a more detailed video of the map creation AFTER Bundle adjustment is below     
https://drive.google.com/file/d/1-qPUwkg_6AtzXwxWl2v5JOYI7UqGpA7P/view?usp=sharing



<br>

#  OVERVIEW
Robot car model drives through a gazebo city model and maps a 3D pointcloud of the environment.<br>
This SFM model is constructed in real-time in gazebo as the robot moves through the environment.
![feature_problem](https://user-images.githubusercontent.com/17696533/109409814-ba583c00-7963-11eb-9385-cb7c55848e75.PNG)

<br>

Simple car model used due to scaling issues.<br>
The camera on the correctly scaled car doesn't match the city scale and features are hard to detect.<br>
Six 180 degree fisheye cameras..... 60 degrees apart <br>
![car to robot scale](https://user-images.githubusercontent.com/17696533/109409977-fe980c00-7964-11eb-9c1e-4e1a96be4958.PNG)

<br>

![image](https://user-images.githubusercontent.com/17696533/109410163-54b97f00-7966-11eb-8d6d-d0c2580656dc.png)

# RESULTS
Map before bundle adjustment.<br>
![image](https://user-images.githubusercontent.com/17696533/109410598-f393aa80-7969-11eb-9e17-081a873e8864.png)

<br>
Map after bundle adjustment (birds eye view).<br>

![image](https://user-images.githubusercontent.com/17696533/109410886-322a6480-796c-11eb-94dc-691ce45ca88e.png)

## CONCLUSIONS
-Image features tend to be scarce in gazebo regardless of model. Most of the design features in the original city model did not translate to gazebo.<br>
-Scarce features in gazebo causes for a trade-off between RANSAC accuracy and point cloud density causing a more noisy point cloud.<br>
-Bundle adjustment tends to slow down the computation even more and accounts for a lag in point cloud mapping; I should have opted for an asynchronous approach instead of synchronous. <br>
-The point cloud seems to be most accurate at the right side of the model which was reasonably closer to the car robot and had its features well defined and visible.<br>
-Overall, this project gave me a lot of experience in applying concepts I learned and gave me a better understanding and intuition in their application.


