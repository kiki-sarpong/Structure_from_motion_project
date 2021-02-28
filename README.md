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
Robot car model drives through a gazebo city model and maps a 3D pointcloud of the environment.
![feature_problem](https://user-images.githubusercontent.com/17696533/109409814-ba583c00-7963-11eb-9385-cb7c55848e75.PNG)

<br><br>

Simple car model used due to scaling issues. The camera on the correctly scaled car doesn't match the city scale and features are hard to detect.

