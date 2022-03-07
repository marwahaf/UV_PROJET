# Robot d'accueil
## General information

    UV : UV Projet
    Participants : Gaétan Helie et Marwa Hafsi
    Date : 04/02/2021 -> 04/03/2022

## Objectif of the project : 

This project aims to create a robot that : 
	- Locates a person and goes toward her 
	- Has a visual interface with all the rooms of the building 
	- Interacts with the person in order to know where she wants to go 
	- guides the person to the wanted room
	- goes back to its original position once the goal achieved
	
## Content of the repository  

- This repository contains 3 python programs . They are located in the package "robot_accueil", in the folder "scripts". Their purposes are :
 
 1 - The program used to control the graphic interface using the "tkinter" library in python 
 > gui.py
	 
 2- The program used to move the robot : it allows the robot to go toward a person once spotted, to go to a certain goal once set through the graphic interface and allows the robot to come back to its original position.
 All of the above while avoiding obstacles. 
> get_goal.py
	 
 3-The program used to analyze the data of the camera : it identifies people and calculates the distance to get to them returning a goal.
 > front_camera.py
	 
- It contains two launch files "robot.launch" and robot_scripts.launch.
"robot.launch" file launches : 
	- If the rviz parameter is set to true, it launches a visualization with rviz tool in a specific config rviz file "config.rviz" . The Rviz files can also be found in the "robot_acceuil" package under the rviz file .
	- AMCL : that localize the robot on the map using the laser scans and Monte Carlo method
	- MapServer: that loads the map saved 

"robot_scripts.launch" file launches: 
	- all 3 python scripts
WARNING : some bugs may appear if robot_scripts.launch is launched before robot.launch !
Make sure to launch robot.launch before robot_scripts.launch. 

> roslaunch robot_accueil robot.launch rviz:=true

> roslaunch robot_accueil robot_scripts.launch

- It also contains a "dnn-model" file which has all the data allowing the recognition of a person .

- The repository contains a "map" file which has the saved map of the environment created using map_server map_saver

## Amelioration perspectives : 

- The people detection can be upgraded
- The robot moving decision can be optimized
- In the script "front_camera.py" we can get the size of the object detected and then send it with the marker so we have aproximately the size of the object.

