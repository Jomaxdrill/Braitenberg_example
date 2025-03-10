# Braitenberg_example
Very Simple example of braitenberg vehicle behaviour as homework 3 for ENMP690 Robot Learning 
## Operating system
Ubuntu 20.04
## Required installations
- Python >= 3.8
- ROS2 galatic
- Gazebo classic (consider migration for 2025 to Gazebo Garden)
- RVIZ2
## GENERAL SETUP
-Verify you have ROS2 galatic distribution installed and also CMAKE necessary installations. Refer to: https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html
On command line run:
```sh
echo $ROS_DISTRO
```
-Install previously the following packages and any additional in your linux distribution running on the terminal the command:
```sh 
sudo apt install python3-colcon-common-extensions
```
-Install all necessary dependencies and libraries with pip for instance. Recommended to run in your own python environment.

### Create the workspace and install all dependencies priorly
Create a folder in your system and locate the src pkg
```sh
  mkdir -p ~/test_ws/src
  cd ~/test_ws/src
```
Clone the repository package inside this folder

```sh
   git clone https://github.com/Jomaxdrill/Braitenberg_example.git
```

You should get two folders called **braitenberg_vehicle** and **braitenberg_vehicle_description**. These folders are the main pkg with the running scripts and gazebo launch and the other pkg with the urdf files to display the robot respectively. 


