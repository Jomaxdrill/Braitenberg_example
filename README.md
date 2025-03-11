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
  mkdir -p ~/ROBOT_WS/src
  cd ~/ROBOT_WS/src
```
Clone the repository package inside this folder

```sh
   git clone https://github.com/Jomaxdrill/Braitenberg_example.git
```

You should get two folders called **braitenberg_vehicle** and **braitenberg_vehicle_description**. These folders are the main pkg with the running scripts and gazebo launch and the other pkg with the urdf files to display the robot respectively. 

Copy them to the **ROBOT_WS/src** folder so your folder structure look like this:

![image](https://github.com/user-attachments/assets/39566412-4aa2-4217-af5d-293d5e314960)


Downlod all the dependencies before runnning a build.
```sh
rosdep install -i --from-path src --rosdistro galactic -y
```
Run this command to be at root of your workspace (~/ROBOT_WS) and build your workspace
```sh
cd ../
colcon build 
```
![image](https://github.com/user-attachments/assets/9da418cf-b92b-4d76-950f-1f648b35f652)

Source ROS (Package will be identified) However you can do make this default when opening the terminal by modifying the .bashrc file. <ins>Don't forget your user password to give permissions </ins>
```sh
sudo nano ~/.bashrc
```
![image](https://github.com/user-attachments/assets/56625fea-d3f4-4354-8d2e-7433444ea24b)

```sh
 source /opt/ros/galactic/setup.bash
```
**Source package to be identified**. Do this for every new terminal you open locating the Workspace folder:

```sh
source install/setup.bash
```
### Running scripts 

#### Run gazebo

```sh
ros2 run braitenberg vehicle empty_world.launch.py
```

In case you want to reset use the option reset world. From this point also you can add objects as you consider for the simulation
![image](https://github.com/user-attachments/assets/78555c92-20c4-4e2f-b460-d4077f9803e9)

### Run teleoperation

In other terminal run 

```sh
source install/setup.bash
ros2 run braitenberg_vehicle teleop_keyboard.py
```

You should get the following terminal screen. Use it as stated to move, turn and stop the robot. CTRL+C to end script.
![image](https://github.com/user-attachments/assets/bde8dd9f-9b37-4c2a-92e4-b9a1d8030bad)

### Run autonomous mode

The autonomous scripts provides to user defined parameters to pass:
speed: default to 0.22 m/s if ommitted. Set the turtlebot3 max linear speed.
behaviour: default is fearful. Set currently between fearful (vehicle 2A) and aggressive (vehicle 2B)

In other terminal run:
```sh
source install/setup.bash
ros2 run braitenberg_vehicle braintenberg_behavior.py --ros-args -p behavior:="BEHAVIOR_HERE" -p speed:="SPEED_HERE"
```
You should get the following terminal screen. Add objects to the gazebo enviroment in the Robot LIDAR range to check out for the changes the robot will do depending the behavior.

![image](https://github.com/user-attachments/assets/3aa61375-ac5a-48ef-ac26-065d75d6fd4b)

![image](https://github.com/user-attachments/assets/dff325f3-62f1-4acd-9755-4ae7a4f49f1e)

## Video demonstrations
Teleoperation https://drive.google.com/file/d/1Zs254Z68APxohSS3amW5wCYu_IokDcOV/view?usp=sharing
Autnomous mode https://drive.google.com/file/d/1EJPw2p9TDKc5ZOOGISl_-vvAzcBT-rRw/view?usp=sharing

