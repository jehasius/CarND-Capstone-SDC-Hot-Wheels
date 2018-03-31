# CarND-Capstone-SDC-Hot-Wheels
---
1. Introduction
2. Team
3. Setup and Compilation
4. Original Udacity README

[//]: # (Image References)
[image1]: ./20180301_plannertest.png  "Planner in action"

##1. Introduction
This project was the final project of the Udacity Self-Driving Car Enginer Nanodegree course. The goal of this project was to enable a [self driving car called Carla](https://medium.com/udacity/how-the-udacity-self-driving-car-works-575365270a40) to drive around our test track using waypoint navigation. Carla should do this while avoiding obstacles and stopping at traffic lights by using waypoint navigation. Waypoints are simply an ordered set of coordinates that Carla uses to plan a path around the track. Each of these waypoints also has an associated target velocity. Carla's planning subsystem updates the target velocity for the waypoints ahead of the vehicle depending on the desired vehicle behavior. Carla's control subsystem actuates the throttle, steering, and brake to successfully navigate the waypoints with the correct target velocity.
To fullfill this tasks it was necessary to implement components of the perception, planning, and the control subsystems. In the perception subsystem a traffic light detection was implemented. In the planning subsystem the team implemented a node called the waypoint updater. This node was used to set the target velocity for each waypoint based on the upcoming traffic lights and obstacles. In the control subsystem the team has implemented a drive by wire ROS node that takes target trajectory information is input and sends control commands to navigate the vehicle.
Do do the ROS framework for these nodes as well as a version of the simulator that includes traffic lights and obstacles were provided by Udacity. The provided ROS framework works with both the Udacity simulator and with Carla.

##2. Team
For this project it was crucial to work as a groups of up to 5 students with one team lead for the group. For communication we used a private Slack channel. To coordinate and divide the work a Trello Board was used and the source code was shared by this repository. The consist of the following members (in alphabetical order).

* K. Daubner(Germany)
* J. Hampe(Germany)
* B. Khalid(Dubai)
* A. Menge, Team Lead (Germany)
* T. Steinert(Germany)

##3. Setup and Compilation
You can find detailed information about the dependencies and compilation in chapter [5.Original Udacity README](#Udacity).

<a name="Udacity"></a>
##4. Original Udacity README

This is the project repo for the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. For more information about the project, see the project introduction [here](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/5ab4b122-83e6-436d-850f-9f4d26627fd9).

Please use **one** of the two installation options, either native **or** docker installation.

### Native Installation

* Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir. [Ubuntu downloads can be found here](https://www.ubuntu.com/download/desktop).
* If using a Virtual Machine to install Ubuntu, use the following configuration as minimum:
  * 2 CPU
  * 2 GB system memory
  * 25 GB of free hard drive space

  The Udacity provided virtual machine has ROS and Dataspeed DBW already installed, so you can skip the next two steps if you are using this.

* Follow these instructions to install ROS
  * [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu 16.04.
  * [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) if you have Ubuntu 14.04.
* [Dataspeed DBW](https://bitbucket.org/DataspeedInc/dbw_mkz_ros)
  * Use this option to install the SDK on a workstation that already has ROS installed: [One Line SDK Install (binary)](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/81e63fcc335d7b64139d7482017d6a97b405e250/ROS_SETUP.md?fileviewer=file-view-default)
* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases).

### Docker Installation
[Install Docker](https://docs.docker.com/engine/installation/)

Build the docker container
```bash
docker build . -t capstone
```

Run the docker file
```bash
docker run -p 4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone
```

### Port Forwarding
To set up port forwarding, please refer to the [instructions from term 2](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77)

### Usage

1. Clone the project repository
```bash
git clone https://github.com/udacity/CarND-Capstone.git
```

2. Install python dependencies
```bash
cd CarND-Capstone
pip install -r requirements.txt
```
3. Make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
4. Run the simulator

### Real world testing
1. Download [training bag](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/traffic_light_bag_file.zip) that was recorded on the Udacity self-driving car.
2. Unzip the file
```bash
unzip traffic_light_bag_file.zip
```
3. Play the bag file
```bash
rosbag play -l traffic_light_bag_file/traffic_light_training.bag
```
4. Launch your project in site mode
```bash
cd CarND-Capstone/ros
roslaunch launch/site.launch
```
5. Confirm that traffic light detection works on real life images
