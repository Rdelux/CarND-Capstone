# **Term 3 Capstone Project: Programming a Real Self-Driving Car**
***Self-Driving Car Engineer Nanodegree Program***

***Team Name:*** Blue Bird Dynamics

***Project Due:*** August 13, 2018

***Course Ends:*** August 27, 2018

***Team Members:***

| Member        | Role           | Email     |
| ------------- |----------------|:-------------:|
| Richard Lee (Team Leader) | DBW Nodes | richard-lee@outlook.com |
| Adam Preston | Waypoint Updater Full | adam.preston@gmail.com |
| Eric Tang | Waypoint Updater Partial / TL Detection| tang8156@gmail.com |



***The goals of this project are the following:***

* Team members will develop ROS nodes to implement core functionality of the autonomous vehicle system - traffic light detection, control, and waypoint following
* Carla will smoothly follow the waypoints in the simulator
* Carla will be driven at the target top speed set for the waypoints
* Carla will stop at traffic lights when needed
* The ROS nodes will publish throttle, steering, and brake commands at 50hz

[image1]: ./imgs/architecture.png "P1"

***Architecture***

The system architecture diagram showing the ROS nodes and topics are shown below:

![alt text][image1]

***Implementation***
The development was divided into 3 parts: drive-by-wire nodes, traffic light detection, and waypoint updater.  Each team member is responsible for developing the nodes required for each part and the team leader is also responsible to combine all these nodes to create the final implementation.

***DBW Nodes***

The purpose of the drive-by-wire (DBW) node is to publish the appropriate throttle, steering and brake command to either the simulator or Carla.  The waypoint_follower node received final_waypoints messages from the waypoint_updater node, and then publish twist messages on the twist_cmd topic to provide the linear and angular velocities to the DBW node.  The parameters that are required to calculate the throttle, brake and steering outputs include, wheel radius, wheel base, steer ratio, maximum lateral acceleration, maximum steering angle, maximum throttle, and maximum braking torque.  These values are obtained from the ROS parameter server. The various topics that need to be subscribed and publish in order to drive the vehicle are implemented in the dbw_node.py file under the twist_controller package. Control of the steering, throttle, and brake output are done in the twist_controller.py file, where a PID controller scheme was used.  The controller also distinguish between manual control and autonomous control.  Effectively, when one switch from manual to autonomous control, the controller will reset itself so that the cross track error will not be accumulated. A low-pass filter is also used to make the steering output less erratic and smoother.

***Traffic Light Detection***

The Traffic Light (TL) detection node need to determine the state of the up-coming traffic light and assign the appropriate speed profile for the vehicle based on the waypoint and the pose of the vehicle.  In the initial implementation, the ground truth of the traffic light was given, and algorithm for the stopping characteristics of the vehicle was development.  Although it was suggested in the project note that a classifier should be developed and many previous projects used a machine learning approach for the problem, our team thinks it is much more efficient to use a machine vision approach.  The machine vision approach involves first converting the images on the /image_color topic into grayscale image if the image is within 120 waypoints away.  Using Hough Circle to detect the circular traffic signals, the state of the traffic light can be determined by looking at the grayscale.  Using trial-and-error, anything between 140 and 190 is classified to be "Red Light", and anything over 230 is classified to be "Green Light".  Once the state of the traffic light is determined, the appropriate speed profile is assigned to the waypoints, leading up to the traffic line stop line.

***Waypoint Updater Full***

The waypoint_updater is responsible for providing a set of final waypoints for the vehicle to follow. It achieves this by updating the reference base_waypoints velocity when traffic_waypoints indicates that a red or amber light has been detected. If a valid index is received ahead of the vehicle then the velocity is adjusted to fall to zero at the received waypoint until it is safe to proceed. To improve efficiency when braking only the waypoints leading up to the target stopping point are modified and published. Once the light has changed full set of look ahead waypoints (200) are broadcast. A debounce class has also been implemented to reduce the effect of a classifier error.





## Project Repo

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
