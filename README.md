# **Capstone Project: Programming a Real Self-Driving Car**

[//]: # (Image References)


[image0]: ./data/43mph.png "running_at_43mph"
[image1]: ./writeup/Goal.png "architecture"
[image2]: ./writeup/ROS.png "ROS pubsub-architecture"
[image3]: ./writeup/speed1_curv.png "auto-tunning on curv at speed1"


---

![alt text][image0]

[video](https://www.youtube.com/watch?v=AJfq0BIkAko)

---


## Overview

This is the project repo for the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. For more information about the project, see the project introduction [here](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/5ab4b122-83e6-436d-850f-9f4d26627fd9).


## Goal

The goal of this project was to enable Carla(Udacity’s real self-driving car) to drive around the test track using waypoint navigation. Carla did this while avoiding obstacles and stopping at traffic lights. Waypoints are simply an ordered set of coordinates that Carla uses to plan a path around the track. Each of these waypoints also has an Associated target velocity. Carla’s planning subsystem updates the target velocity for the waypoints ahead of the vehicle depending on the desired vehicle behavior. Carla’s control subsystem actuates the throttle, steering and brake to successfully navigate with the correct target velocity. Components of perception, planning, and control subsystems were implemented.

![alt text][image1]


## Implementation

The project was implemented on the ROS framework of Udacity. It works both in Udacity simulator and Carla. 

![alt text][image2]

- **Perception**

Traffic light detection node can be split into 2 parts:
Detection: Detect the traffic light and its color from the /image_color. The topic /vehicle/traffic_lights contains the exact location and status of all traffic lights in simulator, so you can test your output.
Waypoint publishing: Once you have correctly identified the traffic light and determined its position, you can convert it to a waypoint index and publish it on the topic /traffic_waypoint


- **Planning**

Waypoint updater node subscribes to /base_waypoints and /current_pose and publishes to /final_waypoints. The /base_waypoints topic publishes a list of all waypoints for the track, so this list includes waypoints both before and after the vehicle. The list published to /final_waypoints include just a fixed number of waypoints currently ahead of the vehicle. The first waypoint in the list published to /final_waypoints is the first waypoint that is currently ahead of the car. The total number of waypoints ahead of the vehicle that is included in the /final_waypoints list is provided by the LOOKAHEAD_WPS, which may vary depending on the computing limits.

Waypoint updater node sets target velocity for each waypoint based on upcoming traffic lights and obstacles. For example, if Carla sees a red traffic light on the horizon, it sets decelerating velocities at the nodes leading up to the traffic light. Based on /traffic_waypoint information, it changes the waypoint target velocities before publishing to /final_waypoints. 


- **Control**

Once your waypoint updater node is publishing /final_waypoints, the waypoint_follower node will start publishing messages to the /twist_cmd topic. Drive-By-Wire(DBW) node takes target trajectory information as input and sends control commands to navigate the vehicle. Based on /twist_cmd published by waypoint_follower node, the car could should drive in the simulator and stop at red traffic lights and move when they are green.



## Testing

Udacity Self-Driving Car Harware Specs

- 31.4 GiB Memory
- Intel Core i7-6700K CPU @ 4 GHz x 8
- TITAN X Graphics
- 64-bit OS


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
To set up port forwarding, please refer to the "uWebSocketIO Starter Guide" found in the classroom (see Extended Kalman Filter Project lesson).

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

### Other library/driver information
Outside of `requirements.txt`, here is information on other driver/library versions used in the simulator and Carla:

Specific to these libraries, the simulator grader and Carla use the following:

|        | Simulator | Carla  |
| :-----------: |:-------------:| :-----:|
| Nvidia driver | 384.130 | 384.130 |
| CUDA | 8.0.61 | 8.0.61 |
| cuDNN | 6.0.21 | 6.0.21 |
| TensorRT | N/A | N/A |
| OpenCV | 3.2.0-dev | 2.4.8 |
| OpenMP | N/A | N/A |

We are working on a fix to line up the OpenCV versions between the two.
