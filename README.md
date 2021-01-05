# **Capstone Project: Programming a Real Self-Driving Car**

[//]: # (Image References)


[image0]: ./writeup/covershot.png "cover"
[image1]: ./writeup/Goal.png "architecture"
[image2]: ./writeup/ROS.png "ROS pubsub-architecture"
[image3]: ./writeup/perception.png "nn tl_detection"


---

![alt text][image0]

---


## Overview

This is the project repo for the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. For more information about the project, see the project introduction [here](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/5ab4b122-83e6-436d-850f-9f4d26627fd9).


## Goal

The goal of this project was to enable Carla(Udacity’s real self-driving car) to drive around the test track using waypoint navigation. Carla did this while avoiding obstacles and stopping at traffic lights. Waypoints are simply an ordered set of coordinates that Carla uses to plan a path around the track. Each of these waypoints also has an Associated target velocity. Carla’s planning subsystem updates the target velocity for the waypoints ahead of the vehicle depending on the desired vehicle behavior. Carla’s control subsystem actuates the throttle, steering and brake to successfully navigate with the correct target velocity. Components of perception, planning, and control subsystems were implemented.

![alt text][image1]


---

## Implementation

The project was implemented on the ROS framework and Ubuntu Linux environment(refer to 'install' section), such that it run on both Udacity simulator and Carla. The beloww is the ROS nodes and pub-sub architecture of SDC system.

![alt text][image2]

Udacity Self-Driving Car Harware Specs:

- 31.4 GiB Memory
- Intel Core i7-6700K CPU @ 4 GHz x 8
- TITAN X Graphics
- 64-bit OS

#### Waypoint updater node

The eventual purpose of this node is to publish a fixed number of waypoints ahead of the vehicle with the correct target velocities, depending on traffic lights and obstacles. The goal for the first version of the node should be simply to subscribe to the topics

    - /base_waypoints
    - /current_pose

and publish a list of waypoints to

    - /final_waypoints

- **waypoint_updater.py**

The /base_waypoints topic publishes a list of all waypoints for the track, so this list includes waypoints both before and after the vehicle (note that the publisher for /base_waypoints publishes only once). For this step in the project, the list published to /final_waypoints should include just a fixed number of waypoints currently ahead of the vehicle:

The first waypoint in the list published to /final_waypoints should be the first waypoint that is currently ahead of the car.
The total number of waypoints ahead of the vehicle that should be included in the /final_waypoints list is provided by the LOOKAHEAD_WPS variable in waypoint_updater.py.


Waypoint updater node sets target velocity for each waypoint based on upcoming traffic lights and obstacles. For example, if Carla sees a red traffic light on the horizon, it sets decelerating velocities at the nodes leading up to the traffic light. Based on /traffic_waypoint information, it changes the waypoint target velocities before publishing to /final_waypoints. 
From the code in waypoint_updater.py, both the /final_waypoints and /base_waypoints topics have message type Lane as below. To decelerate, a square root function emulates pressing the brake pedal to be getting exponentially harder as distance decreases and the output velocity values are written as the linear velocity values of the waypoint. 

```Shell
std_msgs/Header header
:
styx_msgs/Waypoint[] waypoints
  geometry_msgs/PoseStamped pose
    std_msgs/Header header
    :
    geometry_msgs/Pose pose
      geometry_msgs/Point position
        float64 x
        float64 y
        float64 z
      geometry_msgs/Quaternion orientation
      :
  geometry_msgs/TwistStamped twist
    std_msgs/Header header
    :
    geometry_msgs/Twist twist
      geometry_msgs/Vector3 linear
        float64 x
        float64 y
        float64 z
      geometry_msgs/Vector3 angular
      :
```

#### DBW node

Once messages are being published to /final_waypoints, the vehicle's waypoint follower publish twist commands to the /twist_cmd topic. The drive-by-wire node (dbw_node.py) subscribe to /twist_cmd and use various controllers to provide appropriate throttle, brake, and steering commands. These commands can then be published to the following topics:

    - /vehicle/throttle_cmd
    - /vehicle/brake_cmd
    - /vehicle/steering_cmd

Since a safety driver may take control of the car during testing, you should not assume that the car is always following your commands. If a safety driver does take over, your PID controller mistakenly accumulate error, so you need to be mindful of DBW status to be turned off appropriately. The DBW status can be found by subscribing to /vehicle/dbw_enabled. When operating the simulator please check DBW status and ensure that it is in the desired state. DBW can be toggled by clicking "Manual" in the simulator GUI.

- **dbw_node.py**

This python file implements the dbw_node publishers and subscribers. There are ROS subscribers for the /current_velocity, /twist_cmd, and /vehicle/dbw_enabled topics. This file also imports the Controller class from twist_controller.py which is used for the necessary controllers. The function used to publish throttle, brake, and steering is publish.

The throttle values passed to publish are in the range 0 to 1, although a throttle of 1 means the vehicle throttle is fully engaged. Brake values passed to publish are in units of torque (N*m). The correct values for brake can be computed using the desired acceleration, weight of the vehicle, and wheel radius.

- **twist_controller.py**
    
This file contains a stub of the Controller class. We can use this class to implement vehicle control. For example, the control method can take twist data as input and return throttle, brake, and steering values. Within this class, we can import and use the provided pid.py and lowpass.py if needed for acceleration, and yaw_controller.py for steering. 

**Note** that dbw_node.py is currently set up to publish steering, throttle, and brake commands at 50hz. The DBW system on Carla expects messages at this frequency, and would disengage (reverting control back to the driver) if control messages are published at less than 10hz. This is a safety feature on the car intended to return control to the driver if the software system crashes. 
Additionally, although the simulator displays speed in mph, all units in the project code use the metric system, including the units of messages in the /current_velocity topic (which have linear velocity in m/s).
Finally, Carla has an automatic transmission, which means the car will roll forward if no brake and no throttle is applied. To prevent Carla from moving requires about 700 Nm of torque.


#### Traffic Light Detection Node

Once the vehicle is able to process waypoints, generate steering and throttle commands, and traverse the course, it also need stop for traffic lights and obstacles. Traffic light detection node can be split into 2 parts, detection and waypoint publishing.

![alt text][image3]

- **tl_detector.py**

This python file processes the incoming traffic light data and camera images. It uses the light classifier to get a color prediction, and publishes the location of any upcoming red lights. The topic /vehicle/traffic_lights contains the exact location and status of all traffic lights in simulator, so you can test your output. The traffic light detection node (tl_detector.py) subscribes to four topics:
   
        - /base_waypoints provides the complete list of waypoints for the course.
        - /current_pose can be used to determine the vehicle's location.
        - /image_color which provides an image stream from the car's camera. These images are used to determine the color of upcoming traffic lights.
        - /vehicle/traffic_lights provides the (x, y, z) coordinates of all traffic lights.
        
Once you have correctly identified the traffic light and determined its position, you can convert it to a waypoint index and publish it on the topic /traffic_waypoint. The node should publish the index of the waypoint for nearest upcoming red light's stop line to a single topic:

        - /traffic_waypoint

- **tl_classifier.py**
    
This file contains the TLClassifier class. The get_classification method can take a camera image as input and return an ID corresponding to the color state of the traffic light in the image. Note that Carla currently has TensorFlow 1.3.0 installed.

---


## Install

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

