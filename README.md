# Object-Track-and-Follow

Quadcopters are flying drones with a highly constrained payload capacity, limited com-
putation power as well as communication capabilities. Such quadcopter can be equipped
with a low weight camera and additional computer. There are many applications where
the autonomous tracking and following of a moving object is desired. For example filming
of fast moving athletes during sport events, studying wild life in difficult to access regions
or surveillance and pursuing of criminals.
This reasons made us create a new vision-based dynamic object track and follow solution
for quadcopters. The solution can be deployed on typical consumer grade drones, as
it respects their payload, computation and communication constrains. Outcome of our
solution is a drone able to track and follow an object that is chosen by an user in a video
frame during initialisation.
In our project we first evaluate few promising computer vision algorithms for object
tracking based on their suitability for a quadcopter. Than we choose the most suitable
trackers which performance is tested on a use case video and evaluated based on various
criteria. The most suitable tracker is implemented in
Robot Operating System (ROS)
framework which is used for communication between the drone and the
Ground Control
Station
. The setup is tested on a two drone platforms as well as in a simulation.
We found the right combination of on-board computer and tracker which are suitable for
our project. We created test-video which evaluates the trackers and another video which
shows the final result during the flight.

#Structure
<ul>
<li>mavlink interpreter is a helping package which translated the override RC commands in the APM</li>
<li>object_follow is the tracking and following solution</li>
</ul>

#Compile and run the code
Thic code was developed under ROS Indigo. Follow ROS Webpage for Indigo [installation](http://wiki.ros.org/indigo/Installation/Ubuntu). 

Create a workspace as is in ros [docummentation indicated](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment).
In case you are not familiar with ROS environment, I suggest you to work through all tutorials from the ROS Webpage. Specialy launch files!
Clone object_follow, ros_opentld, mavros, tum_simulator from git to the workspace. 

```
cd src/
git clone https://github.com/kabzo/ros_opentld.git
git clone https://github.com/kabzo/Object-Track-and-Follow.git
git clone -b indigo-devel https://github.com/mavlink/mavros.git
git clone https://github.com/dougvk/tum_simulator.git
cd ..
rosdep install --from-paths src --ignore-src --rosdistro indigo -y

```
In case you will not use apm controll board erase package ```mavlink_apm_interpreter```

Now you need to install some additional packages
```
sudo apt-get install ros-indigo-ardrone-autonomy ros-indigo-control-toolbox
```
You might to need compile some of the packages separately:
```
catkin_make
catkin_make --pkg tld_msgs
catkin_make --pkg object_follow
catkin_make
```

Launch tum_simulator if you dont have your own camera source
```
roslaunch cvg_sim_gazebo ardrone_testworld.launch
```

Now you can launch the control GUI and tracker with 
```
roslaunch object_follow gui.launch
roslaunch object_follow tld_tracker.launch 
```

in respective launch files (```roscd object_follow/launch```) you can change the camera source, by default it is set for ```tum_simulator``` ardrone camera topic
```
<arg name="image_topic" default="ardrone/front/image_raw"/>
```

The GUI control:
In tab1 you can choose which object to follow. By pressing ```F5``` you refresh the image and by drawing a rectangle around the object of choice you pick the bounding box. Than pres Enter to start tracking. If the drone sees the object launch the tracking by ticking the TRACKING OBJECT box. You might need to change some PID constants which you can change dynamicaly by [ROS parameter server](http://wiki.ros.org/rqt) (no need of recompilation).
For closer information follow [http://www.kabzanj.net/file/object_follow](http://www.kabzanj.net/file/object_follow)

I highly suggest to install qtcreator for development and debugging. Open is as CMake project and you are ready to go.


