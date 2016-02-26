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
