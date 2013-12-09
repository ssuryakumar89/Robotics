%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

CSE 510 - Robotics Algorithms

Final Project
Script : Matlab

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


TOPIC : Kalman Filtering and Parricle filtering for localization and tracking of a robot in a 2D planar environment


Files Submitted :
----------------------------

1) kf.m - Matlab file for the implementation of Kalman Filtering
2) robot.m - The class robot which implements all methods needed for Particle Filtering
3) temp.m - The matlab file which implements the Particle filtering



Specifications and Values used
-------------------------------

1) Kalman Filtering
   
    a) Initial Position = (x,y,orientaion) = (1,1,0)
	b) Intial Covariance = eye(3) * 10000
	c) Forward_noise = 0.05
	d) Number of time steps = 10
	e) Distance for each step = 1
	
2) Particle Filtering
   
    a) forward_noise = 0.1
    b) sense_noise = 0.1
    c) turn_noise = 0
    d) Initial Config of Robot = (5,5,0)
	e) No of Time steps = 15
	f) No of Particles Used = 1000
