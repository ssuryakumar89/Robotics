%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

CSE 510 - Robotics Algorithms
Home Work 5
Spring 2013
Date of Submission : 05/02/2013

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Files Submitted : 

1) robot.m - The class file which gives the definition about the robot ( and the particles )
2) temp.m  - The script which implements the particle filtering using the robot object
3) prior,pf1 ... pf15 - Gives the plot about initial particles and after each of the 15 steps

Main Parameters used :

1) N = number of particles used = 500
2) T = number of time steps = 15
3) forward_noise = the forward movement noise of the robot = 0.25
4) sense_noise = the observation noise of the robot = 0.2

Note :

Since there was no wall mentioned, I have not ignored the points which goes to the negative points ( either negative x direction or negative y direction )
If the negative points have to be ignored, I have written a piece of code to eremove the negative points, please comment that out

