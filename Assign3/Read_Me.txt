%-----------------------------------------------------%
%-----------------------------------------------------%
CSE 510 - HW3 Path Planning Problems

 - Submitted by Surya Kumar Selvam
 - UB# 50026556
 - Date of Submission = 04/10/2013
 - language used = matlab

%-----------------------------------------------------%
%-----------------------------------------------------%

% Files submitted

i) PRM2.m  - For the PRM path finding problem
ii) RRT.m  - For the RRT path finding problem
iii) Read_Me.txt - This file
iv) PRM_theta1_vs_theta2 - an example plot of theta 1 vs theta 2 in PRM with the path shown
v) logprm.txt - log file for the running of PRM2.m 
vi) logrrt.ext - log file for the running of RRT.m
vii) Arms_plotted - plot of various configs of the robot arms generated
viii) RRT_theta1_vs_theta2 - an example plot of the theta1 vs theta2 in RRT

%-----------------------------------------------------%
% Values and parameters used

i) PRM
     * neighbour hood radius = 0.1
	 * distance function used = w * norm(fistconfig - secondconfig) + (1 - w) *(abs( theta1_diff) + abs(theta2_diff ))
	 * weight = 0.1 ( Reason - Since the first point is fixed, the difference in angles value more here ) 
	 * Number of samples = 100
	 * 1 of the Run of the program gives 
		Milestones = 65
		Rejected = 37
		Number of nodes in path = 50
	 
ii) RRT
	 * goal sampling probability = 0.1
	 * Number of Samples = 1000
	 * distance function used = w * norm(fistconfig - secondconfig) + (1 - w) *(abs( theta1_diff) + abs(theta2_diff ))
	 * weight = 0.15 ( Reason - Since the first point is fixed, the difference in angles value more here ) 
	 * Number of samples = 1000
	 
%-----------------------------------------------------%	 
% Results Obtained 

i) RRT
	* Total No of Samples : 100
	* Mile Stones (including start,goal) : 54
	* Rejected Nodes : 48
	* Could not connect to goal within the sample space 
	 
ii) PRM
	 * Number of nodes = 102
	 * Edges = 50
	 * length of shortest_path = 50
%-----------------------------------------------------%
% Utility Functions Used 

PRM & RRT
		* find_arm() - To find the postion of the arm given a configuration
		* plot_arm() - To plot an arm 
		* draw_arena() - To draw the Config space with obstacles and to plot all the arms
		* check_collision() - To check if a given configuration of arm collides with the obstacle
		* find_neighbours() - To find the neighbours of a given node ( In case of RRT, it gives out the index of the nearest node in the tree)
		* isConnected() - to check if two points can be connected in the given config space 
		* final_plot() - to plot all the required graph plots
		
%-----------------------------------------------------%

Notes : PRM2.m which implements PRM algorithm takes very high time.I could not find out the reason for that
   









	 
