%%%%%%%%%%%%%%%%%%%%

CSE 510 : Robotics Algorithms
Term    : Spring 2013
HW#     : 2
Date of Submission : 03/14/2013

By

Surya Kumar Selvam
suryakum@buffalo.edu
UB# 50026556

File Name : finiteLQR.m

%%%%%%%%%%%%%%%%%%%%%

Details :

	This Homework deals with solving the finite LQR problem of a robot. The main functions in the script file are enumerated below
	
finiteLQR() - The main function which calls the problem 1 and problem 2
finiteLQRprob1() - Function to determine and initialize the initial conditions of problem 1 of the HW
finiteLQRprob2() - Function to determine and initialize the initial conditions of problem 2 of the HW
solveLQRprob1()  - Function to solve and plot the path of LQR in problem 1
solveLQRprob2()  - Function to solve and plot the path of LQR in problem 2

Hierrarchy :

finiteLQR()
	- finiteLQRprob1()
		- solveLQRprob1()
	- finiteLQRprob2() 
		- solveLQRprob2()
