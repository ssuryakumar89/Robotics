%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

CSE 510 - Robotics Algorithms
Home Work 4
Histogram Filter 
By Surya Kumar Selvam
UB# 50026556
suryakum@buffalo.edu

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Files Submitted :

1) histogram_filter.m
2) prior.pdf 
3) 8 plots( hist_1 through hist_8 for the 8 timestamps )

____________________________________________________________________________________________

Noise Probablities Used :

Process model ( The function move in the program)

1) Probability of agent moving 1 step = 0.9
2) Probability of agent failing to take a move = 0.1

Observation model ( The function sense in the program )

1) Probablity of correctly identigying colour = 0.8
2) Probablity of error in the colour sense = 0.2
___________________________________________________________________________________________
Notes :

The states are dneoted as 

     0     0     0     0     0     1
     2     3     4     5     6     7
     0     0     0     0     0     8
     9    10    11    12    13    14
     0     0     0     0     0    15
    16    17    18    19    20    21
     0     0     0     0     0    22

______________________________________________________________________________________________
Sample Output

string =

Prob after timestep 1


prob_distribution =

         0         0         0         0         0    0.0064
    0.0064    0.0637    0.0637    0.0637    0.0637    0.0637
         0         0         0         0         0    0.0064
    0.0064    0.0637    0.0637    0.0637    0.0637    0.0637
         0         0         0         0         0    0.0064
    0.0064    0.0637    0.0637    0.0637    0.0637    0.0637
         0         0         0         0         0    0.0064


string =

Prob after timestep 2


prob_distribution =

         0         0         0         0         0    0.0008
    0.0008    0.0150    0.0791    0.0791    0.0791    0.0791
         0         0         0         0         0    0.0008
    0.0008    0.0150    0.0791    0.0791    0.0791    0.0791
         0         0         0         0         0    0.0008
    0.0008    0.0150    0.0791    0.0791    0.0791    0.0791
         0         0         0         0         0    0.0008


string =

Prob after timestep 3


prob_distribution =

         0         0         0         0         0    0.0001
    0.0001    0.0028    0.0274    0.1010    0.1010    0.1010
         0         0         0         0         0    0.0001
    0.0001    0.0028    0.0274    0.1010    0.1010    0.1010
         0         0         0         0         0    0.0001
    0.0001    0.0028    0.0274    0.1010    0.1010    0.1010
         0         0         0         0         0    0.0001


string =

Prob after timestep 4


prob_distribution =

         0         0         0         0         0    0.0000
    0.0000    0.0005    0.0073    0.0478    0.1389    0.1389
         0         0         0         0         0    0.0000
    0.0000    0.0005    0.0073    0.0478    0.1389    0.1389
         0         0         0         0         0    0.0000
    0.0000    0.0005    0.0073    0.0478    0.1389    0.1389
         0         0         0         0         0    0.0000


string =

Prob after timestep 5


prob_distribution =

         0         0         0         0         0    0.0000
    0.0000    0.0001    0.0019    0.0181    0.0910    0.2222
         0         0         0         0         0    0.0000
    0.0000    0.0001    0.0019    0.0181    0.0910    0.2222
         0         0         0         0         0    0.0000
    0.0000    0.0001    0.0019    0.0181    0.0910    0.2222
         0         0         0         0         0    0.0000


string =

Prob after timestep 6


prob_distribution =

         0         0         0         0         0    0.0000
    0.0000    0.0000    0.0003    0.0026    0.0130    0.0317
         0         0         0         0         0    0.2857
    0.0000    0.0000    0.0003    0.0026    0.0130    0.0317
         0         0         0         0         0    0.2857
    0.0000    0.0000    0.0003    0.0026    0.0130    0.0317
         0         0         0         0         0    0.2857


string =

Prob after timestep 7


prob_distribution =

         0         0         0         0         0    0.0000
    0.0000    0.0000    0.0000    0.0004    0.0019    0.0045
         0         0         0         0         0    0.0816
    0.0000    0.0000    0.0000    0.0004    0.0019    0.3719
         0         0         0         0         0    0.0816
    0.0000    0.0000    0.0000    0.0004    0.0019    0.3719
         0         0         0         0         0    0.0816


string =

Prob after timestep 8


prob_distribution =

         0         0         0         0         0    0.0000
    0.0000    0.0000    0.0000    0.0000    0.0002    0.0005
         0         0         0         0         0    0.0133
    0.0000    0.0000    0.0000    0.0000    0.0002    0.1202
         0         0         0         0         0    0.3725
    0.0000    0.0000    0.0000    0.0000    0.0002    0.1202
         0         0         0         0         0    0.3725



