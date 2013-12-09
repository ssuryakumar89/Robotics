function kf

clc;
close all;
clear all;
diary(fullfile('G:', 'myDiary2.txt'));

% The main two parts here are the state transition function and the
% measurement update function

% State transition function 
% [ Xt+1            [ 1 0 0      [Xt          [ costheta 0   [ d 
%   Yt+1      =       0 1 0    *  Yt      +     sintheta 0 *   alpha]
%   THEt+1 ]          0 0 1 ]     THEt]         0 1  ]

% Alpha is the steering angle of the robot

% The initial state of the robot 
x = [1
     1
     0];
 
 % The covariance matrix with high initial uncertainity
 P = eye(3) * 10000;
 
 % State Transition function
 F = eye(3);
 
 % The measurement update function
 H = [1 0 0
      0 1 0
      0 0 1];
  
 % The error in measurement
 R = [1 0 0
       0 1 0
       0 0 1];
   
 % Identity matrix
 I = eye(3);
 
 % Forward noise
 forward_noise = 0.05;
 
 % no of time steps
 T = 10;
 
 % Distance of each step
 d = 1;
 
 
 
    
      
      for i=1:T
         figure(i);
         hold on;
         axis([-20 20 -20 20]);
         pause(0.5);  
         display('Time Step 1');
         % measurement update step
         alpha = randn()*pi/2;
         Z = move(x,alpha,forward_noise,d);
         display('The measurement is ');
         display(Z);
         y = Z - H*x;
         S = H*P*H.' + R;
         K = P*H.'*inv(S);
         x = x + K*y;
         P = (I - K*H)*P;
         
         display('The covariance matrix is ');
         display(P);

         % motion update
         plot(Z(1),Z(2),'bo');
         x =F*x + [cos(x(3)*d)
                   sin(x(3)*d)
                   alpha];
          P = F*P*F.';
          display('Updated motion is ');
          display(x);
          plot(x(1),x(2),'r*');
          display('-------------------------------');
          hold off;
     end
     diary off; 
     
end
 
    function Z = move(x,alpha,forward_noise,d)
        dist = d + randn()*(forward_noise);
        x(1) = x(1)+cos(x(3))*dist;
        x(2) = x(2)+sin(x(3))*dist;
        x(3) = x(3)+alpha;
        a(1) = x(1);
        a(2) = x(2);
        a(3) = x(3);
        Z = a';       
    end
      

