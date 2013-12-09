classdef robot
    properties
        x;
        y;
        orient;
        forw_noise;
        sense_noise;        
    end
    
    methods
        
       function obj = robot(R)
            if nargin==1
                obj.x = R.x;
                obj.y = R.y;
                obj.orient = R.orient;
                obj.forw_noise = R.forw_noise;
                obj.sense_noise = R.sense_noise;
            else
                obj.x = rand()* 7;
                obj.y = rand()* 3;
                obj.orient = rand()* 2* pi;
                obj.forw_noise=0;
                obj.sense_noise=0;
            end
       end
       
       % Function to set values for an object robot
       function obj = set_values(obj,new_x,new_y,new_orient)
            obj.x = new_x;
            obj.y = new_y;
            obj.orient = new_orient;           
       end
        
        % Function to set noise of the robot
        function res = set_noise(obj,new_fnoise,new_snoise)
            obj.forw_noise = new_fnoise;
            obj.sense_noise = new_snoise;
            res = obj;
        end
        
        % The observation function which gives the minimum of the distance
        % to the beacons 
        function Z = sense(obj)
            dist1 = sqrt((obj.x - 2)^2 + (obj.y-0)^2);
            dist2 = sqrt((obj.x - 5)^2 + (obj.y-0)^2);
            Z = min(dist1,dist2);
        end
        
        % The move function 
        function res = move(obj,turn,forward)
            orientation = obj.orient + turn ;
            %orientation = mod(orientation,2 * pi);
            dist = forward + randn()*sqrt(obj.forw_noise); % Forward noise in the movement
            new_x = obj.x + (cos(orientation) * dist);
            new_y = obj.y + (sin(orientation) * dist);
            res = robot();
            yy = res.set_values(new_x,new_y,orientation);
            tt = yy.set_noise(obj.forw_noise, obj.sense_noise);
            res = tt;
                        
        end
        
        % Find the gaussian
        function gaussian = Gaussian(obj,mu,sigma_squared,x_temp)
            gaussian = exp(- ((mu - x_temp)^2) / (sigma_squared) / 2.0) / sqrt(2.0 * pi * (sigma_squared));
        end
        
        % This function gives the similarity of the particle to the distance sensed by the robot
        % measurement is the distance measured by the robot
        % Then for a particular particle this function measures the
        % distance, gets the measurement of the robot and then gives the
        % similartity to the robot
        function prob = measurement_prob(obj,measurement)
            prob = 1.0;
            dist1 = sqrt((obj.x - 2)^2 + (obj.y-0)^2);
            dist2 = sqrt((obj.x - 5)^2 + (obj.y-0)^2);
            dist = min(dist1,dist2);
            %display(obj.sense_noise);
            prob = prob * obj.Gaussian(dist, obj.sense_noise, measurement);
        end
        
        % Fucntion to display the object's coordinates
        function disp(obj)
            display(['x = ' num2str(obj.x) '; y = ' num2str(obj.y) '; orient = ' num2str(obj.orient) ' ; sense_noise = ' num2str(obj.sense_noise) ' ; ']);
        end      
    end
    
end