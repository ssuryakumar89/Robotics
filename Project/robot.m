classdef robot
    properties
        x;
        y;
        orient;
        forw_noise;
        sense_noise;   
        turn_noise;
    end
    
    methods
        
       function obj = robot(R)
            if nargin==1
                obj.x = R.x;
                obj.y = R.y;
                obj.orient = R.orient;
                obj.forw_noise = R.forw_noise;
                obj.sense_noise = R.sense_noise;
                obj.turn_noise = R.turn_noise;
            else
                obj.x = rand()* 100;
                obj.y = rand()* 100;
                obj.orient = 0;
                obj.forw_noise=0;
                obj.sense_noise=0;
                obj.turn_noise =0;
            end
       end
       
       % Function to set values for an object robot
       function obj = set_values(obj,new_x,new_y,new_orient)
            obj.x = new_x;
            obj.y = new_y;
            obj.orient = new_orient;           
       end
        
        % Function to set noise of the robot
        function res = set_noise(obj,new_fnoise,new_snoise,new_tnoise)
            obj.forw_noise = new_fnoise;
            obj.sense_noise = new_snoise;
            obj.turn_noise = new_tnoise;
            res = obj;
        end
        
        % The observation function which gives the minimum of the distance
        % to the beacons 
        function Z = sense(obj)
            dist = obj.find_shortest_distance(obj.x,obj.y,obj.orient);
            dist = dist+randn()*obj.sense_noise;
            Z = dist;
        end
        
        % The move function 
        function res = move(obj,turn,forward)
            orientation = obj.orient + turn +randn()*obj.turn_noise;
            orientation = mod(orientation,2 * pi);
            dist = forward + randn()*(obj.forw_noise); % Forward noise in the movement
            new_x = obj.x + (cos(orientation) * dist);
            new_y = obj.y + (sin(orientation) * dist);
            res = robot();
            yy = res.set_values(new_x,new_y,orientation);
            tt = yy.set_noise(obj.forw_noise, obj.sense_noise,obj.turn_noise);
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
            dist = obj.find_shortest_distance(obj.x,obj.y,obj.orient);
            %display(obj.sense_noise);
            prob = prob * obj.Gaussian(dist, obj.sense_noise, measurement);
        end
        
        % Fucntion to display the object's coordinates
        function disp(obj)
            display(['x = ' num2str(obj.x) '; y = ' num2str(obj.y) '; orient = ' num2str(obj.orient) ' ; sense_noise = ' num2str(obj.sense_noise) ' ; ']);
        end      
        
        function dist = find_shortest_distance(obj,x3,y3,orient)
            if(orient==0)
                x1 = 100;
                x2 = 100;
                y1 = 0;
                y2 = 100;
            end
            if(orient==0.5)
                pause(10);
                x1=0;
                x2=100;
                y1=100;
                y2=100;
            end
            if(orient==1)
                x1 = 0;
                x2 =0;
                y1=0;
                y2=100;
            end
            if(orient==1.5)
                x1=0;
                x2=100;
                y1=0;
                y2=0;
            end
            px = x2-x1;
            py = y2-y1;
            temp = px*px + py*py;
            u = ((x3-x1)*px + (y3-y1)*py)/temp;
            if u>1
                u = 1;
            end
            if u<1
                u =0;
            end
            xx = x1 + u * px;
            yy = y1 + u * py;
            dx = xx - x3;
            dy = yy - y3;
            dist = sqrt(dx*dx + dy*dy); 
        end
        
        
    end
    
end