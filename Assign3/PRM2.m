function PRM2()

    % Initial Parameters 
    
    global no_of_samples;
    no_of_samples = 100;           % No of samples
    arena_limits = [-3 3 -1 3];   % The limits of the arena
    start_theta1 = 45;
    start_theta2 = -45;
    goal_theta1 = 135;
    goal_theta2 = 45;
    qstart = find_arm(0,0,start_theta1,start_theta2); % Start Configuration
    qgoal = find_arm(0,0,goal_theta1,goal_theta2);    % Goal Configuration
    
   
    i=0;
    j=1;
    k=1;
    while(i<no_of_samples)
        i=i+1;
        x1_tmp = 0;
        y1_tmp = 0;
        theta1 = rand() *180;
        theta2 = rand() *360;
        tmp_robot = find_arm(x1_tmp, y1_tmp, theta1, theta2); % Getting a qrand
        isCollision = check_collision(tmp_robot); % Checking if the config is valid i.e., without colliding with the obstacles
        if(isCollision == 0)
            milestones(j,1) = {tmp_robot}; %coordinates of robot
            milestones(j,2) = {theta1}; % robot's first theta for first link
            milestones(j,3) = {theta2}; % robot's second theta for second link
            milestones(j,4) = {[]}; % neighbors
            j=j+1;
            display([num2str(i),' - added to the PRM graph']);
        else
            rejected(k,1) = {tmp_robot};
            rejected(k,2) = {theta1};
            rejected(k,3) = {theta2};
            rejected(k,4) = {[]};
            display([num2str(i),'- Config Rejected']);
            k = k+1;
        end
            
    end
    
    display('Finding and adding neighbours');
    % adds neighbors which are connected with the current node
    for j = 1 : length(milestones)
        Nq = find_neighbors(milestones,j,0.1);
        for m = 1 : length(Nq)
            if ((isConnected(milestones{Nq(m),1}(:,1),milestones{j,1}(:,1)) == 1) && (isConnected(milestones{Nq(m),1}(:,2),milestones{j,1}(:,2)) == 1) ...
                    && (isConnected(milestones{Nq(m),1}(:,3),milestones{j,1}(:,3)) == 1))
                milestones{j,4}(:,end+1) = Nq(m);
            end
        end
    end
    
    
    % solves query algorithm
    display('Adding Start Information');
    %adds start information of robot to milestones
    milestones(end+1,1) = {qstart}; %coordinates of robot
    milestones(end,2) = {start_theta1}; % robot's first theta for first link
    milestones(end,3) = {start_theta2}; % robot's second theta for second link
    milestones(end,4) = {[]}; % neighbors

    %adds goal information of robot to milestones
    display('Adding Goal Information');
    milestones(end+1,1) = {qgoal}; %coordinates of robot
    milestones(end,2) = {goal_theta1}; % robot's first theta for first link
    milestones(end,3) = {goal_theta2}; % robot's second theta for second link
    milestones(end,4) = {[]}; % neighbors

    Nq_init = find_neighbors(milestones, length(milestones), 0.1);
    Nq_goal = find_neighbors(milestones, length(milestones), 0.1);

       
    % adds neighbors which are connected with the start node
    display('Adding neighbours for start and goal');
    for m = 1 : length(Nq_init)
        a = (isConnected(milestones{Nq_init(m),1}(:,1),milestones{end - 1,1}(:,1)) == 1);
        b = (isConnected(milestones{Nq_init(m),1}(:,2),milestones{end - 1,1}(:,2)) == 1);
        c = (isConnected(milestones{Nq_init(m),1}(:,3),milestones{end - 1,1}(:,3)) == 1);
        if ( a && b && c )
            milestones{end - 1,4}(:,end+1) = Nq_init(m);
        end
    end


    % adds neighbors which are connected with the goal node
    for m = 1 : length(Nq_goal)
        a = isConnected(milestones{Nq_goal(m),1}(:,1),milestones{end - 1,1}(:,1)) == 1 ;
        b = (isConnected(milestones{Nq_goal(m),1}(:,2),milestones{end - 1,1}(:,2)) == 1);
        c = (isConnected(milestones{Nq_goal(m),1}(:,3),milestones{end - 1,1}(:,3)) == 1);
        if (a && b && c)
            milestones{end,4}(:,end+1) = Nq_goal(m);  
        end
    end
    
    % Doing again to include the added goal and start configuartion
    for j = 1 : length(milestones)
        Nq = find_neighbors(milestones,j,0.1);
        for m = 1 : length(Nq)
            if ((isConnected(milestones{Nq(m),1}(:,1),milestones{j,1}(:,1)) == 1) && (isConnected(milestones{Nq(m),1}(:,2),milestones{j,1}(:,2)) == 1) ...
                    && (isConnected(milestones{Nq(m),1}(:,3),milestones{j,1}(:,3)) == 1))
                milestones{j,4}(:,end+1) = Nq(m);
            end
        end

    end
      
    
    display('Searching for a path...');    
    % Implementing the BFS algorithm to find the path from goal to start
    for m=1:length(milestones)
        A = milestones{m,4};
        B = A(1:ceil(end/2));
        milestones{m,4} = B;
    end
    
    marked =[];
    for l = 1: length(milestones)
        marked(l) = 0;
    end
    

    flag2 = 0; 
    queue = [];
    queue(1) =length(milestones);
    start = length(milestones)-1;
    v = -1;
    pause(8);
    visited = [];
    while ( not(isempty(queue)) || ( v==start) )
        v = queue(1);
        visited = [visited;v];
        queue(1) = [];
        for j = milestones{v,4}
            if(marked(j)==0)
                  queue = [queue;j];
                  marked(j) = 1;
            end
        end
        if ( ( v==start ) )
            display('Start reached from Goal ');
            flag2 = 1;
        end
    end
    
    if(flag2==1)
        visited = fliplr(visited);
        %plot_path(milestones,visited);
    else
        display('Start Not Found from Goal.So No path');
    end
    
    
    figure(1);
    final_plot(milestones,visited,rejected);
    
    
    figure(2);
    draw_arena(milestones,visited,rejected);
    
    
end

  % Function to the configuration for given theta1,theta2
 function result = find_arm(x1,y1,theta1,theta2)
    
    % Adding the first point
    firstPoint = [x1;y1];
    result = firstPoint;
    
    % finds x2,y2 and adds to result
    nextPoint = firstPoint + [cosd(theta1); sind(theta1) ];
    result = [result, nextPoint];
    
    % finds x3,y3 and adds to result
    nextPoint = nextPoint + [cosd(theta1+theta2); sind(theta1+theta2)];
    result = [result, nextPoint];
 end

 
% Function to plot the arm
function [h] =  plot_arm(result,c)
    if(nargin==2)
        h = plot(result(1,1:2),result(2,1:2),c);
        h = plot(result(1,2:3),result(2,2:3),c);
    else
        c='b';
        h = plot(result(1,1:2),result(2,1:2),c);
        h = plot(result(1,2:3),result(2,2:3),c);
    end
end
        
    
function draw_arena(milestones,visited,rejected)
    xmin = -3;
    xmax = 3;
    ymin = -1;
    ymax = 3;
    start_theta1 = 45;
    start_theta2 = -45;
    goal_theta1 = 135;
    goal_theta2 = 45;
    line([xmin xmin xmax xmax xmin], [ymin ymax ymax ymin ymin]);
    arena_map{1} = [-3 0;
                    -3 -1;
                    3 -1;
                    3 0];
    arena_map{2} = [-1 1;
                    -1 3;
                    -3 3;
                    -3 1];
    arena_map{3} = [1 1;
                    3 1;
                    3 3;
                    1,3];
    for i = 1:length(arena_map);
        obstacle = arena_map{i};
        patch(obstacle(:,1), obstacle(:,2),'red');
    end
    
    hold on;
    
    n =1;
    for i= 1: length(milestones)
        config = milestones{i,1};
        h(n) = plot_arm(config,'k');
    end
    n=n+1;
    
    
    for i= 1: length(visited)
        y = visited(i);
        config = milestones{y,1};
        h(n) = plot_arm(config,'b');
    end
    n=n+1;
    
    
    [m,o] = size(rejected);
    for i= 1:m
        config = rejected{i,1};
        h(n) = plot_arm(config,'r');
    end
    n=n+1;
    
    % start position in one color
    result = find_arm(0,0,start_theta1,start_theta2);
    h(n) = plot_arm(result,'y');
    n=n+1;
    
    
    % end position in another_color
    result= find_arm(0,0,goal_theta1,goal_theta2);
    h(n) = plot_arm(result,'g');
    n=n+1;
    
    
    legend(h,'Milestones','Visited Arms','Rejected Arms','Start','Goal','Location','Best');
    
    
    hold off;
    axis tight;
    axis square;
    grid on;
end
    
    
% Checks if there is any collision with robot and obstacles 
% Takes robots vertices and returns 1 for collision or 0 for collision free
function isCollision = check_collision( tmp_robot )

        arena_map{1} = [-3 0;
                        -3 -1;
                        3 -1;
                        3 0];
        arena_map{2} = [-1 1;
                        -1 3;
                        -3 3;
                        -3 1];
        arena_map{3} = [1 1;
                        3 1;
                        3 3;
                        1,3];
                
        arena_limits = [-3 3 -1 3]; 
        isCollision = 0;

    %Checks if robots vertices are outside the boundry
    IN = inpolygon(tmp_robot(1,:),tmp_robot(2,:), [arena_limits(1:2), arena_limits(1,2), arena_limits(1,1)], ...
            [arena_limits(1,3), arena_limits(1,3), arena_limits(1,4), arena_limits(1,4)]);
    [r, c] = find(IN == 0);
    [r1,c1] = size(r);
    if(c1 ~= 0)
        isCollision = 1;
        return;
    end
    
    for i = 1 : length(arena_map)
        [a, b, ii] = polyxpoly(tmp_robot(1,:),tmp_robot(2,:),arena_map{i}(:,1),arena_map{i}(:,2));
        [s1,s2] = size(ii);

        if(s1 ~= 0)
            isCollision = 1;
            return;
        end   

        %Checks if robots vertices are inside any obstacle
        [IN ON] = inpolygon(tmp_robot(1,:),tmp_robot(2,:), arena_map{i}(:,1),arena_map{i}(:,2));
        [r, c] = find(IN == 1);
        [s,d] = find(ON==1);
        [r1,c1] = size(r);
        [s1,d1] = size(s);
        if(c1 ~= d1)
            isCollision = 1;
            return;
        end
    
    end

end



% takes milestones, index of the node and number of neighbors
% returns neighbors' index of the node
function IX = find_neighbors( milestones, j,neigh_radius )
    w = 0.09; % if it is taken 1, distance will be just euclidean distance
    k_neighbors = 2;
    Nq = [];
    k=1;
    for i = 1 : length(milestones)
        if(i == j) 
            Nq(i) = 100;
        else
          theta1_diff = milestones{j,2} - milestones{i,2};
          theta2_diff = milestones{j,3} - milestones{i,3};
          
          if( theta1_diff > 180) % robot can go opposite way so that take shorter way
              theta1_diff = theta1_diff - 2*180;
          end
          if( theta1_diff < -180) % robot can go opposite way so that take shorter way
              theta1_diff = 2*180 + theta1_diff;
          end
                   
          if( theta2_diff > 180) % robot can go opposite way so that take shorter way
              theta2_diff = theta2_diff - 2*180;
          end
          if( theta2_diff < -180) % robot can go opposite way so that take shorter way
              theta2_diff = 2*180 + theta2_diff;
          end
           d = w * norm(milestones{j,1} - milestones{i,1}) + (1 - w) * ...
               (abs( theta1_diff) + abs(theta2_diff )); % calculate distance
           if(d<=neigh_radius)
               Nq(k)= d;
               k=k+1;
           end
               
        end
    end
    [B, IX] = sort(Nq);
    IX = IX(1:length(Nq)); % sends indexes of the k closest neighbors
 end

%checks if two milestones are connected
%takes two points as input and 
%returns boolean
function flg = isConnected(p1, p2)

    arena_map{1} = [-3 0;
                        -3 -1;
                        3 -1;
                        3 0];
        arena_map{2} = [-1 1;
                        -1 3;
                        -3 3;
                        -3 1];
        arena_map{3} = [1 1;
                        3 1;
                        3 3;
                        1,3];
    eps  = 0.001;
    flg = 1;

    len = norm(p1-p2);
    samples  = fix(len/eps/10);

    x = linspace(p1(1,1), p2(1,1), samples);
    y = linspace(p1(2,1), p2(2,1), samples);

    x = x(2:end-1);
    y = y(2:end-1);

    for i = 1:length(x)
        for j = 1 : length(arena_map)
            if inpolygon(x(i), y(i), arena_map{j}(:,1), arena_map{j}(:,2)) % if they are not connected, it will be in polygon
                flg = 0; % so return false if not connected
                return;
            end

        end
    end
end


function plot_path(milestones,visited)
    figure(3);
    hold on;
    for i= 1: length(visited)
        y = visited(i);
        config = milestones{y,1};
        h = plot_arm(config);
    end
    hold off;
end


function final_plot(milestones,visited,rejected)
    start_theta1 = 45;
    start_theta2 = -45;
    goal_theta1 = 135;
    goal_theta2 = 45;
    
    temp = visited;
    visited=[];
    temp2 = temp(1);
    for j = 1:length(temp)
        visited(j) = temp(j);
        if(temp(j)==temp2-1)
            break;
        end
    end
    
    
    
    a = length(milestones);
    [b,c] = size(rejected);
    
    display(['Total No of Samples(including start,goal) : ',num2str(a+b)]);
    display(['Mile Stones : ',num2str(a)]);
    display(['Rejected Nodes : ',num2str(b)]);
    display(['Nodes in Path : ',num2str(length(visited))]);
    display(['Length of Shortest Path : ',num2str(length(visited))]);
    
    hold on;
    l = 1;
    h(l) = plot(start_theta1,start_theta2,'rs');
    l=l+1; 
    h(l) = plot(goal_theta1,goal_theta2,'gs');
    l=l+1;
    
    
    theta1s = milestones(1:length(milestones)-2,2);
    theta2s = milestones(1:length(milestones)-2,3);
    theta1s = cell2mat(theta1s);
    theta2s = cell2mat(theta2s);
    h(l) = scatter(theta1s,theta2s,'b');
    l = l+1;
    %hleg2 = legend('Milestones');
    
    theta1s = rejected(:,2);
    theta2s = rejected(:,3);
    theta1s = cell2mat(theta1s);
    theta2s = cell2mat(theta2s);
    h(l) = plot(theta1s,theta2s,'rx');
    l = l+1;
    %hleg3 = legend('Rejected Configuration');
    
    theta1=[];
    theta2=[];
    for i = 1: length(visited)
        y = visited(i);
        theta1 = [theta1;milestones{y,2}];
        theta2 = [theta2;milestones{y,3}];
    end
    h(l) = plot(theta1,theta2,'kx-');
    l = l+1;
    
    legend(h,'Start Configuration','Goal Configuration','Milestones','Rejected Configuration','Path followed','Location','SouthOutside');
    %hleg4 = legend('The path followed');
    
    xlabel('Theta1');
    ylabel('Theta2');
    
    hold off;
    
end