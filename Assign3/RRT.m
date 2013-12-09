function RRT

    N = 100;
    start_theta1 = 45;
    start_theta2 = -45;
    goal_theta1 = 135;
    goal_theta2 = 45;
    qstart = find_arm(0,0,start_theta1,start_theta2);
    qgoal = find_arm(0,0,goal_theta1,goal_theta2);
           
    Nodeslist(1,1) = {1};
    Nodeslist(1,2) = {qstart};
    Nodeslist(1,3) = {start_theta1};
    Nodeslist(1,4) = {start_theta2};
    
    Tree = {1,[1 0]};
    % Tree = [ Nodeindex nearestNode distancetonearestnode ]
    
    flag = 1;
    i=0;
    k=1;
    rejected = {[]};
    
    
    while(flag)
        i = i+1;
        x1_tmp = 0;
        y1_tmp = 0;
        theta1 = rand() *180;
        theta2 = rand() *360;
        q_rand = find_arm(x1_tmp, y1_tmp, theta1, theta2);
        
        isCollision = check_collision(q_rand); 
        
        if isCollision==0
                  
            if(rand<0.1)
                ix = find_closest(q_rand,theta1,theta2,Nodeslist);
                nearest_node = Nodeslist{ix,2};
                alpha1 = Nodeslist{ix,3};
                alpha2 = Nodeslist{ix,4};
                
                j = length(Tree(:,1));
                Tree(end+1,1) = {j+1};
                Tree(end,2) = {[j+1 ix ]};
                Nodeslist(end+1,1) = {length(Nodeslist) + 1};
                Nodeslist(end,2) = {q_rand};
                Nodeslist(end,3) = {theta1};
                Nodeslist(end,4) = {theta2};
                
                d_node_to_tree = calc_distance(q_rand,theta1,theta2,nearest_node,alpha1,alpha2);
                d_node_to_goal = calc_distance(q_rand,theta1,theta2,qgoal,goal_theta1,goal_theta2);
                
                if(d_node_to_goal < d_node_to_tree )
                    flag = 0;
                    display([num2str(i),' Tried to connect goal and RRT Tree - success']);
                    plot_path(Nodeslist,Tree,qstart,qgoal);
                else
                    display([num2str(i),' Tried to connect goal and RRT Tree - Failure']);
                end
            else
                ix = find_closest(q_rand,theta1,theta2,Nodeslist);
                display([num2str(i),' Added to the RRT Tree']);
                j = length(Tree(:,1));
                Tree(end+1,1) = {j+1};
                Tree(end,2) = {[j+1 ix ]};
                Nodeslist(end+1,1) = {length(Nodeslist) + 1};
                Nodeslist(end,2) = {q_rand};
                Nodeslist(end,3) = {theta1};
                Nodeslist(end,4) = {theta2};
            end
            
        else
            rejected(k,1) = {q_rand};
            rejected(k,2) = {theta1};
            rejected(k,3) = {theta2};
            rejected(k,4) = {[]};
            display([num2str(i),'- Config Rejected']);
            k = k+1;
            
        end
        
        
        
        if ( isSame(q_rand,qgoal) ||  i>N )
            if( isSame(q_rand,qgoal))
                plot_path(Nodeslist,Tree,qstart,qgoal);
            end
            flag = 0;
        end
        
        
    
        
        
    end    
    [a,d] = size(Nodeslist);
    [b,c] = size(rejected);
    display(['Total No of Samples : ',num2str(i-1)]);
    display(['Mile Stones (including start,goal) : ',num2str(a)]);
    display(['Rejected Nodes : ',num2str(b)]);
    
    if(i>N)
        display(['Could not connect to goal within the sample space']);
        visited = {[]};
    else
        y = Tree(:,2);
        y = cell2mat(y);
        ix = y(end,1);
        k=1;
        config = Nodeslist{ix,2};
        visited(k) = Nodeslist{ix,1};
        k=k+1;
        while(~isequal(config,qstart))
            ix = y(ix,2);
            config = Nodeslist{ix,2};
            visited(k) = Nodeslist{ix,1};
            k=k+1;
        end
        
    end
    
    figure(1);
    final_plot(Nodeslist,visited,rejected);
    
    draw_arena();
       


end
 
function [out] = isSame(q1,q2)
    if( isequal(q1,q2) )
        out = 1;
    else
        out = 0;
    end

end
 

function plot_arm(result,c)
    if(nargin==2)
        plot(result(1,:),result(2,:),c);
    else
        c='b';
        plot(result(1,:),result(2,:),c);
    end
end

 function [result] = find_arm(x1,y1,theta1,theta2)
    
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
 
  
 function [index] = find_closest(qrand,theta1,theta2,Nodeslist)
    infinity = 1e5;
    smallest_d = infinity;
    for i=1:(length(Nodeslist(:,1)))
        d = calc_distance(qrand,theta1,theta2,Nodeslist{i,2},Nodeslist{i,3},Nodeslist{i,4});
        if d<smallest_d
            smallest_d = d;
            index = i;
        end
        
    end
    
 end
 
 function [distance] = calc_distance(q1,theta1,theta2,q2,alpha1,alpha2)
       
    X1 = [q1(:,2),q2(:,2)];
    X2 = [q1(:,3),q2(:,3)];
    
    d1 = pdist(X1,'euclidean');
    d2 = pdist(X2,'euclidean');
    
    angle1_diff = theta1 - alpha1;
    angle2_diff = theta2 - alpha2;
    
    w = 0;
          
          if( angle1_diff > 180) % robot can go opposite way so that take shorter way
              angle1_diff = angle1_diff - 2*180;
          end
          if( angle1_diff < -180) % robot can go opposite way so that take shorter way
              angle1_diff = 2*180 + angle1_diff;
          end
                   
          if( angle2_diff > 180) % robot can go opposite way so that take shorter way
              angle2_diff = angle2_diff - 2*180;
          end
          if( angle2_diff < -180) % robot can go opposite way so that take shorter way
              angle2_diff = 2*180 + angle2_diff;
          end
          
    distance = w * sqrt( d1.^2 - d2.^2 ) + (1 - w) *(abs( angle1_diff) + abs(angle2_diff )); % calculate distance
 
 end
 
 function draw_arena()
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
      
    % start position in one color
    result = find_arm(0,0,start_theta1,start_theta2);
    plot_arm(result,'r');
    
    % end position in another_color
    result= find_arm(0,0,goal_theta1,goal_theta2);
    plot_arm(result,'g');
    
    hold off;
    axis tight;
    axis square;
    grid on;
 end

 function plot_path(Nodeslist,Tree,qstart,qgoal)
    y = Tree(:,2);
    y = cell2mat(y);
    ix = y(end,1);
    config = Nodeslist{ix,2};
    plot_arm(config);
    hold on;
    while(~isequal(config,qstart))
        ix = y(ix,2);
        config = Nodeslist{ix,2};
        plot_arm(config);
        mark = config(:,3);
        plot_arm(mark);
    end
    hold off;
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


function final_plot(Nodeslist,visited,rejected)
    start_theta1 = 45;
    start_theta2 = -45;
    goal_theta1 = 135;
    goal_theta2 = 45;
        hold on;
            l = 1;
            h(l) = plot(start_theta1,start_theta2,'rs');
            l=l+1; 
            h(l) = plot(goal_theta1,goal_theta2,'gs');
            l=l+1;


            theta1s = Nodeslist(1:end,3);
            theta2s = Nodeslist(1:end,4);
            theta1s = cell2mat(theta1s);
            theta2s = cell2mat(theta2s);
            h(l) = scatter(theta1s,theta2s,'b');
            l = l+1;
            %hleg2 = legend('Nodeslist');

            theta1s = rejected(:,2);
            theta2s = rejected(:,3);
            theta1s = cell2mat(theta1s);
            theta2s = cell2mat(theta2s);
            h(l) = plot(theta1s,theta2s,'rx');
            l = l+1;
            %hleg3 = legend('Rejected Configuration');
            
            
            goal_theta1 = 135;
            goal_theta2 = 45;
            qgoal = find_arm(0,0,goal_theta1,goal_theta2);
            ix = find_closest(qgoal,goal_theta1,goal_theta2,Nodeslist);
            theta1 = Nodeslist{ix,3};
            theta2 = Nodeslist{ix,4};
            h(l) = plot(theta1,theta2,'ko');
            l = l+1;
            
            if(length(visited)>=1)
                legend(h,'Start Configuration','Goal Configuration','Nodeslist','Rejected Configuration','Closest','Location','SouthOutside');
            else
                theta1=[];
                theta2=[];
                for i = 1: length(visited)
                    y = visited(i);
                    y=cell2mat(y);
                    theta1 = [theta1;Nodeslist{y,3}];
                    theta2 = [theta2;Nodeslist{y,4}];
                end
                theta1 = cell2mat(theta1);
                theta2 = cell2mat(theta2);
                 h(l) = plot(theta1,theta2,'kx-');
                l = l+1;
                legend(h,'Start Configuration','Goal Configuration','Nodeslist','Rejected Configuration','Path followed','Location','SouthOutside');
                %hleg4 = legend('The path followed');
            end
            xlabel('Theta1');
            ylabel('Theta2');

            hold off;
end

 
 
 
 
 
        
 
 
 
 
        
 
 
 
 
        
        
        
        
        
        
 