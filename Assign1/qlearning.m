function WindyGridWorldQLearning()

        %Initializing the Q matrix and the R matrix
        Q=zeros(30,30);
        R=zeros(30,30);
        
        %Intializing the previous Q to a large value, used to check for
        %convergence
        Q1=ones(size(R))*inf;
        count2=0;
        [r,~]=size(R);
        
        %Initializing the rewards
        R(1:r+1:end)=R(1:r+1:end)-1;
        R(29,30)=10;
        R(29,28)=10;
        R(29,23)=10;
        
        statemap=zeros(5,6);
        count=1;
        windystates = [27 21 15 9 10 11 16 17 22 23 28]
        
        %labelling each state
        for i=1:5
            for j=1:6
                statemap(i,j)=count;
                count=count+1;
            end
        end
        
        display(statemap);
        
        %the coordinates for each state
        index=zeros(30,2);
        count=1;
        for i=1:5
            for j=1:6
                index(count,:)=[i j];
                count=count+1;
            end
        end
        
        display(index);
        
        %initial values
        ep_count=50000;
        start=25;
        goal=29;
        
        %starting the episodes
        for episode=1:ep_count
            curpos = start;
            steps=0;
            Episode_reward =0 ;
            while(curpos~=goal && steps<=30)
                steps=steps+1;
                actions = findactionstates(curpos,index,windystates,statemap);
                
                %If random<epsilon, take random action from the possible
                %states else take the action with highest Q value
                if(rand<0.1)
                    idx=randi(length(actions));
                    randval = actions(idx);
                else
                    randval=findactionwithhighQ(index,curpos,Q,windystates,statemap);
                end
                
                %If the action is valid
                if(randval~=-1)
                    qmax = findQmax(index,randval,Q,windystates,statemap);
                    Episode_reward = Episode_reward + 0.1*( R(curpos,randval)+0.9*qmax-Q(curpos,randval));
                    Q(curpos,randval) = Q(curpos,randval)+ 0.1*( R(curpos,randval)+0.9*qmax-Q(curpos,randval));
                    curpos=randval;

                end

                %If the agent has reached the goal state, take one action
                %before the loop ends
                if(curpos==goal)
                    actions = findactionstates(curpos,index,windystates,statemap);
                    idx=randi(length(actions));
                    randval = actions(idx);
                    if(randval~=-1)
                        qmax = findQmax(index,randval,Q,windystates,statemap);
                        Q(curpos,randval) = Q(curpos,randval)+ 0.1*( R(curpos,randval)+0.9*qmax-Q(curpos,randval));
                    end
                end
             end % while loop ends 
             
             %check for convergence
             if(sum(sum(Q1-Q)))<0.0000001 && sum(sum(Q))>0
                if count2>1000
                    display('Q converged');
                    %the path followed by agent
                    agtpath = displaypath(start,goal,index,Q,windystates,statemap);
                    break
                else
                    count2=count2+1;
                end
            else
                Q1=Q;
                count2=0;
             end
             %storing the reward for this episode
             rewards(episode) = Episode_reward;

        end % ending the for loop
        
        %normalizing the Q table for better results
        g=max(max(Q));
        if(g>0)
            Q=100*Q/g;
        end
        
        display(['Total number of episodes to converge']);
        display(episode);
        %plot the path
        DrawGrid(5,6,index,agtpath);
        %plot episode vs rewardper episode plot
        rewardsplot(episode,rewards);



% Function to find all possible actions from the current position taking
% the sind into consideration
function[actions]=findactionstates(curpos,index,windystates,statemap)
    cd=index(curpos,:);
    x=cd(1);
    y=cd(2);
    
    %Down
    if(x+1<=5)
        pt = [x+1 y];
        [~,indx]=ismember(pt,index,'rows');
        %Move up if in windy state
        if( any(indx==windystates))
            pos = index(indx,:);
            x=pos(1);
            x=x-1;
            y=pos(2);
            indx=statemap(x,y);
        end
        actions(1)= indx;
    else
        % -1 if it goes out of the grid
        actions(1)=-1;
    end
    
    %Right
    if(y+1<=6)
        pt = [x y+1];
        [~,indx]=ismember(pt,index,'rows');
        %Move up if in windy state
        if( any(indx==windystates))
            pos = index(indx,:);
            x=pos(1);
            x=x-1;
            y=pos(2);
            indx=statemap(x,y);
        end
        actions(2)= indx;
    else
        actions(2)=-1;
    end
    
    %Up
    if(x-1>0)
        pt = [x-1 y];
        [~,indx]=ismember(pt,index,'rows');
        %Move up if in windy state
        if( any(indx==windystates))
            pos = index(indx,:);
            x=pos(1);
            x=x-1;
            y=pos(2);
            indx=statemap(x,y);
        end
        actions(3)= indx;
    else
        actions(3)=-1;
    end
    
    %Left
    if(y-1>0)
        pt = [x y-1];
        [~,indx]=ismember(pt,index,'rows');
        %Move up if in windy state
        if( any(indx==windystates))
            pos = index(indx,:);
            x=pos(1);
            x=x-1;
            y=pos(2);
            indx=statemap(x,y);
        end
        actions(4)= indx;
    else
        actions(4)=-1;
    end
    
    
    %Function to find the max Q value of the actions from the given state
    function[qmax] = findQmax(index,state,Q,windystates,statemap);
        actionset = findactionstates(state,index,windystates,statemap);
        j=1;
        for i=1:4
            % add only if its a valid movement
            if(actionset(i)~=-1)
                posactions(j)=actionset(i);
                j=j+1;
            end
        end
        
        for i=1:length(posactions)
            qvalues(i)=Q(state,posactions(i));
        end
        qmax = max( qvalues );
        
        
     
     %Function to find the action with Highest Q value from the current
     %state
     function[nstate] = findactionwithhighQ(index,state,Q,windystates,statemap)
            actionset = findactionstates(state,index,windystates,statemap);
            j=1;
            for i=1:4
                if(actionset(i)~=-1)
                    posactions(j)=actionset(i);
                    j=j+1;
                end
            end
            k=1;
            for i=1:length(posactions)
                qvalues(k)=Q(state,posactions(i));
                k=k+1;
            end
            qmax = max( qvalues );
            index=find(qvalues==qmax);
            if(length(index)>1)
                nstate=posactions(index(1));
            else
                nstate=posactions(index);
            end
            
 
%Function to track the path followed by the agent
function[agentpath] = displaypath(start,goal,index,Q,windystates,statemap)
    curstate=start;
    path=[start];
     l=2;
     while(curstate~=goal)
          nstate=findactionwithhighQ(index,curstate,Q,windystates,statemap);
          path(l)=nstate;
          l=l+1;
          curstate=nstate;
    end
    agentpath=path;
            

    %Fucntion to plot the path followed by the agetnt
function DrawGrid(gridrows, gridcols,index,path)
    x=[0];
    y=[0];
    %So that the label does not overlap
    dx = 0.2; dy = 0.2; 
    figure('name','Q-learning Path');
    hold on
    for i=1:length(path)
        cd=index(path(i),:);
        x(i)=cd(1);
        y(i)=cd(2);
        % y-1 and 5-x to transform from matrix dimensions to carteisan
        % dimensions
        plot(y-1,5-x,'d');
        label = cellstr( num2str(i));
        display(label);
        text(y(i)-1+dx,5-x(i)+dy,label);
        
    end
    
    display(x);
    display(y);
    hold off
    title('Path Taken by agent');
    xlabel('X-Axis');
    ylabel('Y-Axis');
    axis([0 7 0 5])
    
    
    
    %Function to plot the episodes vs rewardperepisode
    function rewardsplot(episodes_max,rewards)
        figure('name','Episode vs Rewards');
        hold on
        x=1:1:episodes_max-1;
        y=rewards(x);
        plot(x,y);
        hold off
        title('Episodes vs Rewards Per episode');
        xlabel('Episode');
        ylabel('Episode_Reward');
        

            
            
        





    