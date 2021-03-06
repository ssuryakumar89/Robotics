function temp
    close all;
    clear all;
    clc;
    diary(fullfile('G:', 'myDiary.txt'));
    forward_noise = 0.25;
    sense_noise = 0.2;
    myrobot = robot();
    myrobot = myrobot.set_values(1,1,0);
    myrobot = myrobot.set_noise(forward_noise,sense_noise);
    
    figure(1);
    title('Intial Plot of the particles and robot');
    hold on;
    line([0 0 7 7 0], [0 3 3 0 0]);
    N = 500;
    T = 15;
    plot(myrobot.x,myrobot.y,'bx');
    p(N) = robot;
    xvals = [];
    yvals = [];
    for i= 1:N
        tt = robot();
        y = tt.set_noise(forward_noise,sense_noise);
        p(i)= y;
        xvals = [xvals,tt.x];
        yvals = [yvals,tt.y];
    end
    plot(xvals,yvals,'r*');
    hold off;
    
    
    
    
    for m = 1:T
        myrobot = myrobot.move(0, sense_noise);
        Z = myrobot.sense();
        
        figure(m+1);
        title(['Plot after time step' int2str(m)]);
        hold on;
        line([0 0 7 7 0], [0 3 3 0 0]);
        x_temp=myrobot.x;
        y_temp=myrobot.y;
        plot(x_temp,y_temp,'bo');

        % Applying move to all the particles created 
        p2(N) = robot;
        for i=1:N
            p2(i) = p(i).move(0, sense_noise);
        end
        p = p2;
        
        % Applying move to all the particles created - removing the
        % negative points ? - comment out this 
        
%         p6 = p2;
%         l = 1;
%         for i=1:length(p2)
%             if ( p2(i).x>=0.0 && p2(i).y>=0.0 )
%                 p6(l) = p2(i);
%                 l = l+1;
%             end
%         end
%         p = p6;
%         N = length(p6);
        
        

        % Calculating the importance weights of each particle
        % measurement_prob is the function which gives similarity between
        % the distance measured by the robot and the distance measured by
        % the particle
        w = [];
        for i=1:N
            w(i)=p(i).measurement_prob(Z);
        end
        total = sum(w);
        for i=1:N
            w(i) = w(i)/total;
        end
       
        
        % Resampling. The resampling technique used is stratified
        % resampling
        Q = cumsum(w);
        indx=[];
        for i=1:N,
            T(i) = rand(1,1)/N + (i-1)/N;
        end
        T(N+1) = 1;
        i=1;
        j=1;
        while (i<=N),
            if (T(i)<Q(j)),
                indx(i)=j;
                i=i+1;
            else
                 j=j+1;        
            end
        end
        p3(N) = robot;
        for i=1:N
            p3(i) = p(indx(i));
        end
        p=p3;
        
        
        
        %Finding the distinct no of particles
        c=[];
        for i=1:N
            c(i,:) = [p(i).x p(i).y p(i).orient];
        end
        d = unique(c,'rows');
        no = length(d);
        
        % Displaying all the particles chosen after resampling 
        % and their respective wnormalized weights
        display(['After step number ' int2str(m) ' Distinct Number of Particles : ' int2str(no)]);
        show(p,N,w);
        
        % Plotting all the particles
        xvals=[];
        yvals=[];
        for k=1:N
            xvals = [xvals p(k).x];
            yvals = [yvals p(k).y];
        end
        plot(xvals,yvals,'r*');
        hold off;        
    end
    diary off;
   
    
    
end


function show(p,N,w)
    for i=1:N
            display(['Particle ' int2str(i) ' : x = ' num2str(p(i).x) ' , y = ' num2str(p(i).y) ' , weight = ' num2str(w(i))]);
    end
end