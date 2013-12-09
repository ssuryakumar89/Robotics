function histogram_filter()

arena = zeros(7,6);
arena(2,:) = 1;
arena(4,:) = 1;
arena(6,:) = 1;
arena(:,6) = 1;
no_of_states = sum(sum(arena==1));
prob_distribution = arena/no_of_states;

% Assuming the agent arrives to state 1 by moving to right by one step

path = [[0 1];[0 1];[0 1];[0 1];[0 1];[1 0];[1 0];[1 0]];

k=1;
temp = arena;
for i=1:7
    for j=1:6
        if(arena(i,j)==1)
            temp(i,j) = k;
            k = k+1;
        end
    end
end

display(temp);

draw_hist(prob_distribution,arena,0);



for l=1:8
    display(['timestamp ' int2str(l)]);
    temp = move(path,prob_distribution,l,arena);
    temp = sense(arena,temp,l);
    prob_distribution = temp;
%     string = ['Prob after timestep ' int2str(l)];
%     display(string);
%     display(prob_distribution);
    draw_hist(prob_distribution,arena,l);
    
end




    

end


function temp = move(path,prob_distribution,step_no,arena)
    p_stay = 0.1;
    p_move = 0.9;
    temp = zeros(7,6);
    
    for i=1:7
        for j=1:6
            % Calculating the previous state according to the path specified
            prev_row = i - path(step_no,1);
            prev_col = j - path(step_no,2);

            % If the previous state is out of bounds, it means the agent arrived to the 
            % present state only by failing to take a move.In that case
            % previous_state_probability is 0. Else it is the probability of
            % the previous state
            if(prev_row==0 || prev_col==0)  
                previous_state_prob = 0;
            else
                previous_state_prob = prob_distribution(prev_row,prev_col);
            end

            present_state_prob = prob_distribution(i,j);

            % The robot could have arrived at this state by either taking one
            % step from the previous state ( represented by p_move *
            % previous_state_prob) or failing to take one step from the current
            % postion ( represented by p_stay * present_state_prob )

            % After step_no 5 we concentrate only on the 6th column(done by step_no>5)
            % where there are two ways of coming to a state.If it is not the 6th column,
            % ( done by else part of if j==6 ) the only chance of coming to a state is by 
            % failing to take a step from the state becuase the movement is only vertical now.
            if(step_no>5)
                if(j==6)
                    temp(i,j) = p_move * previous_state_prob + p_stay * present_state_prob;
                else
                    temp(i,j) = p_stay * present_state_prob;
                end
            else
                temp(i,j) = p_move * previous_state_prob + p_stay * present_state_prob;
            end

        end
        
    end
    write_output('Posterior Probablity after process update for state ',step_no,temp,arena);

    total = sum(sum(temp));
    for i = 1:7
            for j=1:6
                temp(i,j) = temp(i,j) / total;
            end
    end
    write_output('Normalized Posterior Probablity after process update for state ',step_no,temp,arena);



end

function prob_distribution = sense(arena,prob_distribution,step_no)
    Prob_error = 0.2;
    Prob_noerror = 1 - Prob_error;
    total = 0;
    
    for i=1:7
        for j=1:6
            if(arena(i,j)==1)
                prob_distribution(i,j) = prob_distribution(i,j) * Prob_noerror;
            else
                prob_distribution(i,j) = prob_distribution(i,j) * Prob_error;
            end
            total = total+prob_distribution(i,j);
            
        end
    end
    
    write_output('Posterior Probablity after measurement update for state ',step_no,prob_distribution,arena);
    
    
    for i = 1:7
        for j=1:6
            prob_distribution(i,j) = prob_distribution(i,j) / total;
        end
    end
    
    write_output('Normalized Probablity after measurement update for state ',step_no,prob_distribution,arena);
end


function draw_hist(prob_distr,arena,timestamp)
    
    temp = [];
    for i=1:7
        for j=1:6
            if(arena(i,j) == 1 )
                temp = [temp,prob_distr(i,j)];
            end
        end
    end
    
    if(timestamp==0)
        figure('name','Prior');
        bar(temp);
        xlabel('State');
        ylabel('Probability');
        title('Prior Probabilities');
    else
        
        figure('name',['histogram ' int2str(timestamp)]);
        hold on;
        bar(temp);
        xlabel('State');
        ylabel('Probability');
        title(['Probability distribution after time stamp ' int2str(timestamp)]);
        hold off;
    end
end

function write_output(string,time_stamp,prob_distribution,arena)

    
    k=1;
    for i=1:7
        for j=1:6
            if(arena(i,j)==1)
                x = prob_distribution(i,j);
                y = num2str(x);
                display([string int2str(k) '=' y]);
                k=k+1;
            end
        end
    end





end