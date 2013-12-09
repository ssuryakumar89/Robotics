function finiteLQR()
    finiteLQRprob1();
    finiteLQRprob2();

function finiteLQRprob1()
    
    % Initializing the given values of Qf,R,T,dT,b
    Qf=eye(4,4)*100;
    R=eye(2,2)*1;
    Q=eye(4,4)*1;
    T=1000;
    dT=0.025;
    b = 1;
    
    % Declaring new variables for P and K
    P = zeros(4,4,T);
    K = zeros(2,4,T);
    
    % S denotes the position variable
    S = zeros(4,1,T);
    
    
    % After solving the given conditions and using laws of motion 
    A = [ 1 0 dT 0
        0 1 0 dT
        0 1*b 0 dT*b
        0 0 0 0];
    B = [0 0
        0 0
        1 0
        0 1];
    
    display('Problem 1 - Values of A and B');
    display(A);
    display(B);
    
    
    % The initial postion of the robot (1,0)
    S(:,:,1) = [1
                0
                0
                0];
    
    solveLQRprob1(P,Qf,Q,R,A,B,S,K,T);
    % The initial postion of the robot (-1,0)
    S(:,:,1) = [-1
                0
                0
                0];
    solveLQRprob1(P,Qf,Q,R,A,B,S,K,T);
    
    
function solveLQRprob1(P,Qf,Q,R,A,B,S,K,T)
    % Declaring the action varialbes
    u = zeros(2,1,T);
    
    % Solving the finite LQR
    P(:,:,T) = Qf;
    for t=T:-1:2
        P(:,:,t-1)= Q +  A.'*P(:,:,t)*A   - ( A.'*  P(:,:,t)*  B*  inv( R + B.'* P(:,:,t)*B) *   B.'   * P(:,:,t) * A );
    end
    for t =1:1:T-1
        K(:,:,t) = - (inv( R+ B.' * P(:,:,t+1) * B ) * B.' * P(:,:,t+1) * A) ;
    end
    for t = 1:1:T-1
        u(:,:,t) = K(:,:,t) * S(:,:,t);
        S(:,:,t+1)  = A * S(:,:,t) + B * u(:,:,t);
    end
    
    % Plotting the path followed by LQR
    x=zeros(1000);
    y=zeros(1000);
    figure('name','Path by LQR - Problem 1');
    hold on
    %axis([-4e-10 0 -4e-10 0]);
    text(S(1,:,1),S(2,:,1),'Starting position');
    for i=1:1000
        x(i)=S(1,:,i);
        y(i)=S(2,:,i);
    end
    plot(x,y);
    hold off
    title('Path followed by the LQR');
    xlabel('x coordinate');
    ylabel('y coordinate');
    
    
    
% Solving the finite LQR problem 2
    
function finiteLQRprob2()
    
    % Initializing the given values of Qf,R,T,dT,b
    Qf=eye(3,3)*100;
    R=eye(1,1)*1;
    Q=eye(3,3)*1;
    T=1000;
    dT=0.025;
    b = 1;
    
    
    
    % Declaring new variables for P and K
    P = zeros(3,3,T);
    K = zeros(1,3,T);
    
    % S denotes the position variable
    S = zeros(3,1,T);
    
    % After solving the given conditions and using laws of motion
    c1 = ((dT*dT)/2);
    c2 = (b*dT);
    A = [ 1 dT c1
          c2 1 0 
          b c2 b*c1 ];
    B = [0 
         dT
         1 ];
     
    display('Problem 2 - Values of A and B');
    display(A);
    display(B);
    
    % The initial postion of the robot (1,0)
    S(:,:,1) = [1
                0
                0];
    solveLQRprob2(P,Qf,Q,R,A,B,S,K,T);
    
    % The initial postion of the robot (-1,0)
    S(:,:,1) = [-1
                0
                0];
    solveLQRprob2(P,Qf,Q,R,A,B,S,K,T);
    
function solveLQRprob2(P,Qf,Q,R,A,B,S,K,T)
    % Declaring action variables
    u = zeros(1,1,T);
    
    % Solving the finite LQR
    P(:,:,T) = Qf;
    for t=T:-1:2
            P(:,:,t-1)= Q +  A.'*P(:,:,t)*A   - ( A.'*  P(:,:,t)*  B*  inv( R + B.'* P(:,:,t)*B) *   B.'   * P(:,:,t) * A );
    end
    for t =1:1:T-1
        K(:,:,t) = - (inv( R+ B.' * P(:,:,t+1) * B ) * B.' * P(:,:,t+1) * A) ;
    end

    for t = 1:1:T-1
        u(:,:,t) = K(:,:,t) * S(:,:,t);
        S(:,:,t+1)  = A * S(:,:,t) + B * u(:,:,t);
    end
    
    % Plotting the path followed by LQR
    x=zeros(1000);
    y=zeros(1000);
    figure('name','Path by LQR - Problem 2');
    hold on
    %axis([-4e-10 0 -4e-10 0]);
    text(S(1,:,1),1,'Starting Position');
    for i=1:1000
       x(i)=S(1,:,i);
       y(i)=i;
    end
    plot(x,y);
    hold off
    title('Position of the Robot at time T');
    xlabel('Position');
    ylabel('Time');
        
        









