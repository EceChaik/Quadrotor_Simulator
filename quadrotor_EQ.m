function [ state_dot ] = quadrotor_EQ( ~,state,~,Ixx,Iyy,Izz,m,g,l,control)
%   [state_dot] = standalone (time,state,[],Ixx,Iyy,Izz,m,g,l)
%   To be used by an ode solver. Models a standalone quadrotor
%   Inputs: 
%   -state = [x;x_d;y;y_d;z;z_d;roll;roll_d;pitch;pitch_d;yaw;yaw_d]
%   -variables in SI
%   Output:
%   -Derivative of state

%   Analyse desired state
%     xdes = desiredState(1); ydes = desiredState(3); 
%     xddes = desiredState(2); yddes = desiredState(4);
%     zdes = desiredState(5);     zddes = desiredState(6);
%     roll_des = desiredState(7); roll_ddes = desiredState(8);
%     pitch_des = desiredState(9); pitch_ddes = desiredState(10);
%     yaw_des = desiredState(11);  yaw_ddes = desiredState(12);
    
    
%   Specify output
    state_dot = zeros(12,1);

%   Speed of rotors and coeffs  useless here
%    b=1; d=1;                          % thrust and drug coefficients
%    n1 = m*g/4; n2=n1; n3=n1; n4=n2; % hover,zero yaw,zero pitch/roll

%   Make our lives easier
    roll = state(7); pitch = state(9); yaw = state(11);
    roll_d = state(8); pitch_d = state(10); yaw_d = state(12);
    x = state(1); y = state(3); 
    z = state(5);
    xdot = state(2);    ydot = state(4);    
    zdot = state(6);
    U1 = control(1);    U2 = control(2);
    U3 = control(3);    U4 = control(4);
    
%   The easy dots
    state_dot(1) = state(2);
    state_dot(3) = state(4);
    state_dot(5) = state(6);
    state_dot(7) = state(8);
    state_dot(9) = state(10);
    state_dot(11) = state(12);
    
%   Useful variables
    ux = cos(roll)*sin(pitch)*cos(yaw) + sin(roll)*sin(yaw);
    uy = cos(roll)*sin(pitch)*sin(yaw) - sin(roll)*cos(yaw);
    a1 = (Iyy-Izz)/Ixx;     a2 = (Izz-Ixx)/Iyy;
    a3 = (Ixx - Iyy)/Izz;
    b1 = l/Ixx;  b2 = l/Iyy;  b3 = 1/Izz;
    
%   The inputs
%    U1 = b*( n1 + n2 + n3 + n4);   % Thrust
%    U2 = b*( -n2 + n4);            % Roll
%    U3 = b*( n1 - n3);             % Pitch
%    U4 = d*( -n1 + n2 - n3 + n4);  % Yaw




%   The hard dots
    
    state_dot(2) = ux*U1/m;
    state_dot(4) = uy*U1/m;
    state_dot(6) = -g + (cos(roll)*cos(pitch)*U1/m);
    
    state_dot(8) = pitch_d*yaw_d*a1 + b1*U2;
    state_dot(10) = roll_d*yaw_d*a2 + b2*U3;
    state_dot(12) = roll_d*pitch_d*a3 + b3*U4;

end