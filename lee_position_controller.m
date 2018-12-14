function [ control_signals ] = lee_position_controller(state,desiredState)

    % assumed state =
    % [x;xdot;y;ydot;z;zdot;roll;roll_dot;pitch;pitch_dot;yaw;yaw_dot]
    % assumed desired accelerations = 
    % [xddot;yddot;zddot]   [roll_ddot;pitch_ddot;yaw_ddot]
    
    m = 1; la=0.21; g = 9.81; Kp=4.5; Kd=3.5; gvec=[0;0;g];
    
    position = zeros(3,1);  desPosition = zeros(3,1);
    position(:) = state(1:2:5);  desPosition(:) = desiredState(1:2:5);
    position_error = position - desPosition ;
    
    velocity = zeros(3,1); desVelocity = zeros(3,1);
    velocity(:) = state(2:2:6);  desVelocity(:) = desiredState(2:2:6);
    velocity_error = velocity - desVelocity(:) ;
    
    acceleration = (position_error.*Kp + velocity_error.*Kd)/m - gvec;% - desiredAcc;
    %if desiredAcc = true -> add input argument as in description
                                
    
    yaw = desiredState(11);
    b1_des = [ cos(yaw) ; sin(yaw) ; 0 ];
    b3_des = -acceleration./norm(acceleration);
    b2_des = cross(b3_des,b1_des); 
    b2_des = b2_des./norm(b2_des) ;
    R_des = zeros(3,3);
    R_des(:,1) = cross(b2_des,b3_des);
    R_des(:,2) = b2_des;
    R_des(:,3) = b3_des;
%---------------------------------------------------------------------------------------------------------------------------------------%
    phi = state(7); theta = state(9); psi = state(11);
    R = [ cos(theta)*cos(psi), sin(phi)*sin(theta)*cos(psi) - cos(phi)*sin(psi), cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi);
            cos(theta)*sin(psi), sin(phi)*sin(theta)*sin(psi) + cos(phi)*cos(psi), cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi);
           -sin(theta),          sin(phi)*cos(theta),                              cos(phi)*cos(theta)                               ];
%---------------------------------------------------------------------------------------------------------------------------------------%

    angle_error_mat = 0.5*( R_des'*R - R'*R_des );
    angle_error = zeros(3,1);
    angle_error(1) = angle_error_mat(3,2);
    angle_error(2) = angle_error_mat(1,3);
    angle_error(3) = angle_error_mat(2,1);
    
    angular_rate = state(8:2:12);
    angular_rate_des = desiredState(8:2:12);
    angular_rate_error = angular_rate - R_des'*R*angular_rate_des;
    
    norm_att_gain = 1;  norm_ang_rate_gain = 0.22;
    angular_acceleration = -angle_error.*norm_att_gain - angular_rate_error.*norm_ang_rate_gain;
    
    thrust = -m*dot(acceleration,R(:,3));
    angular_acc_thrust = [angular_acceleration ; thrust];
    control_signals = zeros(4,1);
    control_signals(1) = angular_acc_thrust(4);
    control_signals(2) = angular_acc_thrust(1)/la;
    control_signals(3) = angular_acc_thrust(2)/la;
    control_signals(4) = angular_acc_thrust(3);
       
end