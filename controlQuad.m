function [ control ] = controlQuad( state,desiredState )

    %state = [
    %x;xdot;y;ydot;z;zdot;phi;phi_dot;theta;theta_dot;psi;psi_dot]
    xdes = desiredState(1); ydes = desiredState(3); 
    xddes = desiredState(2); yddes = desiredState(4);
    zdes = desiredState(5);     zddes = desiredState(6);
    roll_des = desiredState(7); roll_ddes = desiredState(8);
    pitch_des = desiredState(9); pitch_ddes = desiredState(10);
    yaw_des = desiredState(11);  yaw_ddes = desiredState(12);
    
    roll = state(7); pitch = state(9); yaw = state(11);
    roll_d = state(8); pitch_d = state(10); yaw_d = state(12);
    x = state(1); y = state(3); 
    z = state(5);
    xdot = state(2);    ydot = state(4);    
    zdot = state(6);


    m=1; g=9.81; l=0.21; Ixx=0.01; Iyy=Ixx; Izz=0.02;
    Kd = 20;   Kp = 80;
    KdX = 1.7;  KpX = 2; % 9 , 5
    KdY = 1.7;  KpY = 2;
    xpd = KdX*(xdot - xddes) + KpX*(x - xdes) ;
    ypd = KdY*(ydot - yddes) + KpY*(y - ydes) ;
    
    U1 = m*(g - Kd*(zdot - zddes) - Kp*(z - zdes))/(cos(roll)*cos(pitch));
    U2 = (Ixx/l)*(-Kd*(roll_d - roll_ddes) - Kp*(roll - roll_des)) + ypd*cos(yaw) - xpd*sin(yaw);
    U3 = (Iyy/l)*(-Kd*(pitch_d - pitch_ddes) - Kp*(pitch - pitch_des)) - xpd*cos(yaw) - ypd*sin(yaw);
    U4 = (Izz)*(-Kd*(yaw_d - yaw_ddes) - Kp*(yaw - yaw_des));
    
     U1=max(min(U1,80),0);           
     U2=max(min(U2,20),-20);         
     U3=max(min(U3,20),-20);        
     U4=max(min(U4,20),-20);
    
    control = [U1, U2, U3, U4];

end

