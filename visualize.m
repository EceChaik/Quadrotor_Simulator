%% Load the results and plot them
clear; clc; close all;
load('uav_pid_results.mat'); % in variable named state 12x1001
time = 0 : 0.01 : 10;
l=0.21;
x = state(1,:); y = state(3,:); z = state(5,:);
phi = state(7,:); theta = state(9,:); psi = state(11,:);

% Positions of rotors and centerOfMass
CoM = [ x ; y ; z ] ;

% Plotting
C = [ 0 0 0 ; 1 0 0 ; 0 1 0 ; 0 0 1 ; 1 0 1 ] ;
for i=1:1000
    R=[cos(theta(i))*cos(psi(i)), ...
           sin(phi(i))*sin(theta(i))*cos(psi(i)) - cos(phi(i))*sin(psi(i)), ...
               cos(phi(i))*sin(theta(i))*cos(psi(i)) + sin(phi(i))*sin(psi(i)); ...
       cos(theta(i))*sin(psi(i)), ...
           sin(phi(i))*sin(theta(i))*sin(psi(i)) + cos(psi(i))*cos(phi(i)), ...
               cos(phi(i))*sin(theta(i))*sin(psi(i)) - sin(phi(i))*cos(psi(i)); ...
       -sin(theta(i)), ...
           sin(phi(i))*cos(theta(i)), ...
               cos(phi(i))*cos(theta(i))];
           
    rotor1 = CoM(:,i) + R*[l;0;0];
    rotor2 = CoM(:,i) + R*[0;-l;0];
    rotor3 = CoM(:,i) + R*[-l;0;0];
    rotor4 = CoM(:,i) + R*[0;l;0];
    
    Xpositions = [ CoM(1,i) rotor1(1) rotor2(1) rotor3(1) rotor4(1) ];
    Ypositions = [ CoM(2,i) rotor1(2) rotor2(2) rotor3(2) rotor4(2) ];
    Zpositions = [ CoM(3,i) rotor1(3) rotor2(3) rotor3(3) rotor4(3) ];
    scatter3(Xpositions,Ypositions,Zpositions,100,C,'filled'); 
    hold on;
    plot3([CoM(1,i) rotor1(1)],[CoM(2,i) rotor1(2)],[CoM(3,i) rotor1(3)],'k');
    plot3([CoM(1,i) rotor2(1)],[CoM(2,i) rotor2(2)],[CoM(3,i) rotor2(3)],'k');
    plot3([CoM(1,i) rotor3(1)],[CoM(2,i) rotor3(2)],[CoM(3,i) rotor3(3)],'k');
    plot3([CoM(1,i) rotor4(1)],[CoM(2,i) rotor4(2)],[CoM(3,i) rotor4(3)],'k');
    axis([-1.5 1.5 -1.5 1.5 -0.5 1.5]); % correct depending on the movement
    %view(0,90); 
    hold off;
    if(i==1)
        pause(1);
    end
    pause(0.01);
end

