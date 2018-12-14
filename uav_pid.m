%%   UAV quadrotor state representation
% takes around 20 seconds %

clear; clc; close all;
syms x xdot y ydot z zdot roll roll_d pitch pitch_d yaw yaw_d %guideline

%% Choose controller
% 0: RotorS Lee Position Controller -- 1: Our PD Controller 
controller = 0; % 0 or 1

%% Initial conditions and physics values (in SI)

initial=[0;0;0;0;0;0;0;0;0;0;0;0];
state=zeros(12,1000); state(:,1)=initial;

m=1; g=9.81; l=0.21; Ixx=0.01; Iyy=Ixx; Izz=0.02;
desiredState=[1;0;0.4;0;1;0;0;0;0;0;0;0];
controls = zeros(4,1000);
%% Simulation based on ode45 and standalone.m

time=0:0.01:10;
t=0;
for i=1:1000
    if(controller==1)
        control = controlQuad(state(:,i),desiredState); % -- our PD controller
    else
        control = lee_position_controller(state(:,i),desiredState); % -- rotorS lee controller
    end
    [~,xnew] = ode45('quadrotor_EQ',[t t+0.01],state(:,i),[],Ixx,Iyy,Izz,m,g,l,control);
    szxnew=size(xnew); xnew=xnew(szxnew(1),:); xnew=xnew';
    state(:,i+1)=xnew;
    
    t=t+0.01;
end

save('uav_pid_results','state');
plot(time,state(1,:),'g'); hold on;
plot(time,state(3,:),'r'); hold on;
plot(time,state(5,:),'b');  hold on;
plot(time,desiredState(1)*ones(1,length(time)),'--g');
plot(time,desiredState(3)*ones(1,length(time)),'--r');
plot(time,desiredState(5)*ones(1,length(time)),'--b');
xlabel('Time (seconds)');  
ylabel('Position (meters)');
legend('X','Y','Z','X Desired','Y Desired','Z Desired');
