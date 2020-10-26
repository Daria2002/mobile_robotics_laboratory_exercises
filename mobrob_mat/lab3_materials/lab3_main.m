%% setup the robot simulator
rosshutdown
clear all; close all; clc;

rosinit
sim = ExampleHelperRobotSimulator('simpleMap');
controlRate = robotics.Rate(10);
dt = controlRate.DesiredPeriod;
% Originalna putanja
%setRobotPose(sim, [0.75 0.5 pi/2]);

% Nova 1
setRobotPose(sim, [1, 11, -pi/2]);

% Nova 2
%setRobotPose(sim, [12, 11, -pi/2]);

sim.LaserSensor.NumReadings = 8;
sim.LaserSensor.SensorNoise = 0.025;
orientationNoise = 0.01;
translationNoise = 0.05;
updateCounter = 1;
enableROSInterface(sim, true);

% load map obtained in lab2, it should be robotics.BinaryOccupancyGrid object called 'map' 
% load(...)

%% set-up the path following control algorith
% Originalni put
% path = [2 3; 3.25 6.25; 2 11; 6 7; 11 11; 8 6; 10 5; 7 3; 11 1.5];

% Nova 1
path = [2 10; 9 11; 7 2];

% Nova 3
%path = [12 9; 12 8; 12 7; 11 7; 9.5 7;  8 7; 6 8.5; 4 6; 2 4; 1 2];

plot(path(:,1), path(:,2),'k--d');
controller = robotics.PurePursuit('Waypoints', path);
controller.DesiredLinearVelocity = 0.2;
%controller.MaxAngularVelocity = 0.3;
goalRadius = 0.1;
robotCurrentLocation = path(1,:);
robotGoal = path(end,:);
distanceToGoal = norm(robotCurrentLocation - robotGoal);

PoseHistory = [];
OdomHistory = [];
CorrectionHistory = [];
PdetHistory = [];
PtraceHistory = [];

% initial values of odometry and EKF output
robotOdom = sim.Robot.Pose;
correctedPose = sim.Robot.Pose;
% measured translational and rotational robot velocity
v_measured = 0;
w_measured = 0;


% initialize figure for localization
figureHandle = figure('Name', 'EKF');
axesHandle = axes('Parent', figureHandle);
title(axesHandle, 'EKF: Update 0');
xlim(sim.Map.GridSize / sim.Map.Resolution);

% set initial value of matrix P
% P = ...

%% start the robot control loop

% Define Robot odometry publisher
o_pub = rospublisher('/odom_noise', 'nav_msgs/Odometry');
o_msg = rosmessage(o_pub);

x_k_subscriber = rossubscriber('/x_hat', 'geometry_msgs/Pose');

while (distanceToGoal > goalRadius)
     
    % update robot pose and odometry data
    robotPose = sim.Robot.Pose;   
    PoseHistory = [PoseHistory; robotPose];
    
    robotOdom = robotOdom + dt * [v_measured * cos(robotOdom(3)), ...
        v_measured * sin(robotOdom(3)), w_measured];
    OdomHistory = [OdomHistory; robotOdom];
    
    % Publish noised message
    o_msg.Pose.Pose.Position.X = robotOdom(1);
    o_msg.Pose.Pose.Position.Y = robotOdom(2);
    o_msg.Pose.Pose.Position.Z = robotOdom(3);
    o_msg.Twist.Twist.Linear.X = v_measured;
    o_msg.Twist.Twist.Angular.Z = w_measured;
    send(o_pub, o_msg);
    
    % get sensor readings
    SensorReadings = sim.LaserSensor.getReading(robotPose);
    SensorRe25adings(isnan(SensorReadings)) = sim.LaserSensor.MaxRange; 
    AngleSweep = sim.LaserSensor.AngleSweep;  
    
    %x_k = receive(x_k_subscriber, 10);
    x_k = x_k_subscriber.LatestMessage;
    
    % perform EKF update after a short waiting period
    if updateCounter > 5
        
        try
            correctedPose = [x_k.Position.X, x_k.Position.Y, x_k.Position.Z];
        catch
            disp("No message recieved")
            % Did not recieve message from subscriber... 
            % Pass
        end
    else
        correctedPose=robotOdom;
    end    
    CorrectionHistory = [CorrectionHistory; correctedPose];

    % plot result
    if ~mod(updateCounter, 20)
        figure(2), plot(PoseHistory(:,1), PoseHistory(:,2), 'b', 'LineWidth', 4);
        hold on;
        figure(2), plot(OdomHistory(:,1), OdomHistory(:,2), 'r', 'LineWidth', 2); 
        figure(2), plot(CorrectionHistory(:,1), CorrectionHistory(:,2), 'g', 'LineWidth', 2); 
        figure(2), title(axesHandle, ['EKF: Update ' num2str(updateCounter)]);
        figure(2), legend('True pose', 'Odometry', 'EKF')
        figure(2), xlim([0, sim.Map.GridSize(1)/sim.Map.Resolution]);
        figure(2), ylim([0, sim.Map.GridSize(2)/sim.Map.Resolution]);
    end
    
    % for lab3 use ground truth pose for driving the robot
    [v, w] = controller(robotPose);
    drive(sim, v , w);
    % simulated noisy robot velocity measurements
    v_measured = v + (abs(v) > 0) * normrnd(0, translationNoise);
    w_measured = w + (abs(w) > 0) * normrnd(0, orientationNoise);
    % clear true v and w since they are not available to the EKF
    clear v w;
    
    % update the counter and distance to goal.
    updateCounter = updateCounter + 1;
    distanceToGoal = norm(robotPose(1:2) - robotGoal);
    
    % wait for control rate to ensure 10 Hz rate.
    waitfor(controlRate);
end
%% pretvaranje occGrid.mat u occGrid.csv
file = load('checkpoints2.mat')
csvwrite('checkpoints2.csv', file.checkpoints2)
%%
file = importdata('occGrid.mat');
for i=1:260
    for j=1:270
        if file(i,j) <= 0.5
            file(i, j)=0;
        else
            file(i, j)=1;
        end
    end
end
cellsize = 20;
map = robotics.OccupancyGrid(13.5,13,cellsize);
map.FreeThreshold = 0.4;
map.OccupiedThreshold = 0.6;
figureHandle = figure('Name', 'Map');
axesHandle = axes('Parent', figureHandle);
mapHandle = show(map, 'Parent', axesHandle);
mapHandle.CData = file;

%% PLOTANJE
% initialize figure for localization
% Pokrenuti prvu celiju !
load('maloBolje.mat')

figureHandle = figure('Name', 'EKF');
axesHandle = axes('Parent', figureHandle);
title(axesHandle, 'EKF: Update 0');
xlim(sim.Map.GridSize / sim.Map.Resolution);
figure(2), plot(PoseHistory(:,1), PoseHistory(:,2), 'b', 'LineWidth', 4);
hold on;
figure(2), plot(OdomHistory(:,1), OdomHistory(:,2), 'r', 'LineWidth', 2); 
figure(2), plot(CorrectionHistory(:,1), CorrectionHistory(:,2), 'g', 'LineWidth', 2); 
figure(2), title(axesHandle, ['EKF: Update ' num2str(updateCounter)]);
figure(2), legend('True pose', 'Odometry', 'EKF')
figure(2), xlim([0, sim.Map.GridSize(1)/sim.Map.Resolution]);
figure(2), ylim([0, sim.Map.GridSize(2)/sim.Map.Resolution]);