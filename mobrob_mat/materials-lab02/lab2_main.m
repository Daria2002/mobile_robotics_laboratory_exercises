%% setup the robot simulator
rosshutdown
clear all; close all; clc;

rosinit
sim = ExampleHelperRobotSimulator_lamor('simpleMap');
controlRate = robotics.Rate(10);
setRobotPose(sim,[0.75 0.5 0]);
sim.LaserSensor.NumReadings = 8;
sim.LaserSensor.SensorNoise=0.025;
updateCounter = 1;  
enableROSInterface(sim, true);

%% set sensor, map parameters and the path following control algorithm
% sonar model and and map parameters
rov = 1; 
th3db = 0.5; 
pE = 0.4; 
pO = 0.6;
cellsize = 20;
deltark = 0.05; 
map = robotics.OccupancyGrid(13.5,13,cellsize);
map.FreeThreshold = pE;
map.OccupiedThreshold = pO;

% plot the robot's path and configure the path following controller
path = [1.5 0.5;1 0.5; 1 2; 1 3; 1 4; 1 5; ...
    1 5.5; 1.5 5.5; 2 5.5; 2.25 5.5; ...
    2.25 5.25; 2.25 5.5; 2.25 6; 2.25 6.5; 2.25 7; 2 7; 1.75 7; 1.5 7; 1 7;...
    1 8; 1 9; 1 10; 1 11; 1 12; ...
    2 12; 3 12; 4 12; 5 12; 6 12; 6 11.5; 3.5 11.5;3.5 11;6 11; 6 10.5; 6 10.35;...
    5.5 10.35; 5.25 10.35; 5 10.35; 4.75 10.35; 4.5 10.35; 4.15 10.35;...
    4.15 10; 4.15 9.75; 4.15 9.5; 4.15 9; 4.5 9; 5 9; ...
    6.5 9; 7 9; 7.45 9; 7.45 9.5; 7.45 10;...
    7.45 11; 7.45 11.5; 7.45 12; 8 12; 8.5 12; ...
    11.5 12; 12 12; 12.3 12; 12.3 11.5; 12.3 11; ...
    12.3 8;11.5 8; 12.3 7.5; 12.3 7; 12 7; 11.5 7;...
    10.5 7; 10 7; 9.75 7; 9.75 6.5; 9.75 6; ...
    9.75 5.5; 10 5.5; 10.5 5.5;...
    11.5 5.5; 12 5.5; 12.3 5.5; 12.3 5; 12.3 4.5;...
    12.3 3.85; 12 3.85; 11.5 3.85;...
    10.5 3.85; 10 3.85; 9.75 3.85; 9.75 3.5; 9.75 3;...
    9.75 2.6; 10 2.6; 10.5 2.6;...
    11.5 2.6; 12 2.6; 12.3 2.6; 12.3 2.5; 12.3 2;...
    10 2; 10 1.5;12.3 1.5; 12.3 1; 12 1; 11.5 1;...
    7 1; 6.5 1; 6 1; 6 1.5; 6 2;...
    6 4.5; 6 5; 6 5.5; 6.25 5.5; 6.5 5.5;...
    7 5.5; 7.25 5.5; 7.25 6; 7.25 6.5; 7.25 7; 7 7; 6.5 7;...
    5 7; 4.5 7; 3.3 7; 4.4 6.75; 4.4 6.25; 4.4 6;... 
    3.9 6;3.9 5;4.4 5;4.4 4;3.9 4;4.4 3; 4.4 2.5; 4.4 2.35; 4.25 2.35; 4 2.35;...
    3.5 2.35; 3 2.35; 3 2; 3 1.75; 3 0.5;...
    2 0.5; 2 1; 2 2; 2 3; 2 4; 3 3; 3 4; 3 5; 3 6; 3 7;...
    3 8; 2 8; 2 9; 2 10; 2 11; 3 11; 3 10; 3 9; 3 8;...
    4 8; 5 8; 6 8; 7 8; 8 8; 9 8; 9 9; 8 9; 8 9.5;9 9.5; 9 10; 8 10;...
    8 10.5;9 10.5;8 11; 9 11; 10 11; 12 11;...
    12 10.5; 10 10.5;10 9; 11.5 9.5; 11 8; 10.5 8.5; 10 8; 9 8; 8.5 8;...
    7.5 7; 8.5 7;8.8 7.3; 9.3 5.6;8.5 6;7.7 6.3; 8.5 5;...
    8 5; 7 5; 7 3; 7 2; 7.5 2; 7.5 3;...
    7.5 4;8 4; 8 2; 8 3; 8 4;9 2; 9 3; 9 4; 9 5; 10 5; 12 5;12 4.5; 9.75 4.5];

plot(path(:,1),path(:,2),'k--d');
controller = robotics.PurePursuit('Waypoints',path);
controller.DesiredLinearVelocity = 0.05;
controller.MaxAngularVelocity = 0.375;
controller.LookaheadDistance = 0.1;
goalRadius = 0.1;
robotCurrentLocation = path(1,:);
robotGoal = path(end,:);
distanceToGoal = norm(robotCurrentLocation - robotGoal);

% initialize figure for map building
figureHandle = figure('Name', 'Map');
axesHandle = axes('Parent', figureHandle);
mapHandle = show(map, 'Parent', axesHandle);
title(axesHandle, 'OccupancyGrid: Update 0');

map_subscriber = rossubscriber('/occupancy_grid', 'std_msgs/Float32MultiArray');
%% start the robot control loop
% robot control loop
while( distanceToGoal > goalRadius )
    
    % read robot pose and sonar data    
    RobotPose = sim.Robot.Pose;    
    SonarReadings = sim.LaserSensor.getReading(RobotPose);
    SonarReadings(isnan(SonarReadings)) = sim.LaserSensor.MaxRange;     
    AngleSweep = sim.LaserSensor.AngleSweep;  
    
    % update the occupancy grid map
    % student algorithm 
    
    map_data = receive(map_subscriber, 10);
    
    % visualize the map after every 50th update.
    if ~mod(updateCounter, 5)
         tMap = reshape(flip(map_data.Data), [260, 270]);
         tMap = rot90(rot90(rot90(rot90(tMap))));
         mapHandle.CData = tMap;
         title(axesHandle, ['OccupancyGrid: Update ' num2str(updateCounter)]);
    end
    
    % robot path following5.45
    [v, w] = controller(RobotPose);
    drive(sim, v, w);
    
    % Update the counter and distance to goal.
    updateCounter = updateCounter + 1;
    distanceToGoal = norm(RobotPose(1:2) - robotGoal);
    
    % Wait for control rate to ensure 10 Hz rate.
    waitfor(controlRate);
end
%% pretvaranje occGrid.mat u occGrid.csv
file = load('bestMap.mat');
csvwrite('occGrid.csv', file.bestMap)