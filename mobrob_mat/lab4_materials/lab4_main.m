%% setup the robot simulator
rosshutdown
clear all; close all; clc;
rosinit
sim = ExampleHelperRobotSimulator('simpleMap');
controlRate = robotics.Rate(10);
dt = controlRate.DesiredPeriod;
%setRobotPose(sim, [1.5 0.75 pi/2]);

% Nova 1
setRobotPose(sim, [1, 11, -pi/2]);

% Nova 2
%setRobotPose(sim, [8, 8, 0]);

sim.LaserSensor.NumReadings = 8;
sim.LaserSensor.SensorNoise = 0.025;
orientationNoise = 0.01;
translationNoise = 0.05;
updateCounter = 1; 
robotPose = sim.Robot.Pose; 
goalRadius = 0.25;

enableROSInterface(sim, true);

% load map obtained in lab2, it should be robotics.BinaryO2ccupancyGrid object called 'map' 
% load(...)

% load checkpointsf
load('checkpoints.mat')
checkpoints = checkpoints;
%%
% calculate path through checkpoints in real world coordinates from robot's current pose

% example of calculating the path between robot's current pose and the first
% checkpoint with map inflation=0.5:

% start = world2grid(map, robotPose(1:2));
% goal = world2grid(map, checkpoints(1,:));
% path_example = grid2world(sim.Map, Astar(map.copy(), start, goal, 0.5));


% concatenate paths for each checkpoint and define subgoals
path = [[10,2]];
% subgoals = ...
% plot(path(:,1), path(:,2),'k--d');

PoseHistory = [];
OdomHistory = [];
CorrectionHistory = [];

% initial values of odometry and EKF output
robotOdom = sim.Robot.Pose;
correctedPose = sim.Robot.Pose;
robotGoal = path(end,:);
distanceToGoal = norm(robotGoal - correctedPose(1:2));
% measured translational and rotational robot velocity
v_measured = 0;
w_measured = 0;


% initialize figure for localization
figureHandle = figure('Name', 'EKF');
axesHandle = axes('Parent', figureHandle);
title(axesHandle, 'EKF: Update 0');
xlim(sim.Map.GridSize/sim.Map.Resolution);

% set initial value of matrix P
% P = ...
brzine_sub = rossubscriber('/brzine', 'geometry_msgs/Twist');
x_k_subscriber = rossubscriber('/x_hat', 'geometry_msgs/Pose');

% Define Robot odometry publisher
o_pub = rospublisher('/odom_noise', 'nav_msgs/Odometry');
o_msg = rosmessage(o_pub);
v = 0;
w = 0;
%% start the robot control loop
tic 
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
    
    % get sensor readings2


    SensorReadings=sim.LaserSensor.getReading(robotPose);
    SensorReadings(isnan(SensorReadings)) = sim.LaserSensor.MaxRange; 
    AngleSweep = sim.LaserSensor.AngleSweep;  
    
    %drive(sim, 0, 0);
    x_k = x_k_subscriber.LatestMessage;
    %drive(sim, v, w);
    
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
    
    % plot EKF result
    if ~mod(updateCounter, 50)
        figure(2), plot(PoseHistory(:,1), PoseHistory(:,2), 'b', 'LineWidth', 4);
        hold on;
        figure(2), plot(OdomHistory(:,1), OdomHistory(:,2), 'r', 'LineWidth', 2); 
        figure(2), plot(CorrectionHistory(:,1), CorrectionHistory(:,2), 'g', 'LineWidth', 2); 
        figure(2), title(axesHandle, ['EKF: Update ' num2str(updateCounter)]);
        figure(2), legend('True pose', 'Odometry', 'EKF')
        figure(2), xlim([0, sim.Map.GridSize(1)/sim.Map.Resolution]);
        figure(2), ylim([0, sim.Map.GridSize(2)/sim.Map.Resolution]);
        figure(2), viscircles(checkpoints,goalRadius*ones(length(checkpoints),1), 'Color','k');
      end
    
    % calculate the velocities with respect to subgoals and drive the robot 
    % velocities MUST be calculated using the corrected pose
    
    % student algorithm start
    %drive(sim, 0, 0);
    brzine = brzine_sub.LatestMessage;
    %drive(sim, v, w);
    
    try
        v = brzine.Linear.X;
        w = brzine.Angular.Z;
    catch
        disp("No message recieved - vel")
        v = 0;
        w = 0;
        % Did not recieve message from subscriber... 
        % Pass
    end
    % [v, w] = ...
    % student algorithm end
    

    drive(sim, v , w);
    % simulated noisy robot velocity measurements
    v_measured = v + (abs(v) > 0) * normrnd(0, translationNoise);
    w_measured = w + (abs(w) > 0) * normrnd(0, orientationNoise);
    % clear true v and w since they are not available to the EKF
    %clear v w;
    
    % update the counter and distance to goal.
    updateCounter = updateCounter+1;
    %distanceToGoal = norm(robotGoal - correctedPose(1:2));
    
    % FOR TESTING
    distanceToGoal = norm(robotGoal - robotPose(1:2)); 
    
    % wait for control rate to ensure 10 Hz rate.
    waitfor(controlRate);
end
toc


figure(2), plot(PoseHistory(:,1), PoseHistory(:,2), 'b', 'LineWidth', 4);
hold on;
figure(2), plot(OdomHistory(:,1), OdomHistory(:,2), 'r', 'LineWidth', 2); 
figure(2), plot(CorrectionHistory(:,1), CorrectionHistory(:,2), 'g', 'LineWidth', 2); 
figure(2), title(axesHandle, ['EKF: Update ' num2str(updateCounter)]);
figure(2), legend('True pose', 'Odometry', 'EKF')
figure(2), xlim([0, sim.Map.GridSize(1)/sim.Map.Resolution]);
figure(2), ylim([0, sim.Map.GridSize(2)/sim.Map.Resolution]);
figure(2), viscircles(checkpoints,goalRadius*ones(length(checkpoints),1), 'Color','k');
%% pretvaranje occGrid.mat u occGrid.csv
file = load('checkpoints.mat')
csvwrite('checkpoints.csv', file.checkpoints)