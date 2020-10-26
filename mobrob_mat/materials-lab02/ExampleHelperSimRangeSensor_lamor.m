classdef ExampleHelperSimRangeSensor_lamor < robotics.algs.internal.GridAccess & handle
    %ExampleHelperSimRangeSensor_lamor Simulation of range sensor readings
    %   This class is included only for demonstration purposes. The name of
    %   this class and its functionality may change without notice in a
    %   future release, or the class itself may be removed.
    %
    %   This class simulates the readings of a range sensor, e.g. a LIDAR,
    %   in a given map. The opening angle of the range sensor is 180
    %   degrees and the number of returned readings within this opening
    %   angle can be adjusted dynamically.
    %   The simulated sensor also has a maximum range. If for a given
    %   sensor reading, no obstacle is found within the maximum range, that
    %   sensor reading is returned as NaN.
    %
    %   ExampleHelperSimRangeSensor_lamor properties:
    %       Map         - Map to be used by the range sensor
    %       MaxRange    - Maximum detection range of sensor (in meters)
    %       NumReadings - Number of sensor readings in the 180 opening cone
    %       SensorNoise - Gaussian sensor noise of range readings (stdev in meters)
    %       AngleSweep  - (Read-only) The sweep angles for the range sensor
    %
    %
    %   ExampleHelperSimRangeSensor_lamor methods:
    %       getReading - Return the range readings
    %
    %   See also RobotSimulator.
    
    %   Copyright 2015-2016 The MathWorks, Inc.
    
    properties
        %Map - Map to be used by the range sensor
        %   This is expected to be a robotics.BinaryOccupancyGrid object.
        Map
        
        %MaxRange - Maximum detection range of sensor (in meters)
        %   This also corresponds to the maximum length of each ray in world
        %   units. If an obstacle is encountered within this limit, the range
        %   is the euclidean distance to that obstacle. If it is beyond
        %   that limit, the range is NaN.
        %   Set this value to be appropriate to the desired characteristics of
        %   your sensor.
        %   Default: 5 meters
        MaxRange = 5
        
        %NumReadings - Number of sensor readings in the 180 opening cone
        %   Default: 21 readings
        NumReadings = 360
        
        %SensorNoise - Gaussian sensor noise of range readings (stdev in meters)
        %   Default: 0 meters
        SensorNoise = 0
    end
    
    properties (SetAccess = private)
        %AngleSweep - The sweep angles for the range sensor
        %   Here, 0 is straight ahead and -pi/2 is to the right of the
        %   robot.
        AngleSweep
    end
    
    methods
        function obj = ExampleHelperSimRangeSensor_lamor()
            %ExampleHelperSimRangeSensor_lamor Constructor for range sensor object
            
            obj.AngleSweep = linspace(-pi, pi*(1-2/obj.NumReadings), obj.NumReadings);
        end
        
        function [ranges, angles, collisionLoc] = getReading(obj, robotPose)
            %getReading Return the range readings for simulated sensor
            %   [ranges, angles, collisionLoc] = getReading(obj, robotPose)
            %   returns laser measurements at robotPose [x y theta]. Laser
            %   measurement includes ranges, which is euclidean distance
            %   from robot to obstacle, angles which is the absolute angles
            %   used for the sweep (that is not relative to robot
            %   orientation), and collisionLoc, a Nx2 matrix of collision
            %   locations (world coordinates).
            
            validateattributes(robotPose, {'numeric'}, {'nonempty', 'vector', ...
                'numel', 3, 'nonnan', 'real', 'finite'}, 'getReading', 'robotPose');
            
            angles = obj.AngleSweep';
            
            [ranges, collisionLoc] = robotics.algs.internal.calculateRanges(double(robotPose), angles, obj.MaxRange, ...
                obj.Map.Grid, obj.Map.GridSize, obj.Map.Resolution, obj.Map.GridLocationInWorld);
            
            %[dmy, ~] = laser2sonar(ranges, angles, 8);
            %for i=1:length(ranges)-1
            %    int8(floor(i*8.0/length(ranges)))+1;
            %    ranges(i) = dmy(int8(floor(i*8.0/length(ranges)))+1);
            %end
            
            % Add noise to ranges
            if obj.SensorNoise ~= 0
                ranges = obj.addGaussianNoise(ranges, 0, obj.SensorNoise);
            end
        end
    end
    
    methods
        function set.Map(obj, map)
            %set.Map Setter function for Map property
            validateattributes(map, {'robotics.BinaryOccupancyGrid'}, ...
                {'nonempty', 'scalar'}, 'ExampleHelperSimRangeSensor_lamor', 'Map');
            obj.Map = map;
        end
        function set.MaxRange(obj, maxRange)
            %set.MaxRange Setter function for MaxRange property
            validateattributes(maxRange, {'double'}, ...
                {'nonempty', 'scalar', 'nonnan', 'real', 'finite'}, 'ExampleHelperSimRangeSensor_lamor', 'MaxRange');
            obj.MaxRange = maxRange;
        end
        function set.NumReadings(obj, numReadings)
            %set.NumReadings Setter function for NumReadings property
            validateattributes(numReadings, {'double'}, ...
                {'nonempty', 'scalar', 'nonnan', 'real', 'finite'}, 'ExampleHelperSimRangeSensor_lamor', 'NumReadings');
            obj.NumReadings = numReadings;
            
            % Also update the angle sweep
            obj.AngleSweep = linspace(-pi, pi*(1-2/obj.NumReadings), obj.NumReadings); %#ok<MCSUP>
        end
        function set.SensorNoise(obj, sensorNoise)
            %set.SensorNoise Setter function for SensorNoise property
            validateattributes(sensorNoise, {'double'}, ...
                {'nonempty', 'scalar', 'nonnan', 'real', 'finite'}, 'ExampleHelperSimRangeSensor_lamor', 'SensorNoise');
            obj.SensorNoise = sensorNoise;
        end
    end
    
    methods (Static, Access = private)
        function B = addGaussianNoise(A, mean, stdev)
            %addGaussianNoise Add Gaussian noise to input values
            %   B = addGaussianNoise(A, mean, stdev) adds Gaussian noise
            %   with a given MEAN and standard deviation STDEV to the input
            %   A and return the "noisy" output in B.
            
            B = A + randn(size(A))*stdev - stdev/2 + mean;   %# add gaussian noise
        end
    end
    
    
end

