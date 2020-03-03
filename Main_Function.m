import java.util.*;
clc
clear all;
close all;

%% Hardware Initialization  
mylego = legoev3('usb'); %Create a connection to the EV3 brick

rightMotor = motor(mylego,'B');
leftMotor = motor(mylego,'D');

resetRotation(rightMotor)
resetRotation(leftMotor)

%% Variable Initialization

%Vehicle Parameters and Pose:
PI = 3.14159;
radius = 0.0558/2.0; % wheel radius in meters
circumference = 2*PI*radius; %wheel circumference
L = 0.127 ;%0.108 %Distance between wheels (Differential drive) in meters

X = -0.3 ;                %robot start location X
Y = -0.10;                %robot start location Y
heading = 90.0*PI/180;    %robot Heading

%disance errors + tolerances
distanceTol = 0.03; 
currentError = 0;
previousError = 0;

%Heading errors + tolerances (note: H appended to end of variables)
headingTol = 5; %degree
currentErrorH = 0;
previousErrorH = 0;

lastR1 = 0;
lastR2 = 0;

%motor parameters
motorSpeed = 0;

Kp = 0.6;% 0.6;       %proportional gain
%Ki = 0;        %integral gain
Kd = 0.31; %0.28      %derivative gain  

wStandard = 40; % w = omega = angular wheel velocity
KpH = 0.2;      %proportional gain -- H indicates heading parameter... 
%KiH = 0;       %integral gain
KdH = 0.2;      %derivative gain

%sampling time
ts = 0.05;


%% Waypoint Setup
waypointX = -0.30; %robot first waypoint inside environment
waypointY = +0.30;
collision = 0;     % collision state
%Waypoint Counter
waypointIndex = 1;
%Waypoint Timer - Start
tic;

%% Sensors and Sensor Motors
 
mysonicsensor = sonicSensor(mylego,1);
mapMotor = motor(mylego,'C');
zero_odometers = getRotation(mapMotor);

bumpSenseRight = colorSensor(mylego,4);
bumpSenseLeft = colorSensor(mylego,2);

%% Map Creation
mapSize = 240; %size of grid (width and height) 182cm / 6ft
cellSize = 2;  %size of each cell (cm)
numberOfCells = floor(mapSize/cellSize); %number of cells required for width and height of grid

mapHit = ones(numberOfCells,numberOfCells)*0.5; %create empty grid of all zeros
mapMiss = ones(numberOfCells,numberOfCells)*0.5; %create empty grid of all zeros

%Create map Stacks
angleStack = Stack();
distanceStack = Stack();

%% Main Program

behaviour = 1; % Variable for bahviour selection. 

%Start motors
rightMotor.Speed = motorSpeed;
leftMotor.Speed = motorSpeed;
start(rightMotor);
start(leftMotor);

done = false;

while done == false
    %% DRIVE LOOP: Obtain next waypoint OR stop due to obstacle:
    timer = 0;
    bumper = 0;
    
    while behaviour == 1 
        % Bump Sensor Reading
        rightBump = readLightIntensity(bumpSenseRight, 'reflected');
        leftBump =  readLightIntensity(bumpSenseLeft, 'reflected');
        
        if (rightBump > 11 || leftBump > 14) && collision ~= 1
            % Only enters if last loop was also not collision avoidance
            disp("Collision Detected");
            rightMotor.Speed = 0; 
            leftMotor.Speed = 0;
            collision = 1;
            
            %% Map to grid
            if(rightBump > 11 && leftBump <= 14) %right bumper triggered
                
                [angleStack, distanceStack] = addObjectToStack(angleStack, distanceStack, 1);
                disp(join(["Right Bumper Hit --- R: " rightBump " L: " leftBump]));
                bumper = 1;
            elseif(rightBump <= 11 && leftBump > 14) %left bumper triggered
                
                [angleStack, distanceStack] = addObjectToStack(angleStack, distanceStack, 2);
                disp(join(["Left Bumper Hit --- R: " rightBump " L: " leftBump]));
                bumper = 2;
            elseif(rightBump > 11 && leftBump > 14) %both bumpers triggered
            
                [angleStack, distanceStack] = addObjectToStack(angleStack, distanceStack, 3);
                disp(join(["Both Bumpers Hit --- R: " rightBump " L: " leftBump]));
                bumper = 3;
            end
            
            %% Generate map update / new reading
            [mapHitUpdate, mapMissUpdate] = generateGrid(angleStack, distanceStack, ((X-0.3)*100/cellSize), ((Y+0.3)*100/cellSize), rad2deg(heading), mapSize, cellSize);

            %% update overall map 
            mapHit = mapHit + mapHitUpdate;
            mapMiss = mapMiss + mapMissUpdate;
        end
        
        % MOTION DRIVER

        % Read rotation counter in degrees
        r1 = readRotation(leftMotor) ;          
        r2 = readRotation(rightMotor) ;  

        %current error between required and actual distance from waypoint
        currentError = getDistance_fn(waypointX, waypointY, X, Y);

        %current error between required and actual heading to waypoint
        currentErrorH = wrapToPi(getAngle_fn(X, Y, heading, waypointX, waypointY)) *180.0/PI;

        % IF waypoint Reached:
        if(abs(currentError) < distanceTol) && collision ~= 1
            rightMotor.Speed = 0;
            leftMotor.Speed = 0;
            break    %break loop when waypoints completed

        % Make heading correction (TURN):
        elseif abs(currentErrorH) > headingTol && collision ~= 1
            if currentErrorH >= 0
                    w = 1;
            else 
                    w = -1;
            end
                % Compute command signal (PD Control Signal)
                xn = (KpH*currentErrorH) + (KdH*((currentErrorH-previousErrorH)/ts));                  

                % Saturate command signal between 8 and 15
                xn = saturate(10, 15, xn); % 8 15

                rightMotor.Speed = w*xn; 
                leftMotor.Speed = -rightMotor.Speed;

                % Update error between timesteps:
                previousErrorH = currentErrorH;

        % Make Position correction (DRIVE):
        elseif currentError > distanceTol && collision ~= 1

                % Compute command signal (PD Control Signal)
                xn = (Kp*currentError) + (Kd*((currentError-previousError)/ts));                      

                % Saturate command signal between 15 and 50
                xn = saturate(20, 55, xn);

                rightMotor.Speed = xn; 
                leftMotor.Speed = xn;

                % Update error between timesteps:
                previousError = currentError;
        
        % Reverse Motion (only used after a collision i.e Collision > 0
         elseif collision == 1 
                %disp("Reversing From Obstacle")
                rightMotor.Speed = -16; 
                leftMotor.Speed = -15;
                timer = timer + 1;
        end

        % STATE UPDATE (Odometry):
        wl = double(r1-lastR1)/(360.0*ts)*2.0*PI;  % wheel rad/second 
        wr = double(r2-lastR2)/(360.0*ts)*2.0*PI;  % wheel rad/second 

        turningRate = double(radius*(wr-wl)/L);    % Vehicle Turning Rate rad/s

        % Compute heading:
        % Map heading to (0, +-180) (ie. no wrap around):
        heading = wrapToPi(heading + (turningRate*ts));
        %disp(rad2deg(heading));
        lastR1 = r1; % update encoder count between timesteps
        lastR2 = r2; % update encoder count between timesteps

        vlinear = double(radius*(wr+wl)/2); %Linear velocity

        X = double(X + (cos(heading)*vlinear*ts)); %update X 
        Y = double(Y + (sin(heading)*vlinear*ts)); %update y 
        
        if timer > 10 %20 before but was a bit much
            rightMotor.Speed = 0; 
            leftMotor.Speed = 0;
            break;
        end 

        %Delay the next iteration by the sampling time specified
        pause(ts);
    end
    
    %%  Ultrasonic Sensing,Map Updating, and Path Planning:

    %% Call Sensor reading function
    [angleStack, distanceStack] = getMapValues (mysonicsensor, mapMotor, angleStack, distanceStack, zero_odometers);
    angleStack2 = angleStack.clone();
    distanceStack2 = distanceStack.clone();

    %% Generate map update / new reading
    [mapHitUpdate, mapMissUpdate] = generateGrid(angleStack, distanceStack, ((X-0.3)*100/cellSize), ((Y+0.3)*100/cellSize), rad2deg(heading), mapSize, cellSize);

    %% Update Hit/Miss/Occupancy Map 
    mapHit = mapHit + mapHitUpdate;
    mapMiss = mapMiss + mapMissUpdate;
    
    map = zeros(numberOfCells,numberOfCells); %create empty grid of all zeros
    hits = 0;       %initialize variable
    misses = 0;     %initialize variable

    %for each x and y cell, calculate the ratio based on the hits and
    %misses of that cell
    for i = 1 : numberOfCells
       for j = 1 : numberOfCells

           hits = mapHit(i,j);
           misses = mapMiss(i,j);

           map(i,j) = (hits/(hits+misses));
       end
    end
    
    %% Plot map results
    I = mat2gray(map,[1 0]);
    figure();
    imshow(I);
    
    %end
    
    %% Function Calls (Sensor readings, SLAM etc.)
    
    %SLAM (Future Work)
    
    %Path Planner
    [waypointX, waypointY, collision, waypointIndex, done] = path_planning(done, mylego, distanceStack2, angleStack2, X, Y, rad2deg(heading), collision, bumper, waypointIndex); % Function call path planner for next waypoint
    behaviour = 1;
    
end

disp("Mission Completed");

%stop motors
rightMotor.Speed = 0;
leftMotor.Speed = 0;
clear; %clear all connections


