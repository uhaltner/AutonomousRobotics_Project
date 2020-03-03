function [angleStack, distanceStack] = getMapValues (mysonicsensor, mapMotor, angleStack, distanceStack, zero_odometers)

    import java.util.*;

    %% Variables
    angularSpeed = +12;  %angular rotation speed of top sensor
    startingAngle = 180; %180 deg offset to account for backwards position

    %% Sensing Rotation

    mapMotor.Speed = angularSpeed; %set angular speed

    mapAngle = getRotation(mapMotor) - zero_odometers; %reset encoder count of motor that rotates ultrasonic sensor

    start(mapMotor); %start the motor
    while mapAngle < 360 %while not rotated 360 segrees
        mapAngle = getRotation(mapMotor) - zero_odometers;
        distance = getDistance(mysonicsensor); %get the distance reading from the ultrasonic sensor
      
        if(distance < 200) %dont include distance reading if above a determined distance
            angleStack.push(mapAngle+startingAngle); %180 starting angle of sensor in relation to front of robots
            distanceStack.push(distance);
        end
        pause(0.01) %sampling time for sensor = 0.01 sec
    end
    stop(mapMotor);

    pause(1);   %small pause between sensing and resetting

    %% Return rotation / position of sensor
    mapMotor.Speed = -1.5*angularSpeed;   %speed is 1.5 times faster to return to original position

    start(mapMotor);

    mapAngle = getRotation(mapMotor) - zero_odometers;
    while mapAngle >= 0
       mapAngle = getRotation(mapMotor) - zero_odometers;
    end

    stop(mapMotor);
end

%% Functions

%function to gt distance value from sonic sensor in cm
function d = getDistance(mysonicsensor)
        d = readDistance(mysonicsensor)*100;
end