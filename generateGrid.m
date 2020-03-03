function [mapHit, mapMiss] = generateGrid(angleStack, distanceStack, xRobot, yRobot, headingRobot, mapSize, cellSize)
 
    import java.util.*;

    %% Main
    numberOfCells = floor(mapSize/cellSize); %number of cells required for width and height of grid

    mapHit = zeros(numberOfCells,numberOfCells); %create empty grid to record number of times rays hit a cell
    mapMiss = zeros(numberOfCells,numberOfCells); %create empty grid to record number of times rays miss a cell
    
    %% Main Loop
    %while theere are values in the stack - assuming both stacks are same size
    while distanceStack.size() ~= 0

        d = round(distanceStack.pop());    %get distance rom stack
        th = angleStack.pop();      %get angle from stack
        th = th + headingRobot;     %orient robot readings based on grid orientation 
        
        %% at each heading loop by a range resolution until reaching d
        for D = 1 : d

            dx = cosd(th)*D;            %caluclate the horizontal distance
            dy = sind(th)*D;            %calculate the vertical distance

            cellX = round(dx/cellSize);    %determine translation into cells based on cell size
            cellY = round(dy/cellSize);    %+ using round instead of floor as there are negtive values

            xIndex = -round(xRobot) - cellX;      %determine the cell x value from the robot
            yIndex = round(yRobot) + cellY;       %determine the cell y value / subtract as up is smaller value in matrix

            %if the cell is located inside the matrix
            if((xIndex >= 1)&&(xIndex < numberOfCells)&&(yIndex >= 1)&&(yIndex <= numberOfCells))

                %if the full distance, aka the point of collision
                if(D == d)
                    if(d < 65) %only plot hits up to 65cm from robot
                        mapHit(yIndex, xIndex) = mapHit(yIndex, xIndex) + 7; %increment by weight of each hit
                        mapMiss(yIndex, xIndex) = 0;
                    end

                %if interval is smaller than the colision point/ a miss
                elseif(D <= (d-cellSize))
                    if(d < 100) %only plot miss up to 100cm from robot
                        mapMiss(yIndex, xIndex) = mapMiss(yIndex, xIndex) + 0.5; %increment by weight of each miss
                    end
                end
            end
        end
    end

    
    %% [Analyzing] Print Hit and Miss Occupancy map
%     I = mat2gray(mapHit,[1 0]);
%     figure(1);
%     imshow(I);

%     J = mat2gray(mapMiss,[10 0]);
%     figure(2);
%     imshow(J);
end