% hehehehehe time to break all the things :D
set(0, 'RecursionLimit', 1000)

% <maze information>
% maze is 6x3
% 4 yards (365 cm) long
% walls are 2 cm thick
% represented as a 120 x 120 grid. each grid square represents a 6x6cm area
% grid values:
%  0. unknown
%  1. wall
%  2. open unknown
%  3. endpoint
%  4. open
%  5. nearwall
%  6. a boarder line that indicates a split between an unknown region and
%       open area. these areas are endpoints for explore mode
%  7. barriers to separate the unknown and the open
% 
% open unknown is an area where the underlying color is unknown
% open means the area has been explored and the color is not needed for
%   later
% endpoint indicates that the destination color was detected
% nearwall are spaces that are near walls and should not be traversed over
%   for fear of striking the wall
% wall is the wall (:O)
% unknown is an area that the distance sensor has not passed over.

% <task information>
% 0. determine a new destination, exploration gaps first
% 1. determine a new destination, unidentified open spaces first
% 2. determine a new destination, final goal first
% 3. perform a scan

% <variable information>
% mode: "auto" or "manual", decides whether to determine movements based on
%   the navigation AI or user control
% brick: the brick object being controled by the script
% throttle: value ranging from -100 to 100, defines forward/backward motion
% turn: value ranging from -100 to 100, defines differential in speed
%   between left and right motors
% lift: value repersenting the current state of the lift, can be "up" or
%   "down"
% deltaTime: approximate time since last itteration of control loops
% logTime: a datetime value representing when the last control loop began
% throttleTime: the time in seconds it takes the throttle to go from 0 to
%   100
% key: the current key that is being pressed
% brickName: name of the brick that will be controled
% distance: current measured distance
% lastDistance: distance measured on the last cycle
% jailbroken: overrides idiot proofing measures, allows for full control
 
% <port information> 
% A: right drive motor, - is forwards, + is backwards
% B: left drive motor, - is forwards, + is backwards
% D: lift motor, behavior currently undefined
% 4: sonar sensor, behavior currently undefined
% 2: color sensor, behavior currently undefined
% 3: gyroscope, behavior currently undefined

% <behavior of system>
% 1. get key input
% 2. log sensor data
% 3. reset gyro and time tracking
% 4. perform mapping functions
% 5. determine motor functions
% 6. perform movements

% <behavior of AI>
% 1. initial mapping
% 2. searching for pickup
% 3. transfer to manual control for pickup
% 4. searching dropoff
% 5. dropoff

% <searching behavior>
% 1. take current position and position of walls, find an unexplored gap
%   and move towards the gap
% 2. check if there is a color of interest detected
% 3. for red, stop for a specified length of time
% 4. for yellow and no passenger, log position for dropoff
% 5. for yellow and a passenger, perform dropoff
% 6. for blue, switch to manual for pickup
% 7. if the robot is outside of the bounds of the currently generated
%   lines, perform a mapping sweep

% <initial mapping behavior>
% 1. rotate 360*, logging distance points
% 2. turn points into connected lines, and attempt to simplify those lines
%   for the map
% 3. after rotation and mapping is complete, go to search mode

% <dropoff behavior>
% 1. stop
% 2. lower lift
% 3. reverse for a short length of time to clear the wheelchair

%brick connection
brickName = "Tanker";
while(true)
    %get status
    connected = input("Is the brick already connected?\n>", "S");
    
    %connect if no, exit if yes or no, display error message otherwise
    switch connected
        case {"Yes", "yes", "Y", "y"}
            break;
        case {"No", "no", "N", "n"}
            brick = ConnectBrick(brickName);
            %test tone
            brick.playTone(1000,10,500);
            break;
        otherwise
            disp("Unknown status <" + connected + ">");
    end
end

%startup parameters
while(true)
    %get startmode
    startMode = input("Enter mode to start in:\n1.Manual\n2.Auto\n3.Dumb\n>", "s");

    %set mode or display error message
    switch startMode
        case {"1", "Manual", "manual", "M", "m"}
            mode = "manual";
            break;
        case {"2", "Auto", "auto", "A", "a"}
            mode = "auto";
            break;
        case {"3", "Dumb", "dumb", "D", "d"}
            mode = "dumb";
            break;
        otherwise
            disp("Unknown mode <" + startMode + ">");
    end
end
while(true)
    %get startmode
    liftState = input("Enter current state of the lift:\n1.up\n2.down\n>", "s");

    %set mode or display error message
    switch liftState
        case {"up", "u"}
            lift = "up";
            break;
        case {"down", "d"}
            lift = "down";
            break;
        otherwise
            disp("Unknown state <" + liftState + ">");
    end
end
while(true)
    %desired state
    free = input("Jailbreak robot? (NOT RECOMMENDED! BREAKS A LOT OF STUFF!)\n>", "S");
    
    %set jailbroken state
    switch free
        case {"Yes", "yes", "Y", "y"}
            jailbroken = true;
            break;
        case {"No", "no", "N", "n"}
            jailbroken = false;
            break;
        otherwise
            disp("Unknown state <" + free + ">");
    end
end

%keyboard setup
global key;
InitKeyboard();

%misc variable setup
subMode = "scan";
subMode = "forward";
brick.GyroCalibrate(3);
currentAngle = 0;
throttle = 0;
turn = 0;
logTime = datetime('now');
deltaTime = 0;
throttleTime = 2;
stopCooldown = 0;
distance = 0;
lastDistance = 0;
newAngle = 0;
deltaDistance = 0;
deltaAngle = 0;
lastAngle = 0;
x = 0;
y = 0;
offsetX = -7;
offsetY = 9;
colorOffsetX = 6;
colorOffsetY = 16;
hasPassenger = false;

%navigation variables
map = zeros(120, 120);
scanPoints = [];
angleTraversed = 0;
instructions = [0 ; 0];
task = 0;

%sensor setup
brick.SetColorMode(2, 2);
detectedColor = 0;

%input info
disp("WASD and arrows to move in manual mode.")
disp("Space to stop robot.")
disp("Q to abort and stop robot.")
disp("E to manually switch control modes.")
disp("L to activate lift. Vehicle must be still and in manual mode for this command to be acknowledged.")
disp("It is recommended to only turn in place and move straight forward. Combining turning and forward movement is more likely to cause positional desync. Doing so is disabled by default, but jailbreaking the device will allow for unregulated movement.")

%running
while(true)
    % 1. get key input for priority tasks
    switch key
        case 'q'
            break;
        case 'e'
            if(mode == "manual")
                mode = "auto";
            else
                mode = "manual";
            end
    end

    % 2. log sensor data
    lastDistance = distance;
    distance = brick.UltrasonicDist(4);
    deltaDistance = distance - lastDistance;

    detectedColor = brick.ColorCode(2);

    newAngle = brick.GyroAngle(3);
    if(~isnan(newAngle))
        lastAngle = currentAngle;
        currentAngle = newAngle;
        deltaAngle = currentAngle - lastAngle;
    end
    if(currentAngle < 0)
        currentAngle = currentAngle + 360;
    elseif(currentAngle >= 360)
        currentAngle = currentAngle - 360;
    end

    % 2.5. display debug info for sensors
    disp("Color: " + detectedColor);
    disp("Angle: " + currentAngle);
    disp("Delta time: " + deltaTime);
    disp("Distance: " + distance);
    disp("Position: (" + x + ", " + y + ")");
    disp(" ");

    % 3. reset deltatime and decrement stopCooldown
    deltaTime = seconds(datetime('now') - logTime);
    logTime = datetime('now');
    if(stopCooldown > 0)
        stopCooldown = stopCooldown - deltaTime;
    end

    % 4. perform mapping functions. Very prone to breaking under non-ideal
    %    conditions. This is prevented by setting jailbroken to false.
    if(throttle ~= 0 && distance ~= 255)
        x = x + deltaDistance * cos(currentAngle * pi / 180);
        y = y + deltaDistance * sin(currentAngle * pi / 180);
    end

    % 5. determine motor functions
    if(mode == "manual")
        switch key
            case {'w', 'uparrow'}
                if(jailbroken)
                    throttle = min(100, throttle + (deltaTime / throttleTime * 100));
                elseif(turn == 0)
                    throttle = min(100, throttle + (deltaTime / throttleTime * 100));
                end
            case {'s', 'downarrow'}
                if(jailbroken)
                    throttle = max(-100, throttle - (deltaTime / throttleTime * 100));
                elseif(turn == 0)
                    throttle = max(-100, throttle - (deltaTime / throttleTime * 100));
                end
            case {'a', 'leftarrow'}
                if(jailbroken)
                    turn = max(-100, turn - (deltaTime / throttleTime * 100));
                elseif(throttle == 0)
                    turn = max(-100, turn - (deltaTime / throttleTime * 100));
                end
            case {'d', 'rightarrow'}
                if(jailbroken)
                    turn = min(100, turn + (deltaTime / throttleTime * 100));
                elseif(throttle == 0)
                    turn = min(100, turn + (deltaTime / throttleTime * 100));
                end
            case 'space'
                throttle = 0;
                turn = 0;
            case 'l'
                if(jailbroken)
                    lift = toggleLiftState(brick, lift);
                elseif(throttle == 0 && turn == 0)
                    lift = toggleLiftState(brick, lift);
                    hasPassenger = true;
                end
        end
        %color red is detected, stop for 4 seconds
        if(detectedColor == 5 && stopCooldown <= 0)
            stopCooldown = 10;
            brick.StopMotor('A');
            brick.StopMotor('B');
            pause(4);
        end

    elseif(mode == "dumb")
        switch subMode
            case "forward"
                if(distance < 30)
                    subMode = "turnRight";
                    throttle = 0;
                else
                    throttle = 50;
                end
            case "turnLeft"
                %rotate in place
                throttle = 0;
                turn = -50;

                %track progress
                angleTraversed = angleTraversed + deltaAngle;
                disp(angleTraversed);

                %add point detected to scanning matrix
                scanPoints = add(scanPoints, toPoint(x, y, distance, currentAngle, offsetX, offsetY));
                
                %if a full rotation has occured, begin plotting points on
                %   the map. this is likely an intensive process, and will
                %   take a while.
                if(angleTraversed < -90)
                    subMode = "forward";
                    turn = 0;
                    angleTraversed = 0;
                end
            case "turnRight"
                %rotate in place
                throttle = 0;
                turn = 50;

                %track progress
                angleTraversed = angleTraversed + deltaAngle;
                disp(angleTraversed);

                %add point detected to scanning matrix
                scanPoints = add(scanPoints, toPoint(x, y, distance, currentAngle, offsetX, offsetY));
                
                %if a full rotation has occured, begin plotting points on
                %   the map. this is likely an intensive process, and will
                %   take a while.
                if(angleTraversed > 90)
                    subMode = "forward";
                    turn = 0;
                    angleTraversed = 0;
                end
        end
    elseif(mode == "auto")
        switch subMode
            case "scan"
                %rotate in place
                throttle = 0;
                turn = 30;

                %track progress
                angleTraversed = angleTraversed + deltaAngle;
                disp(angleTraversed);

                %add point detected to scanning matrix
                scanPoints = add(scanPoints, toPoint(x, y, distance, currentAngle, offsetX, offsetY));
                
                %if a full rotation has occured, begin plotting points on
                %   the map. this is likely an intensive process, and will
                %   take a while.
                if(angleTraversed > 360)
                    %stop moving
                    turn = 0;
                    rightThrottle = getRightPower(throttle, turn);
                    leftThrottle = getLeftPower(throttle, turn);
                    runMotors(brick, rightThrottle, leftThrottle);
                    angleTraversed = 0;
                    disp("Scan complete, mapping.");

                    %create walls on map
                    pointSize = size(scanPoints);
                    pointCount = pointSize(2);
                    index = 1;
                    %create a line between two adjacent points, based on
                    %length determine if it is a wall gap or an open gap.
                    while(index <= pointCount)
                        %if the length is greater than 20, then it is a
                        %gap, otherwise it is a wall segment. distance
                        %might have to change.
                        A = scanPoints(:, index);
                        B = scanPoints(:, mod(index, pointCount) + 1);
                        disp("Building wall " + index + " of " + pointCount)
                        if(gapSize(A, B) > 20)

                            %mark midpoint as a point to explore and seal
                            %an open barrier
                            midPoint = (A + B) / 2;
                            midPoint = pointToGraph(midPoint);
                            if(map(midPoint(2, 1), midPoint(1, 1)) == 0)
                                map(midPoint(2, 1), midPoint(1, 1)) = 6;
                            end

                            A = pointToGraph(A);
                            B = pointToGraph(B);
                            wallX = min(A(1, 1), B(1, 1));
                            wallY = min(A(2, 1), B(2, 1));
                            targetWallX = max(A(1, 1), B(1, 1));
                            targetWallY = max(A(2, 1), B(2, 1));
                            while(wallX <= targetWallX)
                                if(map(wallY, wallX) == 0)
                                    map(wallY, wallX) = 7;
                                end
                                wallX = wallX + 1;
                            end
                            wallX = wallX - 1;
                            while(wallY <= targetWallY)
                                if(map(wallY, wallX) == 0)
                                    map(wallY, wallX) = 7;
                                end
                                wallY = wallY + 1;
                            end
                            wallY = wallY - 1;
                            
                        else
                            %create a wall segment in an L shape.
                            A = pointToGraph(A);
                            B = pointToGraph(B);
                            wallX = min(A(1, 1), B(1, 1));
                            wallY = min(A(2, 1), B(2, 1));
                            targetWallX = max(A(1, 1), B(1, 1));
                            targetWallY = max(A(2, 1), B(2, 1));
                            while(wallX <= targetWallX)
                                map(wallY, wallX) = 1;
                                offX = -2;
                                while(offX <= 2)
                                    offY = -2;
                                    while(offY <= 2)
                                        if(map(wallY + offY, wallX + offX) == 0)
                                            map(wallY + offY, wallX + offX) = 5;
                                        end
                                        offY = offY + 1;
                                    end
                                    offX = offX + 1;
                                end
                                wallX = wallX + 1;
                            end
                            wallX = wallX - 1;
                            while(wallY <= targetWallY)
                                map(wallY, wallX) = 1;
                                offX = -2;
                                while(offX <= 2)
                                    offY = -2;
                                    while(offY <= 2)
                                        if(map(wallY + offY, wallX + offX) == 0)
                                            map(wallY + offY, wallX + offX) = 5;
                                        end
                                        offY = offY + 1;
                                    end
                                    offX = offX + 1;
                                end
                                wallY = wallY + 1;
                            end
                            wallY = wallY - 1;
                            
                        end

                        %increment loop
                        index = index + 1;
                    end

                    %fill open areas with open unknown tag
                    done = false;
                    while(~done)
                        converts = 0;
                        searchX = 1;
                        while(searchX <= 120)
                            searchY = 1;
                            while(searchY <= 120)
                                if(map(searchY, searchX) == 2)
                                    if(map(y - 1, x) == 0)
                                        converts = converts + 1;
                                        map = spreadToValue(map, 2, 0, searchX, searchY, 5);
                                    elseif(map(y + 1, x) == 0)
                                        converts = converts + 1;
                                        map = spreadToValue(map, 2, 0, searchX, searchY, 5);
                                    elseif(map(y, x - 1) == 0)
                                        converts = converts + 1;
                                        map = spreadToValue(map, 2, 0, searchX, searchY, 5);
                                    elseif(map(y, x + 1) == 0)
                                        converts = converts + 1;
                                        map = spreadToValue(map, 2, 0, searchX, searchY, 5);
                                    end
                                end
                                searchY = searchY + 1;
                            end
                            searchX = searchX + 1;
                        end
                        if(converts == 0)
                            done = true;
                        end
                    end
                    
                    subMode = "follow";
                    task = 0;
                end
            case "follow"
                instructionSize = size(instructions);
                if(instructionSize(2) > 1)
                    %proceed towards current objective
                    %check if angle is good, if not, stop and rotate
                    diffX = instructions(1, 2) - x;
                    diffY = instructions(2, 2) - y;
                    if(diffX == 0 && diffY >= 0)
                        angleTo = 90;
                    elseif(diffX == 0 && diffY < 0)
                        angleTo = 270;
                    else
                        angleTo = atan(diffY / diffX);
                        if(diffX < 0)
                            angleTo = angleTo + 180;
                        end
                    end

                    if(mod(abs(angleTo - currentAngle), 360) < 15)
                        turn = 0;
                        throttle = 50;
                    else
                        throttle = 0;
                        %decide which way to turn
                        if(mod(angleTo - currentAngle, 360) > 180)
                            turn = 50;
                        else
                            turn = -50;
                        end
                    end
                    
                    %check if objective has been reached
                    if(abs(diffX) < 8 && abs(diffY) < 8)
                        instructions(:, 2) = [];
                        throttle = 0;
                        turn = 0;
                    end
                else
                    %perform destination task
                    switch(task)
                        case 0
                            instructions = [instructions getNewInstructions(map, 6, 2, 3, x, y)];
                            
                        case 1
                            instructions = [instructions getNewInstructions(map, 2, 6, 3, x, y)];

                        case 2
                            instructions = [instructions getNewInstructions(map, 3, 6, 2, x, y)];
                            
                        case 3
                            subMode = "scan";
                            throttle = 0;
                            turn = 0;
                    end
                    %determine what next task should be
                    if(hasPassenger)
                        task = 2;
                    elseif(task == 0)
                        task = 0;
                    end
                    
                end
        end
    else
        disp("Unknown mode <" + mode + ">, aborting.")
        break;
    end

    % 5.5. get color data and update map accordingly
    if(true)
    switch detectedColor
        case 5 %red
            if(stopCooldown <= 0)
                stopCooldown = 10;
                brick.StopMotor('A');
                brick.StopMotor('B');
                pause(4);
            end
        case 4 %blue
            if(~hasPassenger)
                disp("Enabled manual control for pickup")
                mode = "manual";
            end

        case 2 %yellow
            if(hasPassenger)
                disp("Arrived")
                mode = "manual";
            end
            %set located goal space to 3 (endpoint)
            colorPoint = toPoint(x, y, 0, currentAngle, colorOffsetX, colorOffsetY);
            colorGrid = pointToGraph(colorPoint);
            map(colorGrid(2, 1), colorGrid(1, 1)) = 3;

        otherwise
            %create a 5x5 patch of open by replacing open unknown with open
            %(2 -> 4)
            colorPoint = toPoint(x, y, 0, currentAngle, colorOffsetX, colorOffsetY);
            colorGrid = pointToGraph(colorPoint);
            displacementX = -3;
            displacementY = -3;
            while(displacementX <= 3)
                while(displacementY <= 3)
                    if(map(colorGrid(2, 1) + displacementY, colorGrid(1, 1) + displacementX) == 2)
                        map(colorGrid(2, 1) + displacementY, colorGrid(1, 1) + displacementX) = 4;
                    end
                    displacementY = displacementY + 1;
                end
                displacementX = displacementX + 1;
            end
    end
    end

    % 6. calculate motor functions
    rightThrottle = getRightPower(throttle, turn);
    leftThrottle = getLeftPower(throttle, turn);

    % 7. perform motor functions
    runMotors(brick, rightThrottle, leftThrottle);

    % 8. pause to increase deltaTime for sensor relief
    if(deltaTime < .5)
        pause(.5 - deltaTime)
    end
end

%shutdown
CloseKeyboard();
brick.StopAllMotors();

%functions
function r = getRightPower(throttle, turn)
    %calculate power, and adjust if value would be over 100
    r = throttle - turn;
    if(abs(throttle) + abs(turn) > 100)% || abs(throttle - turn) > 100)
        r = r / 2;
    end

    %adjust for reverse motor positioning
    r = -r;
end
function r = getLeftPower(throttle, turn)
    %calculate power, and adjust if value would be over 100
    r = throttle + turn;
    if(abs(throttle) + abs(turn) > 100)% || abs(throttle - turn) > 100)
        r = r / 2;
    end

    %adjust for reverse motor positioning
    r = -r;
end
function runMotors(brick, rightThrottle, leftThrottle)
    if(rightThrottle == 0)
        brick.StopMotor('A');
    else
        brick.MoveMotor('A', rightThrottle);
    end
    if(leftThrottle == 0)
        brick.StopMotor('B');
    else
        brick.MoveMotor('B', leftThrottle);
    end
end
function r = toggleLiftState(brick, liftState)
    %raise or lower lift based on current state
    switch liftState
        case "up"
            brick.MoveMotorAngleRel('D',25,720,'Brake')
            r = "down";
            disp("Lowering lift");
        case "down"
            brick.MoveMotorAngleRel('D',-25,720,'Brake')
            r = "up";
            disp("Raising lift");
    end

    %allow 2 seconds for process to complete
    pause(2);
end
function r = crossProduct(A, B, C)
    r = crossProductXY(A(1, 1), A(2, 1), B(1, 1), B(2, 1), C(1, 1), C(2, 1));
end
function r = crossProductXY(xA, yA, xB, yB, xC, yC)
    %cross product of A, B, C.
    r = (xB - xA) * (yC - yA) - (yB - yA) * (xC - xA);
end
function r = linesIntersect(A, B, C, D)
    r = linesIntersectXY(A(1, 1), A(2, 1), B(1, 1), B(2, 1), C(1, 1), C(2, 1), D(1, 1), D(2, 1));
end
function r = linesIntersectXY(x1, y1, x2, y2, x3, y3, x4, y4)
    %some funky cross product shenanigans. finds if AB intersects CD. only
    %accounts for proper intersections (because I'm lazy, and will fix it
    %later if need be :D).
    if(crossProduct(x1, y1, x2, y2, x3, y3) * crossProduct(x1, y1, x2, y2, x4, y4) < 0 && crossProduct(x3, y3, x4, y4, x1, y1) * crossProduct(x3, y3, x4, y4, x2, y2) < 0)
        r = true;
    else
        r = flase;
    end
end
function r = add(matrix, newValue)
    %appends a new x,y pair (newValue) to a matrix of points
    r = [matrix newValue];
end
function r = toPoint(x, y, distance, angle, offX, offY)
    pointX = x + (distance + offX) * cos(angle / 180 * pi) + offY * sin(angle / 180 * pi);
    pointY = y + (distance + offX) * sin(angle / 180 * pi) + offY * cos(angle / 180 * pi);
    r = [pointX ; pointY];
end
function r = pointToGraph(point)
    r = [round(point(1, 1) / 6, 0) + 60 ; round(point(2, 1) / 6, 0) + 60];
end
function r = graphToPoint(graphPoint)
    r = [(graphPoint(1, 1) - 60) * 6 ; (graphPoint(2, 1) - 60) * 6];
end
function r = gapSize(A, B)
    r = sqrt((A(1, 1) - B(1, 1)).^2 + (A(2, 1) - B(2, 1)).^2);
end
function r = spreadToValue(map, value, spreadValue, x, y, maxSpread)
    r = map;
    if(maxSpread > 0)
        r(y, x) = value;
        if(r(y - 1, x) == spreadValue)
            r = spreadToUnknow(r, value, spreadValue, x, y - 1, maxSpread - 1);
        end
        if(r(y + 1, x) == spreadValue)
            r = spreadToUnknow(r, value, spreadValue, x, y + 1, maxSpread - 1);
        end
        if(r(y, x - 1) == spreadValue)
            r = spreadToUnknow(r, value, spreadValue, x - 1, y, maxSpread - 1);
        end
        if(r(y, x + 1) == spreadValue)
            r = spreadToUnknow(r, value, spreadValue, x + 1, y, maxSpread - 1);
        end
    end
end
function r = getNewInstructions(map, first, second, third, x, y)
    robotCoordinate = pointToGraph([x ; y]);
    potentialTargets = [0 ; 0 ; 0];
    searchX = 1;
    %find targets
    disp("Searching for targets");
    while(searchX <= 120)
        disp("SearchX: " + searchX)
        searchY = 1;
        while(searchY <= 120)
            if(map(searchY, searchX) == first)
                potentialTargets = add(potentialTargets, [searchX; searchY; gapSize(robotCoordinate, [searchX ; searchY])]);
            end
            searchY = searchY + 1;
        end
        searchX = searchX + 1;
    end
    targetsSize = size(potentialTargets);
    if(targetsSize(2) == 1)
        disp("SearchX: " + searchX)
        searchX = 1;
        while(searchX <= 120)
            searchY = 1;
            while(searchY <= 120)
                if(map(searchY, searchX) == second)
                    potentialTargets = add(potentialTargets, [searchX; searchY; gapSize(robotCoordinate, [searchX ; searchY])]);
                end
                searchY = searchY + 1;
            end
            searchX = searchX + 1;
        end
    end
    targetsSize = size(potentialTargets);
    if(targetsSize(2) == 1)
        searchX = 1;
        while(searchX <= 120)
            disp("SearchX: " + searchX)
            searchY = 1;
            while(searchY <= 120)
                if(map(searchY, searchX) == third)
                    potentialTargets = add(potentialTargets, [searchX; searchY; gapSize(robotCoordinate, [searchX ; searchY])]);
                end
                searchY = searchY + 1;
            end
            searchX = searchX + 1;
        end
    end

    %choose target closest to robot
    bestPoint = [potentialTargets(1, 2) ; potentialTargets(2, 2)];
    bestDistance = potentialTargets(3, 2);
    compareIndex = 3;
    targetCount = targetsSize(2);
    while(compareIndex <= targetCount)
        if(potentialTargets(3, compareIndex) < bestDistance)
            bestPoint = [potentialTargets(1, compareIndex) ; potentialTargets(2, compareIndex)];
            bestDistance = potentialTargets(3, compareIndex);
        end
        compareIndex = compareIndex + 1;
    end
    disp("target found: (" + bestPoint(1, 1) + ", " + bestPoint(2, 1) + ")");

    %create a grid-by-grid path to the target using A*
    disp("generating path");
    path = generatePath(map, robotCoordinate, bestPoint);

    %attempt to simplify by removing redundant sections of the path


    %convert grid path to set of x,y destinations
    r = [];
    pathSize = size(path);
    pathLength = pathSize(2);
    index = 1;
    while(index <= pathLength)
        node = path(1:2, index);
        node = graphToPoint(node);
        %center node in middle of each grid square
        node(1, 1) = node(1, 1) + 3;
        node(2, 1) = node(2, 1) + 3;
        r = add(r, node);
    end
end
function r = generatePath(map, start, finish)
    %Using A* algorithm similar to how it is explained here: https://www.geeksforgeeks.org/a-search-algorithm/

    %open list is tiles to be searched, closed list is tiles that have been
    %searched already
    openTileList = [start(1, 1) ; start(2, 1) ; 0 ; 0 ; start(1, 1) ; start(2, 1) ; 0];
    closedTileList = [];
    openListSize = 1;
    closedListSize = 0;
    stop = false;
    while(openListSize > 0 && ~stop)
        %find node on open list with lowest F value
        bestNode = openTileList(: , 1);
        bestF = bestNode(3, 1) + bestNode(4, 1);
        bestIndex = 1;
        searchIndex = 2;
        while(searchIndex <= openListSize)
            newNode = openTileList(: , searchIndex);
            newF = newNode(3, 1) + newNode(4, 1);
            if(newF < bestF)
                bestNode = newNode;
                bestF = newF;
                bestIndex = searchIndex;
            end
        end

        %move node to closed list and remove from open list
        closedTileList = add(closedTileList, bestNode);
        openTileList(:, bestIndex) = [];

        %generate 4 children and add to open list if appropriate
        children = [bestNode bestNode bestNode bestNode];
        %update parent info
        children(5, :) = bestNode(1, 1);
        children(6, :) = bestNode(2, 1);
        children(7, :) = bestNode(3, 1);
        children(3, :) = bestNode(3, 1) + 1;
        %give children unique locations
        children(1, 1) = children(1, 1) + 1;
        children(1, 2) = children(1, 1) - 1;
        children(2, 3) = children(2, 1) + 1;
        children(2, 4) = children(2, 1) - 1;
        %calculate h for each child
        children(4, 1) = quickDistance(finish, [children(1, 1) ; children(2, 1)]);
        children(4, 2) = quickDistance(finish, [children(1, 2) ; children(2, 2)]);
        children(4, 3) = quickDistance(finish, [children(1, 3) ; children(2, 3)]);
        children(4, 4) = quickDistance(finish, [children(1, 4) ; children(2, 4)]);

        %loop through children
        index = 1;
        while(index <= 4)
            %check if the child's position is a valid position. positions
            %are invalid if they are unknown, walls or near walls
            %(0, 1, 5). if the child is invalid, skip the child
            if(~(map(children(2, index), children(1, index)) == 0 || map(children(2, index), children(1, index)) == 1 || map(children(2, index), children(1, index)) == 5))

                %check if the child is the destination. if so, move to closed
                %list and terminate search
                if(children(1, index) == finish(1, 1) && children(2, index) == finish(2, 1))
                    finalChild = children(:, index);
                    closedTileList = add(closedTileList, finalChild);
                    closedListFullSize = size(closedTileList);
                    closedListSize = closedListFullSize(2);
                    stop = true;
                    break;
                else
                    %check if the child is already in the open list, and if the
                    %currently existing entry already has a lower F value. if it
                    %does, then skip this child
                    currentChild = children(:, index);
                    if(~(existsLower(currentChild, openTileList, openListSize)))
                        %check if the child is already in the closed list, and if the
                        %existing entry has a lower f value. if it does, then skip the
                        %child. otherwise, add the child to the open list
                        if(~(existsLower(currentChild, closedTileList, closedListSize)))
                            openTileList = add(openTileList, currentChild);
                            openListFullSize = size(openTileList);
                            openListSize = openListFullSize(2);
                        end
                    end
                end
            end
            index = index + 1;
        end
        
        %update list size
        openListFullSize = size(openTileList);
        openListSize = openListFullSize(2);
        closedListFullSize = size(closedTileList);
        closedListSize = closedListFullSize(2);
        disp("Open list size: " + openListSize);
        disp("Closed list size: " + closedListSize);
    end

    %generate the path, working back from the destination and following the
    %chain of parents with the lowest g values
    path = [];
    currentNode = [];
    index = closedListSize;
    while(index >= 1)
        if(closedTileList(1, index) == finish(1, 1) && closedTileList(2, index) == finish(2, 1))
            currentNode = closedTileList(: , index);
            break;
        end
        index = index - 1;
    end
    %construct path
    done = false;
    while(~done)
        %add node at first index in path
        path = add(currentNode, path);

        %get parent info
        parentX = currentNode(5, 1);
        parentY = currentNode(6, 1);
        parentG = currentNoed(7, 1);

        %check if end has been reached
        if(parentX == finish(1, 1) && parentY == finish(2, 1))
            done = true;
        else
            %find parent and update current node
            index = 1;
            while(index <= closedListSize)
                if(closedTileList(1, index) == parentX && closedTileList(2, index) == parentY && closedTileList(3, index) == parentG)
                    currentNode = closedTileList(: , index);
                    break;
                end
            end
        end
    end

    %return finished path
    r = path;
end
function r = quickDistance(A, B)
    r = abs(A(1, 1) - B(1, 1) + A(2, 1) - B(2, 1));
end
function r = existsLower(child, list, count)
    r = false;
    index = 1;
    while(index <= count)
        if(list(1, index) == child(1, 1) && list(2, index) == child(2, 1)) %same position
            if(list(3, index) <= child(3, 1)) %the list entry has a lower or equal g value
                r = true;
                break;
            end
        end
        index = index + 1;
    end
end
