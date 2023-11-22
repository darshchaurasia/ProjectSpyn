% hehehehehe time to break all the things :D
set(0, 'RecursionLimit', 1000)

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
% path: vector of Destinations to be followed, path(1) processed first
% map: a matrix of MapNodes to store info about the maze
% destinationTask: task to perform once size(path)(2) == 0, can be "map" or
%   "finish"
% 
% 
% 
% 
% 
% 
% 
 
% <port information> 
% A: right drive motor, - is forwards, + is backwards
% B: left drive motor, - is forwards, + is backwards
% C: optics motor, <optics info>
% D: lift motor, behavior currently undefined
% 4: sonar sensor, behavior currently undefined
% 2: color sensor, behavior currently undefined
% 3: gyroscope, behavior currently undefined

% <behavior of system>
% 1. get key input
% 2. log sensor data
% 3. update data
% 4. perform mapping functions
% 5. determine motor functions
% 6. perform movements

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
    startMode = input("Enter mode to start in:\n1.Manual\n2.Auto\n>", "s");

    %set mode or display error message
    switch startMode
        case {"1", "Manual", "manual", "M", "m"}
            mode = "manual";
            break;
        case {"2", "Auto", "auto", "A", "a"}
            mode = "auto";
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

%sensor setup
brick.GyroCalibrate(3);
brick.SetColorMode(2, 2);

%keyboard setup
global key;
InitKeyboard();

%sensor variables
offsetX = -7;
offsetY = 9;
colorOffsetX = 6;
colorOffsetY = 16;
currentAngle = 0;
distance = 0;
detectedColor = 0;

%cycle variables
logTime = datetime('now');
deltaTime = 0;
stopCooldown = 0;
newDistance = 0;
deltaDistance = 0;
lastDistance = 0;
newAngle = 0;
deltaAngle = 0;
lastAngle = 0;

%state variables
throttle = 0;
turn = 0;
throttleTime = 6;
hasPassenger = false;

%navigation variables
x = 0;
y = 0;
facing = "xPos";
arrivedTolerance = 10;

%map variables
defaultNode = MapNode();
map = [defaultNode defaultNode defaultNode defaultNode defaultNode defaultNode defaultNode defaultNode defaultNode defaultNode defaultNode ; defaultNode defaultNode defaultNode defaultNode defaultNode defaultNode defaultNode defaultNode defaultNode defaultNode defaultNode ; defaultNode defaultNode defaultNode defaultNode defaultNode defaultNode defaultNode defaultNode defaultNode defaultNode defaultNode ; defaultNode defaultNode defaultNode defaultNode defaultNode defaultNode defaultNode defaultNode defaultNode defaultNode defaultNode ; defaultNode defaultNode defaultNode defaultNode defaultNode defaultNode defaultNode defaultNode defaultNode defaultNode defaultNode ; defaultNode defaultNode defaultNode defaultNode defaultNode defaultNode defaultNode defaultNode defaultNode defaultNode defaultNode ; defaultNode defaultNode defaultNode defaultNode defaultNode defaultNode defaultNode defaultNode defaultNode defaultNode defaultNode ; defaultNode defaultNode defaultNode defaultNode defaultNode defaultNode defaultNode defaultNode defaultNode defaultNode defaultNode ; defaultNode defaultNode defaultNode defaultNode defaultNode defaultNode defaultNode defaultNode defaultNode defaultNode defaultNode ; defaultNode defaultNode defaultNode defaultNode defaultNode defaultNode defaultNode defaultNode defaultNode defaultNode defaultNode ; defaultNode defaultNode defaultNode defaultNode defaultNode defaultNode defaultNode defaultNode defaultNode defaultNode defaultNode];
path = [Destination(7, 6)];
destinationTask = "map";

%input info & warnings
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
    newDistance = brick.UltrasonicDist(4);
    if(~isnan(newDistance))
        lastDistance = distance;
        distance = newDistance;
        deltaDistance = lastDistance - newDistance;
    end

    newAngle = brick.GyroAngle(3);
    if(~isnan(newAngle))
        lastAngle = currentAngle;
        currentAngle = newAngle;
        deltaAngle = currentAngle - lastAngle;
        currentAngle = mod(currentAngle, 360);
        if(currentAngle < 45)
            facing = "xPos";
        elseif(currentAngle < 135)
            facing = "yPos";
        elseif(currentAngle < 225)
            facing = "xNeg";
        elseif(currentAngle < 315)
            facing = "yNeg";
        else
            facing = "xPos";
        end
    end

    detectedColor = brick.ColorCode(2);

    % 2.5. display debug info for sensors
    disp("Color: " + detectedColor);
    disp("Angle: " + currentAngle);
    disp("Delta time: " + deltaTime);
    disp("Distance: " + distance);
    disp("Position: (" + x + ", " + y + ")");
    disp("Throttle: " + throttle);
    disp("Turn: " + turn);
    disp(" ");

    % 3. reset deltatime and decrement stopCooldown if needed
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
                    hasPassenger = true;
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
    elseif(mode == "auto")
        %color red is detected, stop for 4 seconds
        if(detectedColor == 5 && stopCooldown <= 0)
            stopCooldown = 10;
            brick.StopMotor('A');
            brick.StopMotor('B');
            pause(4);
        end
        %color blue is detected, swap to manual
        if(detectedColor == 2 && ~hasPassenger)
            mode = "manual";
            throttle = 0;
            turn = 0;
            break;
        end

        %check if vehicle has arrived at the next destination
        deltaX = path(1).x - x;
        deltaY = path(1).y - y;
        arrived = abs(deltaX) < arrivedTolerance && abs(deltaY) < arrivedTolerance;
        if(arrived)
            %pop destination
            path(1) = [];

            %check if path is empty
            pathSize = size(path);
            empty = pathSize(2) == 0;
            if(empty)
                %do task
                throttle = 0;
                turn = 0;
                brick.StopMotor('A');
                brick.StopMotor('B');
                switch(destinationTask)
                    case "map"
                        %get node location
                        nodeRow = round(y/MapNode.scaleFactor) + 6;
                        nodeColumn = round(x/MapNode.scaleFactor) + 6;

                        %get node data
                        front = distance;
                        brick.MoveMotorAngleRel('C', -10, 90, 'Brake');
                        pause(2);
                        left = brick.UltrasonicDist(4);
                        brick.MoveMotorAngleRel('C', 10, 180, 'Brake');
                        pause(2);
                        right = brick.UltrasonicDist(4);
                        brick.MoveMotorAngleRel('C', -10, 90, 'Brake');
                        pause(2);

                        %record node data
                        map(nodeRow, nodeColumn) = map(nodeRow, nodeColumn).mapThisNode(detectedColor, left, front, right, facing);

                        %generate new path & task
                        startingMapNode = map(nodeRow, nodeColumn);
                        startingNode = Node(nodeRow, NodeColumn, [], startingMapNode, [], true, facing);

                        %check for special color case
                        if(hasPassenger && startingMapNode.isFinish)
                            destinationTask = "finish";
                        else
                            %check if the finish is located and a passenger
                            %is onboard

                        end
                    case "finish"
                        %lower lift
                        lift = toggleLiftState(brick, lift);
                        %reverse for 2 seconds
                        brick.MoveMotor('A', 50);
                        brick.MoveMotor('B', 50);
                        pause(2);
                        brick.StopMotor('A');
                        brick.StopMotor('B');
                end
            end
        else
            %calculate turn ammount
            turn = path(1).turnTo(x, y, currentAngle);

            %if no turn, forwards
            if(turn == 0)
                throttle = 50;
            end
        end
    else
        disp("Unknown mode <" + mode + ">, aborting.")
        break;
    end

    % 6. calculate motor functions
    rightThrottle = getRightPower(throttle, turn);
    leftThrottle = getLeftPower(throttle, turn);

    % 7. perform motor functions
    runMotors(brick, rightThrottle, leftThrottle);

    % 8. pause to increase deltaTime for sensor relief
    if(deltaTime < .4)
        pause(.4 - deltaTime)
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
