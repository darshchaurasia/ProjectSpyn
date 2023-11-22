% Author: Darsh Chaurasia
% Updated: 12-1-22
% Sad version of the maze solving AI. Features high rate user input
% polling, wall detection, and color detection. Capable of solving most
% mazes, however it will struggle with managing some isolated sections. The
% AI from version 2 is capable of inteligently solving any given maze, and
% this capablility was demonstrated with its accurate mapping of the maze
% it was placed in for a limited time. The pathfinding algorithm also seems
% to have been working correctly.
% However the imprecision of the lego sensors prevents accurate tracking of
% the position of the robot, and it suffers from compounding drift. I
% implemented systems to attempt to correct for this drift, however they
% are insufficient to counter the flutuations in distance as reported by
% the sensor (up to +/- 70 cm observed).
% Strangely the gyroscope, which was reported by others to have accuracy
% issues behaved normally when installed in my current vehicle design. I
% believe this may be due to the centeralized location of the gyroscope,
% which is mounted in the lower chassis above the lift motor. Being mounted
% centerally, it is never rotated and transformed at the same time, which
% might be the reason that it does not suffer from severe drift.
% For this reason I created this simple AI in around half an hour, which is
% sufficient for the majority of maze cases, but is not nearly as cool. :(

% <port information> 
% A: right drive motor, - is forwards, + is backwards
% B: left drive motor, - is forwards, + is backwards
% C: optics motor, - is left, + is right, 7:3 gear ratio
% D: lift motor, - raises lift, + lowers lift
% 4: sonar sensor, gives distance from a point on the robot
% 2: color sensor, give the color detected under the front right track
%   guard
% 3: gyroscope, gives the angle relitive to the initial direction the robot
%   was facing when started

% <initial variables>

brickName = "Tanker";       %name of the brick to connect to
stopDistanceCM = 30;        %distance from the wall to stop at
minPower = 25;              %lowest motor power the AI will use
maxPower = 45;              %maximum power the robot will apply to the drive motor
sensorTimeSec = .35;         %length of time, in seconds, that must elapse to read from the sensors
cyclePauseSec = .05;        %extra delay, in seconds, to wait at the end of a cycle
setModeCooldownSec = 1;     %length of time, in seconds, that must elapse before the robot can switch between manaual and auto control modes
setStopCooldownSec = 10;    %length of time, in seconds, that must elapse before the robot stops at a red marker
angleTolerance = 5;         %angle, in degrees, the robot can be off of its target angle and still begin forward movement
squareSizeCM = 60.9;        %length, in cm, of the side of one grid in the maze
throttleTimeSec = 3;        %time, in seconds, it takes for the throttle to go from 0 to max
debug = false;

% <setup>

% user parameters
brickConnected = getInput("Is the brick connected?", "Enter yes or no", {"Yes", "yes"}, "yes", {"No", "no"}, "no");
if(brickConnected == "no")
    brick = ConnectBrick(brickName);
    %test tone
    brick.playTone(1000,10,500);
end
mode = getInput("Enter mode to start in", "Unknown mode", {"A", "a", "Auto", "auto"}, "auto", {"M", "m", "Manual", "manual"}, "manual");
liftState = getInput("Enter the current state of the lift", "Unknown state", {"Up", "up"}, "up", {"Down", "down"}, "down");

%keyboard setup
global key;
InitKeyboard();

%sensor setup
brick.GyroCalibrate(3);     %reset zero on gyroscope
brick.SetColorMode(2, 2);   %set to color reading mode

%user instruction
disp("WSAD or arrows to move.");
disp("Space to stop");
disp("Q to quit")
disp("E to switch between manual and auto control");
disp("L to actuate the lift, must be correctly configured beforehand");

% <variable setup>

hasPassenger = false;       %if the robot is carrying a passenger, assumed based on if the lift was raised while in manual mode
distance = 999;             %distance, in cm, read from the ultrasonic sensor
angle = 0;                  %angle, in degrees, the robot is currently facing, referenced of the direction the robot was facing at startup
targetAngle = 0;            %angle, in degrees, for the robot to attempt to face
throttle = 0;               %value to determine if the robot should move forward or backward
turn = 0;                   %offsets the power of the left and right drive motors to turn the robot
modeCooldownSec = 0;        %countdown, in seconds, until the robot can change control modes again
stopCooldownSec = 0;        %countdown, in seconds, until the robot can stop again
logTime = datetime('now');  %time recorded at the start of the current cycle
runningTime = 0;            %time accumulated since the last sensor cycle
detectedColor = 0;          %color detected by the color sensor

% <execution loop>

while(true)
    %priority key commands
    switch key
        case 'q'
            break;
        case 'e'
            if(modeCooldownSec == 0)
                modeCooldownSec = setModeCooldownSec;
                if(mode == "manual")
                    mode = "auto";
                else
                    mode = "manual";
                end
            end
    end

    %update time
    deltaTime = seconds(datetime('now') - logTime);
    logTime = datetime('now');
    runningTime = runningTime + deltaTime;
    if(stopCooldownSec > 0)
        stopCooldownSec = max(stopCooldownSec - deltaTime, 0);
    end
    if(modeCooldownSec > 0)
        modeCooldownSec = max(modeCooldownSec - deltaTime, 0);
    end
    if(deltaTime > 1)
        disp("Warning, delta time took longer than expected (" + deltaTime + " sec)");
    end

    %get sensor data (if applicable)
    if(runningTime > sensorTimeSec)
        runningTime = mod(runningTime, sensorTimeSec);
        newDistance = brick.UltrasonicDist(4);
        if(~isnan(newDistance))
            distance = newDistance;
        end
        newAngle = brick.GyroAngle(3);
        if(~isnan(newAngle))
            angle = mod(newAngle, 360);
        end
        detectedColor = brick.ColorCode(2);
    end

    %debug 
    if(debug)
        disp("Target Angle: " + targetAngle);
        disp("Current Angle: " + angle);
        disp("Distance: " + distance);
        disp("Delta Time: " + deltaTime);
        disp(" ");
    end

    %robot control
    switch(mode)
        case "manual"
            switch key
                case {'w', 'uparrow'}
                    throttle = min(maxPower, throttle + (deltaTime / throttleTimeSec * 100));
                case {'s', 'downarrow'}
                    throttle = max(-maxPower, throttle - (deltaTime / throttleTimeSec * 100));
                case {'a', 'leftarrow'}
                    turn = max(-maxPower, turn - (deltaTime / throttleTimeSec * 100));
                case {'d', 'rightarrow'}
                    turn = min(maxPower, turn + (deltaTime / throttleTimeSec * 100));
                case 'space'
                    throttle = 0;
                    turn = 0;
                case 'l'
                    liftState = toggleLiftState(brick, liftState);
                    hasPassenger = true;
            end
        case "auto"
            turn = turnTo(angle, targetAngle, angleTolerance, minPower, maxPower);
            if(turn ~= 0)
                throttle = 0;
            else
                %color red is detected, stop for 4 seconds
                if(detectedColor == 5 && stopCooldownSec <= 0)
                    disp("Stop marker detected, stopping");
                    stopCooldownSec = setStopCooldownSec;
                    brick.StopMotor('A');
                    brick.StopMotor('B');
                    pause(4);

                %color blue is detected, swap to manual
                elseif(detectedColor == 2 && ~hasPassenger)
                    disp("Pickup zone detected, waiting for manual pickup");
                    mode = "manual";
                    throttle = 0;
                    turn = 0;

                %color yellow is detected, reverse for 6 seconds
                elseif(detectedColor == 4 && hasPassenger)
                    disp("Dropoff zone detected, passenger disembarking");
                    brick.StopMotor('A');
                    brick.StopMotor('B');
                    liftState = toggleLiftState(brick, liftState);
                    brick.MoveMotor('A', maxPower);
                    brick.MoveMotor('B', maxPower);
                    pause(6);
                    mode = "manual";
                    throttle = 0;
                    turn = 0;
                
                %no color detected, wall detected
                elseif(distance <= stopDistanceCM)
                    %stop
                    brick.StopMotor('A');
                    brick.StopMotor('B');
                    throttle = 0;
                    
                    %check if there is a wall on the left
                    brick.MoveMotorAngleRel('C', -20, 210, 'Brake');
                    pause(2);
                    left = brick.UltrasonicDist(4);

                    %check if there is a wall on the right
                    brick.MoveMotorAngleRel('C', 20, 420, 'Brake');
                    pause(3);
                    right = brick.UltrasonicDist(4);

                    %reset sensor direction
                    brick.MoveMotorAngleRel('C', -20, 210, 'Brake');
                    pause(2);

                    %determine which way to go
                    if(left >= squareSizeCM)
                        targetAngle = targetAngle - 90;
                    elseif(right >= squareSizeCM)
                        targetAngle = targetAngle + 90;
                    else
                        targetAngle = targetAngle + 180;
                    end

                    %restrict to domain [0, 360)
                    targetAngle = mod(targetAngle, 360);

                %no wall detected, move forward
                else
                    throttle = maxPower;
                end
            end
    end

    %Calculate motor values and run motors
    rightThrottle = getRightPower(throttle, turn);
    leftThrottle = getLeftPower(throttle, turn);
    runMotors(brick, rightThrottle, leftThrottle);

    %pause
    pause(cyclePauseSec);
end

% <shutdown>

brick.StopAllMotors();
CloseKeyboard();

% <methods>

%get the user to select one of two options based on a prompt
function value = getInput(prompt, error, inOne, outOne, inTwo, outTwo)
    while(true)
        rawInput = input(prompt + "\n>", "S");
        switch rawInput
            case inOne
                value = outOne;
                break;
            case inTwo
                value = outTwo;
                break;
            otherwise
                disp(error);
        end
    end
end

%toggle the state of the lift
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

    %allow 4 seconds for process to complete
    pause(4);
end

%get value to pass to right motor
function r = getRightPower(throttle, turn)
    %calculate power, and adjust if value would be over 100
    r = throttle - turn;
    if(abs(throttle) + abs(turn) > 100)% || abs(throttle - turn) > 100)
        r = r / 2;
    end

    %adjust for reverse motor positioning
    r = -r;
end

%get value to pass to left motor
function r = getLeftPower(throttle, turn)
    %calculate power, and adjust if value would be over 100
    r = throttle + turn;
    if(abs(throttle) + abs(turn) > 100)% || abs(throttle - turn) > 100)
        r = r / 2;
    end

    %adjust for reverse motor positioning
    r = -r;
end

%run right and left motors for the given brick
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

function value = turnTo(angle, targetAngle, tolerance, minPower, maxPower)
    %get way to turn
    deltaAngle1 = targetAngle - angle;
    if(deltaAngle1 <= 0)
        deltaAngle2 = deltaAngle1 + 360;
    else
        deltaAngle2 = deltaAngle1 - 360;
    end
    if(abs(deltaAngle1) < abs(deltaAngle2))
        deltaAngle = deltaAngle1;
    else
        deltaAngle = deltaAngle2;
    end

    %if the angle is withing parameters, return 0, else return turn
    %value to traverse over the smaller angle
    if(abs(deltaAngle) < tolerance)
        value = 0;
    else
        drivePower = min(maxPower, abs(deltaAngle) / tolerance * minPower);
        if(deltaAngle > 0)
            value = drivePower;
        else
            value = -drivePower;
        end
    end
end
