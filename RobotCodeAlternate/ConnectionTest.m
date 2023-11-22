%Connect Brick
Brick = ConnectBrick('TANKER');

%Play Test Frequency
Brick.playTone(1000, 10, 1000);

%Motor Rotation Test
while(true)
    %get rotation
    run = input("Enter rotation time, sign for direction, 0 to stop: ");

    %check to quit
    if(run == 0)
        break
    end

    %get direction
    direction = 0;
    if(run < 0)
        direction = -1;
    else
        direction = 1;
    end

    %rotate
    Brick.MoveMotor('A', direction * 50);
    Brick.MoveMotor('B', direction * 50);
    pause(abs(run));
    Brick.StopMotor('A');
    Brick.StopMotor('B');
end
