%Connect Brick
Brick = ConnectBrick('TANKER');

%Play Test Frequency
Brick.playTone(1000, 10, 1000);

%get keyboard
global key;
InitKeyboard();

direction = 0;
turn = 0;

%Motor Movement Loop
while(true)
    pause(.1);
    

    %get key input
    switch key
        case 'uparrow'
            direction = 1;
        case 'downarrow'
            direction = -1;
        case 'rightarrow'
            turn = 1;
        case 'leftarrow'
            turn = -1;
        case 's'
            direction = 0;
            turn = 0;
        case 'q'
            break;
    end

    %action control
    if(direction == 0)
        if(turn == 0)
            Brick.StopAllMotors();
        else
            Brick.MoveMotor('A', turn * 50);
            Brick.MoveMotor('B', turn * -50);
        end
    else
        Brick.MoveMotor('A', (direction * -50) + (turn * 25));
        Brick.MoveMotor('B', (direction * -50) + (turn * 25));
    end
end
CloseKeyboard();

Brick.StopAllMotors();