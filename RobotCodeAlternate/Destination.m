classdef Destination
    properties (Constant)
        angleTolerance = 5
    end
    properties
        x
        y
        column
        row
        isNull
    end
    methods (Static)
        %null object constructor
        function obj = NullDestination()
            obj = Destination(-1, -1);
            obj.isNull = true;
        end
    end
    methods
        %Constructor
        function obj = Destination(column, row)
            obj.column = column;
            obj.row = row;
            obj.x = (column - 6) * MapNode.scaleFactor;
            obj.y = (row - 6) * MapNode.scaleFactor;
            obj.isNull = false;
        end

        %gets the turn required to face a given destination
        function r = turnTo(dest, x, y, angle)
            %get angle to face
            targetAngle = 180 / pi * atan((dest.y - y) / (dest.x - x));
            if(dest.x - x < 0)
                targetAngle = targetAngle + 180;
            end
            targetAngle = mod(targetAngle, 360);
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
            if(abs(deltaAngle) < Destination.angleTolerance)
                r = 0;
            else
                drivePower = min(50, abs(deltaAngle) * 4);
                if(deltaAngle > 0)
                    r = drivePower;
                else
                    r = -drivePower;
                end
            end
        end
    end
end