classdef MapNode
    properties (Constant)
        scaleFactor = 60.9
    end
    properties
        visited
        isFinish
        xPosWall
        xNegWall
        yPosWall
        yNegWall
    end
    methods
        %constructor
        function obj = MapNode()
            obj.visited = false;
            obj.isFinish = false;
            obj.xPosWall = -1;
            obj.yPosWall = -1;
            obj.xNegWall = -1;
            obj.yNegWall = -1;
        end

        %map based on sensor data
        function obj = mapThisNode(mapNode, color, left, front, right, facing)
            obj = mapNode;
            obj.visited = true;
            if(color == 4) %yellow
                obj.isFinish = true;
            end
            switch(facing)
                case "xPos"
                    obj.xNegWall = 0;
                    if(left < MapNode.scaleFactor)
                        obj.yNegWall = 1;
                    else
                        obj.yNegWall = 0;
                    end
                    if(front < MapNode.scaleFactor)
                        obj.xPosWall = 1;
                    else
                        obj.xPosWall = 0;
                    end
                    if(right < MapNode.scaleFactor)
                        obj.yPosWall = 1;
                    else
                        obj.yPosWall = 0;
                    end
                case "xNeg"
                    obj.xPosWall = 0;
                    if(left < MapNode.scaleFactor)
                        obj.yPosWall = 1;
                    else
                        obj.yPosWall = 0;
                    end
                    if(front < MapNode.scaleFactor)
                        obj.xNegWall = 1;
                    else
                        obj.xNegWall = 0;
                    end
                    if(right < MapNode.scaleFactor)
                        obj.yNegWall = 1;
                    else
                        obj.yNegWall = 0;
                    end
                case "yPos"
                    obj.yNegWall = 0;
                    if(left < MapNode.scaleFactor)
                        obj.xPosWall = 1;
                    else
                        obj.xPosWall = 0;
                    end
                    if(front < MapNode.scaleFactor)
                        obj.yPosWall = 1;
                    else
                        obj.yPosWall = 0;
                    end
                    if(right < MapNode.scaleFactor)
                        obj.xNegWall = 1;
                    else
                        obj.xNegWall = 0;
                    end
                case "yNeg"
                    obj.yPosWall = 0;
                    if(left < MapNode.scaleFactor)
                        obj.xNegWall = 1;
                    else
                        obj.xNegWall = 0;
                    end
                    if(front < MapNode.scaleFactor)
                        obj.yNegWall = 1;
                    else
                        obj.yNegWall = 0;
                    end
                    if(right < MapNode.scaleFactor)
                        obj.xPosWall = 1;
                    else
                        obj.xPosWall = 0;
                    end
            end
        end
    end
end