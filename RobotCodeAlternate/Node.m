classdef Node
    properties
        row
        column
        parent
        cost
        mapNode
        isParent    %node can spread in 4 directions instead of 3, allows for backing out of a dead end
    end
    methods
        %constructor
        function obj = Node(row, column, parent, mapNode, cost, isParent, facing)
            if(~isParent)
                obj.row = row;
                obj.column = column;
                obj.parent = parent;
                obj.cost = parent.cost + cost;
                obj.mapNode = mapNode;
                obj.isParent = false;
            else
                obj.row = row;
                obj.column = column;
                switch(facing)
                    case "xPos"
                        obj.parent = Node(row, column - 1, [], [], [], true, "");
                    case "yPos"
                        obj.parent = Node(row - 1, column, [], [], [], true, "");
                    case "xNeg"
                        obj.parent = Node(row, column + 1, [], [], [], true, "");
                    case "yNeg"
                        obj.parent = Node(row + 1, column, [], [], [], true, "");
                    otherwise
                        obj.parent = [];
                end
                obj.cost = 0;
                obj.mapNode = mapNode;
                obj.isParent = true;
            end
        end

        %generates the vector of children that can come from a given node
        function children = getChildren(node, map)
            children = [];
            if(node.parent.row == node.row)
                if(node.parent.column > node.column)
                    facing = "xNeg";
                else
                    facing = "xPos";
                end
            else
                if(node.parent.row > node.row)
                    facing = "yNeg";
                else
                    facing = "yPos";
                end
            end

            %children generation
            if(node.mapNode.xPosWall == 0)
                switch facing
                    case "xPos"
                        child = Node(node.row, node.column + 1, node, map(node.row, node.column + 1), 1, false, "");
                        children = [children child];
                    case "yPos"
                        child = Node(node.row, node.column + 1, node, map(node.row, node.column + 1), 2, false, "");
                        children = [children child];
                    case "xNeg"
                        if(node.isParent)
                            child = Node(node.row, node.column + 1, node, map(node.row, node.column + 1), 3, false, "");
                            children = [children child];
                        end
                    case "yNeg"
                        child = Node(node.row, node.column + 1, node, map(node.row, node.column + 1), 2, false, "");
                        children = [children child];
                end
            end
            if(node.mapNode.yPosWall == 0)
                switch facing
                    case "xPos"
                        child = Node(node.row + 1, node.column, node, map(node.row + 1, node.column), 2, false, "");
                        children = [children child];
                    case "yPos"
                        child = Node(node.row + 1, node.column, node, map(node.row + 1, node.column), 1, false, "");
                        children = [children child];
                    case "xNeg"
                        child = Node(node.row + 1, node.column, node, map(node.row + 1, node.column), 2, false, "");
                        children = [children child];
                    case "yNeg"
                        if(node.isParent)
                            child = Node(node.row + 1, node.column, node, map(node.row + 1, node.column), 3, false, "");
                            children = [children child];
                        end
                end
            end
            if(node.mapNode.xNegWall == 0)
                switch facing
                    case "xPos"
                        if(node.isParent)
                            child = Node(node.row, node.column - 1, node, map(node.row, node.column - 1), 3, false, "");
                            children = [children child];
                        end
                    case "yPos"
                        child = Node(node.row, node.column - 1, node, map(node.row, node.column - 1), 2, false, "");
                        children = [children child];
                    case "xNeg"
                        child = Node(node.row, node.column - 1, node, map(node.row, node.column - 1), 1, false, "");
                        children = [children child];
                    case "yNeg"
                        child = Node(node.row, node.column - 1, node, map(node.row, node.column - 1), 2, false, "");
                        children = [children child];
                end
            end
            if(node.mapNode.yNegWall == 0)
                switch facing
                    case "xPos"
                        child = Node(node.row - 1, node.column, node, map(node.row - 1, node.column), 2, false, "");
                        children = [children child];
                    case "yPos"
                        if(node.isParent)
                            child = Node(node.row - 1, node.column, node, map(node.row - 1, node.column), 3, false, "");
                            children = [children child];
                        end
                    case "xNeg"
                        child = Node(node.row - 1, node.column, node, map(node.row - 1, node.column), 2, false, "");
                        children = [children child];
                    case "yNeg"
                        child = Node(node.row - 1, node.column, node, map(node.row - 1, node.column), 1, false, "");
                        children = [children child];
                end
            end
        end

        %convert a node to a destination or a null destination
        function dest = toDestination(node, null)
            if(null)
                dest = Destination.NullDestination();
            else
                dest = Destination(node.column, node.row);
            end
        end

        %get a path of destinations to an unvisited node from a given node
        function dest = getPath(node, map)
            %check if node is unvisited
            if(node.mapNode.visited == false)
                dest = node.toDestination(false);
            else
                children = node.getChildren(map);
                childSize = size(children);
                path = [];
                while(childSize(2) ~= 0)
                    %get best child, pop off vector
                    child = children(1);
                    bestIndex = 1;
                    childIndex  = 2;
                    while(childIndex < childSize(2))
                        if(children(childIndex).cost < child.cost)
                            bestIndex = childIndex;
                            child = children(childIndex);
                        end
                        childIndex = childIndex + 1;
                    end
                    children(bestIndex) = [];

                    %generate path of that child
                    path = child.getPath(map);

                    %check the path of the child, if it is good, break
                    if(path(1).isNull == false)
                        break;
                    end

                    %path of selected child was invalid, reset path and try
                    %again
                    path = [];
                    childSize = size(children);
                end
                pathSize = size(path);
                if(pathSize(2) == 0)
                    dest = node.toDestination(true);
                else
                    dest = path;
                end
            end
        end
    end
end