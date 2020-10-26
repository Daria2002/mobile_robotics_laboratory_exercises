function [path] = Astar(map, start, goal, inflation)
inflate(map, inflation);
map = occupancyMatrix(map);
neighbours = [-1 -1; -1 0; -1 1; 0 -1; 0 1; 1 -1; 1 0; 1 1];
path = [];
MapSize = size(map);
start_node.id = 1;
start_node.f = 0;
start_node.g = 0;
start_node.h = 0;
start_node.pose = start;
start_node.parent = 0;
OpenList = [start_node];
ClosedList = [];
id = 1;
while ~isempty(OpenList)   
    OpenList = nestedSortStruct(OpenList, 'f');
    currentNode = OpenList(1);
    currentNode.pose;
    ClosedList = [ClosedList; currentNode];
    OpenList(1) = [];
    if isequal(currentNode.pose, goal)
        while(currentNode.parent)
            path = [path;currentNode.pose];
            for i = 1:length(ClosedList)
                if ClosedList(i).id == currentNode.parent
                    currentNode = ClosedList(i);
                end
            end
        end
        path = fliplr(path')';
        return
    end
            
    for i = 1:size(neighbours)
        neighbour = neighbours(i,:);
        location = currentNode.pose+neighbour;
        if any(location < 1) || any(location>MapSize)
            continue
        end
        if map(location(1), location(2))
            continue
        end
        in = 0;
        for j = 1:length(ClosedList)
            if isequal(ClosedList(j).pose, location)
                in = 1;
            end
        end
        
        if in
            continue
        end
        
        child.g = currentNode.g+sqrt(sum(neighbour.^2));
        child.h = sqrt(sum((location-goal).^2));
        child.f = child.g+child.h;
            
        in = 0;
        for j = 1:length(OpenList)
            if isequal(OpenList(j).pose, location)
                if child.g >= OpenList(j).g
                    in = 1;
                end
            end
        end
        
        if in
            continue
        end
        
        id = id+1;
        child.id = id;
        child.parent = currentNode.id;
        child.pose = location;
        OpenList = [OpenList; child];
    end   
end
disp('No path found')
path = [];
end

