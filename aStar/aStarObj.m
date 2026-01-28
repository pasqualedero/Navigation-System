classdef aStarObj < handle
    %ASTAROBJ Implementation of A* algorithm

    properties(Access=public)
        map {logical}
        startNode (1,2) {mustBeNonempty}
        goalNode (1,2) {mustBeNonempty}
        useHeuristic {mustBeMember(useHeuristic,["Euclidean","Manhattan"])} = "Manhattan"
    end

    properties(Dependent=true)
        path
    end

    methods
        function obj = aStarObj(map,startNode, goalNode, useHeuristic)
            obj.map = map;
            obj.startNode = startNode;
            obj.goalNode = goalNode;
            if nargin == 4
                obj.useHeuristic = useHeuristic;
            end
        end


        function value = get.path(obj)
            value = obj.findPath;
        end

    end

    methods(Access=private)
        function path = findPath(obj)
            start_node = obj.startNode;
            goal_node = obj.goalNode;

            [rows, cols] = size(obj.map);

            gScore = inf(rows, cols);
            fScore = inf(rows, cols);

            parent_prev = cell(rows, cols);

            gScore(start_node(1), start_node(2)) = 0;
            fScore(start_node(1), start_node(2)) = obj.heuristic(start_node, goal_node);

            openList = [fScore(start_node(1), start_node(2)), start_node(1), start_node(2)];

            closedMap = false(rows, cols);

            neighbors = [-1 0; 1 0; 0 -1; 0 1; 1 1; 1 -1; -1 -1; -1 1];

            while ~isempty(openList)
                % Sort by f-score (first column) and pick the first
                [~, min_idx] = min(openList(:, 1));
                current = openList(min_idx, 2:3); % take point coord.

                % Remove current from openList
                openList(min_idx, :) = [];

                % Check Goal 
                if isequal(current, goal_node)
                    path = obj.reconstruct_path(parent_prev, current, start_node);
                    return;
                end

                % Add to Closed List 
                closedMap(current(1), current(2)) = true;

                % Check Neighbors 
                for i = 1:size(neighbors, 1)
                    diagonalStep = (i>4);
                    neighbor_pos = current + neighbors(i, :);
                    r = neighbor_pos(1);
                    c = neighbor_pos(2);

                    % Check Bounds and Obstacles
                    if r < 1 || r > rows || c < 1 || c > cols || obj.map(r, c) == 1
                        continue;
                    end

                    % Skip if already in closed list (Efficiency fix)
                    if closedMap(r, c)
                        continue;
                    end

                    % Calculate tentative g
                    % Distance is 1 for grid movement
                    increment = 1;
                    if diagonalStep
                        increment = sqrt(2);
                    end
                    tentative_g = gScore(current(1), current(2)) + increment;

                    if tentative_g < gScore(r, c)
                        % This path is better! Record it.
                        parent_prev{r,c} = current;
                        gScore(r, c) = tentative_g;
                        fScore(r, c) = tentative_g + obj.heuristic([r, c], goal_node);

                        % Add to openList if not already there
                        inOpen = false;
                        if ~isempty(openList)
                            inOpen = any(openList(:,2) == r & openList(:,3) == c);
                        end

                        if ~inOpen
                            openList = [openList; fScore(r, c), r, c];          
                        end
                    end
                end
            end

            path = []; % Failure case
        end

    end


    methods(Access=private)
        
        function h = heuristic(obj, pos, goal)
            switch obj.useHeuristic
                case "Manhattan"
                    h = abs(pos(1) - goal(1)) + abs(pos(2) - goal(2));
                case "Euclidean"
                    h = norm(pos - goal);
            end
        end

        function path = reconstruct_path(obj, parent_map, goal, start)
            current = goal;
            path = current;
            while ~isequal(current,start)
                current = parent_map{current(1),current(2)};
                path = [path; current];
            end
            disp("Strategy: "+obj.useHeuristic);
        end

    end

end