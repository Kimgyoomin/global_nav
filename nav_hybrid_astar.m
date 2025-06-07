% nav_hybrid_astar.m
% Hybrid A* algorithm
% Robot's Kinematic characteristic will be in this algo

function path = nav_hybrid_astar(map3D, robot, robotConstraints, ...
                                startPose, goalPose, xLimits, yLimits)
    global INFLATED2D;  % inflated 2D map set in main.m
    
% nav_hybrid_astar : 4 legged robot will navigate through given 3D map
%
% %%%%%%%%%%%%%%%%%%%%%%%%%%% Input %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% map3D             - occupancyMap3D (obj)
% robot             - robotics.RigidBodyTree object (robot model)
% robotConstraints  - struct which has robot's Contraints
% (nav_robot_model.m)
% startPose         - Robot Start Pose [x, y, z, r, p, y] (m, rad)
% goalPose          - Robot Goal Pose  [x, y, z, r, p, y] (m, rad)
%
% %%%%%%%%%%%%%%%%%%%%%%%%%%% Output %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% path              - Navigated Path (Optimal Path)
    fprintf('Hybrid A star Path Planning Start');

    % 1. Hybrid A star Parameter Setting
    % This value would be adjusted 
    parameters.gridResolution   = map3D.Resolution;
    parameters.numControlInputs = 3;        % ex) [lin_vel, ang_vel(yaw),
    % p/r ]
    parameters.stepSize         = 0.2;      % Dist per step [m]
    parameters.maxIterations    = 50000;     % Max Searching steps
    parameters.tolerance        = 0.1;      % [m] / in tol, it will be goal

    % Cost function params
    parameters.costWeight.distance      = 1.0;
    parameters.costWeight.orientation   = 2.0;
    parameters.costWeight.collision     = 1e+4; % If collision ; die

    % -- Define Robot's 2D Motion Primitives
    % Each Motion will be define as [delta_x, delta_y, delta_yaw,
    % costmultiplier] shape
    % r, p, z would be used for path length ( g_cost ), collision checking
    % Motion Primitives will used for X-Y coordinate motion and Yaw

    % Yaw rate
    angularStep = deg2rad(5);  % Angle per step (changable)

    motionPrimitives = {
        % [delta_X, delta_Y, delta_Yaw, cost_multiplier]

        % A. Move Forward
        [parameters.stepSize, 0, 0, 1.0];   % 1.0

        % B. Turn Right
        [parameters.stepSize * cos(-angularStep/2), parameters.stepSize * ...
        sin(-angularStep/2), -angularStep, 1.0];    % 1.5

        % C. Turn Left
        [parameters.stepSize * cos(angularStep/2), parameters.stepSize * ...
        sin(angularStep/2), angularStep, 1.0];  % 1.5

        % D. Move Backward
        [-parameters.stepSize, 0, 0, 2.0];  % 2.0

        % E. In-place Turn Right (제자리 우회전)
        [0, 0, -angularStep, 1.0];  % 2.5

        % F. In-place Turn Left (제자리 좌회전)
        [0, 0, angularStep, 1.0];   % 2.5
    };



    % 2. Define state space
    % Hybrid A* node include  [x, y, z, r, p, y]
    % Iteration per dimensions (different from grid resolution)
    parameters.stateDiscretization.xy      = 0.2; % Discrete between x, y
    parameters.stateDiscretization.z       = 0.2; % Discrete z
    parameters.stateDiscretization.yaw     = deg2rad(5); % Discrete yaw
    % Used for constraints for Robot

    % 3. Start and Goal Node Initialization
    % Real struct/class would be define and initialized in here
    % Node { x, y, z, roll, pitch, yaw, g_cost, h_cost, f_cost, parent_idx, 
    % motion_primitive_idx }
    nodeTemplate = struct(...
        'x', [], ...        % x-coordinate  [m]
        'y', [], ...        % y-coordinate  [m]
        'z', [], ...        % z-coordinate  [m]
        'roll', [], ...     % roll angle    [rad]
        'pitch', [], ...    % pitch angle   [rad]
        'yaw', [], ...      % yaw angle     [rad]
        'g_cost', inf, ...  % Cost from start node to current node
        'h_cost', inf, ...  % Estimated cost from current node to goal node
        'f_cost', inf, ...  % Total Estimated cost (g_cost + h_cost)
        'parent_idx', 0,  ... % Index of the parent node in the closed list
        'motion_primitive_idx', 0 ... % Index of motion primitive used to reach the goal
    );
    
    % Cretae Start Node
    startNode           = nodeTemplate;
    startNode.x         = startPose(1);
    startNode.y         = startPose(2);
    startNode.z         = startPose(3);
    startNode.roll      = startPose(4);
    startNode.pitch     = startPose(5);
    startNode.yaw       = startPose(6);
    startNode.g_cost    = 0;
    % h_cost will not calculate at the time when start (It will be
    % calculated in heuristic function
    % f_cost and h_cost will be same at the start node

    startNode.parent_idx=0; % at start node, it has no parent    

    % Goal Node information (it would not used in real navigation, but used
    % for goal checking and heuristic calculation)
    goalNode.x          = goalPose(1);
    goalNode.y          = goalPose(2);
    goalNode.z          = goalPose(3);
    goalNode.roll       = goalPose(4);
    goalNode.pitch      = goalPose(5);
    goalNode.yaw        = goalPose(6);


    

    % Open List and Closed List Initialized in here
    % Open List : Searching Nodes will be stored based on f_cost (우선순위 큐)
    % All Node will be managed with list, 탐색 시 정렬하여 가져올꺼임
    
    % Closed List : Store Node which searched finished, be aware of search
    % in duplicate

    % Closed List : Store all searched Nodes. It will be used to check
    % parent index of the Node

    closedList      = [startNode];
    closedListSize  = 1;


    % Open List : Store f_cost and index from closedList for searching
    % Nodes
    %                   [f_cost, closedList_index]
    % f_cost from startNode, didnt' calculates h_cost -> Not initialized
    % But in A*, put startNode in OpenList and get out and start for
    % searching
    % calculate h_cost of start Node
    startNode.h_cost = calculateHeuristic(startNode, goalNode, ...
        robotConstraints.minTurningRadius);
    startNode.f_cost = startNode.g_cost + startNode.h_cost;     % f_cost = g_cost + h_cost
    openList         = [startNode.f_cost, 1];   % [f_cost, closedList_index]


    % Closed List Hash Table : Use discreted Node pose as key, store
    % closedList index
    % change Node pose as string and use as key
    % formatSpec = '%.2f_%.2f_%.2f_%.2f_%.2f_%.2f'
    % At first, only X, Y, Yaw can used as discreted position

    % Function which Discreted Node pose as key (Later, it can be kicked
    % out as helper function)
    function key = getNodeKey(node)
        % Discretize Node pose and change as string
        x_disc      = round(node.x / parameters.stateDiscretization.xy);
        y_disc      = round(node.y / parameters.stateDiscretization.xy);
        z_disc      = round(node.z / parameters.stateDiscretization.z);
        yaw_disc    = round(node.yaw / parameters.stateDiscretization.yaw);
        % Roll & Pitch will be not discretized at initial point
        key = sprintf('%d_%d_%d_%d', x_disc, y_disc, z_disc, yaw_disc);
    end


    closedListMap   = containers.Map();
    % Add start Node to Closed List Map, Index is 1
    closedListMap(getNodeKey(startNode))    = 1;    



    % 4.Hybrid A* 알고리즘 pseudo-code ( 나중에 영어로 씁시다잉 )
    % --- Hybrid A star main search Loop ---
    path = [];      % path is final path ; At start, it will be empty
    fprintf('\n === Hybrid A star On going ===\n');

    for iter = 1:parameters.maxIterations
        if isempty(openList)
            fprintf('Open List is empty..... Cannot find optimal path \n');
            break;  % If Open List is empty, there's no searching node so close
        end

        % a. Select the lowest f_cost Node
        % openList structure would be [f_cost, closedList_index]
        % Find smallest f_cost and its index
        [minFcost, minIdxInOpenList] = min(openList(:, 1));
        
        % Node Index of the Closed List
        currentClosedListIndex = openList(minIdxInOpenList, 2);
        % Get real Node data
        currentNode = closedList(currentClosedListIndex);

        % b. Delete selected node from Open List and consider as add to
        % Closed List
        openList(minIdxInOpenList, :) = []; % Delete from open List

        % c. Check whether it arrived at goal point
        if isGoalReached(currentNode, goalNode, parameters.tolerance)
            fprintf('Goal Point Reached!!!!! Path re-constructing \n\n');
            % Call re-construct path function
            path = reconstructPath(currentNode, closedList);
            fprintf('Path re-constructed finished \n');
            break;  % It reached to goal so break / quit
        end

        % d. Node Expansion (make and call expandNode function)
        % In here, create next possible node from currentNode, Collision
        % check, cost calculate
        generatedNodes = expandNode(currentNode, currentClosedListIndex, ...
            map3D, robot, robotConstraints, parameters, goalNode, ...
            nodeTemplate, motionPrimitives);
        
        % == Open List and Closed List Update ==
        for i = 1:length(generatedNodes)
            newNode     = generatedNodes(i);
            newNodeKey  = getNodeKey(newNode);

            % Check if new node already in closed List map
            if closedListMap.isKey(newNodeKey)
                existingNodeIndex   = closedListMap(newNodeKey);
                existingNode        = closedList(existingNodeIndex);
        
                % Original Node's g_cost and new node g_cost compare
                if newNode.g_cost < existingNode.g_cost
                    % if u searched for better route, original node update
                    % Update node in closed List
                    closedList(existingNodeIndex) = newNode;
        
                    % find particular node in Open List and update f_cost (if
                    % existed in OpenList)
                    idxInOpenList = find(openList(:, 2) == existingNodeIndex);
                    if ~isempty(idxInOpenList)
                        % update f_cost
                        openList(idxInOpenList, 1) = newNode.f_cost;
                    else
                        % If u search better path, and move Node to Open List
                        % Hybrid A star algorithm
                        openList = [openList; newNode.f_cost, existingNodeIndex];
                    end
                end
            else
                % If new node doesnt exist in Closed List Map (New Node)
                closedListSize = closedListSize + 1;
                % add new node in closed List
                closedList(closedListSize) = newNode;
                % Add new key-index in Closed List Map
                closedListMap(newNodeKey) = closedListSize;
    
                % Add new Node in Open List
                openList = [openList; newNode.f_cost, closedListSize];
            end
        end
        
        if mod(iter, 100) == 0
            fprintf('Current iter : %d, Open List Size : %d, Current Node f_cost : %.2f\n', ...
+             iter, size(openList, 1), currentNode.f_cost);
        end
    end
    
    % Open List is not empty but cannot find optimal path
    if isempty(path) && size(openList, 1) > 0  
        fprintf('Max Iteration reached. U did not reached the goal\n');
    elseif isempty(path)
        fprintf('Search failed : Open List is empty so u cannot find a path\n');
    end


    %5. Path re-construct (if search succeed)
    fprintf('\n === Hybrid A star search finisehd === \n');
    fprintf('Map, Robot Model, start/Goal pose lets go~~~\n');

    % -- Helper Functions ---
% =========== Heuristic Function =========== %
% calculateHeuristic func : Calculate heristic cost from currentNode to
% goalNode
    function h = calculateHeuristic(currentNode, goalNode, minTurningRadius)
        % For Hybrid A star, Reeds-shepp or Dubins length will be used for
        % heuristic
        % In here, we'll use Reeds-shepp path for planning

        % start pose [x, y, yaw]
        startState  = [currentNode.x, currentNode.y, currentNode.yaw];

        % goal pose [x, y, yaw]
        goalState   = [goalNode.x, goalNode.y, goalNode.yaw];

        % Reeds-shepp Path objects create
        try
            % Use euclidean distance or use very big value 
            % warning('Reeds-Shepp Path calculation failed. Use Euclidean dist for heuristic func\n');
            h = norm([currentNode.x - goalNode.x, currentNode.y - goalNode.y]); % 2D Euclidean distance
        catch
            % If error occured (ex : if u cannot find map or cannot find
            % params)
            % IN, Robotics System Toolbox, there is ReedsSheppPath
            rsPath = robotics.ReedsSheppPath(startState, goalState, minTurningRadius);
            % use path length for heuristic function
            h = rsPath.length;

        end
        % axes z will move respectively or included in h_cost, so it is
        % good to use 2D in h_cost
        % However if u wanna add difference of z axes in h_cost:
        % h = h + abs(currentNode.z - gaolNode.z)
        
    end



    % isGoalReached func : Check whether the Robot reached the goal
    function reached = isGoalReached(currentNode, goalNode, tolerance)
        % 3D orientation , consider yaw angle and check whether Goal
        % reached
        dist = norm([currentNode.x - goalNode.x, currentNode.y - goalNode.y, ...
            currentNode.z - goalNode.z]);

        % U can check yaw diretion, but first lets just simplify by
        % checking only distance, if u need, check following
        % ang_diff = abs(angdiff(currentNode.yaw, goalNode.yaw));
        % ang_diff is in Robotics System Toolbox
        % reached = (dist < tolerance && ang_diff < deg2rad(10));
        % After checking dist, u can check by up line
        
        % But now, lets just check only position
        reached = (dist < tolerance);   % At initial check with just position
    end


    % reconstructPath function : From Closed List, re-construct path
    function p = reconstructPath(finalNode, closedList)
        p = [];
        currentNode = finalNode;

        % path re-construct (finalNode to startNode)
        while true
            % add current node's pose to path (N * 6 shape)
            p = [currentNode.x, currentNode.y, currentNode.z, ...
                currentNode.roll, currentNode.pitch, currentNode.yaw; p];

            % If u get to start node, loop end
            if currentNode.parent_idx == 0
                break;
            end

            % Move to next Node (parent Node)
            currentNode = closedList(currentNode.parent_idx);
        end
    end

    %     while currentNode.parent_idx ~= 0
    %         p = [struct2array(currentNode); p]; % add node data to array
    %         currentNode = closedList(currentNode.parent_idx);
    %     end
    %     % add start node to start point of path
    %     p = [struct2array(startNode); p];
    % 
    % end
    
% ================= checkCollision for 2D plane ========================
    function isColliding = checkCollision2D(x, y, xLimits, yLimits)
        % x, y : world coordinate [m]
        % INFLATED2D : (nRows x nCols) logic array (true = inflated
        % obstacle)
        % xLimits         = map3D.XWorldLimits;
        % yLimits         = map3D.YWorldLimits;
        res             = map3D.Resolution;

        j = round((x - xLimits(1)) * res) + 1;
        i = round((y - yLimits(1)) * res) + 1;
        if i < 1 || i > size(INFLATED2D, 1) || j < 1 || j > size(INFLATED2D, 2)
            isColliding = true;
        else
            isColliding = INFLATED2D(i, j);
        end
    end


    % --- checkCollision function ---
    % We'll check whether roboot collide with world map at current Pose
    % Just think about robot torso -> We'll make robot torso into simple
    % rectangular cubic
    function isColliding = checkCollision(robotPose, map3D, robotConstraints)
        % robotPose : robot pose which we'll check whether robot collides
        % to obstacle
        % map3D : objective of map3D
        % robotContraints : robot contraints (body radius, body
        % Dimensions...)

        isColliding = false;    % Basically, no collision

        % A. Robot Bounding box definition
        % From URDF File
        % <box size="0.3762 0.0935 0.114" />
        bodyLength  = 0.4;
        bodyWidth   = 0.1;
        bodyHeight  = 0.12;

        % Half of bounding box
        halfLength  = bodyLength / 2;
        halfWidth   = bodyWidth / 2;
        halfHeight  = bodyHeight / 2;

        % Change Robot Pose to World Coordinate
        poseX       = robotPose(1);
        poseY       = robotPose(2);
        poseZ       = robotPose(3);
        poseRoll    = robotPose(4);
        posePitch   = robotPose(5);
        poseYaw     = robotPose(6);

        % Rotation considering robot rotation
        % Consider base center as pose x-y-z
        % use MATLAB eul2rotm
        rotm        = eul2rotm([poseYaw, posePitch, poseRoll], 'ZYX');

        % Define vertex(8) of Bounding Box (from Robot Local Coordinate)
        cornersLocal = [
            halfLength, halfWidth, halfHeight;
            halfLength, halfWidth, -halfHeight;
            halfLength, -halfWidth, halfHeight;
            halfLength, -halfWidth, -halfHeight;
            -halfLength, halfWidth, halfHeight;
            -halfLength, halfWidth, -halfHeight;
            -halfLength, -halfWidth, halfHeight;
            -halfLength, -halfWidth, -halfHeight
            ]';

        % Convert each corners to world frame
        cornerWorld = rotm * cornersLocal + [poseX; poseY; poseZ];
        cornerWorld = cornerWorld';

        % B. Check for Collision
        % use map3D.checkOccupancy function and check whether corenrs are
        % in occupied space
        % If the probablistic is over 0.5 -> consider as occupied

        % Consider Torso as 'Set of Points' and check if this set occupying
        % map
        % Or check bounding box surrounding robot

        % Another simple way ; Sample points which covers bounding box
        % IN MATLAB -> using checkMapCollision -> but not in cpp

        % In this code, lets sample following points
        % Center of Bounding Box, Each corner, Centeroid Point
        % Check grid voxel which contains torso

        % Sample Inner points of bounding box (sample based on robot local
        % Orientation
        sampleResolution = map3D.Resolution;    % Sample with map Resolution
        if sampleResolution == 0
            sampleResolution = 0.1;     % Sample by 0.1m if Resolution null
        end

        [localSampleX, localSampleY, localSampleZ] = meshgrid( ...
            (-halfLength+sampleResolution/2):sampleResolution:(halfLength-sampleResolution/2),...
            (-halfWidth+sampleResolution/2):sampleResolution:(halfWidth-sampleResolution/2),...
            (-halfHeight+sampleResolution/2):sampleResolution:(halfHeight-sampleResolution/2));    
        
        % N * 3 Matrix
        localSamplePoints = [localSampleX(:), localSampleY(:), localSampleZ(:)];

        % if localSamplePoints is empty -> Error, too small robot
        if isempty(localSamplePoints)
            isColliding = false;
            return;
        end
        
        % Change local Sample points to world points with robot pose
        % rotm (3X3) * localSamplePoints' (3XN) = 3XN matrix 
        rotatedSamplePoints = rotm * localSamplePoints'; % 3XN Mat
        rotatedSamplePoints = rotatedSamplePoints' + [poseX, poseY, poseZ];
        % (NX3 + 1X3) Mat
        % Finally get NX3 shape world points

        % Get occupancy probability from map
        occupancyProb = map3D.getOccupancy(rotatedSamplePoints);

        % If there any cell which occupied -> consider as collision
        % occupancyMap3D basic parameter for occupancyThreshold is "0.65"
        if any(occupancyProb > map3D.OccupiedThreshold)
            isColliding = true;
            % fprintf('Collision Detected -> for checking code\n');
            % fprintf('Collision Occured at Robot Pose : %.2f, %.2f %.2f
            % .\n', poseX, poseY, poseZ);
        end



    end

    % --- checkRobotConstraints function ---
    % checkRobotConstraints : As the Robot pose, check robot's physical
    % constraints (ex: max slope, max step height)
    function isValid = checkRobotConstraints(robotPose, map3D, ...
            robotConstraints, currentNode)
        % robotPose : we'll check robot pose [x, y, z, r, p, y]
        % map3D : occupancyMap3D object (geographical information)
        % robotConstraints : robots' constraints (maxSlope, maxStepHeight)

        isValid = true;     % Basically considered as valid

        % For slope Estimation, it will be further study in c++ simulation.
        % for now, lets just say 'VALID'

        poseX       = robotPose(1);
        poseY       = robotPose(2);
        poseZ       = robotPose(3);
        poseRoll    = robotPose(4);
        posePitch   = robotPose(5);
        poseYaw     = robotPose(6);

        % 1. Check Robot's roll / pitch slope constraints
        % From now, just check for startPose 
        if abs(poseRoll) > robotConstraints.maxSlope || ...
                abs(posePitch) > robotConstraints.maxSlope
            isValid = false;
            return;
        end

        % 2. Slope estimation and check constraints
        % This part is sturdy so lets write later
        % From now keep'isValid'
        % Later, u can add here as we make traversibility map

        % C. difference of z check
        % If there z moving in motion primitve, this would be important
        if abs(robotPose(3) - currentNode.z) > robotConstraints.maxStepHeight
            isValid = false;
            % fprintf('Constraints : Z difference warning (%.2f m).
            % \n', abs(robotPose(3) - currentNode.z));
            return;
        end

        % Check map's Z (need minZ, maxZ defined at nav_mapping)
        % cannot use map3D.getMapLimits(), hardcoded value, parameters
        % value
        % nav_mapping.m -> minZ = 0, maxZ = 5
        minMapZ = 0;
        maxMapZ = 5;
        if robotPose(3) < minMapZ || robotPose(3) > maxMapZ
            isValid = false;
            % fprintf(' Its out of Constraints : Robot is out of map');
            return;
        end
       % if all constraints fine, valid 

    end




    % --- expandNode function ---
    function generateNodes = expandNode(currentNode, currentClosedListIndex,...
                            map3D, robot, robotConstraints, parameters, goalNode,...
                            nodeTemplate, motionPrimitives)
        generateNodes = []; % Storage where expanded Node will be stored

        % A. get motion Primitives in CurrentNode
        for i  = 1:length(motionPrimitives)
            motion = motionPrimitives{i};   % [delta_x, delta_y, delta_yaw, cost_multiplier]

            delta_x                 = motion(1);
            delta_y                 = motion(2);
            delta_yaw               = motion(3);
            move_cost_multiplier    = motion(4);
            
            % Pose calculation of next node based on current Node
            % X-Y coordinate movement should be considered with yaw rate
            % and transformed into world coordinate
            currentYaw = currentNode.yaw;

            % delta_X, delta_y in world coordinate
            world_delta_x = delta_x * cos(currentYaw) - delta_y * sin(currentYaw);
            world_delta_y = delta_x * sin(currentYaw) + delta_y * cos(currentYaw);

            nextNode        = nodeTemplate;
            nextNode.x      = currentNode.x + world_delta_x;
            nextNode.y      = currentNode.y + world_delta_y;
            nextNode.z      = currentNode.z; % for Future work, stair, slope etc...
            nextNode.roll   = currentNode.roll;  % initial roll would be same
            nextNode.pitch  = currentNode.pitch; % initial pitch would be same
            nextNode.yaw    = currentNode.yaw + delta_yaw;

            % Normalization Yaw angle (~pi ~ pi)
            nextNode.yaw    = atan2(sin(nextNode.yaw), cos(nextNode.yaw));
            
            % B. Collision Checking for Next Node (checkCollision function
            % would be called
            % isCollision = checkCollision([nextNode.x, nextNode.y, nextNode.z, ...
            %     nextNode.roll, nextNode.pitch, nextNode.yaw], ...
            %     map3D, robotConstraints);

            % B-1 checkCollision with 2D inflated Map
            % isCollision = checkCollision2D(nextNode.x, nextNode.y);
            isCollision = checkCollision2D(nextNode.x, nextNode.y, ...
                xLimits, yLimits);
            
            % C. Check Robot Constraints (maxslope, step height)
            isValidConstraints = checkRobotConstraints([nextNode.x, nextNode.y, nextNode.z, ...
                nextNode.roll, nextNode.pitch, nextNode.yaw], ...
                map3D, robotConstraints, currentNode);

            % C - 1. Check Robot Constraints (check both collision, constraints)
            if isCollision || ~isValidConstraints
                continue;   % if robot banned from collision check and contraints -> this node invalid
            end
            
            
            % D. g_cost calculate
            % g_cost = g_cost + current move cost
            % cost will be following = (parameters.stepSize) *
            % move_cost_multiplier + (etc... like traversability)
            distanceCost = parameters.stepSize * move_cost_multiplier;
    
            % traversabilityCost = getTraversibilityCost(nextNode, map3D);
            % traversabilityCost는 나중에 구현!!!!!

            % if traversabilityCost is defined ...
            % nextNode.g_cost = currentNode.g_cost + distanceCost +
            % traversibilityCost;
            % From now, just distance cost
            nextNode.g_cost = currentNode.g_cost + distanceCost;

            % E. calculate h_cost (heuristic function cost)
            nextNode.h_cost = calculateHeuristic(nextNode, goalNode,...
                robotConstraints.minTurningRadius);

            % F. calculate f_cost 
            nextNode.f_cost = nextNode.g_cost + nextNode.h_cost;

            % G. Store parent Node Index and motion primitives Index
            nextNode.parent_idx = currentClosedListIndex;
            nextNode.motion_primitive_idx = i;  % Motion primitive Index
            
            % Store only valid Node to generateNodes
            generateNodes = [generateNodes; nextNode];
        end
        % generateNodes will be used to update Open List and Closed List 
        % Later
    end
    % for iter = 1 : paramters.maxIterations
    %       % a. Open List에서 가장 낮은 f_cost를 가진 노드 선택하기
    %       % b. 선택한 노드를 Closed List에 추가
    %       % c. 목표 지점에 도착했는지 확인
    %       % if current pose != goal pose
    %       % d. Node 확장하기 ( expandNode )
    %       %   - 로봇 움직임의 제약 사항 확인 ( 
    %       %   - 로봇 움직임 충돌 검사 (이게 리소스 개많이 먹어서 Lazy PRM*를 사용한다는
    % 논문들이 많은데, 일단 Hybrid A*로 하고, Lazy PRM*는 추후에 논문 확인하고 구현해봐야
    % 할 듯? GPU도 쓰는데 이거 base단을 일단 짜고, 각 leg edge에 대한 충돌성 검사는 리소
    % 스 적게 먹는 방식으로 슈웃~~
    %       %  충돌 검사 (checkCollision 함수 만들기!, map3D랑 robotConstraints
    %       %  들고 와야할 듯
    %       %   - 로봇의 동역학, 기구학적 모순점 확인!!! (robotConstraints로 긁어?)
    %       %   - 새로운 상태에 대한 g(n) 계산 f(n) + g(n) = h(n)
    %       %   - 휴리스틱 함수 불러와서 h(n) 계산 박기
    %       %   - 새로운 노드를 Open List에 추가 아니면 Update

end
% 250604 4