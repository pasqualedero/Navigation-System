clear; close all; clc;

%% Load Map
origImageMap = im2bw(imread("Image1.bmp"));

resolution = 30;
map = binaryOccupancyMap(~origImageMap, resolution);
% 1 -> obstacle (black), 0 -> free (white)

% In world coordinates
start = [26 17 -pi/2];
goal = [0.3 17 pi/2];

figure;
show(map)
grid on;
title('Workspace of the Robot');

% Inflate the map
inflatedMap = copy(map);
radius = 0.4; % [m]
inflate(inflatedMap, radius);

figure;
show(inflatedMap);
title('Inflated Map');
grid on;

% Compare original and inflated
gridInflated = occupancyMatrix(inflatedMap);
gridOriginal = occupancyMatrix(map);

imgInfo = zeros(map.GridSize(1), map.GridSize(2), 3);

% 3 matrices for every RGB
R = ones(size(gridOriginal));
G = ones(size(gridOriginal));
B = ones(size(gridOriginal));

maskObstacle = (gridOriginal == 1);
maskInflated = (gridInflated == 1) & ~gridOriginal;

% inflated areas are red
R(maskInflated) = 1;
G(maskInflated) = 0;
B(maskInflated) = 0;

% obsttacle areas are black
R(maskObstacle) = 0;
G(maskObstacle) = 0;
B(maskObstacle) = 0;

imgInfo(:,:,1) = R;
imgInfo(:,:,2) = G;
imgInfo(:,:,3) = B;

figure
imshow(imgInfo)

%% Apply Hybrid A*
ss = stateSpaceSE2;
ss.StateBounds = [map.XWorldLimits; map.YWorldLimits; [-pi pi]];

sv = validatorOccupancyMap(ss);
sv.Map = inflatedMap;
sv.ValidationDistance = 0.5; % frequency of "collision-checking" in [m]

planner = plannerHybridAStar(sv);

planner.MinTurningRadius = 1.6; % Minimum turning radius in meters
planner.MotionPrimitiveLength = 2.5; % Arc length of motion primitives
planner.AnalyticExpansionInterval = 20; % How often to try connecting directly to goal
planner.InterpolationDistance = 0.1;

[refPath, solnInfo] = plan(planner, start, goal);
refPath = computeRefAngle(refPath.States, refPath.States(1,3));

% Plot
figure;
show(planner);
grid on;
title('Hybrid A* Path Planning');

%% Apply A*

startGrid = world2grid(inflatedMap, [start(1) start(2)]);
goalGrid = world2grid(inflatedMap, [goal(1) goal(2)]);

aStar = aStarObj(gridInflated, startGrid, goalGrid);
pathAstar = grid2world(inflatedMap,aStar.path);

% Visualize the A* path on the inflated map
figure
show(inflatedMap);
hold on;
plot(pathAstar(:,1), pathAstar(:,2), 'g', 'LineWidth', 2);
hold off;

% plot A* on image normal + inflated

figure;
hold on;
imshow(imgInfo, 'XData', [0 map.XWorldLimits(2)], 'YData', [0 map.YWorldLimits(2)]);
plot(pathAstar(:,1), map.YWorldLimits(2).*ones(size(pathAstar,1))-pathAstar(:,2), 'g', 'LineWidth', 2);
axis([0 map.XWorldLimits(2) 0 map.YWorldLimits(2)])
axis on
hold off;

%% Model
wheelRadius = 0.1; %[m]
wheelDist = [0.3 0.25];
vehicle = FourWheelSteering(wheelRadius, wheelDist);

%% Define tVec
Ts = 0.1;
SimTime = 60; %[s]
tVec = 0:Ts:SimTime;

s = linspace(0,1,size(refPath,1));
s_interp = linspace(0,1,numel(tVec));
x_ref = interp1(s, refPath(:,1), s_interp, 'linear');
y_ref = interp1(s, refPath(:,2), s_interp, 'linear');
theta_ref = interp1(s, refPath(:,3), s_interp, 'linear');

ref = [x_ref(:), y_ref(:), theta_ref(:)];

%% NLMPC
nx = 3;
ny = 3;
nu = 3;

nlmpcController = nlmpc(nx,ny,nu);
nlmpcController.Ts = Ts;
p = 8;
c = 2;
nlmpcController.PredictionHorizon = p;
nlmpcController.ControlHorizon = c;

% Define the state-space model for the NLMPC controller
nlmpcController.Model.StateFcn = @(x,u) FourWheelSteerDyn(x,u,Ts);
nlmpcController.Model.IsContinuousTime = false;

% weights
% wheights MV has second component high because the 4WheelSteer doesnt take
% into account vy to compute inverse kinematics
nlmpcController.Weights.ManipulatedVariables = [30,50,1];
nlmpcController.Weights.ManipulatedVariablesRate = [10,10,10];
nlmpcController.Weights.OutputVariables = [200, 100, 50];

% x
nlmpcController.MV(1).Max = 0.9;
nlmpcController.MV(1).Min = -0.9;
nlmpcController.MV(1).RateMax = 0.2;
nlmpcController.MV(1).RateMin = -0.2;

% y
nlmpcController.MV(2).Max = 0.9;
nlmpcController.MV(2).Min = -0.9;
nlmpcController.MV(2).RateMax = 0.2;
nlmpcController.MV(2).RateMin = -0.2;

% omega
nlmpcController.MV(3).Max = pi/4;
nlmpcController.MV(3).Min = -pi/4;
nlmpcController.MV(3).RateMax = pi/6;
nlmpcController.MV(3).RateMin = -pi/6;

% loop
trajectory = ref;
r = rateControl(1/Ts);

trajectory(end+1:end+p,:) = repmat(trajectory(end,:),[p 1]);

% Initialize the pose array for storing the robot's position
pose = zeros(length(tVec),3);
pose(1,:) = start; % Set the initial pose to the starting position
u = zeros(length(tVec),nu);

wheelSpds = zeros(length(tVec),2);
steerAngFS = zeros(length(tVec),2);

for idx = 2:length(tVec)
    %Run the NLPMC
    [u(idx,:),~,mpcinfo] = nlmpcmove(nlmpcController, pose(idx-1,:), u(idx-1,:), trajectory(idx:idx+p-1,:));
    [wheelSpds(idx,:), steerAngFS(idx,:)] = inverseKinematicsFrontSteer(vehicle, u(idx,1), u(idx,3));
    % If no noise, vel == u(idx,:)
    velBody = forwardKinematics(vehicle,wheelSpds(idx,:),steerAngFS(idx,:));
    vel = bodyToWorld(velBody,pose(idx-1,:));

    pose(idx,:) = pose(idx-1,:) + vel' .* Ts;
end

%% Plots (NLMPC)

figure
show(map);
hold on
plot(pose(:,1),pose(:,2));
plot(trajectory(:,1),trajectory(:,2));
grid on
hold off

figure
hold on
% Plot the trajectory's theta-coordinate over time
plot(1:length(trajectory), trajectory(:,3), 'b', 'LineWidth', 2);
xlabel('Time Step');
ylabel('\theta Coordinate');
title('Trajectory \theta Coordinate Over Time');
% Finalize the trajectory visualization by plotting the robot's pose
plot(1:length(pose), pose(:,3), 'r', 'LineWidth',2)
legend('reference','actual angle')
grid on;
hold off;

figure
hold on
subplot(3,3,[1 2 3])
plot(tVec,steerAngFS(:,1),'LineWidth',2);
title("Front Steering Angle")
ylabel('[rad]')
grid on
hold off

subplot(3,3,[4 5 6])
hold on
plot(tVec,wheelSpds(:,1),'LineWidth',2,'Color','r');
plot(tVec,wheelSpds(:,2),'LineWidth',2,'Color','b');
legend('Front Wheel','Rear Wheel')
ylabel('[rad/s]')
title('Wheel Speeds')
grid on
hold off

subplot(3,3,[7,8,9])
hold on
plot(tVec,u(:,1),'LineWidth',2,'Color','r')
plot(tVec,u(:,2),'LineWidth',2,'Color','b')
legend('V_x','V_y')
ylabel('[m/s]')
title('Control Input')
grid on
hold off

%% Lidar Sensor
lidar = rangeSensor;
lidar.Range = [0 5];                 
lidar.HorizontalAngle = [-pi/2, pi/2];
lidar.HorizontalAngleResolution = pi/50;

%% Pure Pursuit Controller
waypoints = ref(1:10:end,1:2);
controller = controllerPurePursuit;
controller.Waypoints = waypoints;
controller.LookaheadDistance = 1;
controller.DesiredLinearVelocity = 0.75;
controller.MaxAngularVelocity = 1.5;

%% VFH
vfh = controllerVFH;
vfh.DistanceLimits = [0.05 5];
vfh.NumAngularSectors = 36;
vfh.HistogramThresholds = [4 8];
vfh.RobotRadius = .2;
vfh.SafetyDistance = .2;
vfh.MinTurningRadius = 0.75;

%% Create a new obstacle (rectangle) in position (3,7)
p1 = world2grid(map, [3 7]);
p2 = world2grid(map, [5 8]);

rows = sort([p1(1), p2(1)]);
cols = sort([p1(2), p2(2)]);

mapObstacle = copy(map);
[R_grid, C_grid] = meshgrid(rows(1):rows(2), cols(1):cols(2));
setOccupancy(mapObstacle, [R_grid(:), C_grid(:)], 1, 'grid');

figure;
hold on
show(mapObstacle);
plot(pose(:,1),pose(:,2));
plot(trajectory(:,1),trajectory(:,2));
title('Map with New Obstacle Added');
hold off

%% Initialize the pose array for storing the robot's position
posePP = zeros(length(tVec),3);
posePP(1,:) = start; 
uPP = zeros(length(tVec),nu); %[vx,vy,omega]

wheelSpdsPP = zeros(length(tVec),2);
steerAngFSPP = zeros(length(tVec),2);

%% Plot
idxOA = 1;
idxMPC = 1;
poseOA = zeros(600,2);
poseMPC = zeros(600,2);

figure
hold on
show(mapObstacle)
robot = plot(posePP(1,1),posePP(1,2), 'Color','b','Marker','o','MarkerSize',10);
wayP = plot(waypoints(:,1),waypoints(:,2),'r.','Color','r','Marker','x');
lidarPlot = plot(nan, nan,'r.','MarkerSize', 8);
module = 1;
headX = posePP(1,1) + module * cos(posePP(1,3));
headY = posePP(1,2) + module * sin(posePP(1,3));
heading = plot([posePP(1,1) headX], [posePP(1,2) headY], 'LineStyle','-.','Color','black','LineWidth',2);
avoidancePath = plot(nan, nan,'r.', 'Color','g');
mpcPath = plot(nan, nan,'r.', 'Color','c');

%% Loop
idxPose = 2;
idxRef = 2;
for i = 2 : length(tVec)

    % Target reached?
    curPose = posePP(idxPose-1,:);
    if norm(curPose(1:2)-ref(end,1:2)) < 0.4
        break;
    end

    [ranges, angles] = lidar(curPose,mapObstacle);
    scan = lidarScan(ranges, angles);

    targetDir = atan2(trajectory(idxRef+p-1,2)-curPose(2),trajectory(idxRef+p-1,1)-curPose(1)) - curPose(3);

    vfhDirection = vfh(scan.Ranges, scan.Angles, targetDir);
    
    if (~isnan(vfhDirection) && abs(vfhDirection-targetDir) > 0.1) 
        obst = true;
        while obst
            % Get the sensor readings
            curPose = posePP(idxPose-1,:);
            [ranges, angles] = lidar(curPose,mapObstacle);
            scan = lidarScan(ranges, angles);

            % Run the path following and obstacle avoidance algorithms
            [vRef,wRef,lookAheadPt] = controller(curPose);
            uPP(idxPose,:) = vRef';
            targetDir = atan2(lookAheadPt(2)-curPose(2),lookAheadPt(1)-curPose(1)) - curPose(3);
            steerDir = vfh(scan.Ranges,scan.Angles,targetDir);
            if ~isnan(steerDir) && abs(steerDir-targetDir) > 0.1
                wRef = steerDir;
            end

            % Control the robot
            % velB = [vRef * cos(wRef); vRef * sin(wRef); wRef];                   % Body velocities [vx;vy;w]
            % vel = bodyToWorld(velB,curPose);  % Convert from body to world
            [wheelSpdsPP(idxPose,:), steerAngFSPP(idxPose,:)] = inverseKinematicsFrontSteer(vehicle, vRef, wRef);
            velB = forwardKinematics(vehicle,wheelSpdsPP(idxPose,:),steerAngFSPP(idxPose,:));
            vel = bodyToWorld(velB, curPose);

            % Perform forward discrete integration step
            posePP(idxPose,:) = curPose + vel'*Ts;
            poseOA(idxOA,:) = posePP(idxPose,1:2);

            % Update plot
            set(robot, 'XData', posePP(idxPose,1), 'YData', posePP(idxPose,2));
            headX = posePP(idxPose,1) + module * cos(posePP(idxPose,3));
            headY = posePP(idxPose,2) + module * sin(posePP(idxPose,3));
            set(heading, 'XData', [posePP(idxPose,1) headX], 'YData', [posePP(idxPose,2) headY]);
            set(avoidancePath, 'XData', poseOA(1:idxOA,1), 'YData', poseOA(1:idxOA,2));
            set(mpcPath, 'XData', poseMPC(1:idxMPC,1), 'YData', poseMPC(1:idxMPC,2))
            drawnow limitrate;
            waitfor(r);

            % Update Pose index
            idxPose = idxPose+1;
            idxOA = idxOA + 1;
           
            % Check
            [distMin, idxMin] = getClosestPointIndex(trajectory(idxRef:end,:), curPose);
            if distMin < 0.001 %[m]
                obst = false;
                idxRef = idxMin+1;
            end
        end
        
    end

    %Run the NLPMC
    [uPP(idxPose,:),~,mpcinfo] = nlmpcmove(nlmpcController, posePP(idxPose-1,:), uPP(idxPose-1,:), trajectory(idxRef:idxRef+p-1,:));
    [wheelSpdsPP(idxPose,:), steerAngFSPP(idxPose,:)] = inverseKinematicsFrontSteer(vehicle, uPP(idxPose,1), uPP(idxPose,3));
    % If no noise, vel == u(idx,:)
    velBody = forwardKinematics(vehicle,wheelSpdsPP(idxPose,:),steerAngFSPP(idxPose,:));
    vel = bodyToWorld(velBody,posePP(idxPose-1,:));
    posePP(idxPose,:) = posePP(idxPose-1,:) + vel' .* Ts;
    poseMPC(idxMPC,:) =  posePP(idxPose,1:2);
   
    % Update plot
    set(robot, 'XData', posePP(idxPose,1), 'YData', posePP(idxPose,2));
    headX = posePP(idxPose,1) + module * cos(posePP(idxPose,3));
    headY = posePP(idxPose,2) + module * sin(posePP(idxPose,3));
    set(heading, 'XData', [posePP(idxPose,1) headX], 'YData', [posePP(idxPose,2) headY]);
    set(avoidancePath, 'XData', poseOA(1:idxOA,1), 'YData', poseOA(1:idxOA,2));
    set(mpcPath, 'XData', poseMPC(1:idxMPC,1), 'YData', poseMPC(1:idxMPC,2));    
    drawnow limitrate;
    waitfor(r);
   
    % Update
    idxRef = idxRef + 1;
    idxPose = idxPose+1; 
    idxMPC = idxMPC + 1;
    
end

figure
hold on
show(mapObstacle);
plot(posePP(:,1),posePP(:,2),'r.');
plot(trajectory(:,1),trajectory(:,2))

function [distMin, idx] = getClosestPointIndex(reference, queryPoint)
    % reference: Nx3 matrix [x, y, theta]
    % queryPoint: 1x2 [x, y] or 1x3 [x, y, theta] vector
    
    % 1. Extract only X and Y columns for distance calculation
    refXY = reference(:, 1:2);
    ptXY = queryPoint(1:2);
    
    % 2. Calculate Squared Euclidean Distance (Vectorized)
    % (x_ref - x_pt)^2 + (y_ref - y_pt)^2
    % We use squared distance because it is faster (no sqrt) and 
    % preserves the minimum.
    diffs = refXY - ptXY;
    distSq = sum(diffs.^2, 2);
    
    % 3. Find the index of the minimum distance
    [distMin, idx] = min(distSq);
end


