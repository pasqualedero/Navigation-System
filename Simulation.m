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

%% NLMPC
nx = 3;
ny = 3;
nu = 3;

nlmpcController = nlmpc(nx,ny,nu); 
Ts = 0.1;
nlmpcController.Ts = Ts;
p = 5;
c = 2;
nlmpcController.PredictionHorizon = p; 
nlmpcController.ControlHorizon = c; 

% Define the state-space model for the NLMPC controller
nlmpcController.Model.StateFcn = @(x,u) FourWheelSteerDyn(x,u,Ts);
nlmpcController.Model.IsContinuousTime = false;

% weights
nlmpcController.Weights.ManipulatedVariables = [1,1,1];
nlmpcController.Weights.ManipulatedVariablesRate = [10,10,10];
nlmpcController.Weights.OutputVariables = [500, 500, 50];

% x
controller.MV(1).Max = 0.9;
controller.MV(1).Min = -0.9;
controller.MV(1).RateMax = 0.2;
controller.MV(1).RateMin = -0.2;

% y
controller.MV(2).Max = 0.9;
controller.MV(2).Min = -0.9;
controller.MV(2).RateMax = 0.2;
controller.MV(2).RateMin = -0.2;

% omega
controller.MV(3).Max = pi/4;
controller.MV(3).Min = -pi/4;
controller.MV(3).RateMax = pi/4;
controller.MV(3).RateMin = -pi/4;

% loop
trajectory = refPath.States;
r = rateControl(1/Ts);

trajectory(end+1:end+p,:) = repmat(trajectory(end,:),[p 1]);

% Initialize the pose array for storing the robot's position
pose = zeros(length(trajectory),3);
pose(1,:) = start; % Set the initial pose to the starting position
u = zeros(length(trajectory),nu);

wheelSpds = zeros(length(trajectory),2);
steerAngFS = zeros(length(trajectory),2);

for idx = 2:length(trajectory)-p
    %Run the NLPMC
    [u(idx,:),~,mpcinfo] = nlmpcmove(nlmpcController, pose(idx-1,:), u(idx-1,:), trajectory(idx:idx+p-1,:));
    [wheelSpds(idx,:), steerAngFS(idx,:)] = inverseKinematicsFrontSteer(vehicle, u(idx,1), u(idx,3));
    % If no noise, vel == u(idx,:)
    velBody = forwardKinematics(vehicle,wheelSpds(idx,:),steerAngFS(idx,:));
    vel = bodyToWorld(velBody,pose(idx-1,:));

    pose(idx,:) = pose(idx-1,:) + vel' .* Ts;
end

figure
show(map);
hold on
plot(pose(:,1),pose(:,2));
plot(trajectory(:,1),trajectory(:,2));
hold off