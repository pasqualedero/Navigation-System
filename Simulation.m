clear; close all; clc;

%% Load Map 
origImageMap = im2bw(imread("Image1.bmp"));
resolution = 30;
map = binaryOccupancyMap(~origImageMap, resolution);
% 1 -> obstacle (black), 0 -> free (white)

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
sv.ValidationDistance = 0.3; % frequency of "collision-checking"

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



