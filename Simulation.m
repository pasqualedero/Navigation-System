clear; close all; clc;

%% Load Map 
origImageMap = im2bw(imread("Image1.bmp"));
resolution = 30;
map = binaryOccupancyMap(~origImageMap, resolution);

start = [26 17 -pi/2];
goal = [0.3 17 pi/2];

show(map)
grid on;

% Apply Hybrid A*
ss = stateSpaceSE2;

ss.StateBounds = [map.XWorldLimits; map.YWorldLimits; [-pi pi]];

sv = validatorOccupancyMap(ss); 
sv.Map = map; 
sv.ValidationDistance = 1; % Distance between collision checks (in meters)

planner = plannerHybridAStar(sv);

planner.MinTurningRadius = 1.6; % Minimum turning radius in meters
planner.MotionPrimitiveLength = 2.5; % Arc length of motion primitives
planner.AnalyticExpansionInterval = 10; % How often to try connecting directly to goal
planner.InterpolationDistance = 0.1;

[refPath, solnInfo] = plan(planner, start, goal);

% Plot
figure;
show(planner); 
title('Hybrid A* Path Planning');



