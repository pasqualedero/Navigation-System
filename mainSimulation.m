clear; close all; clc;

%% Load Map
origImageMap = im2bw(imread("Image1.bmp"));

resolution = 30;
map = binaryOccupancyMap(~origImageMap, resolution);
% 1 -> obstacle (black), 0 -> free (white)

% In world coordinates
start = [26 17 -pi/2];
goal = [0.3 17 pi/2];

% Define the radius/width of the area you want to clear (in meters)
clearSize = 1.0; 

% Clear start and goal points
[x_s, y_s] = meshgrid(26-clearSize : 0.001 : 26+clearSize, ...
                      19-clearSize : 0.001 : 19+clearSize);
setOccupancy(map, [x_s(:), y_s(:)], 0); 

[x_g, y_g] = meshgrid(0.3-clearSize : 0.001 : 0.3+clearSize, ...
                      19-clearSize : 0.001 : 19+clearSize);
setOccupancy(map, [x_g(:), y_g(:)], 0);

figure;
hold on
show(map)
plot(start(1), start(2), 'Marker','.','Color','g', 'MarkerSize',10)
plot(goal(1), goal(2), 'Marker','.','Color','r','MarkerSize',10)
title('Workspace of the Robot');
grid on;
set(gca, 'Layer', 'top');
hold off

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
title('Hybrid A* Path Planning on Inflated Map');

%% Apply A*

startGrid = world2grid(inflatedMap, [start(1) start(2)]);
goalGrid = world2grid(inflatedMap, [goal(1) goal(2)]);

aStar = aStarObj(gridInflated, startGrid, goalGrid);
pathAstar = grid2world(inflatedMap,aStar.path);

% Visualize the A* path on the inflated map
figure
show(map);
hold on;
title('A^* Algorithm and Generated Path')
plot(pathAstar(:,1), pathAstar(:,2), 'g', 'LineWidth', 2);
legend('Generated Path')
grid on;
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

%% ----------------------- NLMPC ------------------------------------------
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
nlmpcController.Weights.ManipulatedVariables = [30,50,1];
nlmpcController.Weights.ManipulatedVariablesRate = [10,10,10];
nlmpcController.Weights.OutputVariables = [200, 100, 50];

% vx
nlmpcController.MV(1).Max = 0.9;
nlmpcController.MV(1).Min = -0.9;
nlmpcController.MV(1).RateMax = 0.2;
nlmpcController.MV(1).RateMin = -0.2;

% vy
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
pose(1,:) = start; 
u = zeros(length(tVec),nu);

wheelSpds = zeros(length(tVec),2);
steerAngFS = zeros(length(tVec),2);

errX = zeros(length(tVec),1);
errY = zeros(length(tVec),1);
errTheta = zeros(length(tVec),1);

% Dynamic Plot
figure
show(map)
grid on
hold on
robotMpc = plot(pose(1,1),pose(1,2), 'Color','r','Marker','.','MarkerSize',33, 'MarkerEdgeColor','auto');
module = 1;
headXmpc = pose(1,1) + module * cos(pose(1,3));
headYmpc = pose(1,2) + module * sin(pose(1,3));
headingMpc = plot([pose(1,1) headXmpc], [pose(1,2) headYmpc], 'LineStyle','-.','Color','black','LineWidth',2);
mpcPath = plot(nan, nan,'r.', 'Color','b');
trajPred = plot(nan, nan, 'LineStyle','none','Marker','square','MarkerSize',8);
traj = plot(trajectory(:,1),trajectory(:,2), 'LineStyle','--');
plot(start(1), start(2), 'Marker','.','Color','g', 'MarkerSize',10)
plot(goal(1), goal(2), 'Marker','.','Color','r','MarkerSize',10)
title('Path-Following: NLMPC (Cascaded Kinematic Control)')

for idx = 2:length(tVec)

    % Compute input u(t)
    [u(idx,:),~,mpcinfo] = nlmpcmove(nlmpcController, pose(idx-1,:), u(idx-1,:), trajectory(idx:idx+p-1,:));
    
    % Apply u(t)
    [wheelSpds(idx,:), steerAngFS(idx,:)] = inverseKinematicsFrontSteer(vehicle, u(idx,1), u(idx,3));
    velBody = forwardKinematics(vehicle,wheelSpds(idx,:),steerAngFS(idx,:));
    vel = bodyToWorld(velBody,pose(idx-1,:));
    pose(idx,:) = pose(idx-1,:) + vel' .* Ts;

    % Calculate errors
    errX(idx) = (trajectory(idx,1) - pose(idx,1));
    errY(idx) = (trajectory(idx,2) - pose(idx,2));
    errTheta(idx) = (angdiff(trajectory(idx,3), pose(idx,3)));

    % Update plots
    set(robotMpc, 'XData', pose(idx,1), 'YData', pose(idx,2));
    headXmpc = pose(idx,1) + module * cos(pose(idx,3));
    headYmpc = pose(idx,2) + module * sin(pose(idx,3));
    set(headingMpc, 'XData', [pose(idx,1) headXmpc], 'YData', [pose(idx,2) headYmpc]);
    set(mpcPath, 'XData', pose(1:idx,1), 'YData', pose(1:idx,2)); 
    set(trajPred, 'XData', trajectory(idx:idx+p-1,1), 'YData', trajectory(idx:idx+p-1,2))
    drawnow limitrate;
    %waitfor(r);
end
legend([mpcPath,traj,robotMpc], 'Trajectory', 'Ref. Path' , 'Robot Position')
hold off;

%% Plots (NLMPC)

figure
hold on
plot(1:length(trajectory), trajectory(:,3), 'b', 'LineWidth', 2);
xlabel('Time Step');
ylabel('\theta Coordinate');
title('Trajectory \theta Coordinate Over Time');
plot(1:length(pose), pose(:,3), 'r', 'LineWidth',2)
legend('reference angle','actual angle (World Ref. Frame)')
grid on;
hold off;

figure
hold on
subplot(4,3,[1 2 3])
hold on
plot(tVec,steerAngFS(:,1),'LineWidth',2, 'Color','g');
plot(tVec,steerAngFS(:,2),'LineWidth',2,'Color','r')
legend('\phi_f','\phi_r')
title("Front and Rear Steering Angle: \phi_r, \phi_f")
ylabel('[rad]')
grid on
hold off
hold off

subplot(4,3,[4 5 6])
hold on
plot(tVec,wheelSpds(:,1),'LineWidth',2,'Color','r');
plot(tVec,wheelSpds(:,2),'LineWidth',2,'Color','b');
legend('Front Wheel','Rear Wheel')
ylabel('[rad/s]')
title('Wheel Speeds (Body Frame)')
grid on
hold off

subplot(4,3,[7,8,9])
hold on
plot(tVec,u(:,1),'LineWidth',2,'Color','r')
plot(tVec,u(:,2),'LineWidth',2,'Color','b')
yline(nlmpcController.MV(1).Max, 'LineStyle','--')
yline(nlmpcController.MV(1).Min, 'LineStyle','--')
legend('V_x','V_y','Upper Limit','Lower Limit')
ylabel('[m/s]')
title('v_x and v_y')
grid on
hold off

subplot(4,3,[10,11,12])
hold on
title('Angular Velocity \omega')
plot(tVec,u(:,3),'LineWidth',2,'Color','r')
yline(nlmpcController.MV(3).Max, 'LineStyle','--')
yline(nlmpcController.MV(3).Min, 'LineStyle','--')
legend('\omega','Upper Limit','Lower Limit')
ylabel('rad/s')
xlabel('Time [s]')
grid on
hold off

% Tracking Errors Plot
figure('Name', 'Tracking Errors');

% X Error
subplot(3,1,1)
plot(tVec, errX, 'LineWidth', 1.5, 'Color', 'blue') 
title('Longitudinal Tracking Error (X)')
ylabel('Error [m]')
ylim([-0.5 0.5])
yticks(-0.5:0.2:0.5)
grid on

% Y Error
subplot(3,1,2)
plot(tVec, errY, 'LineWidth', 1.5, 'Color', 'blue') 
title('Lateral Tracking Error (Y)')
ylabel('Error [m]')
ylim([-0.5 0.5])
yticks(-0.5:0.2:0.5)
grid on

% Theta Error
subplot(3,1,3)
plot(tVec, errTheta, 'LineWidth', 1.5, 'Color', 'blue') 
title('Heading Tracking Error (\theta)')
xlabel('Time [s]')
ylabel('Error [rad]')
ylim([-0.5 0.5])
yticks(-0.5:0.2:0.5)
grid on


waitforbuttonpress

%%--------------------- Obstacle Avoidance --------------------------------

%% Lidar Sensor 
lidar = rangeSensor;
lidar.Range = [0 5.5];                 
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
vfh.DistanceLimits = [1 5.5];
vfh.NumAngularSectors = 36;
vfh.HistogramThresholds = [3.5 8];
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
set(gca,'Layer','Top')
grid on
robot = plot(posePP(1,1),posePP(1,2), 'Color','r','Marker','.','MarkerSize',33, 'MarkerEdgeColor','auto');
wayP = plot(waypoints(:,1),waypoints(:,2),'r.','Color','r','Marker','x');
lidarPlot = plot(nan, nan,'r.','MarkerSize', 8);
module = 1;
headX = posePP(1,1) + module * cos(posePP(1,3));
headY = posePP(1,2) + module * sin(posePP(1,3));
heading = plot([posePP(1,1) headX], [posePP(1,2) headY], 'LineStyle','-.','Color','black','LineWidth',2);
headingVFH = plot(nan,nan,'LineStyle','-.','Color','m');
avoidancePath = plot(nan, nan,'r.', 'Color','g');
mpcPath = plot(nan, nan,'r.', 'Color','b');
title('NLMPC + Pure Pursuit Control with VFH Integration')

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

    steerDir = vfh(scan.Ranges, scan.Angles, targetDir);

    headXV = curPose(1) + module * cos(steerDir+curPose(3));
    headYV = curPose(2) + module * sin(steerDir+curPose(3));
    set(headingVFH, 'XData', [curPose(1) headXV], 'YData', [curPose(2) headYV]);

    if (~isnan(steerDir) && abs(angdiff(targetDir, steerDir)) > 0.1) 
        obst = true;
        while obst
            % Get the sensor readings
            curPose = posePP(idxPose-1,:);
            [ranges, angles] = lidar(curPose,mapObstacle);
            scan = lidarScan(ranges, angles);

            % Run the path following and obstacle avoidance algorithms
            [vRef,wRef,lookAheadPt] = controller(curPose);         
            targetDir = atan2(lookAheadPt(2)-curPose(2),lookAheadPt(1)-curPose(1)) - curPose(3);
            steerDir = vfh(scan.Ranges,scan.Angles,targetDir);
            
            if ~isnan(steerDir) && abs(angdiff(targetDir, steerDir)) > 0.1
                wRef = 1 * steerDir;
            end
           
            [wheelSpdsPP(idxPose,:), steerAngFSPP(idxPose,:)] = inverseKinematicsFrontSteer(vehicle, vRef, wRef);
            velB = forwardKinematics(vehicle,wheelSpdsPP(idxPose,:),steerAngFSPP(idxPose,:));
            uPP(idxPose,:) = velB;
            vel = bodyToWorld(velB, curPose);

            % Perform forward discrete integration step
            posePP(idxPose,:) = curPose + vel'*Ts;
            poseOA(idxOA,:) = posePP(idxPose,1:2);

            % Update plot
            headXV = curPose(1) + module * cos(steerDir+curPose(3));
            headYV = curPose(2) + module * sin(steerDir+curPose(3));
            set(headingVFH, 'XData', [curPose(1) headXV], 'YData', [curPose(2) headYV]);
            set(robot, 'XData', posePP(idxPose,1), 'YData', posePP(idxPose,2));
            headX = posePP(idxPose,1) + module * cos(posePP(idxPose,3));
            headY = posePP(idxPose,2) + module * sin(posePP(idxPose,3));
            set(heading, 'XData', [posePP(idxPose,1) headX], 'YData', [posePP(idxPose,2) headY]);
            set(avoidancePath, 'XData', poseOA(1:idxOA,1), 'YData', poseOA(1:idxOA,2));
            set(mpcPath, 'XData', poseMPC(1:idxMPC,1), 'YData', poseMPC(1:idxMPC,2))
            drawnow limitrate;
            % waitfor(r);

            % Update Pose index
            idxPose = idxPose+1;
            idxOA = idxOA + 1;
           
            % Check
            [distMin, idxMin] = getClosestPointIndex(trajectory(idxRef:end,:), curPose);
            if ~isnan(steerDir) && abs(angdiff(targetDir, steerDir)) <= 0.1 && distMin < 0.1
                obst = false;
                idxRef = idxRef+idxMin;
            end
        end
        
    end

    %Run the NLPMC
    [uPP(idxPose,:),~,mpcinfo] = nlmpcmove(nlmpcController, posePP(idxPose-1,:), uPP(idxPose-1,:), trajectory(idxRef:idxRef+p-1,:));
    [wheelSpdsPP(idxPose,:), steerAngFSPP(idxPose,:)] = inverseKinematicsFrontSteer(vehicle, uPP(idxPose,1), uPP(idxPose,3));
   
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
    % waitfor(r);
   
    % Update
    idxRef = idxRef + 1;
    idxPose = idxPose+1; 
    idxMPC = idxMPC + 1;
    
end

legend([mpcPath,avoidancePath,wayP,robot], 'MPC Trajectory', 'Pure Pursuit Traj.', 'Waypoints' , 'Robot Position')
hold off;

waitforbuttonpress;

%% ----------------- Localization (orig. map) -----------------------------

landmarks = [
    24.8833,16.0500;
    4.7500,3.9500;
    2.4500,15.5500;
    2.1500,10.7500;
    7.9500,8.1500;
    8.9500,4.8500;
    11.9500,4.6500;
    15.6833,1.8500;
    23.1500,1.8500;
    24.8833,4.0500;
    24.8833,8.0500;
    24.8833,12.0500;    
    2.4500,14.4833;
    3.8833,11.1167;
    1.3167,18.0167;
    3.8833,10.0833;
    1.0833,10.7500];

% % Motion model
% syms xr yr thetar vx vy omega T real
% syms 'x%d' 'y%d' [12 1] real
% 
% motionModel = [xr + (cos(thetar)*vx - sin(thetar)*vy)*T
%                 yr + (cos(thetar)*vy + sin(thetar)*vx)*T
%                 thetar + omega * T
%                 x
%                 y];
% JmotionModel = jacobian(motionModel, [xr; yr; thetar; x; y]);
% 
% % Observation model
% sensorModel = [sqrt((x - xr).^2 + (y - yr).^2)
%                 atan2(y - yr, x - xr) - thetar
% ];
% 
% JsensorModel = jacobian(sensorModel, [xr; yr; thetar; x; y]);
% JsensorModel = simplify(real(JsensorModel));

numLandmarks = size(landmarks, 1);

% Initial Pose
start = start';
trueStates = zeros(length(tVec), 3);
trueStates(1, :) = start;

% Extended State
stateEstimate = [start(:); NaN(2 * numLandmarks, 1)]; 

% Storage for History 
correctedStates = NaN(length(tVec), length(stateEstimate));
correctedStates(1, :) = stateEstimate';

predictedStates = NaN(length(tVec), length(stateEstimate));
predictedStates(1, :) = stateEstimate';

% Covariance Matrices
P = eye(3 + 2 * numLandmarks) * 1e-5; 
P(1:3, 1:3) = eye(3) * 0.1; % Robot pose uncertainty

Q = blkdiag(diag([0.05, 0.05, 0.005]), zeros(2 * numLandmarks)); % Process Noise
R = diag([0.005, 0.001]); % Measurement Noise 

Pcell = cell(1,length(tVec));
Pcell{1} = P;

% Control Inputs
uEFK = zeros(length(tVec), 3); % [vx, vy, omega]

% Plots
figure
show(map)
grid on
hold on
plot(landmarks(:,1), landmarks(:,2),'LineStyle','none','Color','r','Marker','o','MarkerFaceColor','auto');
robotEkf = plot(trueStates(1,1),trueStates(1,2), 'Color','r','Marker','.','MarkerSize',20, 'MarkerEdgeColor','auto');
module = 1;
headXEkf = trueStates(1,1) + module * cos(trueStates(1,3));
headYEkf = trueStates(1,2) + module * sin(trueStates(1,3));
headingEkf = plot([trueStates(1,1) headXEkf], [trueStates(1,2) headYEkf], 'LineStyle','-.','Color','black','LineWidth',2);
mpcPathEkf = plot(nan, nan,'r.', 'Color','b');
plotPred = plot(predictedStates(1,1), predictedStates(1,2), 'Color','m','LineStyle',':','LineWidth',2);
plotCorr = plot(correctedStates(1,1), correctedStates(1,2), 'Color','g','LineStyle','--','LineWidth',2);
[XDataCov, YDataCov] = plotCovariance(stateEstimate, P);
plotCov = plot(XDataCov, YDataCov, 'Color','black','LineWidth',1.5);
%plot(trajectory(:,1),trajectory(:,2), 'LineStyle','-');
title('EKF-SLAM')


% Main Simulation Loop
filter = EkfSlam(start', length(landmarks), P, Q, R);

for idx = 2:length(tVec)
    % Where am I before applying u(t)?
    currentPoseEst = correctedStates(idx-1, 1:3);

    % Compute u(t)
    [u_opt, ~, mpcinfo] = nlmpcmove(nlmpcController, currentPoseEst, uEFK(idx-1, :)', trajectory(idx:idx+p-1, :));
    uEFK(idx, :) = u_opt';    

    % Prediction: store it
    filter.predict(uEFK(idx,:), Ts);
    predictedStates(idx,:) = filter.mu';

    % Update prediction plot
    set(plotPred, 'XData', predictedStates(1:idx,1), 'YData', predictedStates(1:idx,2));
    waitfor(r);

    % Apply u(t)
    [wheelSpds, steerAng] = inverseKinematicsFrontSteer(vehicle, uEFK(idx, 1), uEFK(idx, 3));
    velBody = forwardKinematics(vehicle, wheelSpds, steerAng);
    
    velWorld = bodyToWorld(velBody, trueStates(idx-1, :));
    trueStates(idx, :) = trueStates(idx-1, :) + velWorld' * Ts;
    
    % Measure
    observations = simulateLandmarkObservations(trueStates(idx, :)', landmarks, R);

    % Correct
    filter.correct(observations);
    Pcell{idx} = filter.Sigma;
    correctedStates(idx, :) = filter.mu';

    % Update plots
    set(robotEkf, 'XData', trueStates(idx,1), 'YData', trueStates(idx,2));
    headXEkf = trueStates(idx,1) + module * cos(trueStates(idx,3));
    headYEkf = trueStates(idx,2) + module * sin(trueStates(idx,3));
    set(headingEkf, 'XData', [trueStates(idx,1) headXEkf], 'YData', [trueStates(idx,2) headYEkf]);
    set(mpcPathEkf, 'XData', trueStates(1:idx,1), 'YData', trueStates(1:idx,2)); 
    set(plotCorr, 'XData', correctedStates(1:idx,1), 'YData', correctedStates(1:idx,2));
    [XDataCov, YDataCov] = plotCovariance(trueStates(idx,:), filter.Sigma);
    set(plotCov, 'XData', XDataCov, 'YData', YDataCov);
    landmarksEst = filter.getMap;
    landmarksLoc = plot(landmarksEst(:,1),landmarksEst(:,2), 'LineStyle','none','Marker','x','Color','r','LineWidth',3);
    drawnow %limitrate;
    %waitfor(r);
end
legend([plotCorr, plotPred, robotEkf, landmarksLoc], ...
    {'Estimated Path (EKF)', 'Predicted Path', 'True Robot Position', 'Landmarks Est.'});
hold off;

% Compute trace of Covariance Matrices and norm of estimation error for the robot's pose
traces = zeros(1,length(Pcell));
estErrors = zeros(1,length(Pcell));
for i=1:length(Pcell)
    mat = Pcell{i};
    traccia = trace(mat);
    traces(1,i) = traccia;

    estError = norm(correctedStates(i,1:3)-trueStates(i,:),2);
    estErrors(i) = estError; 
end

figure
hold on
subplot(2,2,[1 2])
title("Trace of Covariance Matrix P")
plot(tVec,traces(1,:), 'LineWidth', 3)
xlabel('Time [s]')
ylabel('Trace of P')
grid on

subplot(2,2,[3 4])
title('Norm of state Error')
plot(tVec,estErrors(1,:), 'LineWidth', 3)
xlabel('Time [s]')
ylabel('Norm Value')
grid on
hold off

error_x = correctedStates(:, 1) - trueStates(:, 1);
error_y = correctedStates(:, 2) - trueStates(:, 2);
error_theta = correctedStates(:, 3) - trueStates(:, 3);
error_theta = atan2(sin(error_theta), cos(error_theta));

figure;
title('EKF-SLAM: Individual State Estimation Errors'); 
subplot(3,1,1)
plot(tVec, error_x, 'LineWidth', 1.5)
ylabel('Error [m]')
grid on

subplot(3,1,2)
plot(tVec, error_y, 'LineWidth', 1.5)
ylabel('Error [m]')
grid on

subplot(3,1,3)
plot(tVec, error_theta, 'LineWidth', 1.5)
xlabel('Time [s]')
ylabel('\theta Error [rad]')
grid on

