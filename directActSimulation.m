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
sv.ValidationDistance = 0.5;

planner = plannerHybridAStar(sv);

planner.MinTurningRadius = 1.6; 
planner.MotionPrimitiveLength = 2.5; 
planner.AnalyticExpansionInterval = 20;
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
nu = 4;

nlmpcController = nlmpc(nx,ny,nu);
nlmpcController.Ts = Ts;
p = 8;
c = 2;
nlmpcController.PredictionHorizon = p;
nlmpcController.ControlHorizon = c;

% Define the state-space model for the NLMPC controller
nlmpcController.Model.StateFcn = @(x,u) DetailedDynamics(x,u,Ts);
nlmpcController.Model.IsContinuousTime = false;

% MV(1): omegaF (Front wheel speed)
nlmpcController.MV(1).Max = 9;        
nlmpcController.MV(1).Min = -9;      
nlmpcController.MV(1).RateMax = 1.0;  
nlmpcController.MV(1).RateMin = -1.0;

% MV(2): omegaR (Rear wheel speed)
nlmpcController.MV(2).Max = 9;
nlmpcController.MV(2).Min = -9;
nlmpcController.MV(2).RateMax = 1.0;
nlmpcController.MV(2).RateMin = -1.0;

% MV(3): phiF (Front steering angle)
nlmpcController.MV(3).Max = pi/6;      % 45 degrees max steering
nlmpcController.MV(3).Min = -pi/6;
nlmpcController.MV(3).RateMax = pi/12; % smoother steering transitions
nlmpcController.MV(3).RateMin = -pi/12;

% MV(4): phiR (Rear steering angle)
nlmpcController.MV(4).Max = 0;
nlmpcController.MV(4).Min = 0;
nlmpcController.MV(4).RateMax = 0;
nlmpcController.MV(4).RateMin = 0;

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

velXbody = zeros(length(tVec),1);
velYbody = zeros(length(tVec),1);

errX = zeros(length(tVec),1);
errY = zeros(length(tVec),1);
errTheta = zeros(length(tVec),1);

% Dynamic Plot
figure
show(map)
grid on
hold on
title('Path-Following: NLMPC (Direct Actuator Control)')
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

for idx = 2:length(tVec)

    % Compute input u(t)
    [u(idx,:),~,mpcinfo] = nlmpcmove(nlmpcController, pose(idx-1,:), u(idx-1,:), trajectory(idx:idx+p-1,:));
    wheelSpds(idx,:) = u(idx,1:2);
    steerAngFS(idx,:) = u(idx,3:4);
    % Apply u(t)
    velBody = forwardKinematics(vehicle,u(idx,1:2),u(idx,3:4));
    velXbody(idx) = velBody(1);
    velYbody(idx) = velBody(2);
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
    waitfor(r);
end
legend([mpcPath,traj,robotMpc], 'Trajectory', 'Ref. Path' , 'Robot Position')
hold off

%% Plots (NLMPC)

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
hold on
plot(tVec,steerAngFS(:,1),'LineWidth',2);
plot(tVec,steerAngFS(:,2),'LineWidth',2);
yline(nlmpcController.MV(3).Max,'LineStyle','--')
yline(nlmpcController.MV(3).Min,'LineStyle','--')
legend('\phi_f','\phi_r','Upper Limit','Lower limit')
title("Front Steering Angles: \phi_f, \phi_r")
ylabel('[rad]')
grid on
hold off
hold off

subplot(3,3,[4 5 6])
hold on
plot(tVec,wheelSpds(:,1),'LineWidth',2,'Color','r');
plot(tVec,wheelSpds(:,2),'LineWidth',2,'Color','b');
yline(nlmpcController.MV(1).Max,'LineStyle','--')
yline(nlmpcController.MV(1).Min,'LineStyle','--')
legend('Front Wheel','Rear Wheel','Upper Limit','Lower limit')
ylabel('[rad/s]')
title('Wheel Speeds')
grid on
hold off

subplot(3,3,[7,8,9])
hold on
plot(tVec,velXbody(:),'LineWidth',2,'Color','r')
plot(tVec,velYbody(:),'LineWidth',2,'Color','b')
legend('V_x','V_y')
ylabel('[m/s]')
title('Wheels linear velocity (Body Frame)')
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


disp('Press any key to continue')
waitforbuttonpress

%% Lidar Sensor 
lidar = rangeSensor;
lidar.Range = [0 2];                 
lidar.HorizontalAngle = [-pi/2, pi/2];
lidar.HorizontalAngleResolution = pi/50;

%% Pure Pursuit Controller
waypoints = ref(1:10:end,1:2);
controller = controllerPurePursuit;
controller.Waypoints = waypoints;
controller.LookaheadDistance = 2;
controller.DesiredLinearVelocity = 0.75;
controller.MaxAngularVelocity = 1.5;

%% VFH
vfh = controllerVFH;
vfh.DistanceLimits = [0.005 2];
vfh.NumAngularSectors = 36;
vfh.HistogramThresholds = [3 8];
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
robot = plot(posePP(1,1),posePP(1,2), 'Color','b','Marker','o','MarkerSize',10);
wayP = plot(waypoints(:,1),waypoints(:,2),'r.','Color','r','Marker','x');
lidarPlot = plot(nan, nan,'r.','MarkerSize', 8);
module = 1;
headX = posePP(1,1) + module * cos(posePP(1,3));
headY = posePP(1,2) + module * sin(posePP(1,3));
heading = plot([posePP(1,1) headX], [posePP(1,2) headY], 'LineStyle','-.','Color','black','LineWidth',2);
headingVFH = plot(nan,nan,'LineStyle','-.','Color','m');
avoidancePath = plot(nan, nan,'r.', 'Color','g');
mpcPath = plot(nan, nan,'r.', 'Color','b');
title('NLMPC + Pure Pursuit Control with VFH Integration (Direct Actuator Control Arch.)');

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
            uPP(idxPose,:) = [wheelSpdsPP(idxPose,:), steerAngFSPP(idxPose,:)];
            velB = forwardKinematics(vehicle,wheelSpdsPP(idxPose,:),steerAngFSPP(idxPose,:));
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
            waitfor(r);

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

    % Compute input u(t)
    [uPP(idxPose,:),~,mpcinfo] = nlmpcmove(nlmpcController, posePP(idxPose-1,:), uPP(idxPose-1,:), trajectory(idxRef:idxRef+p-1,:));
    wheelSpdsPP(idxPose,:) = uPP(idxPose,1:2);
    steerAngFSPP(idxPose,:) = uPP(idxPose,3:4);
    velBody = forwardKinematics(vehicle,uPP(idxPose,1:2),uPP(idxPose,3:4));
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

legend([mpcPath,avoidancePath,wayP,robot], 'MPC Trajectory', 'Pure Pursuit Traj.', 'Waypoints' , 'Robot Position')
hold off;

figure
hold on
subplot(2,3,[1 2 3])
hold on
plot(1:length(steerAngFSPP),steerAngFSPP(:,1),'LineWidth',2);
plot(1:length(steerAngFSPP),steerAngFSPP(:,2),'LineWidth',2);
legend('\phi_f','\phi_r','Upper Limit','Lower limit')
title("Front Steering Angles: \phi_f, \phi_r")
ylabel('[rad]')
xlim([0 length(steerAngFSPP)])
grid on
hold off
hold off

subplot(2,3,[4 5 6])
hold on
plot(1:length(wheelSpdsPP),wheelSpdsPP(:,1),'LineWidth',2,'Color','r');
plot(1:length(wheelSpdsPP),wheelSpdsPP(:,2),'LineWidth',2,'Color','b');
legend('Front Wheel','Rear Wheel','Upper Limit','Lower limit')
ylabel('[rad/s]')
title('Wheel Speeds')
yticks(0:2:10)
xlim([0 length(wheelSpdsPP)])
xlabel('N° of iterations')
grid on
hold off





