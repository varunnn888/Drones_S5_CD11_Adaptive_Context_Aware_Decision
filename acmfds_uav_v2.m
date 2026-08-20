clear;
clc;
close all;

%% ===========================
% Initial Mission Parameters
% ===========================

battery = 85;
windSpeed = 4;
obstacleDistance = 3;
missionPriority = 0.9;

%% ===========================
% Mission Waypoints
% ===========================

missionWaypoints = [
     0   0 -10;
    20   0 -10;
    20  20 -10;
     0  20 -10;
     0   0 -10];

targetIndex = 2;

%% ===========================
% Simulation Parameters
% ===========================

updateRate = 10;
dt = 1/updateRate;
playbackFactor = 2;

%% ===========================
% Initial Context
% ===========================

context = detectContext( ...
    battery,...
    windSpeed,...
    obstacleDistance);

weights = contextManager( ...
    context,...
    missionPriority);

decision = decisionManager(context);

disp("================================")
disp(" ACMFDS UAV SIMULATION")
disp("================================")

fprintf("Initial Context : %s\n",context);
fprintf("Battery         : %.1f %%\n",battery);
fprintf("Wind Speed      : %.1f m/s\n",windSpeed);
fprintf("Mission Priority: %.1f\n",missionPriority);

fprintf("\nAdaptive Weights\n");

fprintf("Energy   : %.2f\n",weights.energy);
fprintf("Safety   : %.2f\n",weights.safety);
fprintf("Time     : %.2f\n",weights.time);
fprintf("Tracking : %.2f\n",weights.tracking);
fprintf("Mission  : %.2f\n",weights.mission);

fprintf("\nDecision\n");

fprintf("Mode     : %s\n",decision.mode);
fprintf("Speed    : %.1f m/s\n",decision.speed);
fprintf("Action   : %s\n",decision.action);

%% ===========================
% UAV Scenario
% ===========================

scene = uavScenario( ...
    UpdateRate=updateRate,...
    StopTime=60);

%% ===========================
% Ground
% ===========================

addMesh(scene,"polygon", ...
    {[-10 -10;
       30 -10;
       30  30;
      -10  30],[0 2]}, ...
    [0.5 0.5 0.5]);

%% ===========================
% Physical Obstacle
% ===========================

addMesh(scene,"polygon", ...
    {[-3 9;
       3 9;
       3 15;
      -3 15],[0 18]}, ...
    [0.8 0.2 0.2]);

%% ===========================
% UAV
% ===========================

orientation = eul2quat([0 0 pi]);

currentPosition = ...
    missionWaypoints(1,:);

uav = uavPlatform( ...
    "UAV",...
    scene,...
    "ReferenceFrame","NED",...
    "InitialPosition",currentPosition,...
    "InitialOrientation",orientation);

updateMesh( ...
    uav,...
    "quadrotor",...
    {4},...
    [0 0.5 1],...
    [0 0 0],...
    orientation);

%% ===========================
% Visualization
% ===========================

ax = show3D(scene);

xlim(ax,[-10 30]);
ylim(ax,[-10 25]);
zlim(ax,[-15 20]);

view(ax,2);

axis(ax,"equal");
hold(ax,"on");

%% ===========================
% Original Mission
% ===========================

plot3(ax,...
    missionWaypoints(:,2),...
    missionWaypoints(:,1),...
    -missionWaypoints(:,3),...
    "k--",...
    "LineWidth",1.5);

plot3(ax,...
    missionWaypoints(:,2),...
    missionWaypoints(:,1),...
    -missionWaypoints(:,3),...
    "ko",...
    "MarkerSize",5,...
    "MarkerFaceColor","k");

%% ===========================
% Setup
% ===========================

setup(scene);

actualPath = currentPosition;

adaptivePlots = gobjects(0);

lastContext = context;

contextHistory = strings(0,1);
timeHistory = [];
batteryHistory = [];
windHistory = [];
obstacleHistory = [];
speedHistory = [];
costHistory = [];

%% ===========================
% Mission Loop
% ===========================

while scene.IsRunning && ...
        targetIndex <= size(missionWaypoints,1)

    %% Current target

    targetPosition = ...
        missionWaypoints(targetIndex,:);

    %% Dynamic environment

    [battery,...
        windSpeed,...
        obstacleDistance,...
        missionPriority] = ...
        dynamicContext( ...
            currentPosition,...
            targetPosition,...
            battery,...
            scene.CurrentTime,...
            dt);

    %% Detect context

    newContext = detectContext( ...
        battery,...
        windSpeed,...
        obstacleDistance);

    %% Context change before planning

    if ~strcmp(newContext,lastContext)

        fprintf("\n============================\n");
        fprintf("CONTEXT CHANGE\n");
        fprintf("Time             : %.2f s\n",scene.CurrentTime);
        fprintf("Battery          : %.2f %%\n",battery);
        fprintf("Wind             : %.2f m/s\n",windSpeed);
        fprintf("Obstacle Distance: %.2f m\n",obstacleDistance);
        fprintf("Mission Priority : %.2f\n",missionPriority);
        fprintf("Previous Context : %s\n",lastContext);
        fprintf("New Context      : %s\n",newContext);
        fprintf("============================\n");

        lastContext = newContext;

    end

    context = newContext;

    %% Adaptive weights

    weights = contextManager( ...
        context,...
        missionPriority);

    %% Adaptive decision

    decision = decisionManager(context);

    %% Check waypoint

    distanceToTarget = ...
        norm(targetPosition-currentPosition);

    if distanceToTarget < 0.4

        currentPosition = targetPosition;

        actualPath = [
            actualPath;
            currentPosition];

        fprintf("\nReached Mission Waypoint %d\n",...
            targetIndex);

        targetIndex = targetIndex + 1;

        continue;

    end

    %% ===========================
    % Adaptive Local Planning
    % ===========================

    [localPath,info] = ...
        adaptivePathPlanner( ...
            currentPosition,...
            targetPosition,...
            context,...
            weights,...
            decision.speed);

    %% Planning information

    fprintf("\nAdaptive Planning\n");
    fprintf("Context : %s\n",context);

    if info.directPathBlocked

        fprintf("Direct Path : BLOCKED\n");

        for i = 1:numel(info.candidateNames)

            fprintf( ...
                "%s | Distance %.2f m | Clearance %.2f m | Cost %.3f\n",...
                info.candidateNames(i),...
                info.candidateDistance(i),...
                info.candidateClearance(i),...
                info.candidates(i));

        end

        fprintf("Selected Detour : %s\n",...
            info.candidateNames(info.bestIndex));

        fprintf("Selected Distance : %.2f m\n",...
            info.distance);

        fprintf("Selected Time : %.2f s\n",...
            info.time);

        fprintf("Selected Clearance : %.2f m\n",...
            info.clearance);

    else

        fprintf("Direct Path : CLEAR\n");

        fprintf("Direct Distance : %.2f m\n",...
            info.distance);

        fprintf("Direct Clearance : %.2f m\n",...
            info.clearance);

    end

    %% ===========================
    % Draw current adaptive path
    % ===========================

    if ~isempty(adaptivePlots)

        delete(adaptivePlots(isgraphics(adaptivePlots)));

    end

    adaptivePlots = plot3( ...
        ax,...
        localPath(:,2),...
        localPath(:,1),...
        -localPath(:,3),...
        "r--",...
        "LineWidth",3);

    %% ===========================
    % Follow local path
    % ===========================

    replanRequired = false;

    for p = 2:size(localPath,1)

        nextPoint = localPath(p,:);

        while true

            %% Dynamic environment update

            [battery,...
                windSpeed,...
                obstacleDistance,...
                missionPriority] = ...
                dynamicContext( ...
                    currentPosition,...
                    targetPosition,...
                    battery,...
                    scene.CurrentTime,...
                    dt);

            updatedContext = detectContext( ...
                battery,...
                windSpeed,...
                obstacleDistance);

            %% Runtime context change

            if ~strcmp(updatedContext,context)

                replanRequired = true;

                context = updatedContext;

                weights = contextManager( ...
                    context,...
                    missionPriority);

                decision = decisionManager(context);

                fprintf("\n*** LIVE CONTEXT UPDATE ***\n");

                fprintf("Time     : %.2f s\n",...
                    scene.CurrentTime);

                fprintf("Context  : %s\n",context);

                fprintf("Battery  : %.2f %%\n",battery);

                fprintf("Wind     : %.2f m/s\n",windSpeed);

                fprintf("Obstacle : %.2f m\n",obstacleDistance);

                fprintf("Speed    : %.1f m/s\n",...
                    decision.speed);

                fprintf("Mode     : %s\n",...
                    decision.mode);

                fprintf("****************************\n");

                break;

            end

            %% Distance to next local point

            segmentVector = ...
                nextPoint-currentPosition;

            segmentDistance = ...
                norm(segmentVector);

            if segmentDistance < 0.4

                break;

            end

            direction = ...
                segmentVector/segmentDistance;

            moveDistance = ...
                min(decision.speed*dt,...
                segmentDistance);

            newPosition = ...
                currentPosition + ...
                direction*moveDistance;

            velocity = ...
                (newPosition-currentPosition)/dt;

            acceleration = [0 0 0];

            angularVelocity = [0 0 0];

            motion = [ ...
                newPosition ...
                velocity ...
                acceleration ...
                orientation ...
                angularVelocity];

            move(uav,motion);

            currentPosition = newPosition;

            actualPath = [
                actualPath;
                currentPosition];

            advance(scene);

            %% Logging

            timeHistory(end+1,1) = ...
                scene.CurrentTime;

            batteryHistory(end+1,1) = ...
                battery;

            windHistory(end+1,1) = ...
                windSpeed;

            obstacleHistory(end+1,1) = ...
                obstacleDistance;

            speedHistory(end+1,1) = ...
                decision.speed;

            contextHistory(end+1,1) = ...
                context;

            if isempty(info.candidates)

                currentCost = 0;

            elseif ~isempty(info.bestIndex)

                currentCost = ...
                    info.candidates(info.bestIndex);

            else

                currentCost = 0;

            end

            costHistory(end+1,1) = ...
                currentCost;

            %% Visualization

            show3D( ...
                scene,...
                "Parent",ax,...
                "FastUpdate",true);

            drawnow;

            pause(dt/playbackFactor);

            end

        %% Stop old path after context change

        if replanRequired

            break;

        end

    end

    %% Outer loop automatically replans

    if replanRequired

        continue;

    end

end

%% ===========================
% Final actual path
% ===========================

plot3( ...
    ax,...
    actualPath(:,2),...
    actualPath(:,1),...
    -actualPath(:,3),...
    "b-",...
    "LineWidth",2);

drawnow;

%% ===========================
% Mission summary
% ===========================

disp("================================")
disp("MISSION COMPLETE")
disp("================================")

fprintf("Final Position : [%.2f %.2f %.2f]\n",...
    currentPosition(1),...
    currentPosition(2),...
    currentPosition(3));

fprintf("Final Battery   : %.2f %%\n",...
    battery);

fprintf("Final Context   : %s\n",...
    context);

fprintf("Simulation Time : %.2f s\n",...
    scene.CurrentTime);

fprintf("Mission Priority: %.2f\n",...
    missionPriority);

fprintf("Recorded Samples: %d\n",...
    numel(timeHistory));

disp("================================")
plotResults( ...
    timeHistory,...
    batteryHistory,...
    windHistory,...
    obstacleHistory,...
    speedHistory,...
    costHistory);
    
save("ACMFDS_MissionData.mat", ...
    "timeHistory", ...
    "batteryHistory", ...
    "windHistory", ...
    "obstacleHistory", ...
    "speedHistory", ...
    "costHistory", ...
    "contextHistory");

function [path,info] = adaptivePathPlanner( ...
    currentPosition,...
    targetPosition,...
    context,...
    weights,...
    speed)

%% ==========================================
% Physical obstacle
% ==========================================

obstacleBounds = [9 15 -3 3];

%% ==========================================
% UAV physical clearance
% ==========================================

vehicleRadius = 2.5;
safetyBuffer = 1.0;

hardClearance = vehicleRadius + safetyBuffer;

%% ==========================================
% Expanded obstacle used for collision check
% ==========================================

northMin = obstacleBounds(1) - hardClearance;
northMax = obstacleBounds(2) + hardClearance;

eastMin = obstacleBounds(3) - hardClearance;
eastMax = obstacleBounds(4) + hardClearance;

%% ==========================================
% Direct path
% ==========================================

directPath = [
    currentPosition;
    targetPosition];

directSamples = samplePath( ...
    directPath,...
    0.1);

directDistance = ...
    sum(vecnorm(diff(directPath,1,1),2,2));

directClearance = ...
    obstacleClearance( ...
        directSamples,...
        obstacleBounds);

directBlocked = ...
    pathBlocked( ...
        directSamples(2:end,:),...
        northMin,...
        northMax,...
        eastMin,...
        eastMax);

%% ==========================================
% Direct path is safe
% ==========================================

if ~directBlocked

    path = directPath;

    candidateCosts = [];
    candidateClearances = [];
    candidateDistances = [];
    candidateNames = strings(0);
    bestIndex = [];
    selectedObjectives = [];

else

    %% ======================================
    % Safe routing offsets
    % ======================================

    eastSafe = eastMax + 1;
    westSafe = eastMin - 1;

    northSafe = northMax + 1;
    southSafe = northMin - 1;

    %% ======================================
    % Candidate 1: EAST
    % ======================================

    eastPath = [
        currentPosition;
        currentPosition(1) eastSafe currentPosition(3);
        targetPosition(1) eastSafe targetPosition(3);
        targetPosition];

    %% ======================================
    % Candidate 2: WEST
    % ======================================

    westPath = [
        currentPosition;
        currentPosition(1) westSafe currentPosition(3);
        targetPosition(1) westSafe targetPosition(3);
        targetPosition];

    %% ======================================
    % Candidate 3: NORTH
    % ======================================

    northPath = [
        currentPosition;
        northSafe currentPosition(2) currentPosition(3);
        northSafe targetPosition(2) targetPosition(3);
        targetPosition];

    %% ======================================
    % Candidate 4: SOUTH
    % ======================================

    southPath = [
        currentPosition;
        southSafe currentPosition(2) currentPosition(3);
        southSafe targetPosition(2) targetPosition(3);
        targetPosition];

    candidates = {
        eastPath
        westPath
        northPath
        southPath
    };

    candidateNames = [
        "EAST"
        "WEST"
        "NORTH"
        "SOUTH"
    ];

    nCandidates = numel(candidates);

    candidateCosts = Inf(1,nCandidates);
    candidateClearances = zeros(1,nCandidates);
    candidateDistances = zeros(1,nCandidates);

    candidateObjectives = ...
        Inf(nCandidates,5);

    feasible = false(1,nCandidates);

    %% ======================================
    % Evaluate candidates
    % ======================================

    for i = 1:nCandidates

        candidate = candidates{i};

        samples = samplePath( ...
            candidate,...
            0.1);

        totalDistance = ...
            sum(vecnorm(diff(candidate,1,1),2,2));

        clearance = ...
            obstacleClearance( ...
                samples,...
                obstacleBounds);

        candidateDistances(i) = ...
            totalDistance;

        candidateClearances(i) = ...
            clearance;

        %% Ignore starting point during collision test

        collision = ...
            pathBlocked( ...
                samples(2:end,:),...
                northMin,...
                northMax,...
                eastMin,...
                eastMax);

        if collision

            continue;

        end

        feasible(i) = true;

        %% ==================================
        % Energy objective
        % ==================================

        extraDistance = ...
            max(0,totalDistance-directDistance);

        energyCost = ...
            min(extraDistance/...
            max(directDistance,1),1);

        %% ==================================
        % Time objective
        % ==================================

        timeCost = ...
            min((totalDistance/max(speed,eps))/30,1);

        %% ==================================
        % Safety objective
        % ==================================

        safetyCost = ...
            max(0,min(1,...
            1-clearance/10));

        %% ==================================
        % Tracking objective
        % ==================================

        trackingError = ...
            meanPointLineDeviation( ...
                samples,...
                currentPosition,...
                targetPosition);

        trackingCost = ...
            min(trackingError/20,1);

        %% ==================================
        % Mission objective
        % ==================================

        missionCost = ...
            min(extraDistance/...
            max(directDistance,1),1);

        %% ==================================
        % Store objectives
        % ==================================

        candidateObjectives(i,:) = [
            energyCost,...
            safetyCost,...
            timeCost,...
            trackingCost,...
            missionCost];

        %% ==================================
        % Multi-objective cost
        % ==================================

        candidateCosts(i) = ...
            weights.energy * energyCost + ...
            weights.safety * safetyCost + ...
            weights.time * timeCost + ...
            weights.tracking * trackingCost + ...
            weights.mission * missionCost;

    end

    %% ======================================
    % Select best feasible candidate
    % ======================================

    if ~any(feasible)

        error( ...
            "No feasible detour found. Check obstacle geometry.");

    end

    validCosts = candidateCosts;

    validCosts(~feasible) = Inf;

    [~,bestIndex] = ...
        min(validCosts);

    path = candidates{bestIndex};

    selectedObjectives = ...
        candidateObjectives(bestIndex,:);

end

%% ==========================================
% Planner information
% ==========================================

info.distance = ...
    sum(vecnorm(diff(path,1,1),2,2));

info.time = ...
    info.distance/max(speed,eps);

info.clearance = ...
    obstacleClearance( ...
        samplePath(path,0.1),...
        obstacleBounds);

info.directClearance = ...
    directClearance;

info.directPathBlocked = ...
    directBlocked;

info.candidates = ...
    candidateCosts;

info.candidateClearance = ...
    candidateClearances;

info.candidateDistance = ...
    candidateDistances;

info.candidateNames = ...
    candidateNames;

info.bestIndex = ...
    bestIndex;

info.selectedObjectives = ...
    selectedObjectives;

info.requiredClearance = ...
    hardClearance;

info.context = ...
    context;

end


%% ==========================================
% Sample path
% ==========================================

function sampledPath = samplePath(path,step)

sampledPath = path(1,:);

for i = 1:size(path,1)-1

    p1 = path(i,:);
    p2 = path(i+1,:);

    distance = norm(p2-p1);

    n = max(2,...
        ceil(distance/step)+1);

    for j = 2:n

        alpha = ...
            (j-1)/(n-1);

        point = ...
            p1 + alpha*(p2-p1);

        sampledPath = [
            sampledPath;
            point];

    end

end

end


%% ==========================================
% Physical obstacle clearance
% ==========================================

function clearance = obstacleClearance(points,bounds)

northMin = bounds(1);
northMax = bounds(2);

eastMin = bounds(3);
eastMax = bounds(4);

north = points(:,1);
east = points(:,2);

dn = max([ ...
    northMin-north,...
    zeros(size(north)),...
    north-northMax],[],2);

de = max([ ...
    eastMin-east,...
    zeros(size(east)),...
    east-eastMax],[],2);

distance = ...
    sqrt(dn.^2 + de.^2);

clearance = min(distance);

end


%% ==========================================
% Collision test
% ==========================================

function blocked = pathBlocked( ...
    points,...
    northMin,...
    northMax,...
    eastMin,...
    eastMax)

north = points(:,1);
east = points(:,2);

inside = ...
    north > northMin & ...
    north < northMax & ...
    east > eastMin & ...
    east < eastMax;

blocked = any(inside);

end


%% ==========================================
% Tracking deviation
% ==========================================

function deviation = meanPointLineDeviation( ...
    points,...
    startPoint,...
    endPoint)

start2D = startPoint(1:2);
end2D = endPoint(1:2);

lineVector = end2D-start2D;

lineLength = norm(lineVector);

if lineLength < 1e-9

    deviation = 0;

    return;

end

lineUnit = lineVector/lineLength;

relativePoints = ...
    points(:,1:2)-start2D;

projection = ...
    relativePoints*lineUnit';

closestPoints = ...
    start2D + projection*lineUnit;

distanceVector = ...
    points(:,1:2)-closestPoints;

deviation = ...
    mean(sqrt(sum(distanceVector.^2,2)));

end