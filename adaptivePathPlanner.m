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