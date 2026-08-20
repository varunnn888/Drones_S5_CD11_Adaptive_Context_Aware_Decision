function [path,info] = multiObstaclePlanner( ...
    currentPosition,...
    targetPosition,...
    obstacles,...
    weights,...
    speed)

vehicleClearance = 1.0;

start = currentPosition(1:2);
target = targetPosition(1:2);

%% Direct path
directPath = [
    start;
    target
    ];

directSamples = sampleWholePath(directPath,0.1);

directBlocked = pathBlocked( ...
    directSamples,...
    obstacles,...
    vehicleClearance);

directDistance = sum( ...
    vecnorm(diff(directPath,1,1),2,2));

directClearance = pathClearance( ...
    directSamples,...
    obstacles);

if ~directBlocked

    path = [
        currentPosition;
        targetPosition
        ];

    info.directBlocked = false;
    info.distance = directDistance;
    info.time = directDistance/max(speed,eps);
    info.clearance = directClearance;
    info.bestIndex = [];
    info.candidateNames = strings(0,1);
    info.candidateCosts = [];
    info.candidateDistance = [];
    info.candidateClearance = [];

    return;

end

%% Obstacle envelope
northMin = min(obstacles(:,1));
northMax = max(obstacles(:,2));

eastMin = min(obstacles(:,3));
eastMax = max(obstacles(:,4));

margin = vehicleClearance + 0.5;

northRoute = northMax + margin;
southRoute = northMin - margin;

%% Candidate routes

northCandidate = [
    start;
    northRoute start(2);
    northRoute target(2);
    target
    ];

southCandidate = [
    start;
    southRoute start(2);
    southRoute target(2);
    target
    ];

candidates = {
    northCandidate
    southCandidate
    };

names = [
    "NORTH"
    "SOUTH"
    ];

costs = Inf(2,1);
distances = Inf(2,1);
clearances = zeros(2,1);

%% Evaluate candidates

for i = 1:2

    candidate = candidates{i};

    samples = sampleWholePath( ...
        candidate,...
        0.1);

    blocked = pathBlocked( ...
        samples,...
        obstacles,...
        vehicleClearance);

    if blocked
        continue;
    end

    d = sum( ...
        vecnorm(diff(candidate,1,1),2,2));

    c = pathClearance( ...
        samples,...
        obstacles);

    timeObjective = ...
        min((d/max(speed,eps))/30,1);

    energyObjective = ...
        min(d/40,1);

    safetyObjective = ...
        max(0,min(1,1-c/10));

    trackingObjective = ...
        min(meanDeviation( ...
        samples,...
        start,...
        target)/20,1);

    missionObjective = ...
        min(d/40,1);

    costs(i) = ...
        weights.energy * energyObjective + ...
        weights.safety * safetyObjective + ...
        weights.time * timeObjective + ...
        weights.tracking * trackingObjective + ...
        weights.mission * missionObjective;

    distances(i) = d;
    clearances(i) = c;

end

%% Select best feasible route

[bestCost,bestIndex] = min(costs);

if isinf(bestCost)

    error("No feasible detour exists for the current obstacle configuration.");

end

selected = candidates{bestIndex};

path = zeros(size(selected,1),3);

path(:,1) = selected(:,1);
path(:,2) = selected(:,2);
path(:,3) = currentPosition(3);

info.directBlocked = true;
info.distance = distances(bestIndex);
info.time = distances(bestIndex)/max(speed,eps);
info.clearance = clearances(bestIndex);
info.bestIndex = bestIndex;
info.candidateNames = names;
info.candidateCosts = costs;
info.candidateDistance = distances;
info.candidateClearance = clearances;

end


function samples = sampleWholePath(path,step)

samples = path(1,:);

for i = 1:size(path,1)-1

    p1 = path(i,:);
    p2 = path(i+1,:);

    d = norm(p2-p1);

    n = max(2,ceil(d/step)+1);

    alpha = linspace(0,1,n)';

    segment = ...
        p1 + alpha.*(p2-p1);

    samples = [
        samples;
        segment(2:end,:)
        ];

end

end


function blocked = pathBlocked( ...
    points,...
    obstacles,...
    clearance)

blocked = false;

for i = 1:size(obstacles,1)

    b = obstacles(i,:);

    nMin = b(1) - clearance;
    nMax = b(2) + clearance;

    eMin = b(3) - clearance;
    eMax = b(4) + clearance;

    inside = ...
        points(:,1) >= nMin & ...
        points(:,1) <= nMax & ...
        points(:,2) >= eMin & ...
        points(:,2) <= eMax;

    if any(inside)

        blocked = true;

        return;

    end

end

end


function clearance = pathClearance(points,obstacles)

clearance = Inf;

for i = 1:size(obstacles,1)

    b = obstacles(i,:);

    dn = max([
        b(1)-points(:,1),...
        zeros(size(points,1),1),...
        points(:,1)-b(2)],...
        [],2);

    de = max([
        b(3)-points(:,2),...
        zeros(size(points,1),1),...
        points(:,2)-b(4)],...
        [],2);

    d = sqrt(dn.^2 + de.^2);

    clearance = min(clearance,min(d));

end

end


function deviation = meanDeviation( ...
    points,...
    start,...
    target)

v = target-start;

L = norm(v);

if L < eps

    deviation = 0;

    return;

end

u = v/L;

relative = points-start;

projection = relative*u';

closest = ...
    start + projection.*u;

err = points-closest;

deviation = ...
    mean(sqrt(sum(err.^2,2)));

end