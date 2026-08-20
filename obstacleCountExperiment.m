clear;
clc;
close all;

%% ===========================
% Mission
% ===========================

start = [0 0 -10];
target = [0 25 -10];

%% ===========================
% Obstacles added progressively
% [NorthMin NorthMax EastMin EastMax]
% ===========================

allObstacles = [
    -3   3    5    8;
     4   8   11   14;
    -8  -4   17   20
    ];

%% ===========================
% Fixed Context
% ===========================

battery = 85;
windSpeed = 4;
missionPriority = 0.9;

context = detectContext( ...
    battery,...
    windSpeed,...
    3);

weights = contextManager( ...
    context,...
    missionPriority);

decision = decisionManager(context);

%% ===========================
% Results
% ===========================

numberOfObstacles = 0:3;

distanceResults = zeros(4,1);
timeResults = zeros(4,1);
clearanceResults = zeros(4,1);
detourUsed = false(4,1);

figure( ...
    "Name","UAV Behaviour vs Number of Obstacles",...
    "Color","w");

%% ===========================
% Progressive obstacle experiment
% ===========================

for count = 0:3

    currentObstacles = allObstacles(1:count,:);

    currentPosition = start;

    %% Plan

    [path,info] = multiObstaclePlanner( ...
        currentPosition,...
        target,...
        currentObstacles,...
        weights,...
        decision.speed);

    %% Metrics

    distance = sum( ...
        vecnorm(diff(path(:,1:2),1,1),2,2));

    flightTime = ...
        distance/max(decision.speed,eps);

    distanceResults(count+1) = distance;

    timeResults(count+1) = flightTime;

    clearanceResults(count+1) = info.clearance;

    detourUsed(count+1) = info.directBlocked;

    %% ===========================
    % Plot
    % ===========================

    subplot(2,2,count+1);

    hold on;
    grid on;
    axis equal;

    xlabel("East (m)");
    ylabel("North (m)");

    title(sprintf( ...
        "%d Obstacle%s",...
        count,...
        ternary(count==1,"","s")));

    %% Obstacles

    for i = 1:size(currentObstacles,1)

        b = currentObstacles(i,:);

        rectangle( ...
            "Position",[ ...
            b(3),...
            b(1),...
            b(4)-b(3),...
            b(2)-b(1)],...
            "FaceColor",[0.8 0.2 0.2],...
            "EdgeColor","k");

        text( ...
            mean([b(3) b(4)]),...
            mean([b(1) b(2)]),...
            sprintf("O%d",i),...
            "HorizontalAlignment","center");

    end

    %% Direct path

    plot( ...
        [start(2) target(2)],...
        [start(1) target(1)],...
        "k--",...
        "LineWidth",1.2);

    %% Planned path

    plot( ...
        path(:,2),...
        path(:,1),...
        "b-",...
        "LineWidth",2.5);

    %% Start

    plot( ...
        start(2),...
        start(1),...
        "ko",...
        "MarkerFaceColor","g",...
        "MarkerSize",7);

    %% Target

    plot( ...
        target(2),...
        target(1),...
        "ko",...
        "MarkerFaceColor","b",...
        "MarkerSize",7);

    %% Information

    if info.directBlocked

        text( ...
            0.02,...
            0.95,...
            sprintf( ...
            "Detour\nDistance: %.2f m\nTime: %.2f s\nClearance: %.2f m",...
            distance,...
            flightTime,...
            info.clearance),...
            "Units","normalized",...
            "VerticalAlignment","top");

    else

        text( ...
            0.02,...
            0.95,...
            sprintf( ...
            "Direct Path\nDistance: %.2f m\nTime: %.2f s",...
            distance,...
            flightTime),...
            "Units","normalized",...
            "VerticalAlignment","top");

    end

    hold off;

end

%% ===========================
% Summary
% ===========================

figure( ...
    "Name","UAV Behaviour Metrics",...
    "Color","w");

subplot(1,3,1);

plot( ...
    numberOfObstacles,...
    distanceResults,...
    "o-",...
    "LineWidth",2);

grid on;

xlabel("Number of Obstacles");
ylabel("Distance (m)");

title("Flight Distance");

subplot(1,3,2);

plot( ...
    numberOfObstacles,...
    timeResults,...
    "o-",...
    "LineWidth",2);

grid on;

xlabel("Number of Obstacles");
ylabel("Time (s)");

title("Flight Time");

subplot(1,3,3);

plot( ...
    numberOfObstacles,...
    clearanceResults,...
    "o-",...
    "LineWidth",2);

grid on;

xlabel("Number of Obstacles");
ylabel("Minimum Clearance (m)");

title("Safety Clearance");

%% ===========================
% Command Window
% ===========================

fprintf("\n============================================\n");
fprintf("OBSTACLE COUNT EXPERIMENT\n");
fprintf("============================================\n");

fprintf( ...
    "%-10s %-12s %-12s %-15s %-12s\n",...
    "Obstacles",...
    "Distance",...
    "Time",...
    "Clearance",...
    "Behavior");

fprintf("--------------------------------------------\n");

for i = 1:4

    if detourUsed(i)

        behavior = "Detour";

    else

        behavior = "Direct";

    end

    fprintf( ...
        "%-10d %-12.2f %-12.2f %-15.2f %-12s\n",...
        numberOfObstacles(i),...
        distanceResults(i),...
        timeResults(i),...
        clearanceResults(i),...
        behavior);

end

fprintf("============================================\n");


function result = ternary(condition,a,b)

if condition

    result = a;

else

    result = b;

end

end