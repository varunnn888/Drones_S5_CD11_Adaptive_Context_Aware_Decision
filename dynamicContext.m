function [battery,windSpeed,obstacleDistance,missionPriority] = ...
    dynamicContext(currentPosition,targetPosition,...
    battery,simulationTime,dt)

battery = max(0,battery - 0.03*dt);

if simulationTime < 3

    windSpeed = 4;

elseif simulationTime < 6

    windSpeed = 12;

elseif simulationTime < 9

    windSpeed = 4;

else

    windSpeed = 8;

end

obstacleBounds = [9 15 -3 3];

north = currentPosition(1);
east = currentPosition(2);

dn = max([ ...
    obstacleBounds(1)-north,...
    0,...
    north-obstacleBounds(2)]);

de = max([ ...
    obstacleBounds(3)-east,...
    0,...
    east-obstacleBounds(4)]);

obstacleDistance = sqrt(dn^2 + de^2);

remainingDistance = ...
    norm(targetPosition-currentPosition);

if remainingDistance > 15

    missionPriority = 0.7;

elseif remainingDistance > 5

    missionPriority = 0.9;

else

    missionPriority = 1.0;

end

end