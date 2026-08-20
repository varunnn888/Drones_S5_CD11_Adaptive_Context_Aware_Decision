function weights = contextManager(context,missionPriority)

missionPriority = max(0,min(1,missionPriority));

switch context

    case "Normal"

        weights.energy   = 0.20;
        weights.safety   = 0.20;
        weights.time     = 0.20;
        weights.tracking = 0.15;
        weights.mission  = 0.25;

    case "Low Battery"

        weights.energy   = 0.45;
        weights.safety   = 0.20;
        weights.time     = 0.10;
        weights.tracking = 0.10;
        weights.mission  = 0.15;

    case "High Wind"

        weights.energy   = 0.15;
        weights.safety   = 0.40;
        weights.time     = 0.10;
        weights.tracking = 0.15;
        weights.mission  = 0.20;

    case "Obstacle Nearby"

        weights.energy   = 0.10;
        weights.safety   = 0.55;
        weights.time     = 0.05;
        weights.tracking = 0.15;
        weights.mission  = 0.15;

    otherwise

        weights.energy   = 0.20;
        weights.safety   = 0.20;
        weights.time     = 0.20;
        weights.tracking = 0.15;
        weights.mission  = 0.25;

end

weights.mission = ...
    weights.mission * (0.5 + missionPriority);

totalWeight = ...
    weights.energy + ...
    weights.safety + ...
    weights.time + ...
    weights.tracking + ...
    weights.mission;

weights.energy   = weights.energy / totalWeight;
weights.safety   = weights.safety / totalWeight;
weights.time     = weights.time / totalWeight;
weights.tracking = weights.tracking / totalWeight;
weights.mission  = weights.mission / totalWeight;

end