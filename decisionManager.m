function decision = decisionManager(context)

switch context

    case "Normal"

        decision.speed  = 10;
        decision.mode   = "Normal Flight";
        decision.action = "Continue Mission";

    case "High Wind"

        decision.speed  = 7;
        decision.mode   = "Safety Mode";
        decision.action = "Reduce Speed";

    case "Low Battery"

        decision.speed  = 8;
        decision.mode   = "Energy Saving";
        decision.action = "Return to Home";

    case "Obstacle Nearby"

        decision.speed  = 5;
        decision.mode   = "Obstacle Avoidance";
        decision.action = "Avoid Obstacle";

    otherwise

        decision.speed  = 10;
        decision.mode   = "Unknown";
        decision.action = "Continue";

end

end