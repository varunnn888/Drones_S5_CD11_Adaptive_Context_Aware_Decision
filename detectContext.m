function context = detectContext(battery, windSpeed, obstacleDistance)

if battery < 20
    context = "Low Battery";

elseif obstacleDistance < 5
    context = "Obstacle Nearby";

elseif windSpeed > 10
    context = "High Wind";

else
    context = "Normal";

end

end