function ACMFDS_Scientific
% ACMFDS_Scientific
% Adaptive Context-Aware Multi-Objective Flight Decision System
    close all force;
    rng(448,'twister');

    %% Simulation constants and state
    app = struct();
    app.W = 60; app.H = 40;
    app.start = [3 3];
    app.goal = [58 38];
    app.position = app.start;
    app.path = [];
    app.obstacles = struct([]);
    app.running = false;
    app.stopRequested = false;
    app.mode = 'READY';
    app.simTime = 0;
    app.batteryNow = 80;
    app.distanceTravelled = 0;
    app.minClearance = inf;
    app.safetyRejections = 0;
    app.replanCount = 0;
    app.pathIndex = 2;
    app.target = app.goal;
    app.previousHeading = [1 0];
    app.trail = app.start;
    app.seedCounter = 0;
    app.lastPlanTime = 0;
    app.noPathTime = 0;
    app.missionSuccess = false;
    app.safeReturn = false;
    app.lastReason = 'Generate a world, then plan or run the mission.';
    app.compareHandles = gobjects(0);
    app.speedMode = 'NORMAL';
    app.lastAlert = '';
    app.safeZones = zeros(0,2);
    app.nextDecisionTime = 0;
    app.forceReplan = false;
    app.context = 'NORMAL';
    app.lastContextForPlanning = 'NORMAL';
    app.obstacleDistance = inf;
    app.missionPriority = 0.90;  % Fixed for the duration of a mission run.

    % Published generic multirotor reference parameters from Gong et al. (2022),
    % arXiv:2209.04128, Table I and parameter statement near the end of the paper.
    % W=20 N is converted to mass; n=4 is one of their considered configurations.
    % Battery energy and nominal/boost speeds are explicit scenario inputs, NOT
    % values supplied by that paper; edit them through the VEHICLE MODEL dialog.
    app.model = struct( ...
        'massKg',20/9.81, ...
        'rotorCount',4, ...
        'airDensity',1.168, ...
        'profileDragCoeff',0.011, ...
        'rotorSolidity',0.045, ...
        'rotorAreaM2',0.214, ...
        'thrustCoeff',0.001195, ...
        'v0Mps',6.325, ...
        'inducedCorrection',0.11, ...
        'fuselageAreaM2',0.009, ...
        'batteryCapacityWh',60, ...
        'cruiseAirspeed',5, ...
        'boostAirspeed',10, ...
        'maxAirspeed',15, ...
        'reserveSOC',20, ...
        'lowBatteryTriggerSOC',30, ...
        'bodyRadiusM',0.35);

    % Theme
    C.bg = [0.035 0.050 0.075];
    C.panel = [0.065 0.085 0.120];
    C.panel2 = [0.085 0.110 0.150];
    C.text = [0.88 0.92 0.98];
    C.muted = [0.49 0.60 0.72];
    C.cyan = [0.10 0.88 1.00];
    C.green = [0.18 0.95 0.53];
    C.yellow = [1.00 0.83 0.22];
    C.orange = [1.00 0.55 0.16];
    C.red = [1.00 0.24 0.27];
    C.purple = [0.72 0.48 1.00];

    %% Main application window
    app.fig = figure('Name','ACMFDS | Drone Navigation Lab', ...
        'NumberTitle','off','Color',C.bg,'MenuBar','none','ToolBar','none', ...
        'Position',[35 45 1540 880],'Resize','on', ...
        'CloseRequestFcn',@onClose);

    % Top brand bar
    uicontrol(app.fig,'Style','text','Units','normalized', ...
        'Position',[0.015 0.948 0.52 0.042], 'String','ACMFDS  /  DRONE NAVIGATION LAB', ...
        'FontSize',16,'FontWeight','bold','HorizontalAlignment','left', ...
        'ForegroundColor',C.cyan,'BackgroundColor',C.bg);
    uicontrol(app.fig,'Style','text','Units','normalized', ...
        'Position',[0.54 0.951 0.445 0.035], ...
        'String','5-OBJECTIVE ADAPTIVE PLANNING  •  FIXED MISSION PRIORITY 0.90', ...
        'FontSize',9,'HorizontalAlignment','right', ...
        'ForegroundColor',C.muted,'BackgroundColor',C.bg);

    %% Left control panel
    app.leftPanel = uipanel('Parent',app.fig,'Units','normalized', ...
        'Position',[0.015 0.025 0.225 0.91], 'BackgroundColor',C.panel, ...
        'ForegroundColor',C.cyan,'Title','MISSION CONTROL', ...
        'FontSize',11,'FontWeight','bold');
    uicontrol(app.leftPanel,'Style','text','Units','normalized', ...
        'Position',[0.05 0.925 0.90 0.035], 'String','ADJUST THE SCENARIO', ...
        'FontSize',10,'FontWeight','bold','HorizontalAlignment','left', ...
        'ForegroundColor',C.text,'BackgroundColor',C.panel);

    app.obsLabel = addLabel(app.leftPanel,'Static + moving obstacles: 16',0.875);
    app.obsSlider = addSlider(app.leftPanel,[0.06 0.842 0.88 0.025],0,40,16,40,@onSettingChanged);
    app.movingLabel = addLabel(app.leftPanel,'Moving obstacles: 4',0.805);
    app.movingSlider = addSlider(app.leftPanel,[0.06 0.772 0.88 0.025],0,12,4,12,@onSettingChanged);
    app.windLabel = addLabel(app.leftPanel,'Wind speed: 4.0 m/s',0.735);
    app.windSlider = addSlider(app.leftPanel,[0.06 0.702 0.88 0.025],0,15,4,15,@onSettingChanged);
    app.dirLabel = addLabel(app.leftPanel,'Wind direction: 45° toward',0.665);
    app.dirSlider = addSlider(app.leftPanel,[0.06 0.632 0.88 0.025],0,359,45,359,@onSettingChanged);
    app.batteryLabel = addLabel(app.leftPanel,'Initial battery: 80%',0.595);
    app.batterySlider = addSlider(app.leftPanel,[0.06 0.562 0.88 0.025],10,100,80,90,@onSettingChanged);
    app.deadlineLabel = addLabel(app.leftPanel,'Mission deadline: 90 s',0.525);
    app.deadlineSlider = addSlider(app.leftPanel,[0.06 0.492 0.88 0.025],15,240,90,225,@onSettingChanged);
    app.safetyLabel = addLabel(app.leftPanel,'Safety margin: 1.0 m',0.455);
    app.safetySlider = addSlider(app.leftPanel,[0.06 0.422 0.88 0.025],0.5,2.5,1.0,2.0,@onSettingChanged);

    uicontrol(app.leftPanel,'Style','text','Units','normalized', ...
        'Position',[0.06 0.382 0.88 0.027], 'String','PLANNING MODE', ...
        'FontSize',9,'FontWeight','bold','HorizontalAlignment','left', ...
        'ForegroundColor',C.muted,'BackgroundColor',C.panel);
    app.strategyPopup = uicontrol(app.leftPanel,'Style','popupmenu', ...
        'Units','normalized','Position',[0.06 0.345 0.88 0.04], ...
        'String',{'ACMFDS Adaptive','Shortest Safe Route', ...
        'Fastest Feasible Route','Energy Efficient Route'}, ...
        'Value',1,'FontSize',10,'BackgroundColor',[0.12 0.15 0.20], ...
        'ForegroundColor',C.text,'Callback',@onStrategyChanged);
    app.randomOnRunCheck = uicontrol(app.leftPanel,'Style','checkbox', ...
        'Units','normalized','Position',[0.06 0.309 0.89 0.027], ...
        'String','Generate a new world before Run', 'Value',0, ...
        'FontSize',9,'ForegroundColor',C.text,'BackgroundColor',C.panel);

    % Actions: two rows plus compare
    app.newButton = addButton(app.leftPanel,[0.055 0.245 0.43 0.048], ...
        'NEW WORLD',C.cyan,C.bg,@onGenerate);
    app.planButton = addButton(app.leftPanel,[0.515 0.245 0.43 0.048], ...
        'REPLAN',C.purple,[1 1 1],@onPlan);
    app.runButton = addButton(app.leftPanel,[0.055 0.178 0.43 0.052], ...
        'RUN MISSION',C.green,[0.02 0.09 0.05],@onRun);
    app.stopButton = addButton(app.leftPanel,[0.515 0.178 0.43 0.052], ...
        'STOP',C.red,[1 1 1],@onStop);
    app.compareButton = addButton(app.leftPanel,[0.055 0.112 0.89 0.047], ...
        'COMPARE ROUTES IN SAME WORLD',C.panel2,C.text,@onCompare);
    app.modelButton = addButton(app.leftPanel,[0.055 0.047 0.89 0.043], ...
        'VEHICLE / MISSION PARAMETERS',C.panel2,C.text,@onModelParameters);

    app.inputControls = [app.obsSlider app.movingSlider app.windSlider app.dirSlider ...
        app.batterySlider app.deadlineSlider app.safetySlider app.strategyPopup ...
        app.randomOnRunCheck app.modelButton];

    %% Central game map
    app.ax = axes('Parent',app.fig,'Units','normalized', ...
        'Position',[0.255 0.095 0.485 0.82], 'Color',C.bg);
    set(app.ax,'YDir','normal','XLim',[0.5 app.W+0.5], ...
        'YLim',[0.5 app.H+0.5],'XTick',5:5:60,'YTick',5:5:40, ...
        'XColor',[0.65 0.73 0.83],'YColor',[0.65 0.73 0.83], ...
        'GridColor',[0.26 0.34 0.44],'GridAlpha',0.25, ...
        'FontSize',8,'Layer','top');
    xlabel(app.ax,'EAST / X (m)','Color',C.text,'FontWeight','bold');
    ylabel(app.ax,'NORTH / Y (m)','Color',C.text,'FontWeight','bold');
    title(app.ax,'LIVE WORLD  /  TOP-DOWN FLIGHT LAYER', ...
        'Color',C.cyan,'FontSize',12,'FontWeight','bold');

    %% Right telemetry panel
    app.rightPanel = uipanel('Parent',app.fig,'Units','normalized', ...
        'Position',[0.755 0.025 0.23 0.91], 'BackgroundColor',C.panel, ...
        'ForegroundColor',C.cyan,'Title','FLIGHT TELEMETRY', ...
        'FontSize',11,'FontWeight','bold');
    addSectionTitle(app.rightPanel,'MISSION STATUS',0.925,C);
    app.statusText = addInfoText(app.rightPanel,[0.045 0.79 0.91 0.12],C,9.2);
    addSectionTitle(app.rightPanel,'PERFORMANCE ESTIMATE',0.755,C);
    app.metricsText = addInfoText(app.rightPanel,[0.045 0.565 0.91 0.18],C,9.0);
    addSectionTitle(app.rightPanel,'WHY THIS ROUTE?',0.525,C);
    app.decisionText = addInfoText(app.rightPanel,[0.045 0.315 0.91 0.20],C,8.8);
    addSectionTitle(app.rightPanel,'ROUTE COMPARISON',0.275,C);
    app.compareText = addInfoText(app.rightPanel,[0.045 0.055 0.91 0.205],C,8.0);

    %% Start initial generated environment
    app.previousN = round(get(app.obsSlider,'Value'));
    app.previousMoving = round(get(app.movingSlider,'Value'));
    createNewWorld();

    %% ======================= NESTED CALLBACKS =======================
    function h = addLabel(parent,str,y)
        h = uicontrol(parent,'Style','text','Units','normalized', ...
            'Position',[0.06 y 0.88 0.027],'String',str,'FontSize',9.5, ...
            'HorizontalAlignment','left','ForegroundColor',C.text, ...
            'BackgroundColor',C.panel);
    end

    function h = addSlider(parent,pos,mn,mx,val,bigStep,callback) %#ok<INUSD>
        h = uicontrol(parent,'Style','slider','Units','normalized', ...
            'Position',pos,'Min',mn,'Max',mx,'Value',val, ...
            'SliderStep',[max(0.01,1/(mx-mn)) 0.12], ...
            'BackgroundColor',[0.20 0.28 0.37],'Callback',callback);
    end

    function h = addButton(parent,pos,str,bg,fg,callback)
        h = uicontrol(parent,'Style','pushbutton','Units','normalized', ...
            'Position',pos,'String',str,'FontSize',8.8,'FontWeight','bold', ...
            'BackgroundColor',bg,'ForegroundColor',fg,'Callback',callback);
    end

    function addSectionTitle(parent,str,y,theme)
        uicontrol(parent,'Style','text','Units','normalized', ...
            'Position',[0.045 y 0.91 0.028],'String',str, ...
            'FontSize',9.5,'FontWeight','bold','HorizontalAlignment','left', ...
            'ForegroundColor',theme.cyan,'BackgroundColor',theme.panel);
    end

    function h = addInfoText(parent,pos,theme,fontSize)
        h = uicontrol(parent,'Style','text','Units','normalized', ...
            'Position',pos,'String','Waiting for scenario...', ...
            'FontName','Consolas','FontSize',fontSize, ...
            'HorizontalAlignment','left', ...
            'ForegroundColor',theme.text,'BackgroundColor',theme.panel2, ...
            'Max',2,'Min',0);
    end

    function cfg = readConfig()
        cfg.numObstacles = round(get(app.obsSlider,'Value'));
        cfg.movingCount = min(cfg.numObstacles,round(get(app.movingSlider,'Value')));
        cfg.windSpeed = get(app.windSlider,'Value');
        cfg.windDirection = round(get(app.dirSlider,'Value'));
        cfg.initialBattery = round(get(app.batterySlider,'Value'));
        cfg.timeLimit = round(get(app.deadlineSlider,'Value'));
        cfg.safetyMargin = get(app.safetySlider,'Value');
        names = get(app.strategyPopup,'String');
        cfg.strategy = names{get(app.strategyPopup,'Value')};
        cfg.randomizeOnRun = logical(get(app.randomOnRunCheck,'Value'));
        cfg.model = app.model;
        cfg.airspeedMps = app.model.cruiseAirspeed;
        cfg.boostAirspeedMps = app.model.boostAirspeed;
        cfg.reserveSOC = app.model.reserveSOC;
        cfg.missionPriority = app.missionPriority;
        cfg.contextWindThreshold = 10;       % m/s, transparent policy threshold.
        cfg.contextObstacleThreshold = 5;     % m surface clearance from UAV body.
        cfg.contextBatteryThreshold = app.model.lowBatteryTriggerSOC;
    end

    function updateLabels()
        cfg = readConfig();
        set(app.obsLabel,'String',sprintf('Static + moving obstacles: %d',cfg.numObstacles));
        set(app.movingLabel,'String',sprintf('Moving obstacles: %d',cfg.movingCount));
        set(app.windLabel,'String',sprintf('Wind speed: %.1f m/s',cfg.windSpeed));
        set(app.dirLabel,'String',sprintf('Wind direction: %d° toward',cfg.windDirection));
        set(app.batteryLabel,'String',sprintf('Initial battery: %d%%',cfg.initialBattery));
        set(app.deadlineLabel,'String',sprintf('Mission deadline: %d s',cfg.timeLimit));
        set(app.safetyLabel,'String',sprintf('Safety margin: %.1f m',cfg.safetyMargin));
    end

    function onSettingChanged(~,~)
        updateLabels();
        if app.running
            % Wind speed/direction remain operator-adjustable during a mission.
            % Force the decision layer to recompute route feasibility and path cost.
            app.forceReplan = true;
            app.nextDecisionTime = app.simTime;
            app.lastPlanTime = -inf;
            app.lastReason = 'Operator changed live wind conditions; route feasibility, ground speed and energy estimate are being recomputed.';
            return;
        end
        cfg = readConfig();
        if cfg.numObstacles ~= app.previousN || cfg.movingCount ~= app.previousMoving
            createNewWorld();
            return;
        end
        resetMissionState(cfg);
        app.obstacles = updateObstaclePositions(app.obstacles,0);
        app.safeZones = computeSafeZones(app.obstacles,cfg);
        [app.path,ok] = planRoute(app.start,app.goal,app.obstacles,cfg,cfg.initialBattery,cfg.timeLimit);
        app.pathIndex = min(2,size(app.path,1));
        if ok
            app.lastReason = decisionReason(cfg,pathMetrics(app.path,cfg,app.obstacles),C);
            app.mode = 'READY / REPLANNED';
        else
            app.lastReason = 'No safe path with these settings. Reduce obstacles or safety margin, or regenerate.';
            app.mode = 'NO SAFE PATH';
        end
        drawWorld();
        updateTelemetry(cfg,ok);
    end

    function onStrategyChanged(~,~)
        if app.running, return; end
        updateLabels();
        cfg = readConfig();
        resetMissionState(cfg);
        app.obstacles = updateObstaclePositions(app.obstacles,0);
        app.safeZones = computeSafeZones(app.obstacles,cfg);
        [app.path,ok] = planRoute(app.start,app.goal,app.obstacles,cfg,cfg.initialBattery,cfg.timeLimit);
        app.pathIndex = min(2,size(app.path,1));
        if ok
            app.lastReason = decisionReason(cfg,pathMetrics(app.path,cfg,app.obstacles),C);
            app.mode = 'READY / STRATEGY UPDATED';
        else
            app.lastReason = 'No safe path for current strategy/settings.';
            app.mode = 'NO SAFE PATH';
        end
        drawWorld();
        updateTelemetry(cfg,ok);
    end

    function onModelParameters(~,~)
        if app.running, return; end
        m=app.model;
        prompts={ ...
            'Total mass [kg] (paper reference W=20 N)', ...
            'Rotor count [even integer 4..12]', ...
            'Air density rho [kg/m^3]', ...
            'Rotor profile drag coefficient delta [-]', ...
            'Rotor solidity s [-]', ...
            'Rotor disk area per rotor A [m^2]', ...
            'Rotor thrust coefficient CT [-]', ...
            'Reference induced velocity v0 [m/s] (paper Table I: 6.325)', ...
            'Induced-power correction k [-]', ...
            'Equivalent flat-plate area S_FP_parallel [m^2]', ...
            'Nominal battery energy [Wh] = nominal V x Ah', ...
            'Normal target airspeed [m/s]', ...
            'Urgent/boost target airspeed [m/s]', ...
            'Model maximum airspeed [m/s]', ...
            'Protected landing/return reserve [SOC %]', ...
            'Low-battery recovery trigger [SOC %]', ...
            'Drone horizontal body radius [m]', ...
            'Mission priority [0..1], fixed during a run'};
        defaults={ ...
            num2str(m.massKg,'%.4g'),num2str(m.rotorCount), ...
            num2str(m.airDensity,'%.5g'),num2str(m.profileDragCoeff,'%.5g'), ...
            num2str(m.rotorSolidity,'%.5g'),num2str(m.rotorAreaM2,'%.5g'), ...
            num2str(m.thrustCoeff,'%.6g'),num2str(m.v0Mps,'%.5g'), ...
            num2str(m.inducedCorrection,'%.5g'),num2str(m.fuselageAreaM2,'%.5g'), ...
            num2str(m.batteryCapacityWh,'%.5g'),num2str(m.cruiseAirspeed,'%.5g'), ...
            num2str(m.boostAirspeed,'%.5g'),num2str(m.maxAirspeed,'%.5g'), ...
            num2str(m.reserveSOC,'%.5g'),num2str(m.lowBatteryTriggerSOC,'%.5g'), ...
            num2str(m.bodyRadiusM,'%.5g'),num2str(app.missionPriority,'%.5g')};
        answer=inputdlg(prompts,'ACMFDS | Vehicle and Mission Parameters',[1 68],defaults);
        if isempty(answer), return; end
        v=str2double(answer);
        if numel(v)~=18 || any(~isfinite(v))
            errordlg('All model inputs must be finite numeric values.','Invalid model parameters');
            return;
        end
        positiveIdx=[1 3 4 5 6 7 8 11 12 13 14 17];
        if any(v(positiveIdx)<=0) || v(9)<0 || v(10)<0
            errordlg('Mass, air density, rotor parameters, battery energy, speeds and body radius must be positive. k and flat-plate area cannot be negative.','Invalid model parameters');
            return;
        end
        if v(2)~=round(v(2)) || v(2)<4 || v(2)>12 || mod(v(2),2)~=0
            errordlg('Rotor count must be an even integer from 4 to 12, matching the cited paper scope.','Invalid rotor count');
            return;
        end
        if v(13)<v(12) || v(14)<v(13)
            errordlg('Require normal airspeed <= boost airspeed <= maximum airspeed.','Invalid airspeed settings');
            return;
        end
        if v(15)<1 || v(15)>60 || v(16)<=v(15) || v(16)>100
            errordlg('Reserve must be 1..60% SOC, and low-battery trigger must be above reserve and at most 100%.','Invalid battery policy');
            return;
        end
        if v(18)<0 || v(18)>1
            errordlg('Mission priority must be between 0 and 1. It remains fixed during a mission run.','Invalid mission priority');
            return;
        end
        app.model.massKg=v(1); app.model.rotorCount=v(2);
        app.model.airDensity=v(3); app.model.profileDragCoeff=v(4);
        app.model.rotorSolidity=v(5); app.model.rotorAreaM2=v(6);
        app.model.thrustCoeff=v(7); app.model.v0Mps=v(8);
        app.model.inducedCorrection=v(9); app.model.fuselageAreaM2=v(10);
        app.model.batteryCapacityWh=v(11); app.model.cruiseAirspeed=v(12);
        app.model.boostAirspeed=v(13); app.model.maxAirspeed=v(14);
        app.model.reserveSOC=v(15); app.model.lowBatteryTriggerSOC=v(16);
        app.model.bodyRadiusM=v(17);
        app.missionPriority=v(18);
        onSettingChanged([],[]);
    end

    function onGenerate(~,~)
        if app.running, return; end
        createNewWorld();
    end

    function createNewWorld()
        if app.running, return; end
        updateLabels();
        cfg = readConfig();
        app.seedCounter = app.seedCounter + 1;
        rng(448 + 7919*app.seedCounter,'twister');
        resetMissionState(cfg);
        found = false;
        bestObs = struct([]); bestPath = [];
        for attempt = 1:12
            candidate = generateObstacles(cfg.numObstacles,cfg.movingCount,app.start,app.goal,app.W,app.H);
            candidate = updateObstaclePositions(candidate,0);
            [candidatePath,ok] = planRoute(app.start,app.goal,candidate,cfg,cfg.initialBattery,cfg.timeLimit);
            bestObs = candidate; bestPath = candidatePath;
            if ok
                found = true;
                break;
            end
        end
        app.obstacles = bestObs;
        app.safeZones = computeSafeZones(app.obstacles,cfg);
        app.path = bestPath;
        app.pathIndex = min(2,size(app.path,1));
        app.previousN = cfg.numObstacles;
        app.previousMoving = cfg.movingCount;
        if found
            app.mode = 'READY';
            app.lastReason = decisionReason(cfg,pathMetrics(app.path,cfg,app.obstacles),C);
        else
            app.mode = 'NO SAFE PATH';
            app.lastReason = 'No route found after 12 world attempts. Reduce obstacle count / safety margin.';
        end
        set(app.compareText,'String',sprintf('Current world seed: %d\nActual obstacles: %d\nClick COMPARE ROUTES to test four planners in this same map.',448+7919*app.seedCounter,numel(app.obstacles)));
        drawWorld();
        updateTelemetry(cfg,found);
    end

    function onPlan(~,~)
        if app.running, return; end
        cfg = readConfig();
        resetMissionState(cfg);
        app.obstacles = updateObstaclePositions(app.obstacles,0);
        [app.path,ok] = planRoute(app.start,app.goal,app.obstacles,cfg,cfg.initialBattery,cfg.timeLimit);
        app.pathIndex = min(2,size(app.path,1));
        if ok
            app.mode = 'PLAN READY';
            app.lastReason = decisionReason(cfg,pathMetrics(app.path,cfg,app.obstacles),C);
        else
            app.mode = 'NO SAFE PATH';
            app.lastReason = 'A* could not find a safe path. Try a different world or lower the safety margin.';
        end
        drawWorld();
        updateTelemetry(cfg,ok);
    end

    function onCompare(~,~)
        if app.running, return; end
        cfg = readConfig();
        app.obstacles = updateObstaclePositions(app.obstacles,0);
        drawWorld();
        strategies = {'Shortest Safe Route','Fastest Feasible Route','Energy Efficient Route','ACMFDS Adaptive'};
        colors = [0.98 0.68 0.20; 0.80 0.43 1.00; 0.25 0.90 0.62; 0.05 0.78 1.00];
        lineStyles = {'--','-.',':','-'};
        rows = cell(0,1);
        legendHandles = gobjects(0);
        for ii=1:numel(strategies)
            c2 = cfg; c2.strategy = strategies{ii};
            [p2,ok2] = planRoute(app.start,app.goal,app.obstacles,c2,cfg.initialBattery,cfg.timeLimit);
            if ok2
                hp = plot(app.ax,p2(:,1),p2(:,2),lineStyles{ii},'Color',colors(ii,:), ...
                    'LineWidth',1.5,'DisplayName',strategies{ii});
                legendHandles(end+1)=hp; %#ok<AGROW>
                m = pathMetrics(p2,c2,app.obstacles);
                rows{end+1}=sprintf('%-17s L%4.0f T%4.1f E%4.1f%%/%4.1fWh',shortName(strategies{ii}),m.length,m.time,m.energy,m.energyWh); %#ok<AGROW>
            else
                rows{end+1}=sprintf('%-17s NO SAFE PATH',shortName(strategies{ii})); %#ok<AGROW>
            end
        end
        % The direct line is a reference only; it is not offered as a flight mode.
        direct = [linspace(app.start(1),app.goal(1),100)' linspace(app.start(2),app.goal(2),100)'];
        hd = plot(app.ax,direct(:,1),direct(:,2),'--','Color',[0.62 0.66 0.72], ...
            'LineWidth',1.0,'DisplayName','Direct line (reference only)');
        legendHandles(end+1)=hd;
        directM = pathMetrics(direct,cfg,app.obstacles);
        rows{end+1}=sprintf('Direct reference L%4.0f  T%4.1f  E%4.1f%% / conflicts %d', ...
            directM.length,directM.time,directM.energy,directM.collisionSamples); %#ok<AGROW>
        legend(app.ax,legendHandles,'TextColor',C.text,'Color',C.panel, ...
            'EdgeColor',[0.20 0.31 0.42],'Location','southoutside','FontSize',7);
        set(app.compareText,'String',strjoin(rows,newline));
        app.lastReason = 'Routes computed on the same obstacle map. The direct line is a reference baseline only.';
        updateTelemetry(cfg,~isempty(app.path));
    end

    function onRun(~,~)
        if app.running, return; end
        cfg=readConfig();
        if cfg.randomizeOnRun
            createNewWorld();
            cfg=readConfig();
        end
        app.obstacles=updateObstaclePositions(app.obstacles,0);
        app.safeZones=computeSafeZones(app.obstacles,cfg);
        resetMissionState(cfg);
        [app.path,ok]=planRoute(app.start,app.goal,app.obstacles,cfg,cfg.initialBattery,cfg.timeLimit);
        initialFastFallback=false;
        initialEnergyFallback=false;
        if ~ok || size(app.path,1)<2
            % If the selected profile is not trackable under current wind, test
            % the configured fastest profile. Accept it only if it meets the
            % deadline and protected SOC reserve under the same energy model.
            fastCfg=cfg;
            fastCfg.strategy='Fastest Feasible Route';
            fastCfg.airspeedMps=min(cfg.model.boostAirspeed,cfg.model.maxAirspeed);
            [fastPath,fastOK]=planRoute(app.start,app.goal,app.obstacles,fastCfg,cfg.initialBattery,cfg.timeLimit);
            if fastOK && size(fastPath,1)>=2
                fastCfg.profileMode='FAST';
                fastMetrics=pathMetrics(fastPath,fastCfg,app.obstacles);
                if fastMetrics.time<=cfg.timeLimit && ...
                        fastMetrics.energy+cfg.model.reserveSOC<=cfg.initialBattery
                    app.path=fastPath; ok=true; app.speedMode='BOOST'; initialFastFallback=true;
                    app.lastReason=sprintf('Initial fallback: cruise profile was infeasible; fastest profile predicts %.1f s and %.2f Wh while preserving %.1f%% SOC.', ...
                        fastMetrics.time,fastMetrics.energyWh,cfg.model.reserveSOC);
                end
            end
        end
        if ~ok || size(app.path,1)<2
            app.mode='MISSION NOT STARTED';
            app.lastReason='No collision-free route exists in the current map, even after the faster-profile check. Launch inhibited. Generate a new world or reduce obstacle density / safety margin.';
            app.lastAlert='PREFLIGHT BLOCK: no geometric path to the destination was found. The drone remains at home; this is not a lost-drone event.';
            updateTelemetry(cfg,false);
            return;
        end

        % PREFLIGHT FEASIBILITY GATE: a geometric path is not enough. Evaluate
        % predicted arrival time and energy before allowing the simulated launch.
        if ~initialFastFallback
            preCfg=cfg;
            preCfg.profileMode=profileModeForStrategy(preCfg,cfg.initialBattery, ...
                cfg.timeLimit,app.start,app.goal);
            preMetrics=pathMetrics(app.path,preCfg,app.obstacles);
            preTimeOK=preMetrics.time<=cfg.timeLimit;
            preEnergyOK=(preMetrics.energy+cfg.model.reserveSOC<=cfg.initialBattery);
            if ~(preTimeOK && preEnergyOK)
                % If energy is the problem, first test an energy-minimizing route.
                % Only accept it if it also meets the deadline and keeps the reserve.
                energyFeasible=false;
                if ~preEnergyOK
                    energyCfg=cfg;
                    energyCfg.strategy='Energy Efficient Route';
                    energyCfg.profileMode='ENERGY';
                    energyCfg.airspeedMps=cfg.model.cruiseAirspeed;
                    [energyPath,energyOK]=planRoute(app.start,app.goal,app.obstacles, ...
                        energyCfg,cfg.initialBattery,cfg.timeLimit);
                    if energyOK && size(energyPath,1)>=2
                        energyMetrics=pathMetrics(energyPath,energyCfg,app.obstacles);
                        energyFeasible=energyMetrics.time<=cfg.timeLimit && ...
                            energyMetrics.energy+cfg.model.reserveSOC<=cfg.initialBattery;
                    end
                    if energyFeasible
                        app.path=energyPath;
                        app.speedMode='ENERGY SAVE';
                        initialEnergyFallback=true;
                        app.lastReason=sprintf(['Preflight chose the energy-minimizing route because the selected route ' ...
                            'failed the reserve check. Estimate: %.1f s, %.2f Wh, arrival SOC %.1f%%; reserve %.1f%%.'], ...
                            energyMetrics.time,energyMetrics.energyWh, ...
                            cfg.initialBattery-energyMetrics.energy,cfg.model.reserveSOC);
                    end
                end
                % If energy planning cannot satisfy both constraints, test a faster route.
                if ~energyFeasible
                    fastCfg=cfg;
                    fastCfg.strategy='Fastest Feasible Route';
                    fastCfg.airspeedMps=min(cfg.model.boostAirspeed,cfg.model.maxAirspeed);
                    fastCfg.profileMode='FAST';
                    [fastPath,fastOK]=planRoute(app.start,app.goal,app.obstacles,fastCfg, ...
                        cfg.initialBattery,cfg.timeLimit);
                    fastFeasible=false;
                    if fastOK && size(fastPath,1)>=2
                        fastMetrics=pathMetrics(fastPath,fastCfg,app.obstacles);
                        fastFeasible=fastMetrics.time<=cfg.timeLimit && ...
                            fastMetrics.energy+cfg.model.reserveSOC<=cfg.initialBattery;
                    end
                    if fastFeasible
                        app.path=fastPath;
                        app.speedMode='BOOST';
                        initialFastFallback=true;
                        app.lastReason=sprintf(['Preflight chose the faster route because the original route was infeasible. ' ...
                            'Estimate: %.1f s, %.2f Wh, arrival SOC %.1f%%; reserve %.1f%%.'], ...
                            fastMetrics.time,fastMetrics.energyWh, ...
                            cfg.initialBattery-fastMetrics.energy,cfg.model.reserveSOC);
                    else
                        app.mode='PREFLIGHT HOLD';
                        if ~preTimeOK && ~preEnergyOK
                            why='The selected route violates both the mission deadline and protected energy reserve.';
                        elseif ~preTimeOK
                            why='The selected route exceeds the mission deadline.';
                        else
                            why='The selected route would consume the protected battery reserve.';
                        end
                        if fastOK && size(fastPath,1)>=2
                            fastText=sprintf(' Faster-route estimate: %.1f s and %.2f Wh.',fastMetrics.time,fastMetrics.energyWh);
                        else
                            fastText=' No collision-free faster route exists.';
                        end
                        if ~preEnergyOK && exist('energyMetrics','var')
                            energyText=sprintf(' Energy-route estimate: %.1f s and %.2f Wh.',energyMetrics.time,energyMetrics.energyWh);
                        else
                            energyText='';
                        end
                        app.path=[];
                        app.pathIndex=1;
                        app.target=app.start;
                        app.position=app.start;
                        app.batteryNow=cfg.initialBattery;
                        app.trail=app.start;
                        app.lastAlert='PREFLIGHT INFEASIBLE: launch inhibited; aircraft remains at home.';
                        app.lastReason=[why energyText fastText ' No flight has started; this is not a lost-drone event.'];
                        drawWorld();
                        updateTelemetry(cfg,false);
                        return;
                    end
                end
            end
        end
        app.lastAlert='';
        app.pathIndex=2;
        app.target=app.goal;
        app.mode='OUTBOUND';
        app.running=true;
        app.stopRequested=false;
        app.lastPlanTime=0;
        app.noPathTime=0;
        app.missionSuccess=false;
        app.safeReturn=false;
        app.lastAlert='';
        if ~initialFastFallback && ~initialEnergyFallback, app.speedMode='NORMAL'; end
        app.nextDecisionTime=0;
        if initialFastFallback
            app.lastReason=[app.lastReason ' Mission started using the faster profile after feasibility checks.'];
        elseif initialEnergyFallback
            app.lastReason=[app.lastReason ' Mission started using the energy-minimizing profile after feasibility checks.'];
        else
            app.lastReason='Mission started. ACMFDS will continuously check destination feasibility, battery reserve and recovery options.';
        end
        setRunControls(false);
        drawWorld();
        updateTelemetry(cfg,true);
        drawnow;

        wallPause=0.035;
        maxHold=2.0;
        reservePct=cfg.model.reserveSOC;
        while app.running && isgraphics(app.fig)
            if app.stopRequested
                app.mode='STOPPED BY USER';
                app.lastReason='Mission stopped by user.';
                break;
            end

            cfg=readConfig();
            remaining=cfg.timeLimit-app.simTime;
            app.obstacles=updateObstaclePositions(app.obstacles,app.simTime);

            % CONTEXT DETECTION: use current battery, wind, and surface clearance
            % to the nearest obstacle. A transition requests a fresh plan.
            oldContext=app.lastContextForPlanning;
            [measuredClearance,~]=nearestClearance( ...
                app.position,app.obstacles,app.model.bodyRadiusM);
            app.obstacleDistance=max(0,measuredClearance);
            app.context=detectFlightContext( ...
                app.batteryNow,cfg.windSpeed,app.obstacleDistance,cfg);
            app.lastContextForPlanning=app.context;
            if ~strcmp(app.context,oldContext)
                app.forceReplan=true;
                app.nextDecisionTime=app.simTime;
                app.lastReason=sprintf( ...
                    'Context changed: %s -> %s. Route weights and local plan are being refreshed.', ...
                    oldContext,app.context);
            end

            % Arrival check: target may be the goal, home, or emergency landing pad.
            if norm(app.position-app.target)<0.35
                if norm(app.target-app.goal)<0.35
                    app.missionSuccess=true;
                    app.mode='MISSION SUCCESS';
                    app.lastReason='Destination reached with the current simulation model.';
                elseif norm(app.target-app.start)<0.35
                    app.safeReturn=true;
                    app.mode='SAFE RETURN COMPLETE';
                    app.lastReason='Home reached successfully under the recovery policy.';
                else
                    app.safeReturn=true;
                    app.mode='EMERGENCY LANDING COMPLETE';
                    app.lastReason=sprintf('Reached emergency landing site (%.0f, %.0f). Drone is considered safely landed.',app.target(1),app.target(2));
                end
                break;
            end

            % DECISION LAYER: periodically estimate a route to the current target.
            % If the destination is no longer feasible, try fast flight only when
            % both energy reserve and remaining-time constraints remain feasible.
            % Otherwise divert to the safest reachable landing site/home.
            if app.simTime >= app.nextDecisionTime
                app.nextDecisionTime=app.simTime+0.8;
                isDestination=norm(app.target-app.goal)<0.35;
                % Always evaluate a true normal-speed baseline first, even if the
                % previous decision put the drone into BOOST mode.
                normalCfg=cfg;
                normalCfg.airspeedMps=cfg.model.cruiseAirspeed;
                normalCfg.profileMode=profileModeForStrategy(normalCfg,app.batteryNow,max(0.1,remaining),app.position,app.target);
                [pNormal,normalOK]=planRoute(app.position,app.target,app.obstacles,normalCfg, ...
                    app.batteryNow,max(0.1,remaining));
                if normalOK && ~isempty(pNormal)
                    mNormal=pathMetrics(pNormal,normalCfg,app.obstacles);
                    energyOK=(mNormal.energy+reservePct <= app.batteryNow);
                    timeOK=(mNormal.time <= max(0,remaining));
                else
                    mNormal=struct('time',inf,'energy',inf,'energyWh',inf);
                    energyOK=false; timeOK=false;
                end

                lowBattery=app.batteryNow <= cfg.model.lowBatteryTriggerSOC;
                mustRecover=~energyOK || (~timeOK && isDestination) || (~normalOK);

                % If energy feasibility is the issue, try an energy-minimizing
                % destination route before considering boost or recovery.
                destinationEnergyHandled=false;
                if mustRecover && isDestination && (~energyOK || ~normalOK)
                    energyCfg=cfg;
                    energyCfg.strategy='Energy Efficient Route';
                    energyCfg.profileMode='ENERGY';
                    energyCfg.airspeedMps=cfg.model.cruiseAirspeed;
                    [pEnergy,energyRouteOK]=planRoute(app.position,app.goal,app.obstacles, ...
                        energyCfg,app.batteryNow,max(0.1,remaining));
                    if energyRouteOK && size(pEnergy,1)>=2
                        mEnergy=pathMetrics(pEnergy,energyCfg,app.obstacles);
                        energyRouteFeasible=(mEnergy.energy+reservePct<=app.batteryNow) && ...
                            (mEnergy.time<=max(0,remaining));
                        if energyRouteFeasible
                            app.path=pEnergy;
                            app.pathIndex=min(2,size(app.path,1));
                            app.target=app.goal;
                            app.speedMode='ENERGY SAVE';
                            app.mode='ENERGY-AWARE ROUTE TO DESTINATION';
                            app.lastReason=sprintf(['Current destination route violates the energy check; selected an energy-minimizing ' ...
                                'destination route instead. Estimated time %.1f s, energy %.2f Wh, arrival SOC %.1f%%, reserve %.1f%%.'], ...
                                mEnergy.time,mEnergy.energyWh,app.batteryNow-mEnergy.energy,reservePct);
                            destinationEnergyHandled=true;
                        end
                    end
                end

                if mustRecover && isDestination && ~destinationEnergyHandled
                    % First attempt a boosted destination route. Boost costs more
                    % energy, so it is accepted only if predicted reserve remains.
                    boostCfg=cfg;
                    boostCfg.strategy='Fastest Feasible Route';
                    boostCfg.airspeedMps=cfg.model.boostAirspeed;
                    if boostCfg.airspeedMps>cfg.model.maxAirspeed
                        boostCfg.airspeedMps=cfg.model.maxAirspeed;
                    end
                    [pBoost,boostOK]=planRoute(app.position,app.goal,app.obstacles,boostCfg, ...
                        app.batteryNow,max(0.1,remaining));
                    boostFeasible=false;
                    if boostOK && ~isempty(pBoost)
                        mBoost=pathMetrics(pBoost,boostCfg,app.obstacles);
                        boostFeasible=(mBoost.energy+reservePct <= app.batteryNow) && ...
                            (mBoost.time <= max(0,remaining));
                    end
                    % If goal can be reached safely with boost and reserve, do so.
                    % A boost is NOT allowed to spend the protected reserve.
                    % Boost only if it restores deadline feasibility, or if its
                    % alternate route is both reserve-feasible and uses less
                    % modeled energy than the normal candidate. Low SOC alone is
                    % never a reason to accelerate.
                    boostSavesEnergy = ~energyOK && boostFeasible && ...
                        (mBoost.energyWh < mNormal.energyWh - 1e-9);
                    if boostFeasible && (~timeOK || boostSavesEnergy)
                        app.path=pBoost;
                        app.pathIndex=min(2,size(app.path,1));
                        app.target=app.goal;
                        app.speedMode='BOOST';
                        app.mode='BOOST TO DESTINATION';
                        app.lastReason=sprintf(['Adaptive recovery: normal route infeasible; boosted route is feasible. ' ...
                            'Urgent airspeed %.1f m/s, predicted ETA %.1f s, energy %.1f%% SOC, reserve protected %.1f%% SOC.'], ...
                            boostCfg.airspeedMps,mBoost.time,mBoost.energy,reservePct);
                    else
                        % No feasible destination route: choose home or an emergency pad.
                        [recTarget,recPath,recSpeed,recLabel,recReason,recOK]= ...
                            selectRecoveryRoute(cfg,remaining,reservePct);
                        if recOK
                            app.target=recTarget;
                            app.path=recPath;
                            app.pathIndex=min(2,size(app.path,1));
                            app.speedMode=recSpeed;
                            app.mode=recLabel;
                            app.lastReason=recReason;
                        else
                            app.mode='DRONE LOST ALERT';
                            app.lastAlert=sprintf(['No destination, home, or emergency landing site is reachable ' ...
                                'within the energy reserve. Last known position (%.1f, %.1f), battery %.1f%%, ' ...
                                'simulated time %.1f s. Recovery unavailable; operator intervention required.'], ...
                                app.position(1),app.position(2),app.batteryNow,app.simTime);
                            app.lastReason='MAYDAY: destination infeasible, boost infeasible, and no safe recovery site passes the energy check.';
                            break;
                        end
                    end
                elseif mustRecover && ~isDestination
                    % Already returning/diverting: if current recovery target is no
                    % longer feasible, evaluate another safe target before declaring loss.
                    boostCfg=cfg;
                    boostCfg.strategy='Fastest Feasible Route';
                    boostCfg.airspeedMps=cfg.model.boostAirspeed;
                    if boostCfg.airspeedMps>cfg.model.maxAirspeed
                        boostCfg.airspeedMps=cfg.model.maxAirspeed;
                    end
                    [pBoost,boostOK]=planRoute(app.position,app.target,app.obstacles,boostCfg, ...
                        app.batteryNow,max(0.1,remaining));
                    boostFeasible=false;
                    if boostOK && ~isempty(pBoost)
                        mBoost=pathMetrics(pBoost,boostCfg,app.obstacles);
                        boostFeasible=(mBoost.energy+reservePct <= app.batteryNow);
                    end
                    if energyOK && normalOK
                        app.path=pNormal;
                        app.pathIndex=min(2,size(app.path,1));
                    elseif boostFeasible
                        app.path=pBoost;
                        app.pathIndex=min(2,size(app.path,1));
                        app.speedMode='BOOST';
                        app.lastReason='Recovery route updated in boosted mode; reserve check passed.';
                    else
                        [recTarget,recPath,recSpeed,recLabel,recReason,recOK]= ...
                            selectRecoveryRoute(cfg,remaining,reservePct);
                        if recOK
                            app.target=recTarget;
                            app.path=recPath;
                            app.pathIndex=min(2,size(app.path,1));
                            app.speedMode=recSpeed;
                            app.mode=recLabel;
                            app.lastReason=recReason;
                        else
                            app.mode='DRONE LOST ALERT';
                            app.lastAlert=sprintf(['Recovery failed. Last known position (%.1f, %.1f), battery %.1f%%, ' ...
                                'time %.1f s. No safe site remains reachable within the protected energy reserve.'], ...
                                app.position(1),app.position(2),app.batteryNow,app.simTime);
                            app.lastReason='MAYDAY: all computed recovery routes violate the protected energy reserve.';
                            break;
                        end
                    end
                elseif normalOK && ~destinationEnergyHandled
                    % A critical SOC selects the energy-optimized profile under ACMFDS.
                    % Low battery alone never increases speed.
                    if strcmp(cfg.strategy,'ACMFDS Adaptive') && lowBattery
                        app.speedMode='ENERGY SAVE';
                        app.mode='ENERGY-SAVING FLIGHT';
                        app.lastReason=sprintf('Low-battery response: use the model energy-minimizing airspeed profile. Predicted route energy %.2f Wh; protected reserve %.1f%% SOC.',mNormal.energyWh,reservePct);
                        app.path=pNormal;
                        app.pathIndex=min(2,size(app.path,1));
                    elseif strcmp(app.speedMode,'BOOST') && timeOK && energyOK
                        app.speedMode='NORMAL';
                        if norm(app.target-app.goal)<0.35
                            app.mode='OUTBOUND';
                        end
                        app.path=pNormal;
                        app.pathIndex=min(2,size(app.path,1));
                        app.lastReason='Risk reduced: normal-speed flight is feasible again; boost mode disengaged.';
                    elseif isempty(app.path) || size(app.path,1)<2 || app.pathIndex>size(app.path,1)
                        app.path=pNormal;
                        app.pathIndex=min(2,size(app.path,1));
                    end
                end
            end

            % If mission deadline expires, try a reachable emergency landing rather
            % than instantly declaring loss. The deadline still appears as a breach.
            remaining=cfg.timeLimit-app.simTime;
            if remaining<=0 && norm(app.target-app.goal)<0.35
                [recTarget,recPath,recSpeed,recLabel,recReason,recOK]= ...
                    selectRecoveryRoute(cfg,remaining,reservePct);
                if recOK
                    app.target=recTarget;
                    app.path=recPath;
                    app.pathIndex=min(2,size(app.path,1));
                    app.speedMode=recSpeed;
                    app.mode=recLabel;
                    if ~contains(app.mode,'DEADLINE OVERRUN')
                        app.mode=[app.mode ' / DEADLINE OVERRUN'];
                    end
                    app.lastReason=['Destination deadline missed; prioritizing safe landing. ' recReason];
                else
                    app.mode='DRONE LOST ALERT';
                    app.lastAlert=sprintf(['Mission deadline expired and no energy-feasible recovery site was found. ' ...
                        'Last known position (%.1f, %.1f), battery %.1f%%.'], ...
                        app.position(1),app.position(2),app.batteryNow);
                    app.lastReason='MAYDAY: the destination deadline is missed and safe recovery is infeasible.';
                    break;
                end
            end

            % Apply selected speed profile to actual segment execution.
            flightCfg=cfg;
            if strcmp(app.speedMode,'BOOST')
                flightCfg.airspeedMps=cfg.model.boostAirspeed;
                flightCfg.strategy='Fastest Feasible Route';
            elseif strcmp(app.speedMode,'ENERGY SAVE')
                flightCfg.airspeedMps=cfg.model.cruiseAirspeed;
                flightCfg.strategy='Energy Efficient Route';
            end
            flightCfg.profileMode=profileModeForStrategy(flightCfg,app.batteryNow, ...
                max(0.1,cfg.timeLimit-app.simTime),app.position,app.target);
            targetNow=app.target;

            % Dynamic-obstacle replan; uses current flight profile and recovery target.
            if app.forceReplan || (cfg.movingCount>0 && app.simTime-app.lastPlanTime>=1.0)
                app.forceReplan=false;
                [pNew,okNew]=planRoute(app.position,targetNow,app.obstacles,flightCfg, ...
                    app.batteryNow,max(0.1,cfg.timeLimit-app.simTime));
                if okNew && size(pNew,1)>=2
                    app.path=pNew;
                    app.pathIndex=2;
                    app.lastPlanTime=app.simTime;
                    app.replanCount=app.replanCount+1;
                    if strcmp(app.mode,'OUTBOUND')
                        app.lastReason=sprintf('Dynamic replan #%d: route refreshed for moving obstacles; wind %.1f m/s, battery %.1f%%.', ...
                            app.replanCount,cfg.windSpeed,app.batteryNow);
                    end
                elseif ~okNew && norm(targetNow-app.goal)<0.35
                    app.nextDecisionTime=app.simTime; % trigger recovery logic next iteration
                    app.lastReason='No current path to destination because of moving obstacles; evaluating recovery now.';
                    app.simTime=app.simTime+0.05;
                    app.batteryNow=max(0,app.batteryNow-0.01);
                    updateGraphics(); updateTelemetry(cfg,false); drawnow; pause(wallPause);
                    continue;
                end
            end

            % Replan if the current path was exhausted.
            if isempty(app.path) || size(app.path,1)<2 || app.pathIndex>size(app.path,1)
                [pNew,okNew]=planRoute(app.position,targetNow,app.obstacles,flightCfg, ...
                    app.batteryNow,max(0.1,cfg.timeLimit-app.simTime));
                if okNew && size(pNew,1)>=2
                    app.path=pNew; app.pathIndex=2; app.lastPlanTime=app.simTime;
                else
                    app.nextDecisionTime=app.simTime;
                    app.noPathTime=app.noPathTime+0.15;
                    app.mode='SAFE HOLD / REPLANNING';
                    app.lastReason='No safe next segment; holding briefly while dynamic obstacles move, then recovery logic will retry.';
                    app.simTime=app.simTime+0.15;
                    app.batteryNow=max(0,app.batteryNow-0.03);
                    updateGraphics(); updateTelemetry(cfg,false); drawnow; pause(wallPause);
                    if app.noPathTime>=maxHold
                        [recTarget,recPath,recSpeed,recLabel,recReason,recOK]= ...
                            selectRecoveryRoute(cfg,cfg.timeLimit-app.simTime,reservePct);
                        if recOK
                            app.target=recTarget; app.path=recPath;
                            app.pathIndex=min(2,size(app.path,1)); app.speedMode=recSpeed;
                            app.mode=recLabel; app.lastReason=recReason; app.noPathTime=0;
                        else
                            app.mode='DRONE LOST ALERT';
                            app.lastAlert=sprintf(['No safe segment or energy-feasible recovery route after waiting. ' ...
                                'Last known position (%.1f, %.1f), battery %.1f%%.'], ...
                                app.position(1),app.position(2),app.batteryNow);
                            app.lastReason='MAYDAY: obstacle blockage persisted and recovery options were exhausted.';
                            break;
                        end
                    end
                    continue;
                end
            end

            nextPos=app.path(app.pathIndex,:);
            if norm(nextPos-app.position)<1e-6
                app.pathIndex=app.pathIndex+1;
                continue;
            end
            [segTime,segEnergy,heading,~]=segmentCost(app.position,nextPos,flightCfg,app.previousHeading);
            futureTime=app.simTime+segTime;
            if futureTime>cfg.timeLimit && norm(app.target-app.goal)<0.35
                % Do not step past an infeasible destination segment; force a re-check.
                app.nextDecisionTime=app.simTime;
                app.lastReason='Predicted next step misses the destination deadline; evaluating boost or safe landing.';
                continue;
            end
            futureObstacles=updateObstaclePositions(app.obstacles,futureTime);
            safeSegment=segmentIsSafe(app.position,nextPos,app.simTime,futureTime,futureObstacles,flightCfg);
            if ~safeSegment
                % Reject before movement; the next iteration replans with updated data.
                app.lastPlanTime=-inf;
                app.nextDecisionTime=app.simTime;
                app.lastReason='Safety interlock: next segment rejected because a moving obstacle enters its clearance envelope.';
                app.simTime=app.simTime+0.05;
                app.batteryNow=max(0,app.batteryNow-0.01);
                app.obstacles=updateObstaclePositions(app.obstacles,app.simTime);
                app.noPathTime=app.noPathTime+0.05;
                app.safetyRejections=app.safetyRejections+1;
                updateGraphics(); updateTelemetry(cfg,false); drawnow; pause(wallPause);
                if app.noPathTime>=maxHold
                    [recTarget,recPath,recSpeed,recLabel,recReason,recOK]= ...
                        selectRecoveryRoute(cfg,cfg.timeLimit-app.simTime,reservePct);
                    if recOK
                        app.target=recTarget; app.path=recPath;
                        app.pathIndex=min(2,size(app.path,1)); app.speedMode=recSpeed;
                        app.mode=recLabel; app.lastReason=recReason; app.noPathTime=0;
                    else
                        app.mode='DRONE LOST ALERT';
                        app.lastAlert=sprintf(['Safety interlock blocked all tested routes. Last known position ' ...
                            '(%.1f, %.1f), battery %.1f%%. No feasible recovery route remains.'], ...
                            app.position(1),app.position(2),app.batteryNow);
                        app.lastReason='MAYDAY: repeated collision-avoidance interlocks and no recovery path.';
                        break;
                    end
                end
                continue;
            end

            % Execute validated next step.
            oldPos=app.position;
            app.position=nextPos;
            app.simTime=futureTime;
            app.obstacles=futureObstacles;
            app.distanceTravelled=app.distanceTravelled+norm(app.position-oldPos);
            app.batteryNow=max(0,app.batteryNow-segEnergy);
            app.previousHeading=heading;
            app.trail(end+1,:)=app.position; %#ok<AGROW>
            app.pathIndex=app.pathIndex+1;
            app.noPathTime=0;
            [nearClr,~]=nearestClearance(app.position,app.obstacles,app.model.bodyRadiusM);
            app.minClearance=min(app.minClearance,nearClr);

            if app.batteryNow<=0
                app.mode='DRONE LOST ALERT';
                app.lastAlert=sprintf('Battery depleted at last known position (%.1f, %.1f), simulated time %.1f s. Emergency landing could not be completed.',app.position(1),app.position(2),app.simTime);
                app.lastReason='MAYDAY: simulated battery reached 0%. This is a simulated mission-loss event.';
                break;
            end

            updateGraphics();
            updateTelemetry(cfg,true);
            drawnow;
            pause(wallPause);
        end

        if app.stopRequested && ~contains(app.mode,'STOPPED')
            app.mode='STOPPED BY USER';
        elseif app.missionSuccess
            app.mode='MISSION SUCCESS';
        elseif app.safeReturn && ~strcmp(app.mode,'EMERGENCY LANDING COMPLETE')
            app.mode='SAFE RETURN COMPLETE';
        end
        app.running=false;
        if isgraphics(app.fig)
            setRunControls(true);
            updateGraphics();
            updateTelemetry(readConfig(),app.missionSuccess || app.safeReturn);
            drawnow;
            if strcmp(app.mode,'DRONE LOST ALERT') && ~isempty(app.lastAlert)
                warndlg(sprintf('ACMFDS MISSION ALERT\n\n%s\n\nThis is a simulated ground-station alert, not a real telemetry transmission.',app.lastAlert), ...
                    'ACMFDS - MISSION RECOVERY FAILED','modal');
            elseif contains(app.mode,'EMERGENCY')
                msgbox(sprintf('ACMFDS RECOVERY NOTICE\n\n%s',app.lastReason), ...
                    'ACMFDS - SAFE RECOVERY','help','modal');
            end
        end
    end

    function [bestTarget,bestPath,bestSpeed,bestLabel,bestReason,found] = selectRecoveryRoute(cfg,remaining,reservePct)
        % Prefer home where practical; otherwise choose a reachable clear landing pad.
        candidates=[app.start; app.safeZones];
        if isempty(candidates), candidates=app.start; end
        % Remove duplicates and keep safe targets distinct from the mission goal.
        candidates=unique(round(candidates*10)/10,'rows','stable');
        keep=vecnorm(candidates-app.goal,2,2)>3.0;
        candidates=candidates(keep,:);
        candidates=[app.start; candidates(~all(abs(candidates-app.start)<1e-9,2),:)];
        found=false; bestKey=[inf inf inf]; bestTarget=app.start; bestPath=[];
        bestSpeed='NORMAL'; bestLabel='RETURNING HOME'; bestReason='';
        for ci=1:size(candidates,1)
            tgt=candidates(ci,:);
            for boostTry=0:1
                rcfg=cfg;
                if boostTry==1
                    rcfg.strategy='Fastest Feasible Route';
                    rcfg.airspeedMps=cfg.model.boostAirspeed;
                else
                    rcfg.strategy='Energy Efficient Route';
                    rcfg.airspeedMps=cfg.model.cruiseAirspeed;
                end
                [candPath,candOK]=planRoute(app.position,tgt,app.obstacles,rcfg, ...
                    app.batteryNow,max(0.1,remaining));
                if ~candOK || isempty(candPath), continue; end
                if size(candPath,1)<2 && norm(app.position-tgt)>0.35, continue; end
                rcfg.initialBattery=app.batteryNow;
                rcfg.timeLimit=max(0.1,remaining);
                rcfg.profileMode=profileModeForStrategy(rcfg,app.batteryNow,max(0.1,remaining),app.position,tgt);
                m=pathMetrics(candPath,rcfg,app.obstacles);
                if m.energy+reservePct>app.batteryNow, continue; end
                timeFeasible=m.time<=max(0,remaining);
                % Lexicographic recovery policy, not a hidden weighted score:
                % (1) prefer options meeting the remaining deadline; (2) among
                % deadline-feasible options minimize Wh; otherwise minimize ETA;
                % (3) use lower energy as a tie-breaker. All candidates already
                % satisfy collision constraints and protected reserve.
                if timeFeasible
                    key=[0 m.energyWh m.time];
                else
                    key=[1 m.time m.energyWh];
                end
                better=~found || lexicographicallyLess(key,bestKey);
                if better
                    found=true; bestKey=key; bestTarget=tgt; bestPath=candPath;
                    if boostTry==1, bestSpeed='BOOST'; else, bestSpeed='ENERGY SAVE'; end
                    if norm(tgt-app.start)<0.35
                        bestLabel='RETURNING HOME';
                    else
                        bestLabel='DIVERTING TO SAFE LANDING';
                    end
                    if ~timeFeasible
                        bestLabel=['EMERGENCY ' bestLabel ' / DEADLINE OVERRUN'];
                    end
                    bestReason=sprintf(['Recovery decision: destination was infeasible, so ACMFDS selected %s at (%.0f, %.0f). ' ...
                        'Estimated recovery time %.1f s, energy %.1f%%; protected reserve %.1f%%. %s'], ...
                        bestLabel,tgt(1),tgt(2),m.time,m.energy,reservePct, ...
                        chooseText(boostTry==1,'Urgent airspeed evaluated against the model reserve.','Energy-minimizing airspeed profile selected for recovery.'));
                end
            end
        end
    end

    function out=chooseText(condition,a,b)
        if condition, out=a; else, out=b; end
    end

    function tf=lexicographicallyLess(a,b)
        tf=false;
        for kk=1:numel(a)
            if a(kk)<b(kk)-1e-9, tf=true; return; end
            if a(kk)>b(kk)+1e-9, return; end
        end
    end

    function zones=computeSafeZones(obstacles,cfg)
        % Candidate emergency landing sites are kept away from start/goal and
        % accepted only when obstacle clearance exceeds the selected margin.
        cand=[5 35; 13 35; 22 35; 31 36; 41 35; 53 33; ...
              5 20; 17 20; 29 20; 43 20; 54 18; ...
              7 6; 18 5; 30 5; 42 5; 54 6; 29 9; 38 31];
        zones=zeros(0,2);
        minClr=cfg.safetyMargin;
        for zz=1:size(cand,1)
            pz=cand(zz,:);
            if norm(pz-app.start)<5 || norm(pz-app.goal)<5, continue; end
            [clr,coll]=nearestClearance(pz,obstacles,cfg.model.bodyRadiusM);
            if ~coll && clr>minClr
                zones(end+1,:)=pz; %#ok<AGROW>
            end
        end
        % If randomized obstacles cover the preferred pads, search a coarse lattice.
        if size(zones,1)<3
            for yy=4:4:(app.H-3)
                for xx=4:4:(app.W-3)
                    pz=[xx yy];
                    if norm(pz-app.start)<5 || norm(pz-app.goal)<5, continue; end
                    [clr,coll]=nearestClearance(pz,obstacles,cfg.model.bodyRadiusM);
                    if ~coll && clr>minClr && all(vecnorm(zones-pz,2,2)>5)
                        zones(end+1,:)=pz; %#ok<AGROW>
                    end
                end
            end
        end
        if size(zones,1)>12
            [~,ix]=sort(vecnorm(zones-repmat([app.W/2 app.H/2],size(zones,1),1),2,2),'ascend');
            zones=zones(ix(1:12),:);
        end
    end

    function onStop(~,~)
        if app.running
            app.stopRequested = true;
            app.lastReason = 'Stop requested by user.';
        else
            app.lastReason = 'Mission is not running.';
            updateTelemetry(readConfig(),~isempty(app.path));
        end
    end

    function onClose(~,~)
        app.stopRequested = true;
        if isgraphics(app.fig), delete(app.fig); end
    end

    function setRunControls(enabled)
        if enabled
            state='on';
        else
            state='off';
        end
        for jj=1:numel(app.inputControls)
            if isgraphics(app.inputControls(jj)), set(app.inputControls(jj),'Enable',state); end
        end
        % The operator may change wind speed/direction in-flight. Other controls
        % remain locked so the environment and physics parameters stay coherent.
        if ~enabled
            set(app.windSlider,'Enable','on');
            set(app.dirSlider,'Enable','on');
        end
        set(app.newButton,'Enable',state);
        set(app.planButton,'Enable',state);
        set(app.runButton,'Enable',state);
        set(app.compareButton,'Enable',state);
        set(app.stopButton,'Enable','on');
    end

    function resetMissionState(cfg)
        app.position=app.start;
        app.simTime=0;
        app.batteryNow=cfg.initialBattery;
        app.distanceTravelled=0;
        app.minClearance=inf;
        app.safetyRejections=0;
        app.replanCount=0;
        app.pathIndex=2;
        app.target=app.goal;
        app.previousHeading=[1 0];
        app.trail=app.start;
        app.lastPlanTime=0;
        app.noPathTime=0;
        app.missionSuccess=false;
        app.safeReturn=false;
        app.stopRequested=false;
        app.mode='READY';
        app.speedMode='NORMAL';
        app.lastAlert='';
        app.nextDecisionTime=0;
        app.forceReplan=false;
        app.context='NORMAL';
        app.lastContextForPlanning='NORMAL';
        app.obstacleDistance=inf;
    end

    function drawWorld()
        if ~isgraphics(app.ax), return; end
        cla(app.ax);
        % Subtle checkerboard tiles give the map a game-board feel.
        [GX,GY] = meshgrid(1:app.W,1:app.H);
        board = 1 + mod(GX+GY,2);
        imagesc(app.ax,1:app.W,1:app.H,board);
        colormap(app.ax,[0.035 0.055 0.080; 0.050 0.078 0.105]);
        caxis(app.ax,[1 2]);
        set(app.ax,'YDir','normal','XLim',[0.5 app.W+0.5], ...
            'YLim',[0.5 app.H+0.5],'XTick',5:5:60,'YTick',5:5:40, ...
            'XColor',[0.65 0.73 0.83],'YColor',[0.65 0.73 0.83], ...
            'GridColor',[0.24 0.33 0.43],'GridAlpha',0.30, ...
            'FontSize',8,'Layer','top');
        hold(app.ax,'on'); grid(app.ax,'on');
        xlabel(app.ax,'EAST / X (m)','Color',C.text,'FontWeight','bold');
        ylabel(app.ax,'NORTH / Y (m)','Color',C.text,'FontWeight','bold');
        title(app.ax,'LIVE WORLD  /  TOP-DOWN FLIGHT LAYER', ...
            'Color',C.cyan,'FontSize',12,'FontWeight','bold');

        cfg=readConfig();
        % Route plotted before obstacles; inflated safety halos are dashed outlines.
        app.hPath = plot(app.ax,nan,nan,'-','Color',C.cyan,'LineWidth',2.2, ...
            'DisplayName','Planned safe route');
        if ~isempty(app.path)
            set(app.hPath,'XData',app.path(:,1),'YData',app.path(:,2));
        end
        app.safetyHandles=gobjects(numel(app.obstacles),1);
        app.obstacleHandles=gobjects(numel(app.obstacles),1);
        for jj=1:numel(app.obstacles)
            ob=app.obstacles(jj);
            curv=[0.08 0.08];
            if ob.isCircle, curv=[1 1]; end
            margin=cfg.safetyMargin+cfg.model.bodyRadiusM;
            app.safetyHandles(jj)=rectangle(app.ax,'Position', ...
                [ob.x-ob.rx-margin ob.y-ob.ry-margin 2*(ob.rx+margin) 2*(ob.ry+margin)], ...
                'Curvature',curv,'EdgeColor',[0.78 0.36 0.40], ...
                'LineStyle','--','LineWidth',0.7,'FaceColor','none');
            if ob.moving, fc=C.red; ec=[1.0 0.66 0.68]; else, fc=C.orange; ec=[1.0 0.79 0.46]; end
            app.obstacleHandles(jj)=rectangle(app.ax,'Position', ...
                [ob.x-ob.rx ob.y-ob.ry 2*ob.rx 2*ob.ry], ...
                'Curvature',curv,'FaceColor',fc,'EdgeColor',ec, ...
                'LineWidth',1.1);
        end
        % Green emergency landing pads: safe alternatives for low-battery recovery.
        if ~isempty(app.safeZones)
            plot(app.ax,app.safeZones(:,1),app.safeZones(:,2),'h', ...
                'MarkerSize',8,'MarkerFaceColor',C.green,'MarkerEdgeColor',[0.92 1.00 0.93], ...
                'LineWidth',0.8,'DisplayName','Emergency landing site');
        end
        % Home and goal markers.
        plot(app.ax,app.start(1),app.start(2),'s','MarkerSize',12, ...
            'MarkerFaceColor',C.green,'MarkerEdgeColor',[0.90 1.00 0.93], ...
            'LineWidth',1.1,'DisplayName','HOME');
        plot(app.ax,app.goal(1),app.goal(2),'p','MarkerSize',16, ...
            'MarkerFaceColor',C.yellow,'MarkerEdgeColor',[1.00 0.97 0.77], ...
            'LineWidth',1.1,'DisplayName','GOAL');
        text(app.ax,app.start(1)+1.2,app.start(2)+1.1,'HOME','Color',C.green, ...
            'FontWeight','bold','FontSize',8,'Clipping','on');
        text(app.ax,app.goal(1)-1,app.goal(2)-2.2,'DESTINATION','Color',C.yellow, ...
            'FontWeight','bold','FontSize',8,'HorizontalAlignment','right','Clipping','on');
        app.hTrail=plot(app.ax,app.trail(:,1),app.trail(:,2),'-', ...
            'Color',[0.10 0.98 0.88],'LineWidth',2.8,'DisplayName','Travel trail');
        app.hDrone=plot(app.ax,app.position(1),app.position(2),'p', ...
            'MarkerSize',14,'MarkerFaceColor',C.cyan,'MarkerEdgeColor','w', ...
            'LineWidth',1.1,'DisplayName','Drone');
        h=app.previousHeading/max(norm(app.previousHeading),eps);
        app.hHeading=plot(app.ax,[app.position(1) app.position(1)+1.25*h(1)], ...
            [app.position(2) app.position(2)+1.25*h(2)],'-', ...
            'Color',[0.92 1.00 1.00],'LineWidth',2.3,'HandleVisibility','off');
        app.hDroneLabel=text(app.ax,app.position(1)+1.1,app.position(2)+1.1,'UAV', ...
            'Color','w','FontSize',8,'FontWeight','bold','Clipping','on');
        app.compareHandles=gobjects(0);
        xlim(app.ax,[0.5 app.W+0.5]); ylim(app.ax,[0.5 app.H+0.5]);
        axis(app.ax,'equal'); box(app.ax,'on');
        drawnow limitrate;
    end

    function updateGraphics()
        if ~isgraphics(app.ax), return; end
        cfg=readConfig();
        % Obstacle and safety-halo positions move in place, not as a static picture.
        n=min(numel(app.obstacles),numel(app.obstacleHandles));
        for jj=1:n
            ob=app.obstacles(jj); margin=cfg.safetyMargin+cfg.model.bodyRadiusM;
            if isgraphics(app.obstacleHandles(jj))
                set(app.obstacleHandles(jj),'Position', ...
                    [ob.x-ob.rx ob.y-ob.ry 2*ob.rx 2*ob.ry]);
            end
            if isgraphics(app.safetyHandles(jj))
                set(app.safetyHandles(jj),'Position', ...
                    [ob.x-ob.rx-margin ob.y-ob.ry-margin 2*(ob.rx+margin) 2*(ob.ry+margin)]);
            end
        end
        if isgraphics(app.hPath) && ~isempty(app.path)
            set(app.hPath,'XData',app.path(:,1),'YData',app.path(:,2));
        elseif isgraphics(app.hPath)
            set(app.hPath,'XData',nan,'YData',nan);
        end
        if isgraphics(app.hTrail)
            set(app.hTrail,'XData',app.trail(:,1),'YData',app.trail(:,2));
        end
        if isgraphics(app.hDrone)
            set(app.hDrone,'XData',app.position(1),'YData',app.position(2));
        end
        if isgraphics(app.hHeading)
            h=app.previousHeading/max(norm(app.previousHeading),eps);
            set(app.hHeading,'XData',[app.position(1) app.position(1)+1.25*h(1)], ...
                'YData',[app.position(2) app.position(2)+1.25*h(2)]);
        end
        if isgraphics(app.hDroneLabel)
            set(app.hDroneLabel,'Position',[app.position(1)+1.0 app.position(2)+1.0 0]);
        end
        drawnow limitrate;
    end

    function updateTelemetry(cfg,routeOK)
        if ~isgraphics(app.fig), return; end
        if nargin<1 || isempty(cfg), cfg=readConfig(); end
        cfg.missionVector=app.target-app.position;
        [measuredClearance,~]=nearestClearance( ...
            app.position,app.obstacles,app.model.bodyRadiusM);
        app.obstacleDistance=max(0,measuredClearance);
        app.context=detectFlightContext( ...
            app.batteryNow,cfg.windSpeed,app.obstacleDistance,cfg);
        [wE,wS,wT,wR,wM]=adaptiveWeights( ...
            cfg,app.batteryNow,max(0.1,cfg.timeLimit-app.simTime), ...
            max(1,norm(app.target-app.position)),app.context);
        switch app.context
            case 'LOW BATTERY'
                contextAction='Energy saving / recovery feasibility';
            case 'OBSTACLE NEARBY'
                contextAction='Safety-biased route replanning';
            case 'HIGH WIND'
                contextAction='Wind-aware tracking / safety weighting';
            otherwise
                contextAction='Follow selected safe route';
        end
        if isempty(app.path) || size(app.path,1)<2 || ~routeOK
            s = sprintf('STATE: %s | CONTEXT: %s\nWorld: %d obstacles (%d moving)\nPlanner: %s\nRoute: NO SAFE PATH | Speed: %s', ...
                app.mode,app.context,numel(app.obstacles),sum([app.obstacles.moving]),cfg.strategy,app.speedMode);
            met = sprintf('Remaining path --\nTravel time    --\nEnergy         --\nDeadline       %d s\nActual time    %.1f s\nBattery now    %.1f%%\nTime remaining %.1f s', ...
                cfg.timeLimit,app.simTime,app.batteryNow,max(0,cfg.timeLimit-app.simTime));
        else
            flightCfg=cfg;
            if strcmp(app.speedMode,'BOOST')
                flightCfg.airspeedMps=cfg.model.boostAirspeed;
                flightCfg.strategy='Fastest Feasible Route';
            elseif strcmp(app.speedMode,'ENERGY SAVE')
                flightCfg.airspeedMps=cfg.model.cruiseAirspeed;
                flightCfg.strategy='Energy Efficient Route';
            end
            firstRemaining=max(1,min(size(app.path,1),app.pathIndex-1));
            remainingPath=app.path(firstRemaining:end,:);
            flightCfg.profileMode=profileModeForStrategy(flightCfg,app.batteryNow, ...
                max(0.1,cfg.timeLimit-app.simTime),remainingPath(1,:),remainingPath(end,:));
            m=pathMetrics(remainingPath,flightCfg,app.obstacles);
            slack=cfg.timeLimit-app.simTime-m.time;
            after=app.batteryNow-m.energy;
            fitTime=(app.simTime+m.time)<=cfg.timeLimit;
            fitEnergy=(after>=cfg.model.reserveSOC);
            if fitTime, timeFlag='OK'; else, timeFlag='OVER'; end
            if fitEnergy, energyFlag='OK'; else, energyFlag='TIGHT'; end
            s=sprintf('STATE: %s | CONTEXT: %s\nWorld: %d obstacles (%d moving)\nPlanner: %s\nFeasibility: time %s / energy %s', ...
                app.mode,app.context,numel(app.obstacles),sum([app.obstacles.moving]),cfg.strategy,timeFlag,energyFlag);
            if isinf(m.minClearance), cl='n/a'; else, cl=sprintf('%.1f m',m.minClearance); end
            displayWind=cfg.windSpeed*[cosd(cfg.windDirection) sind(cfg.windDirection)];
            displayUnit=[1 0];
            if size(remainingPath,1)>=2
                displayUnit=remainingPath(2,:)-remainingPath(1,:);
                displayUnit=displayUnit/max(norm(displayUnit),eps);
            end
            displayVa=selectAirSpeed(1,displayUnit,displayWind,flightCfg);
            displayPower=multirotorPowerW(displayVa,flightCfg.model);
            met=sprintf('Path length  %6.1f m\nEst. time    %6.1f s\nEnergy       %6.2f Wh\nSOC used     %6.2f %%\nArrival SOC  %6.1f %%\nModel power  %6.1f W\nMin clearance %s\nDeadline slack %+6.1f s\nActual time  %6.1f s\nActual SOC   %6.1f %%', ...
                m.length,m.time,m.energyWh,m.energy,after,displayPower,cl,slack,app.simTime,app.batteryNow);
        end
        set(app.statusText,'String',s);
        set(app.metricsText,'String',met);
        if isinf(app.obstacleDistance)
            obstacleText='no obstacle in range';
        else
            obstacleText=sprintf('%.1f m',app.obstacleDistance);
        end
        decisionString=sprintf(['%s\nContext: %s | gap: %s\nAction: %s\n' ...
            'Priority: %.2f | E/S/T/R/M: %.2f/%.2f/%.2f/%.2f/%.2f\n' ...
            'Position: (%.0f, %.0f) | distance %.1f m\n' ...
            'Replans: %d | safety rejects: %d\nMin clearance: %s | speed: %s'], ...
            app.lastReason,app.context,obstacleText,contextAction,cfg.missionPriority, ...
            wE,wS,wT,wR,wM,app.position(1),app.position(2),app.distanceTravelled, ...
            app.replanCount,app.safetyRejections,clearanceText(app.minClearance),app.speedMode);
        if ~isempty(app.lastAlert)
            decisionString=sprintf('%s\n\n!!! MISSION ALERT !!!\n%s',decisionString,app.lastAlert);
        end
        set(app.decisionText,'String',decisionString);
        drawnow limitrate;
    end
end

%% ======================== LOCAL MODEL FUNCTIONS ========================
function obstacles = generateObstacles(n,nMoving,startPos,goalPos,W,H)
% Generate circular and rectangular footprints; start / goal retain buffers.
    template=struct('x',0,'y',0,'x0',0,'y0',0,'rx',1.5,'ry',1.5, ...
        'isCircle',true,'moving',false,'ampX',0,'ampY',0, ...
        'phaseX',0,'phaseY',0,'omegaX',0,'omegaY',0);
    obstacles=repmat(template,0,1);
    placed=0; tries=0; maxTries=max(500,n*180);
    while placed<n && tries<maxTries
        tries=tries+1;
        isCircle=rand<0.58;
        if isCircle
            rx=1.1+1.0*rand; ry=rx;
        else
            rx=1.25+1.35*rand; ry=0.9+1.25*rand;
        end
        x=4.0+(W-8.0)*rand;
        y=4.0+(H-8.0)*rand;
        if norm([x y]-startPos)<max(rx,ry)+4.0 || norm([x y]-goalPos)<max(rx,ry)+4.0
            continue;
        end
        tooClose=false;
        for kk=1:numel(obstacles)
            old=obstacles(kk);
            if norm([x y]-[old.x0 old.y0]) < 0.72*(max(rx,ry)+max(old.rx,old.ry))
                tooClose=true; break;
            end
        end
        if tooClose && rand<0.85, continue; end
        placed=placed+1;
        obstacles(placed)=template;
        obstacles(placed).x=x; obstacles(placed).y=y;
        obstacles(placed).x0=x; obstacles(placed).y0=y;
        obstacles(placed).rx=rx; obstacles(placed).ry=ry;
        obstacles(placed).isCircle=isCircle;
    end
    nMoving=min(nMoving,numel(obstacles));
    if nMoving>0
        ids=randperm(numel(obstacles),nMoving);
        for kk=ids
            ob=obstacles(kk);
            obstacles(kk).moving=true;
            marginX=max(0,min(ob.x-ob.rx-1.5,W-ob.x-ob.rx-1.5));
            marginY=max(0,min(ob.y-ob.ry-1.5,H-ob.y-ob.ry-1.5));
            obstacles(kk).ampX=min(3.0,marginX);
            obstacles(kk).ampY=min(2.7,marginY);
            obstacles(kk).phaseX=2*pi*rand;
            obstacles(kk).phaseY=2*pi*rand;
            obstacles(kk).omegaX=0.25+0.45*rand;
            obstacles(kk).omegaY=0.20+0.42*rand;
        end
    end
end

function obstacles=updateObstaclePositions(obstacles,t)
    for ii=1:numel(obstacles)
        ob=obstacles(ii);
        if ob.moving
            obstacles(ii).x=ob.x0+0.5*ob.ampX*(sin(ob.omegaX*t+ob.phaseX)-sin(ob.phaseX));
            obstacles(ii).y=ob.y0+0.5*ob.ampY*(sin(ob.omegaY*t+ob.phaseY)-sin(ob.phaseY));
        else
            obstacles(ii).x=ob.x0; obstacles(ii).y=ob.y0;
        end
    end
end

function [path,ok] = planRoute(startPos,goalPos,obstacles,cfg,battery,timeRemaining)
% 8-neighbour A*. Diagonal corner cutting is forbidden. All obstacle cells are
% inflated by safety buffer plus body radius. ACMFDS Adaptive evaluates the five
% objective cost; baseline modes retain their single-objective costs.
    path=[]; ok=false;
    W=60; H=40;
    xVals=1:W; yVals=1:H;
    [XX,YY]=meshgrid(xVals,yVals);
    margin=cfg.safetyMargin+cfg.model.bodyRadiusM;
    cfg.referenceDistance=max(1,norm(goalPos-startPos));
    cfg.missionVector=goalPos-startPos;
    if ~isfield(cfg,'profileMode') || isempty(cfg.profileMode)
        cfg.profileMode=profileModeForStrategy(cfg,battery,timeRemaining,startPos,goalPos);
    end
    cfg.referenceGroundSpeed=max(0.1,cfg.model.boostAirspeed+cfg.windSpeed);
    cfg.referenceTime=cfg.referenceDistance/cfg.referenceGroundSpeed;
    cfg.referenceEnergy=max(1e-9,energyLowerBoundPerMeter(cfg)*cfg.referenceDistance);
    blocked=false(H,W); clearance=inf(H,W);
    for ii=1:numel(obstacles)
        ob=obstacles(ii);
        dx=abs(XX-ob.x); dy=abs(YY-ob.y);
        if ob.isCircle
            occ=((dx./(ob.rx+margin)).^2+(dy./(ob.ry+margin)).^2)<=1;
            q=sqrt(dx.^2+dy.^2)-ob.rx;
        else
            occ=(dx<=ob.rx+margin)&(dy<=ob.ry+margin);
            outX=max(dx-ob.rx,0); outY=max(dy-ob.ry,0);
            q=sqrt(outX.^2+outY.^2);
            inside=(dx<ob.rx)&(dy<ob.ry);
            q(inside)=-min(ob.rx-dx(inside),ob.ry-dy(inside));
        end
        blocked=blocked|occ;
        clearance=min(clearance,q);
    end
    [sc,sr]=worldToGrid(startPos,W,H);
    [gc,gr]=worldToGrid(goalPos,W,H);
    if blocked(gr,gc), return; end
    blocked(sr,sc)=false; % Allow the drone to leave its current cell after a replan.
    startId=sub2ind([H W],sr,sc); goalId=sub2ind([H W],gr,gc);
    if startId==goalId, path=goalPos; ok=true; return; end

    nNodes=W*H;
    g=inf(nNodes,1); f=inf(nNodes,1); parent=zeros(nNodes,1);
    open=false(nNodes,1); closed=false(nNodes,1);
    g(startId)=0;
    f(startId)=heuristic(startPos,goalPos,cfg,battery,timeRemaining);
    open(startId)=true;
    offsets=[0 1; 1 0; 0 -1; -1 0; 1 1; 1 -1; -1 1; -1 -1]; % dc,dr
    windVec=cfg.windSpeed*[cosd(cfg.windDirection) sind(cfg.windDirection)];
    while any(open)
        ids=find(open); [~,minIdx]=min(f(ids)); cur=ids(minIdx);
        if cur==goalId
            idsPath=cur;
            while cur~=startId
                cur=parent(cur);
                if cur==0, path=[]; return; end
                idsPath(end+1)=cur; %#ok<AGROW>
            end
            idsPath=fliplr(idsPath);
            [rows,cols]=ind2sub([H W],idsPath);
            path=[cols(:) rows(:)];
            path(1,:)=startPos; path(end,:)=goalPos;
            ok=true; return;
        end
        open(cur)=false; closed(cur)=true;
        [r,c]=ind2sub([H W],cur);
        for jj=1:size(offsets,1)
            dc=offsets(jj,1); dr=offsets(jj,2);
            nc=c+dc; nr=r+dr;
            if nc<1||nc>W||nr<1||nr>H||blocked(nr,nc), continue; end
            if dc~=0 && dr~=0
                if blocked(r,nc)||blocked(nr,c), continue; end
            end
            nid=sub2ind([H W],nr,nc);
            if closed(nid), continue; end
            delta=[dc dr]; dist=norm(delta); unit=delta/dist;
            [stepTime,stepEnergy]=edgePhysics(dist,unit,windVec,cfg,0);
            % Wind triangle may make an edge physically untrackable at this
            % commanded airspeed (e.g., crosswind >= airspeed or excessive headwind).
            if ~isfinite(stepTime) || ~isfinite(stepEnergy), continue; end
            physicalClear=max(0,clearance(nr,nc)-cfg.model.bodyRadiusM);
            % Dimensionless clearance-risk index in [0,1]; obstacle inflation is
            % the hard safety constraint; this term only prefers extra clearance.
            risk=1/(1+physicalClear/max(0.25,cfg.safetyMargin));
            switch cfg.strategy
                case 'Shortest Safe Route'
                    edge=dist;
                case 'Fastest Feasible Route'
                    edge=stepTime;
                case 'Energy Efficient Route'
                    edge=stepEnergy;
                otherwise
                    % Five-objective ACMFDS edge cost:
                    % E=energy, S=safety risk, T=time, R=tracking, M=mission progress.
                    localObstacleDistance=max(0,clearance(nr,nc)-cfg.model.bodyRadiusM);
                    localContext=detectFlightContext( ...
                        battery,cfg.windSpeed,localObstacleDistance,cfg);
                    [wE,wS,wT,wR,wM]=adaptiveWeights( ...
                        cfg,battery,timeRemaining,norm(goalPos-startPos),localContext);
                    trackScale=max(2,0.12*cfg.referenceDistance);
                    crossTrack=pointLineDistance([nc nr],startPos,goalPos);
                    trackingCost=min(1,crossTrack/trackScale);
                    dBefore=norm([c r]-goalPos);
                    dAfter=norm([nc nr]-goalPos);
                    progressFraction=max(0,min(1,(dBefore-dAfter)/max(dist,eps)));
                    missionCost=1-progressFraction;
                    edge=wE*(stepEnergy/cfg.referenceEnergy) + ...
                        wS*risk*(dist/cfg.referenceDistance) + ...
                        wT*(stepTime/cfg.referenceTime) + ...
                        wR*trackingCost*(dist/cfg.referenceDistance) + ...
                        wM*missionCost*(dist/cfg.referenceDistance);
            end
            tentative=g(cur)+edge;
            if tentative<g(nid)
                parent(nid)=cur; g(nid)=tentative;
                f(nid)=tentative+heuristic([nc nr],goalPos,cfg,battery,timeRemaining);
                open(nid)=true;
            end
        end
    end
end

function h=heuristic(p,goal,cfg,battery,timeRemaining)
% Admissible lower bounds for each edge objective. The wind-triangle bound
% V_ground <= V_air + |wind| is used as an optimistic (never-too-slow) limit.
    d=norm(goal-p);
    maxGround=max(0.1,cfg.model.boostAirspeed+cfg.windSpeed);
    switch cfg.strategy
        case 'Shortest Safe Route'
            h=d;
        case 'Fastest Feasible Route'
            h=d/maxGround;
        case 'Energy Efficient Route'
            lowerTime=d/maxGround;
            lowerSOC=energyLowerBoundPerMeter(cfg)*d;
            h=lowerSOC;
        otherwise
            refD=max(1,cfg.referenceDistance);
            refT=max(1e-9,cfg.referenceTime);
            refE=max(1e-9,cfg.referenceEnergy);
            lowerTime=d/maxGround;
            lowerSOC=energyLowerBoundPerMeter(cfg)*d;
            % Weights may change with local context. For an admissible heuristic,
            % use the minimum E and T weights across all four contexts; omit the
            % nonnegative safety, tracking and mission penalties.
            contexts={'NORMAL','HIGH WIND','OBSTACLE NEARBY','LOW BATTERY'};
            minWE=inf; minWT=inf;
            for jj=1:numel(contexts)
                [testWE,~,testWT,~,~]=adaptiveWeights( ...
                    cfg,battery,timeRemaining,refD,contexts{jj});
                minWE=min(minWE,testWE);
                minWT=min(minWT,testWT);
            end
            h=minWE*(lowerSOC/refE)+minWT*(lowerTime/refT);
    end
end

function context=detectFlightContext(battery,windSpeed,obstacleDistance,cfg)
% Explicit context priority: LOW BATTERY, OBSTACLE NEARBY, HIGH WIND, NORMAL.
    if battery<=cfg.contextBatteryThreshold
        context='LOW BATTERY';
    elseif obstacleDistance<=cfg.contextObstacleThreshold
        context='OBSTACLE NEARBY';
    elseif windSpeed>=cfg.contextWindThreshold
        context='HIGH WIND';
    else
        context='NORMAL';
    end
end

function [wE,wS,wT,wR,wM]=adaptiveWeights(cfg,battery,timeRemaining,distance,context)
% Transparent five-objective policy; weights are simulation design choices, not
% physical constants or values claimed from the cited power-model paper.
    switch context
        case 'OBSTACLE NEARBY'
            base=[0.08 0.50 0.08 0.16 0.18];
        case 'HIGH WIND'
            base=[0.10 0.42 0.12 0.16 0.20];
        case 'LOW BATTERY'
            base=[0.48 0.22 0.08 0.08 0.14];
        otherwise
            base=[0.18 0.28 0.18 0.18 0.18];
    end

    % Add time pressure when normal-speed ETA would consume most of the deadline.
    if isfield(cfg,'missionVector') && norm(cfg.missionVector)>0
        dir=cfg.missionVector/norm(cfg.missionVector);
    else
        dir=[1 0];
    end
    wind=cfg.windSpeed*[cosd(cfg.windDirection) sind(cfg.windDirection)];
    [vg,canTrack]=groundSpeedForTrack(dir,wind,cfg.model.cruiseAirspeed);
    if canTrack
        etaCruise=distance/vg;
    else
        etaCruise=inf;
    end
    timePressure=min(1,etaCruise/max(0.1,timeRemaining));
    if timePressure>=0.85
        base(3)=base(3)+0.08;
    end

    % Mission priority is fixed for a run. Scale its objective and normalize all
    % five weights to sum to one. cfg.missionPriority is constrained to [0,1].
    missionPriority=min(1,max(0,cfg.missionPriority));
    base(5)=base(5)*(0.5+missionPriority);
    base=base/sum(base);
    wE=base(1); wS=base(2); wT=base(3); wR=base(4); wM=base(5);
end

function d=pointLineDistance(p,a,b)
% Perpendicular distance from point p to the infinite line through a and b.
    ab=b-a;
    if norm(ab)<eps
        d=norm(p-a);
    else
        ap=p-a;
        d=abs(det([ab;ap]))/norm(ab);
    end
end

function mode=profileModeForStrategy(cfg,battery,timeRemaining,startPos,goalPos)
% Speed-mode arbitration. FAST uses the configured urgent airspeed; ENERGY selects
% the minimum model energy-per-segment among discretized admissible airspeeds;
% CRUISE uses the user-configured nominal airspeed. The profile choice is part of
% the system design and is reported as a policy decision, not as a physical law.
    switch cfg.strategy
        case 'Shortest Safe Route'
            mode='CRUISE'; return;
        case 'Fastest Feasible Route'
            mode='FAST'; return;
        case 'Energy Efficient Route'
            mode='ENERGY'; return;
        otherwise
            d=norm(goalPos-startPos);
            if d<eps, mode='CRUISE'; return; end
            available=max(0.1,battery-cfg.model.reserveSOC);
            energyPressure=min(1,(d*energyLowerBoundPerMeter(cfg))/available);
            % Adaptive planning may penalize slow routes under time pressure, but
            % it does not silently boost. A faster profile is selected only by the
            % explicit feasibility/recovery checks in the mission controller.
            if battery<=cfg.model.lowBatteryTriggerSOC || energyPressure>=0.65
                mode='ENERGY';
            else
                mode='CRUISE';
            end
    end
end

function rate=energyLowerBoundPerMeter(cfg)
% Optimistic energy-per-distance lower bound used only by A* heuristics and
% normalization. Since ground speed cannot exceed Va + |wind|, P/(Va+|wind|)
% is an admissible lower bound for each candidate airspeed. The minimum over
% the configured speed interval preserves the lower-bound property.
    low=cfg.model.cruiseAirspeed;
    high=min(cfg.model.boostAirspeed,cfg.model.maxAirspeed);
    candidates=linspace(low,high,11);
    maxGS=@(va) max(0.1,va+cfg.windSpeed);
    rates=zeros(size(candidates));
    for kk=1:numel(candidates)
        P=multirotorPowerW(candidates(kk),cfg.model);
        rates(kk)=P/maxGS(candidates(kk))/3600/cfg.model.batteryCapacityWh*100;
    end
    rate=min(rates);
end

function [ix,iy]=worldToGrid(pos,W,H)
    ix=max(1,min(W,round(pos(1))));
    iy=max(1,min(H,round(pos(2))));
end

function [segTime,energyPct,heading,turnAngle]=segmentCost(p0,p1,cfg,previousHeading)
    delta=p1-p0; dist=norm(delta);
    if dist<eps, heading=previousHeading; segTime=0; energyPct=0; turnAngle=0; return; end
    heading=delta/dist;
    wind=cfg.windSpeed*[cosd(cfg.windDirection) sind(cfg.windDirection)];
    [segTime,energyPct,turnAngle]=edgePhysics(dist,heading,wind,cfg,previousHeading);
end

function [tt,energyPct,turnDeg,energyWh,powerW]=edgePhysics(dist,unit,wind,cfg,previousHeading)
% Ground-track feasibility from the wind triangle. Wind is defined as a vector
% "towards" cfg.windDirection in the 2-D East/North plane. For unit ground-track d,
% a commanded airspeed Va admits Vg = W_parallel + sqrt(Va^2 - W_cross^2).
% If W_cross exceeds Va or Vg <= 0, the edge is not flyable at that airspeed.
    if nargin<5, previousHeading=[1 0]; end
    turnDeg=0;
    if norm(previousHeading)>0 && norm(unit)>0
        prev=previousHeading/max(norm(previousHeading),eps);
        turnDeg=acosd(max(-1,min(1,dot(prev,unit))));
    end
    Va=selectAirSpeed(dist,unit,wind,cfg);
    if ~isfinite(Va) || Va<=0
        tt=inf; energyPct=inf; energyWh=inf; powerW=inf; return;
    end
    wParallel=dot(wind,unit);
    wCross=norm(wind-wParallel*unit);
    discriminant=Va^2-wCross^2;
    if discriminant<0
        tt=inf; energyPct=inf; energyWh=inf; powerW=inf; return;
    end
    groundSpeed=wParallel+sqrt(max(0,discriminant));
    if groundSpeed<=0
        tt=inf; energyPct=inf; energyWh=inf; powerW=inf; return;
    end
    tt=dist/groundSpeed;
    powerW=multirotorPowerW(Va,cfg.model);
    energyWh=powerW*tt/3600;
    energyPct=100*energyWh/cfg.model.batteryCapacityWh;
end

function Va=selectAirSpeed(dist,unit,wind,cfg)
% Select an airspeed from the explicit model/strategy settings. Energy mode tests
% 11 speeds between the user-defined cruise and boost speeds and selects the one
% with lowest modeled segment Wh; this is a finite-resolution optimization, not a
% claim of continuous global optimality.
    mode='CRUISE';
    if isfield(cfg,'profileMode'), mode=cfg.profileMode;
    elseif strcmp(cfg.strategy,'Fastest Feasible Route'), mode='FAST';
    elseif strcmp(cfg.strategy,'Energy Efficient Route'), mode='ENERGY';
    end
    switch mode
        case 'FAST'
            Va=min(cfg.model.boostAirspeed,cfg.model.maxAirspeed);
        case 'ENERGY'
            candidates=linspace(cfg.model.cruiseAirspeed, ...
                min(cfg.model.boostAirspeed,cfg.model.maxAirspeed),11);
            bestWh=inf; Va=nan;
            for jj=1:numel(candidates)
                testVa=candidates(jj);
                [vg,ok]=groundSpeedForTrack(unit,wind,testVa);
                if ~ok, continue; end
                wh=multirotorPowerW(testVa,cfg.model)*(dist/vg)/3600;
                if wh<bestWh
                    bestWh=wh; Va=testVa;
                end
            end
            if ~isfinite(Va), Va=nan; end
        otherwise
            Va=cfg.model.cruiseAirspeed;
    end
end

function [groundSpeed,feasible]=groundSpeedForTrack(unit,wind,Va)
% Exact steady wind-triangle relationship for a commanded ground-track direction.
    parallel=dot(wind,unit);
    cross=norm(wind-parallel*unit);
    disc=Va^2-cross^2;
    if disc<0
        groundSpeed=nan; feasible=false; return;
    end
    groundSpeed=parallel+sqrt(max(0,disc));
    feasible=isfinite(groundSpeed) && groundSpeed>0;
end

function P=multirotorPowerW(Vair,m)
% Steady horizontal-flight case (V_perp=0) of Eq. (19) from Gong et al. (2022),
% "Modelling Power Consumptions for Multi-rotor UAVs", arXiv:2209.04128.
% Parameters delta, rho, solidity, A, k, CT and S_FP_parallel follow their generic
% reference parameter set. W is weight [N]; A is disk area PER rotor [m^2].
% This model omits acceleration, turns, climb/descent, battery voltage dynamics,
% motor/ESC loss changes and gust-response power; it must be parameterized for a
% particular aircraft before interpreting SOC estimates as flight predictions.
    n=m.rotorCount;
    W=m.massKg*9.81;
    rho=m.airDensity;
    A=m.rotorAreaM2;
    delta=m.profileDragCoeff;
    solidity=m.rotorSolidity;
    CT=m.thrustCoeff;
    k=m.inducedCorrection;
    Sfp=m.fuselageAreaM2;
    Wr=W/n;

    % Equation (4)/(5): hover profile power + induced power, summed over rotors.
    profileHoverRotor=Wr^(3/2)*rho^(-1/2)*solidity*A^(-1/2)*CT^(-3/2)*delta/8;
    inducedRotor=(1+k)*Wr^(3/2)/sqrt(2*rho*A);
    Pmh=n*(profileHoverRotor+inducedRotor);
    Pin=n*inducedRotor;
    % Use the paper's tabulated v0 reference (6.325 m/s) as an explicit
    % parameter. When mass/rotor parameters are customized, recalibrate v0 from
    % the selected vehicle's data or document the derivation used.
    v0=m.v0Mps;

    % Horizontal-flight case of Eq. (19), setting vertical speed V_perp=0:
    % forward-flight profile increment, induced-power change, and fuselage drag.
    % Vair is speed relative to the air mass, not ground-track speed.
    profileForward=(3/8)*sqrt(n)*delta*sqrt(W*rho*A/CT)*solidity*Vair^2;
    inner=sqrt(1+Vair^4/(4*v0^4))-Vair^2/(2*v0^2);
    inducedForward=Pin*(sqrt(max(0,inner))-1);
    parasitic=(n/2)*Sfp*rho*Vair^3;
    P=Pmh+profileForward+inducedForward+parasitic;
    if ~isfinite(P) || P<=0
        P=inf;
    end
end

function m=pathMetrics(path,cfg,obstacles)
    m=struct('length',0,'time',0,'energy',0,'energyWh',0,'minClearance',inf,'collisionSamples',0);
    if isempty(path)||size(path,1)<2, return; end
    if ~isfield(cfg,'profileMode')
        cfg.profileMode=profileModeForStrategy(cfg,cfg.initialBattery,cfg.timeLimit,path(1,:),path(end,:));
    end
    prev=[1 0]; wind=cfg.windSpeed*[cosd(cfg.windDirection) sind(cfg.windDirection)];
    for ii=2:size(path,1)
        p0=path(ii-1,:); p1=path(ii,:); delta=p1-p0; d=norm(delta);
        if d<eps, continue; end
        u=delta/d;
        [tt,en,~,eWh]=edgePhysics(d,u,wind,cfg,prev);
        if ~isfinite(tt) || ~isfinite(en)
            m.time=inf; m.energy=inf; m.energyWh=inf; return;
        end
        m.length=m.length+d; m.time=m.time+tt; m.energy=m.energy+en; m.energyWh=m.energyWh+eWh;
        prev=u;
        % Sample each edge to compute minimum clearance and direct-reference conflicts.
        ns=max(2,ceil(d/0.2));
        for kk=0:ns
            p=p0+(kk/ns)*delta;
            [clr,collides]=nearestClearance(p,obstacles,cfg.model.bodyRadiusM);
            m.minClearance=min(m.minClearance,clr);
            if collides, m.collisionSamples=m.collisionSamples+1; end
        end
    end
end

function [clearance,collides]=nearestClearance(p,obstacles,bodyRadiusM)
    clearance=inf; collides=false;
    for ii=1:numel(obstacles)
        ob=obstacles(ii); dx=abs(p(1)-ob.x); dy=abs(p(2)-ob.y);
        if ob.isCircle
            q=hypot(dx,dy)-ob.rx;
            inside=(hypot(dx,dy)<ob.rx);
        else
            ox=max(dx-ob.rx,0); oy=max(dy-ob.ry,0);
            q=hypot(ox,oy);
            if dx<ob.rx && dy<ob.ry
                q=-min(ob.rx-dx,ob.ry-dy);
            end
            inside=dx<ob.rx && dy<ob.ry;
        end
        clearance=min(clearance,q-bodyRadiusM);
        if inside || (ob.isCircle && q<bodyRadiusM) || (~ob.isCircle && dx<ob.rx+bodyRadiusM && dy<ob.ry+bodyRadiusM)
            collides=true;
        end
    end
end

function safe=segmentIsSafe(p0,p1,t0,t1,obstaclesAtT1,cfg)
% Conservative swept-segment checks: multiple samples in space and time.
    safe=true;
    samples=linspace(0,1,7);
    % Recover trajectories from stored base positions. obstaclesAtT1 retains x0/y0.
    for kk=1:numel(samples)
        a=samples(kk); p=p0+a*(p1-p0); t=t0+a*(t1-t0);
        obs=updateObstaclePositions(obstaclesAtT1,t);
        margin=cfg.safetyMargin+cfg.model.bodyRadiusM;
        for jj=1:numel(obs)
            ob=obs(jj); dx=abs(p(1)-ob.x); dy=abs(p(2)-ob.y);
            if ob.isCircle
                hit=(dx/(ob.rx+margin))^2+(dy/(ob.ry+margin))^2 <= 1;
            else
                hit=dx<=ob.rx+margin && dy<=ob.ry+margin;
            end
            if hit
                safe=false; return;
            end
        end
    end
end

function txt=decisionReason(cfg,m,C) %#ok<INUSD>
    bits={};
    P=multirotorPowerW(cfg.airspeedMps,cfg.model);
    groundUpper=cfg.model.boostAirspeed+cfg.windSpeed;
    if strcmp(cfg.strategy,'ACMFDS Adaptive')
        if cfg.initialBattery<=cfg.model.lowBatteryTriggerSOC
            bits{end+1}=sprintf('Low SOC (%.1f%%) enters the energy/recovery policy; reserve %.1f%% is a hard constraint.',cfg.initialBattery,cfg.model.reserveSOC); %#ok<AGROW>
        else
            bits{end+1}='Adaptive planner evaluates energy, safety, time, tracking deviation and mission progress.'; %#ok<AGROW>
        end
        if cfg.windSpeed>0
            bits{end+1}=sprintf('Wind triangle: target airspeed %.1f m/s; ideal upper bound on ground speed %.1f m/s.',cfg.airspeedMps,groundUpper); %#ok<AGROW>
        end
        if cfg.timeLimit>0 && m.time/cfg.timeLimit>=0.85
            bits{end+1}='Predicted ETA uses >=85% of the deadline; time-priority weights are active.'; %#ok<AGROW>
        else
            bits{end+1}='Time pressure is below the deadline-priority threshold.'; %#ok<AGROW>
        end
        bits{end+1}=sprintf('Gong et al. Eq. (19) steady horizontal-flight estimate: %.1f W at %.1f m/s airspeed.',P,cfg.airspeedMps); %#ok<AGROW>
        bits{end+1}='Generic reference parameters are not a vehicle calibration; battery capacity and speed bounds must be supplied/validated.'; %#ok<AGROW>
        bits{end+1}='A* treats inflated obstacle footprints as hard constraints; no overflight is modeled.'; %#ok<AGROW>
    elseif strcmp(cfg.strategy,'Shortest Safe Route')
        bits={'Minimizes geometric path length subject to obstacle inflation and wind-track feasibility.'; ...
            'Blocked cells and diagonal corner cutting are forbidden.'};
    elseif strcmp(cfg.strategy,'Fastest Feasible Route')
        bits={'Minimizes wind-triangle travel time subject to obstacle inflation.'; ...
            'Edges are rejected if the requested airspeed cannot make forward progress against/crossing wind.'};
    else
        bits={'Minimizes energy in Wh converted to SOC using the declared battery capacity.'; ...
            'Steady-flight power is calculated with the cited multirotor model; vehicle-specific calibration is still required.'};
    end
    if isfinite(m.time) && m.time>cfg.timeLimit
        bits{end+1}='WARNING: estimated route exceeds the mission deadline.'; %#ok<AGROW>
    end
    if ~isfinite(m.energy) || cfg.initialBattery-m.energy<cfg.model.reserveSOC
        bits{end+1}=sprintf('WARNING: route violates the %.1f%% SOC reserve.',cfg.model.reserveSOC); %#ok<AGROW>
    end
    if strcmp(cfg.strategy,'ACMFDS Adaptive')
        bits{end+1}=sprintf(['The normalized objective cost includes energy, safety, time, ' ...
            'tracking and mission; mission priority is fixed at %.2f.'],cfg.missionPriority); %#ok<AGROW>
    end
    txt=strjoin(bits,newline);
end

function s=shortName(name)
    switch name
        case 'Shortest Safe Route', s='Shortest';
        case 'Fastest Feasible Route', s='Fastest';
        case 'Energy Efficient Route', s='Energy';
        otherwise, s='ACMFDS';
    end
end

function s=clearanceText(x)
    if isinf(x), s='n/a'; else, s=sprintf('%.1f m',x); end
end
