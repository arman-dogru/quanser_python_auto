function params = config()

    % =========================
    % LOOP SETTINGS
    % =========================
    params.dt = 0.05;                 % control loop time step [s]
    params.maxIterations = 3000;      % safety stop for loop

    % =========================
    % FLIGHT AREA LIMITS
    % Must match Motive coordinate system
    % =========================
    params.xMin = -1.0;               % minimum x position [m]
    params.xMax =  1.0;               % maximum x position [m]
    params.yMin = -1.0;               % minimum y position [m]
    params.yMax =  1.0;               % maximum y position [m]

    % =========================
    % SEARCH SETTINGS
    % =========================
    params.searchHeight = 1.0;        % drone height during grid search [m]
    params.gridSpacing = 0.30;        % distance between grid lines [m]
    params.waypointTolerance = 0.08;  % waypoint reached tolerance [m]

    % =========================
    % LANDING SETTINGS
    % =========================
    params.landingHeight = 0.10;      % height considered close to ground [m]
    params.descentSpeed = 0.06;       % downward landing speed [m/s]

    % =========================
    % CONTROLLER GAINS
    % =========================
    params.kp_waypoint = 0.8;         % gain for waypoint tracking
    params.kp_image = 0.001;          % gain for image-based centering

    % =========================
    % SAFETY LIMITS
    % =========================
    params.maxSpeed = 0.3;            % maximum velocity command [m/s]

    % =========================
    % VISION SETTINGS
    % =========================
    params.centerTolerancePixels = 25;
    
    % Resize large images for faster detection
    params.maxDetectionImageSize = 640;
    
    % General grayscale processing
    params.gaussianSigma = 1.0;
    
    % Circle/ring detection
    params.minCircleRadiusFraction = 0.08;
    params.maxCircleRadiusFraction = 0.48;
    params.circleSensitivity = 0.84;
    params.circleEdgeThreshold = 0.12;
    
    % Circle candidate validation
    params.darkInsideThreshold = 0.45;
    params.brightPadThreshold = 0.50;
    params.minLocalContrast = 0.25;
    params.minCircleDarkRatio = 0.06;
    params.maxCircleDarkRatio = 0.65;
    params.minCircleBrightRatio = 0.35;
    
    % Fallback bright body detector
    params.brightPercentile = 70;
    params.brightThresholdMin = 0.40;
    params.minBrightArea = 100;
    params.brightCloseRadius = 12;
    
    params.minPadAreaFraction = 0.003;
    params.maxPadAreaFraction = 0.95;
    
    params.minPadWidth = 25;
    params.minPadHeight = 25;
    
    params.minAspectRatio = 0.35;
    params.maxAspectRatio = 3.00;
    
    params.minDarkInsideRatio = 0.002;
    params.maxDarkInsideRatio = 0.75;
    
    params.darkRatioWeight = 0.30;

    % =========================
    % GRAYSCALE VISION SETTINGS
    % =========================
    params.darkThreshold = 0.35;     % pixels darker than this are considered black markings
    params.minDarkArea = 200;        % remove small dark noise
    params.closeRadius = 20;         % connect nearby black pad markings
    params.minPadArea = 3000;        % minimum region size to accept as landing pad
 

    % =========================
    % GRAYSCALE PAD BODY DETECTION
    % =========================
    params.gaussianSigma = 1.0;
    
    % Bright pad body threshold
    params.brightThreshold = 0.55;
    
    % Remove small bright noise
    params.minBrightArea = 500;
    
    % Connect bright pad areas
    params.brightCloseRadius = 12;
    
    % Candidate pad size filters
    params.minPadBrightArea = 3000;
    params.minPadWidth = 40;
    params.minPadHeight = 40;
    params.maxPadAreaFraction = 0.85;
    
    % Dark markings inside bright pad
    params.darkInsideThreshold = 0.35;
    params.minDarkInsideRatio = 0.01;
    params.maxDarkInsideRatio = 0.60;

    %================================
    params.useFallbackDetector = false;
    

end