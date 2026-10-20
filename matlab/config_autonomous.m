function cfg = config_autonomous()
%CONFIG_AUTONOMOUS Phase 4 grid-search configuration in corrected SI units.
cfg.qdroneIp = '192.168.2.15';
cfg.rigidBodyId = 1;
cfg.searchHeight = 0.70;
cfg.xLimits = [-1.20 1.20];
cfg.yLimits = [-1.00 1.00];
cfg.gridColumns = 5;
cfg.gridRows = 4;
cfg.arrivalTolerance = 0.12;
cfg.dwellSamples = 100;
% Phase 6 detection interrupt. These values only permit a hover hold; they
% do not permit camera-based translation or descent.
cfg.yoloTcpPort = 18901;
cfg.yoloConfidence = 0.60;
cfg.yoloConfirmFrames = 5;
cfg.yoloReleaseFrames = 10;
cfg.yoloStaleSeconds = 0.50;
% Phase 7 constant-altitude centering. This map converts pixel error to
% Research Studio XY movement at 0.70 m. It comes from synchronized YOLO
% and OptiTrack data recorded on 2026-09-22 at approximately 1.1 m, then
% scaled by height. The calibration yaw was close to zero.
cfg.pixelToWorldAtSearchHeight = [ ...
    0.00163  0.00019; ...
    0.00007 -0.00166];
cfg.visionCalibrationYaw = 0.015; % rad, median yaw of selected fit samples
% The downward camera lens is approximately 0.11 m forward of the drone's
% landing center. At the recorded heading, forward is positive world X.
% Pad under body center: targetPixel = opticalCenter - inv(map)*offset.
cfg.cameraOpticalCenterPixel = [320; 240];
cfg.cameraOffsetBody = [0.11; 0]; % metres, body [forward;left]
cfg.visionTargetPixel = cfg.cameraOpticalCenterPixel - ...
    cfg.pixelToWorldAtSearchHeight \ cfg.cameraOffsetBody;
cfg.centerMaxStep = 0.005;       % metres per fresh YOLO frame
cfg.centerMaxDisplacement = 0.50;% metres from detection/hold location
cfg.centerPixelTolerance = 20;   % pixels
cfg.centerDwellFrames = 10;
cfg.waypoints = generateGridWaypoints(cfg.xLimits, cfg.yLimits, ...
    cfg.gridColumns, cfg.gridRows, cfg.searchHeight);
end
