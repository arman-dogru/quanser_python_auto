% QDrone autonomous landing - Milestone 1 configuration (SI units)
qdrone.targetIP = '192.168.2.15';
qdrone.hostIP = '192.168.2.11';
qdrone.streamingID = 1;
qdrone.searchHeight = 0.70;       % m, corrected Research Studio Z
qdrone.packetWidth = 14;
qdrone.commandWidth = 13;
qdrone.packetOrder = {'isTracking','arm_disarm','takeoff_autoland','emergency_abort','joystick_issue', ...
    'measured_x','measured_y','measured_z','measured_yaw', ...
    'desired_x','desired_y','desired_z','desired_yaw','timestamp'};

% Corrected Research Studio measurements from 2026-08-12. The measured
% boundary is not perfectly axis-aligned, so Phase 3 uses a conservative
% internal rectangle rather than treating the corner extrema as flyable.
qdrone.xMinMeasured = -2.000;
qdrone.xMaxMeasured =  1.777;
qdrone.yMinMeasured = -1.903;
qdrone.yMaxMeasured =  1.875;
qdrone.xMinSafe = -1.40;
qdrone.xMaxSafe =  1.20;
qdrone.yMinSafe = -1.30;
qdrone.yMaxSafe =  1.20;

qdrone.phase3Waypoints = [ ...
   -0.75, -0.65, 0.70; ...
    0.15, -0.65, 0.70; ...
    0.15,  0.15, 0.70; ...
   -0.75,  0.15, 0.70];
qdrone.waypointTolerance = 0.12;
