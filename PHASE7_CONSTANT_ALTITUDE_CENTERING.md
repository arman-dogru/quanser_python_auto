# Phase 7 — Constant-altitude visual centering

This phase creates `Mission_Server_QDrone_YOLO_Centering.mdl` and uses the
unchanged `QDrone_YOLO_Direct_Stream_Commander.mdl`.

The controller begins only after the existing five-frame YOLO confirmation.
It maps pad-center pixel error to small Research Studio XY desired-pose
increments. Each fresh frame is limited to 5 mm, total displacement from the
detection point is limited to 0.50 m, and XY is clamped to the search bounds.

Altitude remains 0.70 m. Loss or staleness of vision freezes the last desired
pose. This phase contains no descent and cannot trigger LAND.

The initial mapping comes from three synchronized samples in `important.mp4`.
It is intentionally conservative and must be verified one axis at a time in
the first flight. If pixel error grows, land immediately; do not reverse signs
during flight.
