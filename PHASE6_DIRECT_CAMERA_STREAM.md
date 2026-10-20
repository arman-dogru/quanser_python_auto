# Phase 6: Direct downward-camera stream

This path keeps YOLO on the ground computer and removes MATLAB/Windows GUI
capture completely.

## Data path

OV7251 device 2 -> QUARC Video Capture in the Commander -> TCP 18910 ->
ground `yolo_pad_direct_stream.py` -> TCP 18901 -> Mission Server interrupt.

The flight Commander is `QDrone_YOLO_Direct_Stream_Commander`.
Do not run `QDrone_Downward_Camera_Passive` or
`QDrone_YOLO_Integrated_Camera_Commander` with this path.

## First test (propellers removed)

1. Build `QDrone_YOLO_Direct_Stream_Commander` using
   `build_phase6_direct_camera_stream(false)` and then `true`.
2. On the ground computer, from the project root, start:
   `python .\python\yolo_pad_direct_stream.py`
3. Start `QDrone_YOLO_Direct_Stream_Commander` in Monitor & Tune. Its QUARC
   Stream Client connects to the ground listener on port 18910.
6. Move the secured landing pad through the camera view. Confirm the direct
   YOLO preview updates and the console reports both detections and misses.
7. Stop both processes. Do not proceed to flight until the preview remains live
   for at least two minutes and frame age remains bounded.

## Flight order after the ground test passes

1. Motive streaming.
2. Ground `yolo_pad_direct_stream.py`.
3. `Mission_Server_QDrone_YOLO_Interrupt` in Monitor & Tune.
4. `QDrone_YOLO_Direct_Stream_Commander` in Monitor & Tune.
6. Arm and take off only after tracking, camera frames, and YOLO observation
   freshness are all healthy.

Landing/termination closes the camera connection. Ground YOLO remains running
and accepts the next Commander connection automatically.
