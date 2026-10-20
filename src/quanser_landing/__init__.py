"""Python port of the Quanser QDrone autonomous-landing Simulink/MATLAB project.

Subpackages
-----------
controllers
    Pure, stateful ports of the embedded MATLAB Function charts (Stage 6
    YOLOPoseCommandGenerator, the earlier LandingPadRecenterLogic) and the
    legacy MATLAB prototype controllers. Bit-for-bit parity with the
    extracted ``.m`` files is checked against GNU Octave.
blocks
    The other Stage 6 charts (vehicle data extraction, stream connection
    loss) plus Simulink/QUARC block equivalents used around the controller.
detection
    The detection contract, the UDP packet codec seen in the model and a
    YOLO (ultralytics) inference wrapper for recorded frames.
io
    Camera, localization, detection and command adapters as interfaces, with
    loopback implementations and a command gate that keeps the model's
    monitor-only default and interlocks.
"""

__version__ = "0.1.0"
