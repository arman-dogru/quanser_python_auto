"""Python port of the QDrone Phase 10 autonomous-landing mission server.

Reference implementation: ``simulink/Mission_Server_QDrone_YOLO_Autonomous_Land.mdl``
(MATLAB R2021b / QUARC), talking to the unchanged vehicle model
``simulink/QDrone_YOLO_Direct_Stream_Commander.mdl`` and the ground vision
publisher ``python/yolo_pad_direct_stream_fresh.py``.
"""

__version__ = "0.1.0"
