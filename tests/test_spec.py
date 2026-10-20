"""The documented Stage 6 facts, checked against the parsed .slx archive."""
from __future__ import annotations

import json
import subprocess
import sys

from conftest import ROOT

SPEC = json.loads((ROOT / "docs/spec/QDrone_YOLO_Stage6_RealYOLO_MonitorOnly.json").read_text())
INTEG = json.loads((ROOT / "docs/spec/QDrone_YOLO_Recenter_Integration_Test.json").read_text())
MS = "MISSION SERVER - LEADER/FOLLOWER with QBot 2e and QDrone/"


def block(spec, path):
    b = next(b for b in spec["blocks"] if b["path"] == path)
    return {**b["defaults_applied"], **b["params"]}


def lines_from(spec, src):
    return {ln["dst"] for ln in spec["lines"] if ln["src"] == src}


def test_spec_outputs_are_fresh():
    r = subprocess.run([sys.executable, str(ROOT / "tools/slx_spec.py"), "--check"], capture_output=True, text=True)
    assert r.returncode == 0, r.stdout + r.stderr


def test_snapshot_still_verifies():
    r = subprocess.run([sys.executable, str(ROOT / "tools/verify_snapshot.py")], capture_output=True, text=True)
    assert r.returncode == 0 and "Verified 22 source files and 13 embedded functions" in r.stdout


def test_solver_and_model_arguments():
    active = SPEC["solver"]["config_sets"]["simulink/configSet2.xml"]
    assert active["FixedStep"] == "0.001" and active["SystemTargetFile"] == "quarc_win64.tlc"
    assert SPEC["model_arguments"] == {"URI_Leader": "tcpip://localhost:18001",
                                       "URI_Follower": "tcpip://localhost:18002"}


def test_detection_stream_block():
    b = block(SPEC, MS + "Stream Server")
    assert b["uri"] == "udp://localhost:5010?peer=any"
    assert b["data_type"] == "double" and b["default_value"] == "[0 0 0]"
    assert b["byte_order"].startswith("little endian") and b["sample_time"] == "0.033"
    assert b["send_options"] == "Do not send data" and b["receive_options"] == "Receive most recent data"
    # rcv (3rd output) -> Demux -> detected / error_x / error_y; 'new' only goes to a Display
    assert MS + "Demux#in:1" in lines_from(SPEC, MS + "Stream Server#out:3")
    assert lines_from(SPEC, MS + "Stream Server#out:4") == {MS + "Display5#in:1"}
    for k in (1, 2, 3):
        assert MS + f"YOLOPoseCommandGenerator#in:{k + 1}" in lines_from(SPEC, MS + f"Demux#out:{k}")


def test_monitor_only_switch_and_finding_f1():
    assert block(SPEC, MS + "use_yolo")["Value"] == "0"
    sw = block(SPEC, MS + "Switch1")
    assert sw["Criteria"] == "u2 > Threshold" and sw["Threshold"] == "0.5"
    assert MS + "Switch1#in:3" in lines_from(SPEC, MS + "Mode switch#out:1")
    assert lines_from(SPEC, MS + "Switch1#out:1") == {MS + "Desired  Pose (m, rad) [4]#in:1"}
    # The preceding integration model fed Switch1 from the follower/home switch instead.
    assert MS + "Switch1#in:3" in lines_from(INTEG, MS + "Switch#out:1")
    assert MS + "Switch1#in:3" not in lines_from(SPEC, MS + "Switch#out:1")


def test_controller_rates():
    assert block(SPEC, MS + "Rate Transition")["OutPortSampleTime"] == "0.033"
    assert MS + "YOLOPoseCommandGenerator#in:1" in lines_from(SPEC, MS + "Rate Transition#out:1")
    assert block(SPEC, MS + "Rate Transition1")["OutPortSampleTime"] == "0.001"


def test_optitrack_and_conditioning_constants():
    ot = block(SPEC, MS + "Data Server (Localization) /OptiTrack Trackables")
    assert ot["rigid_body_ids"] == "[1,2]" and ot["sample_time"] == "qc_get_step_size*10"
    assert block(SPEC, MS + "Vehicle ID - QDrone")["Value"] == "2"
    mc = MS + "Measurement Conditioning/"
    assert block(SPEC, mc + "Saturation")["UpperLimit"] == "[2 2 5]"
    assert block(SPEC, mc + "Saturation")["LowerLimit"] == "[-2 -2 0]"
    bias = next(b for b in SPEC["blocks"] if b["path"].startswith(mc + "Subtract Rigid Body"))
    assert bias["params"]["Bias"] == "[0 0 -0.07]"
    conv = MS + "Data Server (Localization) /Convention Correction (Motive Frame to Research Studio Frame) - QDrone/"
    assert block(SPEC, conv + "Rotate Basis from  RSF to MF")["Value"] == "[0 -1 0;0 0 1;-1 0 0]"


def test_comm_loss_threshold():
    th = block(SPEC, "Communication - QDrone/Detect Communication loss/Threshold")
    assert (th["operator"], th["threshold"], th["duration"]) == (">=", "0.5", "0.1")


def test_qdrone_packet_order():
    root = {ln["dst"]: ln["src"] for ln in SPEC["lines"] if ln["system"] == "<root>"}
    for k in range(1, 6):
        assert root[f"Mux#in:{k}"] == MS[:-1] + f"#out:{k}"
    assert root["Communication - QDrone#in:1"] == "Mux#out:1"


def test_charts_mapped_to_instances():
    inst = {c["member"]: c["instance_path"] for c in SPEC["charts"]}
    assert inst["simulink/stateflow/chart_51.xml"].endswith("YOLOPoseCommandGenerator")
    assert inst["simulink/stateflow/chart_33.xml"] == "Communication - QDrone/Detect Communication loss/MATLAB Function"


def test_active_config_set_and_unused_dialog_uris():
    assert SPEC["solver"]["active_config_set"] == {"part": "simulink/configSet2.xml", "name": "Configuration2"}
    used = {(e["block"], e["uri"]): e["used"] for e in SPEC["endpoints"]}
    assert used[("Communication - QDrone/Stream Server", "tcpip://localhost:18877")] is False
    assert used[("Communication - QDrone/Model Argument", "tcpip://localhost:18002")] is True
    assert used[(MS + "JOYSTICK/Host Initialize", "tcpip://localhost:18001")] is False
    assert used[(MS + "Stream Server", "udp://localhost:5010?peer=any")] is True
