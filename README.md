# QDrone Autonomous Landing — Python port baseline

Source: `E:\berkay\Quanser_Clean\QDrone_Autonomous_Landing`.
This replaces the unrelated synced-project snapshot previously committed here.

## Selected current implementation

- Mission server: `simulink/Mission_Server_QDrone_YOLO_Autonomous_Land.mdl` (Phase 10).
- Vehicle: `simulink/QDrone_YOLO_Direct_Stream_Commander.mdl`.
- Ground vision: `python/yolo_pad_direct_stream_fresh.py`, with its local helper modules included. The original `yolo_pad_direct_stream.py` is also retained; run only one vision publisher at a time.
- Trained detector: `models/qdrone_landing_pad_yolo11n_v3_best.pt`.
- Mission parameters: `matlab/config_autonomous.m`.
- Site calibration: `config/Calibration_2026-08-11.cal` and `config/RigidBody_01.tra`.

This is a copy of the user's current working project, selected by source inspection. No flight, MATLAB model update, or hardware build was performed during packaging. The protected `working_grid_search_2026-08-13` baseline has its own recorded working-status notes.

## Included files

The original folder layout is preserved so relative imports, model generation, weights, and calibration dependencies remain available. All handwritten MATLAB/Python sources, controller templates, tests, source-original models, intermediate phase models, and the protected working grid-search baseline are included. Earlier phases are dependencies and comparison references, not alternative current entry points.

Generated QUARC/C/C++ builds, compiled targets, model caches/autosaves, recordings, runtime detection files, logs, and duplicate backup directories are excluded. The original milestone-1 README is preserved in `docs/ORIGINAL_MILESTONE1_README.md`; it predates the current landing implementation.

## Python vision setup

From this repository root:

```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install -r requirements.txt
python -m unittest discover -s python -p "test_*.py"
python python/yolo_pad_direct_stream_fresh.py
```

The direct stream path receives Commander camera frames on TCP 18910 and publishes observations on TCP 18901. This command starts vision only; it does not launch the MATLAB mission server or vehicle. `python/qdrone_camera_streamer.py` is a separate vehicle-side alternative requiring the separately installed Quanser Python SDK; it is not needed for the existing direct-stream Commander path.

## Python port of the Phase 10 mission server

`src/qdrone_autoland` ports the Phase 10 mission server to Python, matching the
Commander's packets and interoperating with `python/yolo_pad_direct_stream_fresh.py`
as it is. Its mission logic is checked tick for tick against GNU Octave running
the chart code extracted from the model. A safety layer guards its inputs and
outputs. It **has not flown**: no MATLAB, QUARC or hardware was used, and the
OptiTrack and joystick readers are not written yet, so it cannot fly the
vehicle as it stands. Autonomous LAND is off by default.

```bash
pip install -e ".[dev]"
pytest -q
qautoland sim                     # closed-loop scenarios against a stand-in vehicle
qautoland serve --mode dry-run    # real sockets, sends nothing
```

Details, evidence, figures and findings about the saved models:
[docs/PYTHON_PORT.md](docs/PYTHON_PORT.md). The port adds files under `src/`,
`tests/`, `tools/`, `docs/` and `.github/`, plus `pyproject.toml`; of the
existing files it changes only this README, `.gitignore` and `.gitattributes`.

## Porting to Python

Use the saved Phase 10 model as the behavioral reference, with `matlab/phase8_grid_manager_code.txt`, `phase9_landing_gate_code.txt`, `phase9_abort_diagnostic_code.txt`, and `phase10_autoland_arbiter_code.txt`. Preserve the existing packet layouts, camera freshness logic, coordinate transforms, camera offset, flight bounds, operator triggers, communication-loss behavior, and landing-readiness gate. Existing phase 9 and phase 10 MATLAB tests are included as behavioral specifications.

The build scripts form a phase chain and some refuse to overwrite existing models. Do not rerun the full chain against this snapshot without reading each script. The saved models may contain absolute lab paths; resolve those to the copied calibration files in MATLAB before building on another machine. MATLAB R2021b, QUARC, Quanser libraries, Motive/OptiTrack, and the lab hardware remain external prerequisites. Python dependencies are listed without invented version pins; the original environment did not supply a lockfile.

`docs/source_manifest.json` records original file hashes and paths. Apart from this README, copied source files are unchanged. No Python flight-control port was added during packaging; the Python port described above was added afterwards and leaves the copied sources unchanged.
