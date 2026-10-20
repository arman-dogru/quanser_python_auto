# Quanser autonomous landing

Local source repository prepared for a subsequent Python port.

## Baseline and status

The selected baseline is `reference/simulink/QDrone_YOLO_Stage6_RealYOLO_MonitorOnly.slx`, the most advanced named integration stage supplied. Its embedded YOLOPoseCommandGenerator matches the preceding integration and pose-command models. This selection is based on static source inspection, not a flight test or MATLAB execution. Stage 6 is monitor-only by name; descent is explicitly disabled in its controller. This snapshot does not establish a complete working autonomous landing system.

Start the Python port from the extracted Stage 6 controller in `reference/extracted/QDrone_YOLO_Stage6_RealYOLO_MonitorOnly/`. Keep the original Simulink model as the reference for wiring, communication, interlocks, and coordinate transforms.

## Contents

- `reference/simulink/`: Stage 6 baseline plus all four preceding test/integration models for comparison.
- `reference/extracted/`: every embedded MATLAB function from each model, in plain text. File prefixes identify the originating chart.
- `reference/matlab/legacy/`: earlier grayscale detector, grid search, and landing controller prototype. These use a different image-to-motion mapping from the later YOLO models.
- `reference/experiments/`: original live script and screenshot.
- `tools/dataset_legacy/`: original Python preparation/evaluation scripts, retained for algorithm reference. They use machine-specific E: paths; some delete their configured output directories. Review and parameterize them before running.
- `models/yolo11n.pt`: supplied checkpoint, preserved byte-for-byte. Filename alone does not prove it is trained for landing pads. No landing-pad best.pt or training metadata was supplied.
- `docs/PYTHON_PORT.md`: migration contracts and missing inputs.
- `docs/source_manifest.json`: SHA-256 checksums, source mapping, and embedded function provenance.

All supplied source files are included, with older experiments separated from the baseline. Synced project originals were left untouched. No Python flight controller has been implemented yet.

## Verification

Run `python tools/verify_snapshot.py` from this repository to verify all original file copies and extracted embedded functions. This uses only the Python standard library. MATLAB/Simulink and hardware validation remain outstanding.

## Push to your online repository

The local Git repository uses branch `main` and has no remote configured. Create an empty online repository, then run from this folder:

```powershell
git remote add origin <your-repository-url>
git push -u origin main
```

No project license was supplied; copied Quanser/library content and checkpoint retain their existing rights. No new license is asserted by this repository.
