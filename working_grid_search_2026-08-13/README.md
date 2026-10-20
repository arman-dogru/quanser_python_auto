# Verified Working Grid Search Backup

Captured on 2026-08-13 immediately after the user confirmed that this pair
of models worked. These files are the protected pre-YOLO/pre-landing baseline.

Do not develop directly in this folder. Continue development using the models
in the project's `simulink` folder and restore these copies only when needed.

## Files and SHA-256 checksums

- `Mission_Server_QDrone_Grid_Search.mdl`
  - `6C3CBDA011A938F88585868A01CE06753895EE270B7A512F6C4A358D5C73FCE3`
- `QDrone_Grid_Search_Commander.mdl`
  - `C9636E8FC429BE16342D9F5B1D911CADF1A21A8D9220E74C3CE37B4F03694307`

## Verified configuration

- Ground-station IP: `192.168.2.11`
- QDrone IP: `192.168.2.15`
- Commander model URI: `tcpip://192.168.2.15:17001?retries=10`
- Mission-server/Commander stream: port `18001`
- Commander target: `quarc_linux_x64.tlc`
- Mission-server target: `quarc_win64.tlc`
- No YOLO or downward-camera model included in this baseline

## Restore command

Close both models before restoring. Copy the two `.mdl` files from this folder
back into the project's `simulink` folder. Preserve this backup folder itself.
