# 09_PathFollow

Single-layer path-follow module:
- input: camera MJPEG stream
- core: yellow line center extraction + simple follow controller
- output: websocket status (`path_observation` + `cmd_vel`), optional robot command send

## Core File

Only edit this file for algorithm changes:
- `09_PathFollow/core_policy.py`

Bridge/runtime code in `path_follow_bridge.py` is adapter/black-box layer.

## Run

```bash
cd 09_PathFollow
python path_follow_bridge.py
```

Open:
- HTTP panel: `http://localhost:8895`
- WS status: `ws://localhost:8896`

## Safety

By default command send is disabled:
- `ENABLE_SEND_COMMANDS = False` in `path_follow_bridge.py`

Set it to `True` only when you are ready to control robot.
