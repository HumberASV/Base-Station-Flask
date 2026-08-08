# Base-Station-Flask

A Flask + ROS2 bridge that sits between the Loon-E ASV and the web client. It subscribes to the
ASV's ROS2 topics (position/heading/speed, battery, motors/rudder, ZED camera + odometry, ROS logs)
and re-serves that data as JSON over a WebSocket, plus an MJPEG video stream over HTTP.

The base station computer stays on the same network as the ASV and relays its data to the web
client — it does **not** control the ASV or perform navigation.

```text
web client  <--WebSocket (/telemetry)-->  Base-Station-Flask  <--ROS2 topics-->  Loon-E ASV
                <--HTTP (/video_feed)-->
```

> [!NOTE]
> There is no database, authentication, or Docker Compose stack — those were removed. This is a
> single Flask process with a background ROS2 spin thread.

<!-- -->

> [!TIP]
> Setting this up on a fresh machine? See **[INSTALL.md](INSTALL.md)** for a full clean-Windows-11
> walkthrough, including ROS2-on-Windows quirks that don't show up until you hit them. The sections
> below assume you already have a working ROS2 + venv setup.

## Prerequisites

1. **ROS2 Humble**, matching the version/DDS/domain ID used on the ASV's computer, on the same
   network as the ASV.
   - Windows: a source build such as `C:\dev\ros2_humble` (see [Building ROS 2 on Windows](https://docs.ros.org/en/humble/Installation/Alternatives/Windows-Development-Setup.html)).
   - Linux/macOS: a normal `apt`/binary install, sourced via `/opt/ros/humble/setup.bash`.
2. **A Python interpreter matching the ABI that `rclpy` was built against.** ROS2 does not care
   what Python `python app.py` happens to invoke — it cares whether `rclpy`'s compiled extension
   matches that interpreter's version. On Windows this is easy to get wrong silently; see
   [Troubleshooting](#troubleshooting) below if `import rclpy` fails.

## One-time setup

See **[INSTALL.md](INSTALL.md)** for the full setup — venv creation, ROS2 install quirks, and
building the `basestation` package so `ros2 launch` works. Short version, once ROS2 is sourced:

```powershell
cd C:\Users\thefu\my_ros_ws\Base-Station-Flask
python -m venv venv310
.\venv310\Scripts\python.exe -m pip install -r ros2_nodes\src\basestation\requirements.txt
.\build_basestation_pkg.ps1
```

## Optional: building ZED support (`cv_bridge` + `zed_msgs`)

Without these, the bridge logs `cv2, numpy, or cv_bridge not found — ZED image subscription
disabled` and/or `zed_msgs not found — ZED object detection topic will not be subscribed`, and
`/video_feed` only ever serves the local fallback clip even with a live ZED feed. Both are ROS2
packages — not pip-installable — and on a from-source Windows build like `C:\dev\ros2_humble`
aren't included by default (only what was in the checked-out `ros2.repos` gets built; `zed_msgs`
comes from the separate `zed-ros2-interfaces` submodule). Building them from source keeps them
binary-compatible with the rest of a custom-built ROS2 install (mixing in prebuilt RoboStack
packages risks ABI mismatches with `rcl`/`rmw`/message typesupport).

```powershell
# 1. Build-time Python deps the venv needs (separate from requirements.txt's runtime deps —
#    these are used by CMake/ament/rosidl while *generating and compiling* the packages below).
C:\Users\thefu\my_ros_ws\Base-Station-Flask\venv310\Scripts\python.exe -m pip install "numpy<2" catkin_pkg "empy==3.3.4" "lark==1.1.1"
# numpy<2: CMake 3.22's FindPython3 NumPy detection predates numpy 2.x's include-dir layout
# change (numpy/core -> numpy/_core) and can't find the headers otherwise.

# 2. Clone cv_bridge's source (vision_opencv) into its own workspace. (zed_msgs's source,
#    zed-ros2-interfaces, is already a submodule at ros2_nodes/src/zed-ros2-interfaces.)
git clone --branch humble --depth 1 https://github.com/ros-perception/vision_opencv.git C:\dev\ros2_extra_ws\src\vision_opencv

# 3. cv_bridge's CMakeLists.txt looks for a Boost::python<major><minor> CMake target
#    (e.g. Boost::python310). conda-forge's boost-cpp (pulled in transitively for the main
#    ROS2 build) doesn't include Python bindings — add the "boost" package explicitly,
#    pinned to the SAME version boost-cpp already resolved to (check
#    C:\dev\ros2_humble\.pixi\envs\default\conda-meta\boost-cpp-*.json), then reinstall:
#      boost = "==<that version>"   # add under [dependencies] in C:\dev\ros2_humble\pixi.toml
cd C:\dev\ros2_humble
pixi install

# 4. Build, with the VC++ compiler, ROS2, and pixi-provided OpenCV/Boost all on the search path.
& "<path to vcvarsall.bat>" x64   # from an actual cmd.exe, or import its env vars into PowerShell
. "C:\dev\ros2_humble\local_setup.ps1"
$pixiEnv = "C:\dev\ros2_humble\.pixi\envs\default"
$env:Path = "$pixiEnv\Scripts;$pixiEnv\Library\bin;" + $env:Path
$env:CMAKE_PREFIX_PATH = "$pixiEnv\Library;" + $env:CMAKE_PREFIX_PATH
$venvPy = "C:\Users\thefu\my_ros_ws\Base-Station-Flask\venv310\Scripts\python.exe"

cd C:\dev\ros2_extra_ws
colcon build --packages-select cv_bridge --cmake-args "-DPython3_EXECUTABLE=$venvPy" "-DPYTHON_EXECUTABLE=$venvPy" -DBUILD_TESTING=OFF

cd C:\Users\thefu\my_ros_ws\Base-Station-Flask\ros2_nodes
colcon build --packages-select zed_msgs --cmake-args "-DPython3_EXECUTABLE=$venvPy" "-DPYTHON_EXECUTABLE=$venvPy" -DBUILD_TESTING=OFF
```

Build-time snags worth knowing about if step 4 fails:

- **`Could not find a package configuration file ... boost_python310Config.cmake`** — conda-forge's
  boost 1.74.0 exports an unversioned `Boost::python` target, not the per-Python-version
  `Boost::python310` target upstream Boost's own CMake generation produces. `cv_bridge/CMakeLists.txt`
  in this workspace has a small patch (see git history / diff) that falls back to `Boost::python`
  when the versioned component isn't found — keep it if you re-clone `vision_opencv`.
- **`Imported target "...__rosidl_generator_py" includes non-existent path "C:\pixi_ws\..."`** — the
  ROS2 install's own exported `.cmake` files have absolute paths baked in from wherever it was
  *originally* built, which may not match this machine (this repo's copy had `C:\pixi_ws` baked in;
  the real pixi env lives at `C:\dev\ros2_humble\.pixi`). This affects **any** package's CMake
  export that references `rosidl_generator_py`'s numpy include dir (it's what broke both
  `cv_bridge` and `zed_msgs`), so it's already been patched across all of `C:\dev\ros2_humble\share`
  on this machine — but if you rebuild ROS2 from scratch elsewhere, search
  `C:\dev\ros2_humble\share` for the stale prefix and replace it with the real one.
- **`ModuleNotFoundError: No module named 'em'`** (only `zed_msgs`, which generates far more
  message bindings than `cv_bridge`) — `empy` wasn't in the venv; see step 1.
- **`Could not find numpy` referencing `C:/Python310/python.exe`, a totally different interpreter**
  — `rosidl_generator_py`'s own CMake module reads the legacy `PYTHON_EXECUTABLE` cache variable
  specifically (not `Python3_EXECUTABLE`), and falls back to whatever `python` it finds on `PATH`
  if that's unset. Pass **both** `-DPython3_EXECUTABLE` and `-DPYTHON_EXECUTABLE` (as above) — this
  is why `cv_bridge`'s "unused variable" warning for `PYTHON_EXECUTABLE` is safe to ignore, but the
  flag still needs to be there for packages that generate `rosidl` Python bindings.

Then, every time you run the app, source both overlays **after** `local_setup.ps1`:

```powershell
. "C:\dev\ros2_extra_ws\install\local_setup.ps1"                                      # cv_bridge
. "C:\Users\thefu\my_ros_ws\Base-Station-Flask\ros2_nodes\install\local_setup.ps1"     # zed_msgs
```

## Running it

**Quick start:** `.\start_basestation.ps1` (from the repo root) does everything below in one
step — sources ROS2, adds the pixi DLL path, sources the `cv_bridge`/`zed_msgs` overlays if
they've been built (skips them silently otherwise), and runs `ros2 launch basestation
basestation.py`. Requires the `basestation` package to have been built first — see
[INSTALL.md](INSTALL.md) / `.\build_basestation_pkg.ps1`. Supports `-Port`, `-BindHost`,
`-Dashboard`, `-DebugMode`; run `Get-Help .\start_basestation.ps1 -Full` for details.

Otherwise, every new shell needs the ROS2 environment sourced **before** starting the app:

```powershell
# 1. Source the ROS2 install.
. "C:\dev\ros2_humble\local_setup.ps1"

# 2. (Windows source builds only) Put the build-toolchain's DLLs on PATH — this is the
#    directory pixi materializes from C:\dev\ros2_humble\pixi.toml. Without this, rclpy's
#    native extension fails to import (see Troubleshooting).
$env:Path = "C:\dev\ros2_humble\.pixi\envs\default\Library\bin;" + $env:Path

# 3. (Optional) source the cv_bridge/zed_msgs overlays if you built them — see the section
#    above. Skip this if you haven't; the app degrades gracefully without them.
. "C:\dev\ros2_extra_ws\install\local_setup.ps1"
. "C:\Users\thefu\my_ros_ws\Base-Station-Flask\ros2_nodes\install\local_setup.ps1"

# 4. Launch (the basestation package must be built first — see INSTALL.md / build_basestation_pkg.ps1).
ros2 launch basestation basestation.py host:=0.0.0.0 port:=8080
```

For quick local debugging without going through `ros2 launch`, you can still run the Flask app
directly with the matching-ABI venv (steps 1-3 above still required first):

```powershell
cd C:\Users\thefu\my_ros_ws\Base-Station-Flask\ros2_nodes\src\basestation\basestation
C:\Users\thefu\my_ros_ws\Base-Station-Flask\venv310\Scripts\python.exe app.py
```

Command-line flags (all optional):

| Flag | Default | Purpose |
| --- | --- | --- |
| `--host` | `0.0.0.0` (or `$env:HOST`) | Interface to bind |
| `--port` | `8080` (or `$env:PORT`) | Port to bind |
| `--dashboard` | off | Enables the `/dashboard` debug page |

Environment variables:

| Variable | Purpose |
| --- | --- |
| `HOST` / `PORT` | Fallback for `--host`/`--port` if flags aren't passed |
| `FLASK_DEBUG=1` | Enables Flask's interactive debugger + reloader and `DEBUG`-level logging. **Never use in production** — it allows arbitrary code execution. |
| `SECRET_KEY` | Flask session secret (defaults to a placeholder — set a real value for anything beyond local dev) |

If ROS2 isn't reachable (not sourced, `rclpy` unimportable, or no ASV on the network), the app
does **not** crash — it logs `rclpy not found — ROS2 bridge disabled` and serves idle/default
telemetry so the web client can still be exercised against zeros.

## Verifying it's running

```bash
curl http://localhost:8080/health
# {"status": "ok"}
```

```python
# Inspect one /telemetry frame:
import websocket, json
ws = websocket.create_connection("ws://localhost:8080/telemetry", timeout=5)
print(json.dumps(json.loads(ws.recv()), indent=2))
ws.close()
```

With ROS2 actually connected to the ASV (or a test publisher — see below), the `asv`, `battery`,
`motors`, `rudder`, `signal`, and `zed` fields should update as real topics arrive. `map`,
`planning.course/plan`, and `task.data.name` currently have no live Loon-E publisher and will stay
at their idle defaults.

To simulate an ASV without hardware, publish directly to a topic the bridge subscribes to, e.g.:

```python
import rclpy, time
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

rclpy.init()
node = Node("fake_phone")
pub = node.create_publisher(Float32MultiArray, "phone", 10)
time.sleep(1.0)  # let discovery settle
msg = Float32MultiArray(data=[43.6532, -79.3832, 3.4, 128.0])  # lat, lon, speed, heading
pub.publish(msg)
rclpy.spin_once(node, timeout_sec=0.2)
node.destroy_node()
rclpy.shutdown()
```

## Routes

| Route | Method | Description |
| --- | --- | --- |
| `/health` | GET | Liveness check (used by the web client before it attempts the WebSocket) |
| `/telemetry` | WS | Streams the current telemetry state as JSON every 100ms |
| `/video_feed` | GET | MJPEG stream — live ZED feed if ROS2/ZED are available, otherwise a looped local fallback clip |
| `/` | GET | Landing page |
| `/dashboard` | GET | Debug dashboard (only if started with `--dashboard`) |
| `/uh_oh` | GET | "No data from the ASV" error page |

## Topics consumed

| Topic | Type | Populates |
| --- | --- | --- |
| `phone` | `std_msgs/Float32MultiArray` `[lat, lon, speed, heading]` | `asv.*`, `task.location`, `signal.strength` |
| `task` | `std_msgs/Float32MultiArray` `[action, target_heading, target_speed]` | `planning.status`, `task.data.status`, `task.log` |
| `/asv/joint_states` | `sensor_msgs/JointState` | `motors.left/right`, `rudder.angle` |
| `battery_status/prop_l`, `battery_status/prop_r`, `battery_status/main` | `sensor_msgs/BatteryState` | `battery.motors` (avg of prop_l/prop_r), `battery.primary` (main) |
| `zed/zed_node/odom` | `nav_msgs/Odometry` | `zed.odom` |
| `/zed/zed_node/rgb/color/rect/image` | `sensor_msgs/Image` | `zed.camera`, live video frames |
| `zed/obj_det/objects` | `zed_msgs/ObjectsStamped` (optional — skipped if `zed_msgs` isn't built/sourced, see above) | `zed.objects` |
| `/rosout` | `rcl_interfaces/Log` | `task.log` (INFO and above) |

## Troubleshooting

### `... cannot be loaded. The file ... is not digitally signed. You cannot run this script...`

PowerShell's execution policy is blocking `local_setup.ps1` (or any unsigned `.ps1`) from running.
Fix, in the same terminal you're working in:

```powershell
# One-off, just this terminal session (no admin needed):
Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass -Force

# OR persistent, for your user account (no admin needed either):
Set-ExecutionPolicy -Scope CurrentUser -ExecutionPolicy RemoteSigned
```

`RemoteSigned` still requires scripts *downloaded from the internet* to be signed — it only allows
locally-created ones (like ROS2's `setup.ps1`) to run unsigned, so it doesn't meaningfully loosen
security. Then re-run the `. "C:\dev\ros2_humble\local_setup.ps1"` line.

### `ModuleNotFoundError: No module named 'rclpy'`

The ROS2 environment isn't sourced in this shell. Run `. "C:\dev\ros2_humble\local_setup.ps1"`
(or `source /opt/ros/humble/setup.bash` on Linux/macOS) **before** running `app.py`.

### `ImportError: DLL load failed while importing _rclpy_pybind11` (Windows source builds)

This means `rclpy`'s native extension is being found, but one of *its* dependency DLLs isn't on
`PATH`. If you built ROS2 from source using the pixi-based Windows build process, the build
toolchain's `pixi.toml` provides third-party C/C++ libraries (e.g. `spdlog`, `tinyxml2`, `fastrtps`)
that `rclpy`/`rcl`/`rmw` link against — but they only get installed into `<ros2_install>\.pixi\envs\default\Library\bin`
once you run `pixi install` in that directory, and that directory is **not** added to `PATH` by
`local_setup.ps1`. Fix:

```powershell
cd C:\dev\ros2_humble
pixi install
```

Then always put its `Library\bin` on `PATH` alongside sourcing `local_setup.ps1` (see
[Running it](#running-it) above). You can confirm the exact missing DLL with:

```powershell
& "<path to dumpbin.exe from a Visual Studio install>" /dependents "C:\dev\ros2_humble\Lib\site-packages\rclpy\_rclpy_pybind11.cp3XX-win_amd64.pyd"
```

### `import rclpy` fails even with everything above done

Check the Python version. `rclpy`'s compiled extension has a version tag in its filename, e.g.
`_rclpy_pybind11.cp310-win_amd64.pyd` means it was built for **Python 3.10** specifically — a
Python 3.8 or 3.11 interpreter cannot import it, pip cannot fix this, and the error you'll get
(`DLL load failed` / generic import failure) doesn't say so directly. Create your venv from the
matching Python version.

### The bridge logs `rclpy not found — ROS2 bridge disabled` and stays that way

This is the graceful-degradation path, not a crash — the app serves idle telemetry so the web
client still functions. Fix the ROS2 environment per the sections above and restart the app.

### `cv2, numpy, or cv_bridge not found` and/or `zed_msgs not found`

Also graceful degradation, not a crash — ZED image/object-detection subscriptions are just
disabled. See [Optional: building ZED support](#optional-building-zed-support-cv_bridge--zed_msgs)
above.
