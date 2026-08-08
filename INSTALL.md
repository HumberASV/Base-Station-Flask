# Installing Base-Station-Flask on a clean Windows 11 PC

This walks through everything needed to go from a bare Windows 11 machine to running
`.\start_basestation.ps1` successfully, including the ROS2-on-Windows quirks that aren't
obvious from the error messages they produce. Standard way to run this project is via
**`ros2 launch`** (see [Step 7](#7-build-the-basestation-ros2-package) and
[Step 9](#9-run-it)) — running `app.py` directly with `venv310`'s Python still works as a
fallback for quick local debugging, but isn't the supported path going forward.

## 1. Prerequisites

Install these first, in order:

1. **Git for Windows** — https://git-scm.com/download/win
2. **Python 3.10.x** (must be **exactly** 3.10 — see why in step 5) from
   https://www.python.org/downloads/ — during install, check **"Add python.exe to PATH"**.
   Verify: `python --version` → `Python 3.10.x`.
3. **ROS2 Humble for Windows** at `C:\dev\ros2_humble`. This project uses a pixi-based
   source build (see [Building ROS 2 on Windows](https://docs.ros.org/en/humble/Installation/Alternatives/Windows-Development-Setup.html)).
   Get the pre-built `C:\dev\ros2_humble` tree from wherever your team distributes it
   (e.g. a shared archive/drive), or build it from source per the docs above.
4. *(Only needed if you'll build `cv_bridge`/`zed_msgs` from source — Step 6)*
   **Visual Studio Build Tools** with the "Desktop development with C++" workload, for
   `vcvarsall.bat` and the MSVC compiler.

## 2. Patch the ROS2 install's stale build-machine paths

**Skip this step if `ros2 --help` already works after sourcing `local_setup.ps1`** — it
only applies to `C:\dev\ros2_humble` trees that were built elsewhere (e.g. under
`C:\pixi_ws`) and then copied to this machine.

This ROS2 distribution was originally built on a machine where pixi lived at
`C:\pixi_ws`. That absolute path is baked into two different kinds of files, and both
need patching to point at this machine's actual pixi location
(`C:\dev\ros2_humble\.pixi\envs\default`):

**a) CMake export files** (breaks building `cv_bridge`/`zed_msgs` from source, with an
error like `Imported target "...__rosidl_generator_py" includes non-existent path
"C:\pixi_ws\..."`) — search `C:\dev\ros2_humble\share` for the stale prefix and replace
it with the real one.

**b) Console-script launchers** (breaks `ros2`, `ros2doctor`, and every other CLI tool —
manifests as a bare **`failed to create process.`** with no other detail, because the
launcher's embedded shebang points at a `python.exe` that doesn't exist). Fix with:

```powershell
$old = "C:\pixi_ws\.pixi\envs\default\python.exe"
$new = "C:\dev\ros2_humble\.pixi\envs\default\python.exe"
$utf8NoBom = New-Object System.Text.UTF8Encoding $false   # a BOM here breaks shebang parsing
Get-ChildItem "C:\dev\ros2_humble\Scripts\*-script.py" | ForEach-Object {
    $bytes = [System.IO.File]::ReadAllBytes($_.FullName)
    if ($bytes.Length -ge 3 -and $bytes[0] -eq 0xEF -and $bytes[1] -eq 0xBB -and $bytes[2] -eq 0xBF) {
        $bytes = $bytes[3..($bytes.Length - 1)]
    }
    $text = [System.Text.Encoding]::UTF8.GetString($bytes)
    if ($text.StartsWith("#!$old")) {
        [System.IO.File]::WriteAllText($_.FullName, ($text -replace [regex]::Escape($old), $new), $utf8NoBom)
    }
}
```

Verify: `. "C:\dev\ros2_humble\local_setup.ps1"; ros2 --help` should print usage, not
`failed to create process.`.

## 3. Provide the pixi DLL toolchain

`rclpy`'s native extension links against third-party DLLs (spdlog, tinyxml2, fastrtps,
...) that pixi materializes but that aren't on `PATH` by default:

```powershell
cd C:\dev\ros2_humble
pixi install
```

## 4. Clone this repo

```powershell
cd C:\Users\<you>\my_ros_ws
git clone --recurse-submodules <this-repo-url> Base-Station-Flask
cd Base-Station-Flask
```

(`--recurse-submodules` matters — `ros2_nodes/src/zed-ros2-interfaces` is a submodule.)

## 5. Create the venv and install Flask-side dependencies

`rclpy`'s compiled extension has a Python-version tag baked into its filename (e.g.
`_rclpy_pybind11.cp310-win_amd64.pyd` under `C:\dev\ros2_humble\Lib\site-packages\rclpy`)
— `cp310` means it **only** imports under Python 3.10. Any other version fails with a
DLL/import error that doesn't say why.

```powershell
cd C:\Users\<you>\my_ros_ws\Base-Station-Flask
python -m venv venv310
.\venv310\Scripts\python.exe -m pip install -r ros2_nodes\src\basestation\requirements.txt
```

## 6. (Optional) Build ZED support: `cv_bridge` + `zed_msgs`

Skip this if you don't need live ZED camera/object-detection data — the app degrades
gracefully without it (`/video_feed` just serves a local fallback clip). If you do need
it, follow the **"Optional: building ZED support"** section in [README.md](README.md) —
it covers the Boost::python CMake target patch, the build-time Python deps, and the
exact `colcon build --cmake-args` invocations.

## 7. Build the `basestation` ROS2 package

This project runs via `ros2 launch`, which means the `basestation` package needs to be
built with `colcon` — and specifically, built using **`venv310`'s Python**, not
whichever Python `colcon` happens to run under. `ament_python` packages have no
`-DPython3_EXECUTABLE` CMake flag (unlike the compiled `cv_bridge`/`zed_msgs` builds
above); colcon just calls `setup.py` with `sys.executable` of whatever process is
running it. So `colcon` itself needs to be installed **inside** `venv310`:

```powershell
.\build_basestation_pkg.ps1
```

This script (in the repo root) sources ROS2, installs `colcon-common-extensions` into
`venv310` if it isn't there yet, and runs `colcon build --packages-select basestation
--symlink-install` using `venv310`'s own `colcon.exe`.

**If this fails with `WinError 1314: A required privilege is not held by the client`**
on an `os.symlink(...)` call — Windows requires either Administrator rights or
**Developer Mode** to create symlinks as a normal user. Fix:

- Settings → Privacy & security → **For developers** → toggle **Developer Mode** ON
  (no reboot needed, just open a new shell), or
- run `.\build_basestation_pkg.ps1` from an elevated (Run as Administrator) PowerShell.

If you'd rather not touch that Windows setting, edit `build_basestation_pkg.ps1` and
drop `--symlink-install` from the `colcon build` line — it'll do a plain copy install
instead (no special privilege needed), at the cost of needing to re-run the build after
any edit to files under `ros2_nodes/src/basestation`.

Verify: after it finishes, `ros2 pkg executables basestation` should list `basestation
flask` and `basestation stream`.

## 8. Known issue to fix before first run

`ros2_nodes/src/basestation/basestation/__init__.py` currently contains leftover
scaffold code from an earlier prototype — an unconditional `import pyzed.sl as sl`
(the ZED SDK's own Python bindings, not currently a dependency anywhere else in this
project) and `import rospy` (ROS **1**, not ROS2). Because this is the package's
`__init__.py`, it runs on *every* import of `basestation` — including via `ros2 launch`
— and fails immediately with `ModuleNotFoundError: No module named 'pyzed'` before your
own code ever runs, since `pyzed` isn't installed anywhere in this setup.

Nothing else in the package (`app.py`, `annotated_udp_stream.py`, `ros2_bridge.py`,
`telemetry_factory.py`) references anything from this file — `generate_data`/
`generate_frames` aren't called anywhere, and `pyzed`/`rospy`/`socketio` aren't used
elsewhere either. Until this is cleaned up (or the ZED-SDK/SocketIO streaming path it
was sketching out gets actually wired in), replace its contents with an empty file, then
re-run Step 7's build.

## 9. Run it

```powershell
.\start_basestation.ps1
```

This sources ROS2, puts the pixi DLL path on `PATH`, sources the `cv_bridge`/`zed_msgs`
overlays if built, and runs `ros2 launch basestation basestation.py`. Supports `-Port`,
`-BindHost`, `-Dashboard`, `-DebugMode` — see `Get-Help .\start_basestation.ps1 -Full`.

## 10. Verify it's working

```powershell
curl http://localhost:8080/health
# {"status": "ok"}
```

See [README.md § Verifying it's running](README.md#verifying-its-running) for a fuller
check (WebSocket telemetry frame, simulating an ASV publisher).

If anything else goes wrong, check [README.md § Troubleshooting](README.md#troubleshooting)
first — most day-to-day ROS2-environment issues (unsigned script execution policy,
`ModuleNotFoundError: No module named 'rclpy'`, DLL load failures, Python ABI mismatches)
are covered there.
