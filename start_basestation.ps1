<#
.SYNOPSIS
    Starts the Base-Station-Flask ASV telemetry bridge, with the ROS2 Humble
    environment (and the optional cv_bridge / zed_msgs overlays, if built) sourced.

.DESCRIPTION
    Mirrors the manual steps in README.md's "Running it" section:
      1. Sources C:\dev\ros2_humble\local_setup.ps1
      2. Puts the pixi build-toolchain's DLLs (spdlog, etc.) on PATH, which rclpy's
         native extension needs to import.
      3. Sources the cv_bridge / zed_msgs overlays if they've been built (skipped
         silently otherwise -- the app degrades gracefully without them).
      4. Runs `ros2 launch basestation basestation.py`, which starts both the Flask
         app and the annotated ZED UDP stream node. Requires the basestation ROS2
         package to have been built first -- see INSTALL.md / build_basestation_pkg.ps1.

    Safe to re-run: if ROS2 or the optional overlays aren't present, this prints a
    warning and still starts the app (idle/mock telemetry mode).

.PARAMETER Port
    Port to bind the Flask app to. Default 8080.

.PARAMETER BindHost
    Host/interface to bind to. Default 0.0.0.0.

.PARAMETER Dashboard
    Enable the /dashboard debug page.

.PARAMETER DebugMode
    Enable Flask debug mode (FLASK_DEBUG=1) -- interactive debugger + reloader.
    Never use this in production.

.PARAMETER Factory
    Stream factory (fake) telemetry instead of connecting to ROS2/the ASV -- the ROS2 bridge
    thread is never started, and a background thread continuously randomizes speed, heading,
    position, battery, rudder, and signal so the web client has something dynamic to render
    without real hardware. Also switches /video_feed to the looped fallback clip.

.PARAMETER build
    Enable colcon build

.PARAMETER CycloneDdsUri
    Path to a cyclonedds.xml config file. When passed, switches the DDS middleware from
    Humble's default (Fast-RTPS) to Cyclone DDS for this process and everything it launches,
    by setting RMW_IMPLEMENTATION=rmw_cyclonedds_cpp and CYCLONEDDS_URI to that file.

.EXAMPLE
    .\start_basestation.ps1

.EXAMPLE
    .\start_basestation.ps1 -Port 8081 -Dashboard

.EXAMPLE
    .\start_basestation.ps1 -Factory

.EXAMPLE
    .\start_basestation.ps1 -CycloneDdsUri C:\dev\cyclonedds.xml
#>
[CmdletBinding()]
param(
    [string]$Port = "8080",
    [string]$BindHost = "0.0.0.0",
    [switch]$Dashboard,
    [switch]$DebugMode,
    [switch]$Factory,
    [string]$CycloneDdsUri = ""
)

$ErrorActionPreference = "Stop"

$RepoRoot   = Split-Path -Parent $MyInvocation.MyCommand.Path
$Ros2Install = "C:\dev\ros2_humble"
$Ros2ExtraWs = "C:\dev\ros2_extra_ws"
$VenvPython  = Join-Path $RepoRoot "venv310\Scripts\python.exe"

function Write-Step {
    param([string]$Message)
    Write-Host "==> $Message" -ForegroundColor Cyan
}

if (-not (Test-Path $VenvPython)) {
    Write-Error @"
venv310 not found at $VenvPython
Create it first:
    python -m venv "$RepoRoot\venv310"
    "$VenvPython" -m pip install -r "$RepoRoot\ros2_nodes\src\basestation\requirements.txt"
"@
}

# 1. ROS2 core environment.
$ros2Setup = Join-Path $Ros2Install "local_setup.ps1"
if (Test-Path $ros2Setup) {
    Write-Step "Sourcing ROS2 environment ($Ros2Install)"
    . $ros2Setup

    # 2. Build-toolchain DLLs (spdlog, etc.) that rclpy's native extension links against.
    $pixiLib = Join-Path $Ros2Install ".pixi\envs\default\Library\bin"
    if (Test-Path $pixiLib) {
        $env:Path = "$pixiLib;$env:Path"
    }
    else {
        Write-Warning "Pixi env not found at $pixiLib -- if 'import rclpy' fails with a DLL error, run 'pixi install' in $Ros2Install first (see README Troubleshooting)."
    }
}
else {
    Write-Warning "ROS2 install not found at $Ros2Install -- starting in degraded (no-ROS2, mock telemetry) mode."
}

# 1b. (Optional) Switch DDS middleware to Cyclone DDS.
if ($CycloneDdsUri) {
    if (-not (Test-Path $CycloneDdsUri)) {
        Write-Warning "cyclonedds.xml not found at $CycloneDdsUri -- CYCLONEDDS_URI will still be set, but Cyclone DDS will likely fail to start."
    }
    $resolved = Resolve-Path -Path $CycloneDdsUri -ErrorAction SilentlyContinue
    $absolutePath = if ($resolved) { $resolved.Path } else { $CycloneDdsUri }
    $env:RMW_IMPLEMENTATION = "rmw_cyclonedds_cpp"
    $env:CYCLONEDDS_URI = "file:///$($absolutePath -replace '\\', '/')"
    Write-Step "Using Cyclone DDS (CYCLONEDDS_URI=$($env:CYCLONEDDS_URI))"
}

# (optional build)
if ($PSBoundParameters.ContainsKey("build")) {
    Write-Step "Building ROS2 overlays (cv_bridge, zed_msgs)"
    Set-Location $Ros2ExtraWs
    & colcon build --symlink-install
}

# 3. Optional cv_bridge / zed_msgs overlays -- sourced only if they've been built.
$cvBridgeSetup = Join-Path $Ros2ExtraWs "install\local_setup.ps1"
if (Test-Path $cvBridgeSetup) {
    Write-Step "Sourcing cv_bridge overlay"
    . $cvBridgeSetup
}

$zedMsgsSetup = Join-Path $RepoRoot "ros2_nodes\install\local_setup.ps1"
if (Test-Path $zedMsgsSetup) {
    Write-Step "Sourcing zed_msgs overlay"
    . $zedMsgsSetup
}

# 4. Launch.
if ($Dashboard) { Write-Warning "-Dashboard is not yet wired into launch/basestation.py (see TODO)." }
if ($DebugMode) { $env:FLASK_DEBUG = "1" }

$launchArgs = @("host:=$BindHost", "port:=$Port")
if ($Factory) { $launchArgs += "factory:=true" }

$modeNote = if ($Factory) { " [FACTORY MODE -- fake telemetry, no ROS2/ASV required]" } else { "" }
Write-Step "Starting Base-Station-Flask on http://${BindHost}:${Port} (Ctrl+C to stop)$modeNote"
& ros2 launch basestation basestation.py @launchArgs