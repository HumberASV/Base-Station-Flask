<#
.SYNOPSIS
    One-time (re-run after editing setup.py/package.xml) build of the `basestation`
    ROS2 package, targeting venv310's Python so the generated `flask`/`stream`
    console-script executables can actually import Flask, cv2, etc.

.DESCRIPTION
    ament_python packages have no -DPython3_EXECUTABLE CMake flag (unlike the
    compiled cv_bridge/zed_msgs overlays) -- colcon just calls setup.py using
    whichever Python is running colcon itself. So to get executables that import
    correctly, colcon has to be installed into venv310 and run from there.

    ament_package (needed by colcon-ros) is not pip-installed anywhere -- it's
    picked up from C:\dev\ros2_humble\Lib\site-packages via the PYTHONPATH that
    local_setup.ps1 sets, so ROS2 must be sourced before this runs.

.EXAMPLE
    .\build_basestation_pkg.ps1
#>
$ErrorActionPreference = "Stop"

$RepoRoot   = Split-Path -Parent $MyInvocation.MyCommand.Path
$Ros2Install = "C:\dev\ros2_humble"
$VenvPython  = Join-Path $RepoRoot "venv310\Scripts\python.exe"
$VenvColcon  = Join-Path $RepoRoot "venv310\Scripts\colcon.exe"
$WsRoot      = Join-Path $RepoRoot "ros2_nodes"

function Write-Step {
    param([string]$Message)
    Write-Host "==> $Message" -ForegroundColor Cyan
}

if (-not (Test-Path $VenvPython)) {
    Write-Error "venv310 not found at $VenvPython -- see README.md setup steps first."
}

Write-Step "Sourcing ROS2 environment ($Ros2Install)"
. (Join-Path $Ros2Install "local_setup.ps1")

if (-not (Test-Path $VenvColcon)) {
    Write-Step "Installing colcon into venv310 (one-time)"
    & $VenvPython -m pip install colcon-common-extensions
}
else {
    Write-Host "==> colcon already present in venv310, skipping install" -ForegroundColor DarkGray
}

Write-Step "Building basestation package with venv310's colcon"
Push-Location $WsRoot
try {
    & $VenvColcon build --packages-select basestation --symlink-install
    if ($LASTEXITCODE -ne 0) {
        Write-Warning "--symlink-install failed (likely missing the Windows symlink privilege -- see INSTALL.md). Falling back to a plain copy install."
        $staleBuildDir = Join-Path $WsRoot "build\basestation"
        if (Test-Path $staleBuildDir) {
            Remove-Item -Recurse -Force $staleBuildDir
        }
        & $VenvColcon build --packages-select basestation
        if ($LASTEXITCODE -ne 0) {
            Write-Error "colcon build failed even without --symlink-install -- see output above."
        }
    }
}
finally {
    Pop-Location
}

Write-Step "Verifying executables"
. (Join-Path $WsRoot "install\local_setup.ps1")
ros2 pkg executables basestation
