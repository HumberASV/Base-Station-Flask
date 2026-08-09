<#
.SYNOPSIS
    Starts the enviroment for ros
.DESCRIPTION
    Starts enviroment for ros

.PARAMETER CycloneDdsUri
    Path to a cyclonedds.xml config file. When passed, switches the DDS middleware from
    Humble's default (Fast-RTPS) to Cyclone DDS for this process and everything it launches,
    by setting RMW_IMPLEMENTATION=rmw_cyclonedds_cpp and CYCLONEDDS_URI to that file.

.EXAMPLE
    .\setup.ps1

.EXAMPLE
    .\setup.ps1 -CycloneDdsUri C:\dev\cyclonedds.xml
#>

[CmdletBinding()]
param(
    [string]$CycloneDdsUri = ""
)

$ErrorActionPreference = "Stop"

$RepoRoot   = Split-Path -Parent $MyInvocation.MyCommand.Path
$Ros2Install = "C:\dev\ros2_humble"
$Ros2ExtraWs = "C:\dev\ros2_extra_ws"
$VenvPython  = Join-Path $RepoRoot "venv310\Scripts\python.exe"
$env:ROS_DOMAIN_ID=0

function Write-Step {
    param([string]$Message)
    Write-Host "==> $Message" -ForegroundColor Cyan
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
    $env:ROS_DOMAIN_ID=0
    # Prefer constructing a proper file:// URI via the .NET System.Uri class so
    # spaces and special characters are encoded correctly on Windows. Fall back
    # to a simple file:/// conversion if Uri construction fails.
    try {
        $uri = (New-Object System.Uri($absolutePath)).AbsoluteUri
    } catch {
        $uri = "file:///$($absolutePath -replace '\\','/')"
    }

    # Also prepare a plain absolute-path fallback (some Cyclone builds on
    # Windows accept a bare path instead of a file:// URI).
    $plain = $absolutePath -replace '\\','/'

    # On Windows, some Cyclone DDS builds prefer a plain absolute path
    # rather than a file:// URI. Prefer the plain path when running on
    # Windows; otherwise use the encoded file:// URI.
    if ($env:OS -and $env:OS -match 'Windows') {
        $env:CYCLONEDDS_URI = $plain
        Write-Step "Using Cyclone DDS (CYCLONEDDS_URI=$($env:CYCLONEDDS_URI)) (plain path preferred on Windows)"
        Write-Host "==> file-URI fallback: $uri" -ForegroundColor Cyan
    }
    else {
        $env:CYCLONEDDS_URI = $uri
        Write-Step "Using Cyclone DDS (CYCLONEDDS_URI=$($env:CYCLONEDDS_URI))"
        if ($uri -ne $plain) { Write-Host "==> plain fallback path: $plain" -ForegroundColor Cyan }
    }
}

& echo $env:CYCLONEDDS_URI
& echo $env:ROS_DOMAIN_ID
& echo $env:RMW_IMPLEMENTATION
& ros2 topic list