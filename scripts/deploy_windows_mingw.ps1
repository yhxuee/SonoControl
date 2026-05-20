param(
    [string]$QtRoot = "E:\Qt\6.11.0\mingw_64",
    [string]$MingwRoot = "",
    [string]$BuildDir = "cmake-build-release",
    [string]$DistDir = "dist\SonoControl_Windows_Portable",
    [switch]$SkipBuild
)

$ErrorActionPreference = "Stop"
$ProjectRoot = Split-Path -Parent (Split-Path -Parent $MyInvocation.MyCommand.Path)
Set-Location $ProjectRoot

function Fail($msg) {
    Write-Host "ERROR: $msg" -ForegroundColor Red
    exit 1
}

if (!(Test-Path $QtRoot)) { Fail "QtRoot not found: $QtRoot" }
$QtBin = Join-Path $QtRoot "bin"
$WindeployQt = Join-Path $QtBin "windeployqt.exe"
if (!(Test-Path $WindeployQt)) { Fail "windeployqt.exe not found: $WindeployQt" }

if ([string]::IsNullOrWhiteSpace($MingwRoot)) {
    $qtInstallRoot = Split-Path -Parent (Split-Path -Parent (Split-Path -Parent $QtRoot))
    $toolsRoot = Join-Path $qtInstallRoot "Tools"
    $candidates = @(
        Join-Path $toolsRoot "mingw1310_64",
        Join-Path $toolsRoot "mingw1120_64",
        Join-Path $toolsRoot "mingw810_64"
    )
    foreach ($c in $candidates) {
        if (Test-Path (Join-Path $c "bin\g++.exe")) { $MingwRoot = $c; break }
    }
}
if ([string]::IsNullOrWhiteSpace($MingwRoot) -or !(Test-Path (Join-Path $MingwRoot "bin\g++.exe"))) {
    Fail "MinGW root was not found. Pass -MingwRoot E:\Qt\Tools\mingwXXXX_64"
}

$Gcc = Join-Path $MingwRoot "bin\gcc.exe"
$Gxx = Join-Path $MingwRoot "bin\g++.exe"

if (!$SkipBuild) {
    cmake -S . -B $BuildDir -G Ninja -DCMAKE_BUILD_TYPE=Release `
        -DCMAKE_PREFIX_PATH="$QtRoot\lib\cmake" `
        -DCMAKE_C_COMPILER="$Gcc" `
        -DCMAKE_CXX_COMPILER="$Gxx"
    cmake --build $BuildDir --config Release
}

$BinDir = Join-Path $BuildDir "bin"
$GuiExe = Join-Path $BinDir "sonocontrol_gui.exe"
$CliExe = Join-Path $BinDir "sonocontrol.exe"
if (!(Test-Path $GuiExe)) { Fail "GUI executable not found: $GuiExe" }

if (Test-Path $DistDir) { Remove-Item $DistDir -Recurse -Force }
New-Item -ItemType Directory -Force -Path $DistDir | Out-Null
Copy-Item $GuiExe $DistDir
if (Test-Path $CliExe) { Copy-Item $CliExe $DistDir }

& $WindeployQt --release --compiler-runtime --no-translations (Join-Path $DistDir "sonocontrol_gui.exe")

# Some Qt/MinGW combinations do not copy the compiler runtime consistently.
$runtimeDlls = @("libgcc_s_seh-1.dll", "libstdc++-6.dll", "libwinpthread-1.dll")
foreach ($dll in $runtimeDlls) {
    $src = Join-Path (Join-Path $MingwRoot "bin") $dll
    if (Test-Path $src) { Copy-Item $src $DistDir -Force }
}

@"
SonoControl Windows Portable Package
====================================

Run:
  sonocontrol_gui.exe

This folder is self-contained for a normal Windows PC. Qt and MinGW DLLs are bundled.
Do not move sonocontrol_gui.exe out of this folder; keep the platforms/ folder and DLLs beside it.

Release mode notes:
- Debug simulation UI is disabled at compile time.
- Real serial and UDP transport are used.
- If COM/UDP/temperature errors occur, the GUI shows a popup and aborts the experiment.
"@ | Set-Content -Encoding UTF8 (Join-Path $DistDir "README_RUN.txt")

@" 
@echo off
cd /d %~dp0
start "SonoControl" sonocontrol_gui.exe
"@ | Set-Content -Encoding ASCII (Join-Path $DistDir "Run_SonoControl.bat")

$Zip = "$DistDir.zip"
if (Test-Path $Zip) { Remove-Item $Zip -Force }
Compress-Archive -Path (Join-Path $DistDir "*") -DestinationPath $Zip

Write-Host "Portable package created:" -ForegroundColor Green
Write-Host "  $DistDir"
Write-Host "  $Zip"
