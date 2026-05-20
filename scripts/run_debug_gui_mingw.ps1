param(
    [string]$QtRoot = "E:\Qt\6.11.0\mingw_64",
    [string]$MingwBin = "E:\Qt\Tools\mingw1310_64\bin"
)

$ErrorActionPreference = "Stop"
$env:Path = "$QtRoot\bin;$MingwBin;$env:Path"
$env:QT_PLUGIN_PATH = "$QtRoot\plugins"
& ".\build-debug-mingw\bin\sonocontrol_gui.exe"
