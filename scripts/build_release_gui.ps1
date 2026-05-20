param(
    [string]$QtCMakePath = ""
)

$ErrorActionPreference = "Stop"
$opts = @("-S", ".", "-B", "build-release", "-G", "Ninja", "-DCMAKE_BUILD_TYPE=Release")
if ($QtCMakePath -ne "") {
    $opts += "-DCMAKE_PREFIX_PATH=$QtCMakePath"
}
cmake @opts
cmake --build build-release
Write-Host "Built. GUI path: .\build-release\bin\sonocontrol_gui.exe"
