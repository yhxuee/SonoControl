param(
    [string]$QtCMakePath = ""
)

$ErrorActionPreference = "Stop"
$opts = @("-S", ".", "-B", "build-debug", "-G", "Ninja", "-DCMAKE_BUILD_TYPE=Debug")
if ($QtCMakePath -ne "") {
    $opts += "-DCMAKE_PREFIX_PATH=$QtCMakePath"
}
cmake @opts
cmake --build build-debug
Write-Host "Built. GUI path: .\build-debug\bin\sonocontrol_gui.exe"
