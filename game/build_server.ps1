$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $PSScriptRoot
$gcc    = Join-Path $root "tools\w64devkit\bin"
$raylib = Join-Path $root "tools\raylib-6.0_win64_mingw-w64"
if (Test-Path $gcc) { $env:PATH = "$gcc;$env:PATH" }

Push-Location $PSScriptRoot
try {
    gcc -std=c99 -O2 -Wall -Wextra `
        -I"$raylib\include" -I"..\export_c\include" `
        src\server_main.c -o sarpbc_server.exe -lws2_32 -lwinmm
    if ($LASTEXITCODE -ne 0) { throw "build server failed" }
    Write-Host "built game\sarpbc_server.exe successfully"
} finally { Pop-Location }
