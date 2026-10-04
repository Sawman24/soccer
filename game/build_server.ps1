$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $PSScriptRoot
$gcc  = Join-Path $root "tools\w64devkit\bin"
if (Test-Path $gcc) { $env:PATH = "$gcc;$env:PATH" }

Push-Location $PSScriptRoot
try {
    gcc -std=c99 -O2 -Wall -Wextra src\server_main.c -o sarpbc_server.exe -lws2_32
    if ($LASTEXITCODE -ne 0) { throw "build server failed" }
    Write-Host "built game\sarpbc_server.exe successfully"
} finally { Pop-Location }
