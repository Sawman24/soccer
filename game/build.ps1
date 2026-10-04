$ErrorActionPreference = "Stop"
$root   = Split-Path -Parent $PSScriptRoot
$gcc    = Join-Path $root "tools\w64devkit\bin"
$raylib = Join-Path $root "tools\raylib-6.0_win64_mingw-w64"
if (Test-Path $gcc) { $env:PATH = "$gcc;$env:PATH" }

Push-Location $PSScriptRoot
try {
    gcc -std=c99 -O2 -Wall -Wextra `
        -I"$raylib\include" -I"..\export_c\include" `
        src\main.c src\net_client.c -o sarpbc.exe `
        -L"$raylib\lib" -lraylib -lopengl32 -lgdi32 -lwinmm -lws2_32
    if ($LASTEXITCODE -ne 0) { throw "build exe failed" }

    gcc -std=c99 -O2 -Wall -Wextra -shared -DSARPBC_DLL `
        -I"$raylib\include" -I"..\export_c\include" `
        src\main.c src\net_client.c -o sarpbc_game.dll `
        -L"$raylib\lib" -lraylib -lopengl32 -lgdi32 -lwinmm -lws2_32
    if ($LASTEXITCODE -ne 0) { throw "build dll failed" }

    Write-Host "built game\sarpbc.exe and game\sarpbc_game.dll"
} finally { Pop-Location }
