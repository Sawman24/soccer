param(
    [Parameter(ValueFromRemainingArguments = $true)]
    [string[]]$GameArgs
)
$root = Split-Path -Parent $PSScriptRoot
$raylib = Join-Path $root "tools\raylib-6.0_win64_mingw-w64\lib"
$env:PATH = "$raylib;$env:PATH"

$origDir = [System.IO.Directory]::GetCurrentDirectory()
Push-Location $PSScriptRoot
try {
    [System.IO.Directory]::SetCurrentDirectory($PSScriptRoot)

    $sig = @'
    using System;
    using System.Runtime.InteropServices;
    public class SarpbcGameRunner {
        [DllImport("sarpbc_game.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int sarpbc_main(int argc, string[] argv);
    }
'@
    if (-not ([System.Management.Automation.PSTypeName]'SarpbcGameRunner').Type) {
        Add-Type -TypeDefinition $sig
    }

    $allArgs = @("sarpbc.exe")
    if ($GameArgs) { $allArgs += $GameArgs }
    [SarpbcGameRunner]::sarpbc_main($allArgs.Length, $allArgs)
} finally {
    [System.IO.Directory]::SetCurrentDirectory($origDir)
    Pop-Location
}
