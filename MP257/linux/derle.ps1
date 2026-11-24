# telem_udp'yi kart (aarch64 Linux) icin derler. Android NDK'nin clang'i kullanilir; statik oldugu icin kartta kutuphane gerekmez.
# android29: daha eskisinin statik programi TLS hizasi yuzunden kartta acilmiyor.
# Kullanim: powershell -ExecutionPolicy Bypass -File derle.ps1 [-Cikti <dosya>] [-Derleyici <clang.exe>]
param(
    [string]$Cikti = "$env:TEMP\telem_udp",
    [string]$Derleyici = ""
)

$ErrorActionPreference = 'Stop'

if (-not $Derleyici) {
    $bulunan = Get-ChildItem "$env:LOCALAPPDATA\Android\Sdk\ndk\*\toolchains\llvm\prebuilt\windows-x86_64\bin\clang.exe" -ErrorAction SilentlyContinue | Sort-Object FullName | Select-Object -Last 1
    if (-not $bulunan) { throw 'Android NDK bulunamadi. -Derleyici ile clang.exe yolu verin.' }
    $Derleyici = $bulunan.FullName
}

& $Derleyici --target=aarch64-linux-android29 -static -s -O2 -std=c11 -D_GNU_SOURCE -Wall -Wextra -Wpedantic -Werror -o $Cikti "$PSScriptRoot\telem_udp.c"
if ($LASTEXITCODE -ne 0) { throw "derleme basarisiz: $LASTEXITCODE" }

Write-Output "tamam: $Cikti ($((Get-Item $Cikti).Length) bayt)"
