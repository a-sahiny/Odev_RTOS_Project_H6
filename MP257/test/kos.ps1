# USART6 hattinin (Linux mesaji, cerceveler, satirlar) birim testini PC'de derler ve kosar; GECTI / KALDI basar.
$ErrorActionPreference = 'Stop'
$gcc = (Get-Command gcc -ErrorAction SilentlyContinue).Source
if (-not $gcc) { throw 'gcc bulunamadi (ornek: C:\msys64\ucrt64\bin)' }
$exe = Join-Path $env:TEMP 'linux_mesaji_test.exe'
& $gcc -std=gnu11 -Wall -Wextra -Werror -I"$PSScriptRoot\stub" -I"$PSScriptRoot\..\..\App\Inc" -I"$PSScriptRoot\..\zephyr\src" "$PSScriptRoot\linux_mesaji_test.c" "$PSScriptRoot\..\..\App\Src\crc.c" -o $exe
if ($LASTEXITCODE -ne 0) { throw 'derleme hatasi' }
& $exe
exit $LASTEXITCODE
