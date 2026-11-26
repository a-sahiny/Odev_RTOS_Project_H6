# attitude.c birim testini PC'de derler ve kosar; GECTI / KALDI basar.
$ErrorActionPreference = 'Stop'
$gcc = (Get-Command gcc -ErrorAction SilentlyContinue).Source
if (-not $gcc) { throw 'gcc bulunamadi (ornek: C:\msys64\ucrt64\bin)' }
$exe = Join-Path $env:TEMP 'attitude_test.exe'
& $gcc -std=gnu11 -O1 -Wall -Wextra -Werror -I"$PSScriptRoot\..\..\App\Inc" "$PSScriptRoot\attitude_test.c" "$PSScriptRoot\..\..\App\Src\attitude.c" -o $exe -lm
if ($LASTEXITCODE -ne 0) { throw 'derleme hatasi' }
& $exe
exit $LASTEXITCODE
