# Zephyr'in ürettiği zephyr.elf'i OP-TEE'nin yükleyebileceği imzalı .bin'e çevirir.
# Aynı işi CubeIDE'nin CA35TDCID_m33_ns_sign post-build adımı yapar; burada Zephyr çıktısına uygulanır.
# ST bunu Zephyr için belgelemiyor (risk Z-R1): ilk denemede kartta remoteproc çıktısıyla doğrula.
param(
    # west build çıktısı: <uygulama>\build\zephyr\zephyr.elf
    [Parameter(Mandatory = $true)][string]$Elf,
    # STM32CubeMP2 paketinin kökü (içinde Utilities\optee_os var)
    [string]$CubeMP2 = 'C:\Users\Asus\Documents\GitHub\STM32MP257F_DK\resources\05_STM32CubeMP2\STM32CubeMP2',
    # Çıkış adı: fw_cortex_m33.sh <klasör>_CM33_NonSecure_sign.bin arar
    [string]$Name = 'F7Sup_Zephyr'
)
$ErrorActionPreference = 'Stop'

if (-not (Test-Path $Elf)) { throw "ELF yok: $Elf" }
$sign = Join-Path $CubeMP2 'Utilities\optee_os\scripts\sign_rproc_fw.py'
$key = Join-Path $CubeMP2 'Utilities\optee_os\keys\default.pem'
if (-not (Test-Path $sign)) { throw "imza betigi yok: $sign" }

# objcopy: önce PATH, sonra CubeIDE 2.3.0'ın GNU araçları
$objcopy = (Get-Command arm-none-eabi-objcopy -ErrorAction SilentlyContinue).Source
if (-not $objcopy) {
    $objcopy = Get-ChildItem 'C:\ST\STM32CubeIDE_2.3.0' -Recurse -Filter arm-none-eabi-objcopy.exe -ErrorAction SilentlyContinue |
        Select-Object -First 1 -ExpandProperty FullName
}
if (-not $objcopy) { throw 'arm-none-eabi-objcopy bulunamadi' }

$dir = Split-Path $Elf -Parent
$stripped = Join-Path $dir "$($Name)_CM33_NonSecure_stripped.elf"
$out = Join-Path $dir "$($Name)_CM33_NonSecure_sign.bin"

# Hata ayıklama bilgisini at (CubeIDE post-build'deki objcopy -S ile aynı)
& $objcopy -S $Elf $stripped
if ($LASTEXITCODE -ne 0) { throw 'objcopy basarisiz' }

# BOOTADDR 0x80100000: CubeMP2 örnekleriyle ve Zephyr'in ddr_code bölgesiyle aynı adres
& python $sign --in $stripped --out $out --key $key --plat-tlv BOOTADDR 0x80100000
if ($LASTEXITCODE -ne 0) { throw 'imzalama basarisiz' }

# İlk 4 bayt 68 A4 43 35 olmalı (OP-TEE rproc imza başlığı)
$head = [System.IO.File]::ReadAllBytes($out)[0..3] | ForEach-Object { $_.ToString('X2') }
Write-Host "Imzali dosya: $out"
Write-Host "Ilk 4 bayt: $($head -join ' ') (beklenen 68 A4 43 35)"
