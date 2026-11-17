@echo off
rem Host aracini exe'ye cevirir (PyInstaller, tek dosya):
rem   dist\TelemetriHost.exe   grafik arayuz (cift tikla acilir, konsol penceresi yok)
rem   dist\vnet.exe            komut satiri  (vnet.exe --port COM5 hello, vnet.exe --sim test all)
rem Kullanim: Host\build_exe.bat   (her klasorden calisir)
setlocal
cd /d "%~dp0"

python -m PyInstaller --version >nul 2>&1
if errorlevel 1 (
    echo PyInstaller yok, kuruluyor...
    python -m pip install pyinstaller || exit /b 1
)
python -c "import serial" >nul 2>&1
if errorlevel 1 (
    echo pyserial yok, kuruluyor...
    python -m pip install pyserial || exit /b 1
)

rem 'tests' paketi importlib ile adindan yuklendigi icin acikca toplanir.
set COMMON=--noconfirm --clean --onefile --paths "%~dp0." --collect-submodules tests --distpath dist --workpath build --specpath build

python -m PyInstaller %COMMON% --windowed --name TelemetriHost vnet_gui.py || exit /b 1
python -m PyInstaller %COMMON% --console --name vnet vnet.py || exit /b 1

echo.
echo Hazir:
echo   %~dp0dist\TelemetriHost.exe
echo   %~dp0dist\vnet.exe
