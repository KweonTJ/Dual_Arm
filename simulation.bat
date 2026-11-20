@echo off
setlocal
cd /d "%~dp0"
set "PYTHONUTF8=1"
set "PYTHONNOUSERSITE=1"
set "PYTHONPATH="
set "PYTHONHOME="
if exist ".venv-windows\Scripts\python.exe" goto existing
for %%V in (3.12 3.11 3.10) do (
    py -%%V -c "import struct; assert struct.calcsize('P') == 8" >nul 2>&1
    if not errorlevel 1 (
        py -%%V "src\dual_arm_simulation\scripts\windows_bootstrap.py" %*
        goto finished
    )
)
python -c "import sys,struct; assert (3,10) <= sys.version_info[:2] <= (3,12) and struct.calcsize('P') == 8" >nul 2>&1
if errorlevel 1 goto missing
python "src\dual_arm_simulation\scripts\windows_bootstrap.py" %*
goto finished
:existing
".venv-windows\Scripts\python.exe" "src\dual_arm_simulation\scripts\windows_bootstrap.py" %*
:finished
if errorlevel 1 (
    echo.
    echo The app could not start. See the error above and WINDOWS.md.
    pause
    exit /b 1
)
exit /b 0
:missing
echo Install 64-bit Python 3.12 from https://www.python.org/downloads/windows/
echo Enable the Python launcher or Add Python to PATH, then run this file again.
pause
exit /b 1
