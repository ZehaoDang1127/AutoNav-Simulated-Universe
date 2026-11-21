@echo off
setlocal
cd /d "%~dp0"
if not exist ".venv\Scripts\python.exe" (
    echo Create the Python environment and install the GUI dependencies first:
    echo python -m venv .venv
    echo .venv\Scripts\python.exe -m pip install -e ".[gui]"
    pause
    exit /b 2
)
".venv\Scripts\python.exe" -m autonav_universe.cli app --repo "%~dp0." %*
if errorlevel 1 pause
