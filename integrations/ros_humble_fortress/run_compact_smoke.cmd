@echo off
setlocal
set "AUTONAV_TIMEOUT=90"
if not "%~1"=="" set "AUTONAV_TIMEOUT=%~1"
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0run_compact_smoke.ps1" -MissionTimeoutSec %AUTONAV_TIMEOUT%
exit /b %errorlevel%
