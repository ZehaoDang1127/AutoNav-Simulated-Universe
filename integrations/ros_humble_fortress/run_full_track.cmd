@echo off
setlocal
set "AUTONAV_TIMEOUT=900"
if not "%~1"=="" set "AUTONAV_TIMEOUT=%~1"
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0run_compact_smoke.ps1" -FullTrack -MissionTimeoutSec %AUTONAV_TIMEOUT%
exit /b %errorlevel%
