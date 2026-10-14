@echo off
setlocal
for %%I in ("%~dp0..\..") do set "AUTONAV_REPO=%%~fI"
set "AUTONAV_IMAGE=autonav-humble-fortress:20260928"
set "AUTONAV_VOLUME=autonav-full-track-20260928"

docker build -t "%AUTONAV_IMAGE%" -f "%~dp0Dockerfile" "%AUTONAV_REPO%"
if errorlevel 1 exit /b 1

docker volume create "%AUTONAV_VOLUME%" >nul
if errorlevel 1 exit /b 1

docker run --rm -v "%AUTONAV_VOLUME%:/ws" -v "%AUTONAV_REPO%:/host" "%AUTONAV_IMAGE%" bash /host/integrations/ros_humble_fortress/bootstrap_workspace.sh
exit /b %errorlevel%
