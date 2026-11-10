@echo off
rem Double-click me: guided camera calibration with the printed checkerboard.
rem Drone ON, this PC on the TELLO-xxxxxx Wi-Fi, nothing else connected to the drone.
cd /d "%~dp0"
chcp 65001 >nul
"%USERPROFILE%\venvs\tello\Scripts\python.exe" scripts\calibrate_camera.py
echo.
pause
