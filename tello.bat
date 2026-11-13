@echo off
rem Double-click: connects the USB Wi-Fi dongle to the Tello, then starts ROS (driver, localizer, EKF, RViz) in WSL
rem and opens a second window ready for the commands (teleop, missions).
rem   tello.bat          real drone (switch it on first: LED blinking yellow)
rem   tello.bat mock     fake drone, no Wi-Fi needed
setlocal EnableDelayedExpansion

rem ---- your drone and your Wi-Fi dongle (netsh wlan show interfaces)
set "TELLO_SSID=TELLO-5F897A"
set "TELLO_IF=Wi-Fi 2"

set "ROS_ENV=source /opt/ros/jazzy/setup.bash && source ~/tello_ws/install/setup.bash && source ~/tello_ws/src/tello-balloon-hunter/scripts/wsl/ros_env.sh"
set "LAUNCH_ARGS="

if /i "%~1"=="mock" (
    set "LAUNCH_ARGS=mock:=true"
    goto start_ros
)

rem ---- 1. Wi-Fi: the dongle on the Tello (the built-in card stays on the internet)
echo Connecting "%TELLO_IF%" to %TELLO_SSID% ... (drone ON, LED blinking yellow)
set /a tries=0
:wifi_wait
netsh wlan show interfaces | findstr /C:": %TELLO_SSID%" >nul && goto wifi_ok
netsh wlan connect name="%TELLO_SSID%" interface="%TELLO_IF%" >nul 2>&1
timeout /t 3 /nobreak >nul
set /a tries+=1
if !tries! geq 20 (
    echo.
    echo No connection to %TELLO_SSID% after 60 s.
    echo  - Is the drone on, LED blinking yellow?  Switch it off and on again.
    echo  - Is the dongle plugged in?  Its name must be "%TELLO_IF%" ^(netsh wlan show interfaces^).
    pause
    exit /b 1
)
goto wifi_wait
:wifi_ok
echo Connected to %TELLO_SSID%.

:start_ros
rem ---- 2. ROS: driver + localizer + EKF + RViz (band mode, climbs to 1.85 m after takeoff)
echo Starting ROS %LAUNCH_ARGS% ...
start "Tello ROS - Ctrl+C here lands and stops everything" wsl.exe bash -c "%ROS_ENV% && ros2 launch tello_driver real.launch.py %LAUNCH_ARGS%; echo; read -p 'ROS stopped. Press ENTER to close.'"

rem ---- 3. A ready terminal for the commands
timeout /t 8 /nobreak >nul
start "Tello commands" wsl.exe bash -c "%ROS_ENV% && cd ~/tello_ws/src/tello-balloon-hunter && echo && echo '  ros2 run tello_driver tello_teleop          fly by hand (t take off, g land)' && echo '  ros2 run tello_driver tello_goto 49 35      mission: in front of 49, then 35' && echo '  python3 tools/flight_report.py --band 1.7 2.0   after the flight' && echo && exec bash"
endlocal
