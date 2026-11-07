@echo off
rem Double-click me. Menu for the Tello tests: no folder or command to remember.
rem Or run one test directly:  tello_tests.bat 2
setlocal
cd /d "%~dp0"
set "PY=%USERPROFILE%\venvs\tello\Scripts\python.exe"
if not exist "%PY%" (
  echo Python environment not found: %PY%
  echo Create it once with:  python -m venv %USERPROFILE%\venvs\tello
  echo then:                 %USERPROFILE%\venvs\tello\Scripts\python -m pip install -r scripts\requirements.txt
  pause
  exit /b 1
)

:menu
if not "%~1"=="" ( set "CHOICE=%~1" & set "ONESHOT=1" & goto run )
cls
echo ================================================================
echo   TELLO TESTS      (folder: %CD%)
echo ================================================================
echo   Before: drone ON (LED blinking yellow), this PC on TELLO-xxxxxx Wi-Fi,
echo           phone app closed. Run test 1 FIRST after each drone power-on.
echo.
echo   1  Connection check   (motors stay off)
echo   2  Video latency      (drone on the table, camera facing the flashing window, ~60 s, then Esc)
echo   3  Telemetry by hand / manual flight
echo        keys: T take off, G land, arrows move, I/K up/down, J/L yaw,
echo              Space hover, Esc land+quit, Backspace EMERGENCY
echo   4  Automatic hover 30 s
echo   5  Fake drone (no hardware, to try the keys)
echo   Q  Quit
echo.
set "CHOICE="
set /p CHOICE=Your choice:
:run
if /i "%CHOICE%"=="1" ( "%PY%" scripts\tello_udp_check.py & goto done )
if /i "%CHOICE%"=="2" ( "%PY%" scripts\first_flight.py --latency-test & goto done )
if /i "%CHOICE%"=="3" ( "%PY%" scripts\first_flight.py & goto done )
if /i "%CHOICE%"=="4" ( "%PY%" scripts\first_flight.py --auto-hover 30 & goto done )
if /i "%CHOICE%"=="5" ( "%PY%" scripts\first_flight.py --mock & goto done )
if /i "%CHOICE%"=="Q" exit /b 0
if defined ONESHOT ( echo Unknown test "%CHOICE%": use 1-5 & exit /b 1 )
goto menu

:done
echo.
echo Results are in the logs folder: %CD%\logs
if defined ONESHOT exit /b 0
pause
goto menu
