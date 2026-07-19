@echo off
setlocal

cd /d "%~dp0"

if exist "%~dp0.venv\Scripts\python.exe" (
  "%~dp0.venv\Scripts\python.exe" "%~dp0sim\launch_simple_motor_gear_controls.py"
  goto :finished
)

where python >nul 2>nul
if %errorlevel%==0 (
  python "%~dp0sim\launch_simple_motor_gear_controls.py"
) else (
  py -3 "%~dp0sim\launch_simple_motor_gear_controls.py"
)

:finished
if errorlevel 1 (
  echo.
  echo Simulation failed to start. Make sure Python and the mujoco Python package are installed.
  pause
)
