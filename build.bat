@echo off
setlocal

cd /d "%~dp0"

set "BOOTSTRAP_PYTHON="
where py >nul 2>nul
if not errorlevel 1 set "BOOTSTRAP_PYTHON=py -3"

if not defined BOOTSTRAP_PYTHON (
  where python >nul 2>nul
  if not errorlevel 1 set "BOOTSTRAP_PYTHON=python"
)

if not defined BOOTSTRAP_PYTHON (
  echo Python 3 was not found.
  echo Install 64-bit Python from https://www.python.org/downloads/windows/
  exit /b 1
)

if not exist ".venv\Scripts\python.exe" (
  echo Creating virtual environment...
  %BOOTSTRAP_PYTHON% -m venv .venv
  if errorlevel 1 goto :failed
)

set "PYTHON=.venv\Scripts\python.exe"

echo Installing Python tools...
"%PYTHON%" -m pip install --upgrade pip
if errorlevel 1 goto :failed
"%PYTHON%" -m pip install -r requirements.txt
if errorlevel 1 goto :failed

echo Checking launcher syntax...
"%PYTHON%" -m py_compile sim\launch_simple_motor_gear_controls.py
if errorlevel 1 goto :failed

echo Compiling and validating the MuJoCo model...
"%PYTHON%" sim\build_model.py
if errorlevel 1 goto :failed

echo.
echo Build complete.
echo Run the simulation with: runsim.bat
exit /b 0

:failed
echo.
echo Build failed.
exit /b 1
