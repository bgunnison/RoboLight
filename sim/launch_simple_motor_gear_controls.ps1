$ErrorActionPreference = 'Stop'

$root = Split-Path $PSScriptRoot -Parent
$venvPython = Join-Path $root '.venv\Scripts\python.exe'
$python = if (Test-Path $venvPython) { $venvPython } else { 'python' }
$script = Join-Path $PSScriptRoot 'launch_simple_motor_gear_controls.py'

& $python $script
