@echo off
setlocal
set SCRIPT_DIR=%~dp0
if "%~1"=="" (
  echo Usage: export_servo_json.bat ^<gcode_path^> [time_ms]
  exit /b 1
)
set INPUT=%~1
set TIME_MS=%~2
if "%TIME_MS%"=="" set TIME_MS=120
powershell -ExecutionPolicy Bypass -File "%SCRIPT_DIR%pipeline.ps1" -Mode export -InputPath "%INPUT%" -TimeMs %TIME_MS%
endlocal
