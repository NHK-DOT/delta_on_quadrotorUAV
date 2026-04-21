@echo off
setlocal
set SCRIPT_DIR=%~dp0
if "%~2"=="" (
  echo Usage: run_gcode_servo.bat ^<gcode_path^> ^<port^> [time_ms]
  exit /b 1
)
set INPUT=%~1
set PORT=%~2
set TIME_MS=%~3
if "%TIME_MS%"=="" set TIME_MS=120
powershell -ExecutionPolicy Bypass -File "%SCRIPT_DIR%pipeline.ps1" -Mode run -InputPath "%INPUT%" -Port "%PORT%" -TimeMs %TIME_MS%
endlocal
