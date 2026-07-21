@echo off
setlocal

set "SCRIPT_DIR=%~dp0"
set "PROJECT_ROOT=%SCRIPT_DIR%..\.."
set "PYTHON_EXE=%PROJECT_ROOT%\.venv\Scripts\python.exe"
set "SCRIPT_PATH=%SCRIPT_DIR%gamepad_controller.py"

if not exist "%PYTHON_EXE%" (
  echo Missing Python: "%PYTHON_EXE%"
  exit /b 1
)

"%PYTHON_EXE%" "%SCRIPT_PATH%" --port COM19 --start-from-current --slow-start

endlocal
