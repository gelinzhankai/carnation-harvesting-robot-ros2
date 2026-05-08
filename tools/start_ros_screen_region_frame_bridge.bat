@echo off
setlocal

set "PYTHON_EXE=H:\carnation_detection\.venv\Scripts\python.exe"
set "SCRIPT_PATH=\\wsl.localhost\Ubuntu-24.04\root\carnation_harvest\tools\screen_region_frame_bridge.py"

if not exist "%PYTHON_EXE%" (
    echo Python not found: %PYTHON_EXE%
    pause
    exit /b 1
)

if not exist "%SCRIPT_PATH%" (
    echo Script not found: %SCRIPT_PATH%
    pause
    exit /b 1
)

"%PYTHON_EXE%" "%SCRIPT_PATH%" ^
  --output "H:\carnation_detection\ros_screen_region_frame.jpg" ^
  --config "H:\carnation_detection\ros_screen_region_config.json" ^
  --trigger-file "H:\carnation_detection\ros_screen_region_trigger.txt" ^
  --mode triggered ^
  --width 1000 ^
  --height 1000 ^
  --right-margin 0 ^
  --bottom-margin 0 ^
  --fps 2.0

pause
