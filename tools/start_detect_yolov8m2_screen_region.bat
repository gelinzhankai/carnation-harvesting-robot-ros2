@echo off
setlocal

set "PYTHON_EXE=H:\carnation_detection\.venv\Scripts\python.exe"
set "SCRIPT_PATH=%~dp0screen_region_detect_yolov8m2.py"
set "MODEL_PATH=H:\carnation_detection\carnation_yolov8m2_best.pt"
set "CONFIG_PATH=H:\carnation_detection\ros_screen_region_config.json"

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

if not exist "%MODEL_PATH%" (
    echo Model not found: %MODEL_PATH%
    echo Download carnation_yolov8m2_best.pt from GitHub Releases and place it there.
    pause
    exit /b 1
)

"%PYTHON_EXE%" "%SCRIPT_PATH%" ^
  --model "%MODEL_PATH%" ^
  --config "%CONFIG_PATH%"

pause
