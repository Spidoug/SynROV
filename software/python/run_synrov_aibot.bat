@echo off
setlocal
cd /d "%~dp0"
echo Starting SynROV AiBot - Version 1...
echo.
python -c "import cv2, PIL, websocket, speech_recognition, sounddevice" >nul 2>&1
if errorlevel 1 (
  echo Installing/updating SynROV AiBot dependencies...
  python -m pip install -r requirements.txt
  if errorlevel 1 (
    echo.
    echo Could not install the required dependencies.
    echo Run: python -m pip install -r requirements.txt
    pause
    exit /b 1
  )
)
python synrov_aibot.py
if errorlevel 1 (
  echo.
  echo SynROV AiBot exited with an error. The message above shows the cause.
  echo.
  pause
)
endlocal
