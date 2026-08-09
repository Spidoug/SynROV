@echo off
setlocal
cd /d "%~dp0"
echo Starting SynROV AiBot - Version 1...
echo.
python synrov_aibot.py
if errorlevel 1 (
  echo.
  echo SynROV AiBot exited with an error. The message above shows the cause.
  echo.
  pause
)
endlocal
