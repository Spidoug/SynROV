@echo off
setlocal
cd /d "%~dp0"
echo Iniciando SynROV AiBot - Sistema 1.0...
echo.
python synrov_aibot.py
if errorlevel 1 (
  echo.
  echo O SynROV AiBot encerrou com erro. A mensagem acima mostra a causa.
  echo.
  pause
)
endlocal
