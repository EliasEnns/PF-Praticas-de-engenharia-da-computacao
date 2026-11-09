@echo off
REM Roda os dois testes do enlace no Windows.
REM O teste de integracao precisa de um g++ x86 (MinGW-w64, MSYS2 ou WSL).
REM Sem compilador, o teste de contrato ainda roda usando tools\amostra_linhas.jsonl.
setlocal
cd /d "%~dp0.."

where g++ >nul 2>nul
if errorlevel 1 (
    echo [aviso] g++ nao encontrado no PATH. Pulando o teste de integracao.
    echo         Instale MinGW-w64 ou use WSL para rodar tools\rodar_testes.sh.
    goto contrato
)

echo === compilando o teste de integracao ===
g++ -std=c++17 -O1 -Wall -Wextra -Wno-unused-parameter ^
    -I tools\stub -I baja_espnow_tx -I baja_espnow_rx ^
    tools\test_bajalink.cpp -o %TEMP%\test_bajalink.exe
if errorlevel 1 (
    echo [erro] falha ao compilar
    exit /b 1
)
"%TEMP%\test_bajalink.exe"
echo.

:contrato
echo === contrato firmware ^<-^> dashboard ===
python tools\test_formato.py
endlocal
