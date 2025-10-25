@echo off
title ESP32-CAM WebSocket Server
color 0A

echo ===============================================
echo    ESP32-CAM WebSocket Server Auto-Starter
echo ===============================================
echo.

echo [INFO] Navigating to WebSocket server directory...
cd /d "%~dp0websocket-server"

echo [INFO] Checking if Node.js is installed...
node --version >nul 2>&1
if errorlevel 1 (
    echo [ERROR] Node.js is not installed or not in PATH!
    echo [INFO] Please install Node.js from: https://nodejs.org
    echo.
    pause
    exit /b 1
)

echo [INFO] Node.js found!
echo.

echo [INFO] Checking if dependencies are installed...
if not exist "node_modules" (
    echo [INFO] Installing WebSocket dependencies...
    npm install
    if errorlevel 1 (
        echo [ERROR] Failed to install dependencies!
        pause
        exit /b 1
    )
)

echo [INFO] Dependencies ready!
echo.

echo ===============================================
echo     Starting ESP32-CAM WebSocket Server
echo ===============================================
echo.
echo [INFO] Server will start on: http://localhost:8080
echo [INFO] Server will auto-detect available port starting from 8080
echo.
echo [TIP] Choose startup option:
echo   1. Start server only
echo   2. Start server + auto-open browser
echo.
set /p choice="Enter choice (1 or 2): "

:start_server
echo.
echo [INFO] Starting server...
echo [INFO] Press Ctrl+C to stop (then close window or wait)
echo.

if "%choice%"=="2" (
    start "" http://localhost:8080
    timeout /t 2 /nobreak >nul
)

REM Run the server
node camera-server.js

REM After server stops, offer restart option
echo.
echo [INFO] Server stopped.
choice /C YN /M "Do you want to restart the server? (Y/N)"
if errorlevel 2 goto end
if errorlevel 1 goto restart_server

:restart_server
echo [INFO] Restarting server...
taskkill /F /IM node.exe >nul 2>&1
timeout /t 1 /nobreak >nul
goto start_server

:end
echo [INFO] Exiting.
exit