@echo off
rem Double-click launcher for Armstrong. Keeps the window open so setup errors stay visible.
setlocal
cd /d "%~dp0"
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0start.ps1" %*
echo.
echo Armstrong has stopped. Press any key to close this window.
pause >nul
