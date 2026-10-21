@echo off
setlocal enabledelayedexpansion

title F.O.R.G.E Explorer — 3D Reconstruction Platform

:: 1. Navigate to the F.O.R.G.E repository root
cd /d "%~dp0"

echo ===============================================================================
echo   F.O.R.G.E — Fast Omnidirectional Reconstruction for Geometric Environments
echo   Interactive 3D Explorer ^& National Hackathon Demonstration Launcher
echo ===============================================================================
echo.

:: 2. Detect Python Environment
set "PYTHON_CMD="

if exist "C:\Users\25beevdt025\Desktop\forge-spatial\.venv\Scripts\python.exe" (
    set "PYTHON_CMD=C:\Users\25beevdt025\Desktop\forge-spatial\.venv\Scripts\python.exe"
    echo [*] Detected virtual environment: forge-spatial\.venv
) else if exist ".venv\Scripts\python.exe" (
    set "PYTHON_CMD=.venv\Scripts\python.exe"
    echo [*] Detected local virtual environment: .venv
) else (
    where python >nul 2>&1
    if !errorlevel! equ 0 (
        set "PYTHON_CMD=python"
        echo [*] Using system Python
    )
)

if "%PYTHON_CMD%"=="" (
    echo [ERROR] Python was not found in PATH or standard virtual environments.
    echo Please install Python 3.10+ or activate your virtual environment.
    echo.
    pause
    exit /b 1
)

:: Verify Python can run
"%PYTHON_CMD%" --version >nul 2>&1
if !errorlevel! neq 0 (
    echo [ERROR] Failed to execute Python: %PYTHON_CMD%
    pause
    exit /b 1
)

:: 3. Check for Verified Model Release Directory
set "MODELS_DIR=outputs\accuracy_experiment\sprint9_video_to_viewer_release"
if not exist "%MODELS_DIR%\run_manifest.json" (
    echo [ERROR] Sprint 9 reconstruction release assets not found in:
    echo         %MODELS_DIR%
    echo Please ensure the release directory is present before starting the viewer.
    echo.
    pause
    exit /b 1
)
echo [*] Verified model release baseline: %MODELS_DIR%

:: 4. Check for Frontend Build
set "DIST_DIR=viewer\dist"
if not exist "%DIST_DIR%\index.html" (
    echo [WARNING] Production frontend build not found at %DIST_DIR%\index.html
    echo           Falling back to development directory: viewer\
) else (
    echo [*] Verified production frontend build: %DIST_DIR%
)

:: 5. Check if Server is Already Running on Port 5173
set "PORT=5173"
set "SERVER_RUNNING=0"
for /f "tokens=5" %%a in ('netstat -aon ^| findstr /r ":%PORT%\>.*LISTENING"') do (
    set "SERVER_RUNNING=1"
)

if "!SERVER_RUNNING!"=="1" (
    echo [*] F.O.R.G.E Explorer server is already active on port %PORT%.
    echo     Avoiding duplicate server launch.
) else (
    echo [*] Starting F.O.R.G.E Explorer Server on port %PORT%...
    start "FORGE Explorer Server" /min "%PYTHON_CMD%" scripts\serve_viewer.py --port %PORT%
    ping 127.0.0.1 -n 3 >nul
)

:: 6. Launch Default Web Browser
set "APP_URL=http://localhost:%PORT%/"
echo.
echo ===============================================================================
echo   F.O.R.G.E Explorer is live at:
echo   URL: %APP_URL%
echo.
echo   Presentation Controls ^& Shortcuts:
echo     [1] Top View     [2] Front View     [3] Side View     [4] Isometric
echo     [R] Reset Camera [W] Wireframe      [G] Ground Grid   [C] Dual Compare
echo     [P] Pipeline Workflow Architecture Modal
echo ===============================================================================
echo.
echo Opening default web browser...
start "" "%APP_URL%"

echo.
echo Server running in background. You may keep this window open or close it.
echo Press any key to view server process status...
pause >nul

netstat -aon | findstr /r ":%PORT%\>.*LISTENING"
echo.
pause
