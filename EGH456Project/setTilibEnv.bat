@echo off
setlocal

:: Ask user for the path
set /p TILIB_PATH=Enter the full path to TILIB: 

:: Confirm path
echo You entered: %TILIB_PATH%
pause

:: Set system environment variable using setx (requires admin)
echo Setting TILIB system environment variable...

:: Check if script is running as administrator
net session >nul 2>&1
if %errorlevel% neq 0 (
    echo This script must be run as administrator to set system variables.
    pause
    exit /b
)

:: Set the environment variable
setx TILIB "%TILIB_PATH%" /M

echo TILIB has been set to: %TILIB_PATH%
pause
