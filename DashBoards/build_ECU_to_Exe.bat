@echo off
echo === Building the ECU_Dashboard  ===

echo Close any running "ECU_Dashboard.exe" first, or the rebuild will fail with "Access is denied".
pause

:: Install dependencies if not already installed
pip install pyserial pyinstaller

echo.
echo --- Building ECU_Dashboard.exe (ecu_dashboard.py) ---
echo.

pyinstaller --clean --noconfirm ECU_Dashboard.spec

echo.
echo Build complete ==> dist\ECU_Dashboard.exe
echo.

pause
