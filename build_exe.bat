@echo off
echo === ECU Dashboard - Build EXE ===

:: Install dependencies if not already installed
pip install pyserial pyinstaller

:: Build single-file windowed EXE
pyinstaller --onefile --windowed --name ECU_Dashboard ecu_dashboard.py

echo.
echo Build complete. EXE is in dist\ECU_Dashboard.exe
pause
