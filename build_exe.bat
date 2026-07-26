@echo off
echo === UMGT - Build BOTH dashboards ===

:: Install dependencies if not already installed
pip install pyserial pyinstaller

:: Build each dashboard from its spec file (matches how they are released).
:: --noconfirm overwrites the previous dist\*.exe without prompting.
echo.
echo --- Building UMGT_Engine.exe (engine_dashboard.py) ---
pyinstaller --noconfirm UMGT_Engine.spec

echo.
echo --- Building ECU_Dashboard.exe (ecu_dashboard.py) ---
pyinstaller --noconfirm ECU_Dashboard.spec

echo.
echo Build complete:
echo   dist\UMGT_Engine.exe
echo   dist\ECU_Dashboard.exe
echo.
echo NOTE: close any running UMGT_Engine.exe / ECU_Dashboard.exe first,
echo       or the rebuild fails with "Access is denied".
pause
