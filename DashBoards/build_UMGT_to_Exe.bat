@echo off
echo === Building the UMGT_Enbgine_Dashboard  ===

echo Close any running "UMGT_Engine.exe" first, or the rebuild will fail with "Access is denied".
pause

:: Install dependencies if not already installed
pip install pyserial pyinstaller

echo.
echo --- Building UMGT_Engine.exe (engine_dashboard.py) ---
echo.

pyinstaller --clean --noconfirm UMGT_Engine.spec

echo.
echo Build complete ==> dist\UMGT_Engine.exe
echo.

pause
