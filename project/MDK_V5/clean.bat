@echo off
setlocal enableextensions
REM ======================================================================
REM  clean.bat - Remove Keil build outputs (objects/listings)
REM  Usage: clean.bat
REM ======================================================================

set "PROJ_DIR=%~dp0"
pushd "%PROJ_DIR%"

echo [INFO] Cleaning Keil build outputs ...

if exist "objects\*.obj" del /q "objects\*.obj" 2>nul
if exist "objects\*.o"   del /q "objects\*.o"   2>nul
if exist "objects\*.d"   del /q "objects\*.d"   2>nul
if exist "objects\*.crf" del /q "objects\*.crf" 2>nul
if exist "objects\*.axf" del /q "objects\*.axf" 2>nul
if exist "objects\*.hex" del /q "objects\*.hex" 2>nul
if exist "objects\*.htm" del /q "objects\*.htm" 2>nul
if exist "objects\*.lnp" del /q "objects\*.lnp" 2>nul
if exist "objects\*.sct" del /q "objects\*.sct" 2>nul
if exist "objects\*.map" del /q "objects\*.map" 2>nul
if exist "objects\*.dep" del /q "objects\*.dep" 2>nul
if exist "listings\*"    del /q "listings\*"    2>nul

echo [OK] Clean done.
popd
endlocal
