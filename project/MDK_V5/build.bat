@echo off
setlocal enableextensions
REM ======================================================================
REM  build.bat - Compile MPS_MotorDriver Keil project (no flash)
REM  Usage: build.bat        (incremental build)
REM         build.bat clean  (rebuild from scratch)
REM  Output: objects\MPS_MotorDriver.axf + .hex
REM ======================================================================

set "PROJ_DIR=%~dp0"
pushd "%PROJ_DIR%"

set "UV4=C:\Keil_v5\UV4\UV4.exe"
set "PROJ=MPS_MotorDriver.uvprojx"
set "LOG=keil_build.log"

if not exist "%UV4%" (
    echo [ERROR] UV4.exe not found at %UV4%
    echo         Please install Keil MDK or adjust UV4 path in this script.
    popd
    exit /b 3
)

if /i "%~1"=="clean" (
    echo [INFO] Rebuild -- clean + build ...
    "%UV4%" -j0 -r "%PROJ%" -o "%LOG%"
) else (
    echo [INFO] Incremental build ...
    "%UV4%" -j0 -b "%PROJ%" -o "%LOG%"
)
set "RC=%ERRORLEVEL%"

echo.
echo === Build log -- tail ===
powershell -NoProfile -Command "Get-Content '%LOG%' -Tail 20"

if not exist "objects\MPS_MotorDriver.axf" (
    echo.
    echo [FAIL] Build failed: objects\MPS_MotorDriver.axf not generated.
    echo        See full log: %PROJ_DIR%%LOG%
    popd
    exit /b 4
)

REM RC=0 clean, RC=1 warnings -- both OK. RC>=2 error.
if "%RC%"=="0" goto :ok
if "%RC%"=="1" goto :ok_warn
echo.
echo [FAIL] Build failed. RC=%RC%. See: %PROJ_DIR%%LOG%
popd
exit /b %RC%

:ok
echo.
echo [OK] Build succeeded, 0 warnings.
goto :size

:ok_warn
echo.
echo [OK] Build succeeded with warnings. RC=%RC%.
echo      Common cause: RT-Thread cpuport.c context unused warning.

:size
echo.
echo === Program Size ===
powershell -NoProfile -Command "Select-String -Path '%LOG%' -Pattern 'Program Size'"
popd
endlocal
