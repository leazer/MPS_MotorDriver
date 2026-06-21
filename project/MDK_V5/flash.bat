@echo off
setlocal enableextensions
REM ======================================================================
REM  flash.bat - Build if needed, then flash MPS_MotorDriver via JLink
REM  Usage: flash.bat          (build then flash)
REM         flash.bat only     (skip build, flash existing axf)
REM         flash.bat rebuild  (clean build then flash)
REM  Flash uses Keil project's configured debugger (JLink).
REM ======================================================================

set "PROJ_DIR=%~dp0"
pushd "%PROJ_DIR%"

set "UV4=C:\Keil_v5\UV4\UV4.exe"
set "PROJ=MPS_MotorDriver.uvprojx"
set "BUILD_LOG=keil_build.log"
set "FLASH_LOG=keil_flash.log"

if not exist "%UV4%" (
    echo [ERROR] UV4.exe not found at %UV4%
    popd
    exit /b 3
)

REM --- Step 1: Build (unless "only") ---
if /i "%~1"=="only" (
    echo [INFO] Skipping build -- flash only mode.
    if not exist "objects\MPS_MotorDriver.axf" (
        echo [ERROR] No existing axf. Run: flash.bat rebuild
        popd
        exit /b 4
    )
    goto :flash
)

if /i "%~1"=="rebuild" (
    echo [INFO] Rebuild before flash ...
    "%UV4%" -j0 -r "%PROJ%" -o "%BUILD_LOG%"
) else (
    echo [INFO] Incremental build before flash ...
    "%UV4%" -j0 -b "%PROJ%" -o "%BUILD_LOG%"
)
set "RC=%ERRORLEVEL%"

echo.
echo === Build log -- tail ===
powershell -NoProfile -Command "Get-Content '%BUILD_LOG%' -Tail 15"

if not exist "objects\MPS_MotorDriver.axf" (
    echo.
    echo [FAIL] Build failed, axf not generated. Aborting flash.
    popd
    exit /b 4
)

REM RC=0 or 1 OK to flash. RC>=2 error.
if "%RC%"=="0" goto :build_ok
if "%RC%"=="1" goto :build_ok
echo.
echo [FAIL] Build failed. RC=%RC%. Aborting flash.
popd
exit /b %RC%

:build_ok
echo.
echo [OK] Build succeeded. RC=%RC%. Proceeding to flash...

:flash
echo.
echo === Flashing via JLink ===
"%UV4%" -j0 -f "%PROJ%" -o "%FLASH_LOG%"
set "FRC=%ERRORLEVEL%"

echo.
echo === Flash log -- tail ===
powershell -NoProfile -Command "Get-Content '%FLASH_LOG%' -Tail 15"

if "%FRC%"=="0" (
    echo.
    echo [OK] Flash succeeded.
) else (
    echo.
    echo [FAIL] Flash failed. RC=%FRC%. See: %PROJ_DIR%%FLASH_LOG%
    popd
    exit /b %FRC%
)

popd
endlocal
