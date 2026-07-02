@echo off
cd /d E:\WorkSpaces\2_MotorDriver\MPS_MotorDriver\project\MDK_V5
C:\Keil_v5\UV4\UV4.exe -j0 -r MPS_MotorDriver.uvprojx -o keil_build2.log
echo BUILD_EXIT=%errorlevel%
type keil_build2.log
