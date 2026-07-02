@echo off
cd /d E:\WorkSpaces\2_MotorDriver\MPS_MotorDriver\project\MDK_V5
C:\Keil_v5\UV4\UV4.exe -j0 -f MPS_MotorDriver.uvprojx -o keil_flash.log
echo FLASH_EXIT=%errorlevel%
type keil_flash.log
