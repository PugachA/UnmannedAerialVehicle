@echo off 
SET project_name=Getting_started
SET project_dir=E:\Projects\UnmannedAerialVehicle\%project_name%
::SET board=DISCO_L476VG
::SET toolchain=GCC_ARM

IF "%1" == "debug" (SET buildprofile=debug)
IF "%1" == "release" (SET buildprofile=release)
cd ../
mbed compile ^
--source %project_dir% ^
--source BUILDS\mbed-os-build ^
--source libraries ^
--profile mbed-os\tools\profiles\%buildprofile%.json ^
--build %project_dir%\..\BUILDS\BUILD_%project_name%\%buildprofile%