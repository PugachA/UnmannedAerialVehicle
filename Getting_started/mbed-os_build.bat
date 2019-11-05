@echo off 
SET mbed-os_dir=C:\projects\STM32\MBED\mbed-os
SET project_name=Getting_started
SET project_dir=C:\projects\STM32\UnmannedAerialVehicle\Getting_started
SET board=DISCO_L476VG
SET toolchain=GCC_ARM

mbed config -G MBED_OS_DIR %mbed-os_dir%
IF "%1" == "debug" (SET buildprofile=debug)
IF "%1" == "release" (SET buildprofile=release)

mbed compile -t %toolchain% ^
-m %board% --source . ^
--source %mbed-os_dir% ^
--profile %mbed-os_dir%\tools\profiles\%buildprofile%.json ^
--build %project_dir%\..\BUILDS\BUILD_%project_name%\%buildprofile%