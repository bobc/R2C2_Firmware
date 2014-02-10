@echo off
rem
rem Download firmware to Digistump Digi-X
rem
rem NB this requires bossac utility (available in Arduino IDE)
rem
rem change port as required 

set zzport=%1
if /%zzport%/ == // set zzport=COM18

set bossa_path=C:\Programs\arduino-1.5.2\hardware\tools\

rem mode %zzport%:1200
rem sleep 1

echo Press ERASE then RESET on DigiX
pause

rem
rem %bossa_path%bossac.exe --port=%zzport% -U false -e -w -v -b firmware.bin -R
%bossa_path%bossac.exe --port=%zzport% -e -w -v -b firmware.bin -R

set zzport=
