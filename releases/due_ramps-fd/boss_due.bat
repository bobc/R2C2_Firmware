@echo off
rem
rem Download firmware to Due
rem
rem NB this requires bossac utility (available in Arduino IDE)
rem
rem change port as required 

set zzport=%1
if /%zzport%/ == // set zzport=COM18

set bossa_path=C:\Programs\arduino-1.5.2\hardware\tools\

mode %zzport%:1200
rem sleep 1
pause
%bossa_path%bossac.exe --port=%zzport% -U false -e -w -v -b firmware.bin -R

set zzport=
