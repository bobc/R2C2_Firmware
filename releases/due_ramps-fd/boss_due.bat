@echo off
rem
rem Download firmware to Due
rem
rem NB this requires bossac utility (available in Arduino IDE)
rem
rem change port as required 

mode COM18:1200
sleep 1
bossac.exe --port=COM18 -U false -e -w -v -b firmware.bin -R