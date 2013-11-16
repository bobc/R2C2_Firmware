exec EnableFlashDL
exec Device=SAM3X8E
speed 400
h
loadbin flash_due\firmware.bin 0x80000
r
g
exit
