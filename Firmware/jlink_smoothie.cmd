exec EnableFlashDL
exec Device=LPC1768
speed 400
h
loadbin flash_smoothieboard\bootloader.bin 0x0000
r
g
exit
