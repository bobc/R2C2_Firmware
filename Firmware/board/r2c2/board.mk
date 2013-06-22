# Board settings
# - MCU 		used for compiler-option (-mcpu)
# - ARCH_FAMILY	used to select HAL	
# - CHIP		used for linker script name and passed as define	

MCU      	= cortex-m3
ARCH_FAMILY = lpc17xx
CHIP     	= LPC1758

BOARDINC = board/r2c2

# options

USE_FILESYSTEM = YES
# USE_ETHERNET = YES
USE_USB = YES
