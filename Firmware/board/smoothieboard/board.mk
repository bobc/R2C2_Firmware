# Board settings
# - MCU 		used for compiler-option (-mcpu)
# - ARCH_FAMILY	used to select HAL	
# - CHIP		used for linker script name and passed as define	

MCU      	= cortex-m3
ARCH_FAMILY = lpc17xx
CHIP     	= LPC1769

BOARDINC = board/smoothieboard

# options

USE_FILESYSTEM = YES
# USE_ETHERNET = YES
USE_USB = YES
