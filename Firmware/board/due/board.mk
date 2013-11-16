# Board settings
# - MCU 		used for compiler-option (-mcpu)
# - ARCH_FAMILY	used to select HAL	
# - CHIP		used for linker script name and passed as define	

MCU      	= cortex-m3
ARCH_FAMILY = atmel_asf
CHIP     	= sam3x8e

BOARD_FLAGS = -D __SAM3X8E__ -D NDEBUG
BOARDINC = board/due

# options

# USE_FILESYSTEM = YES
# USE_ETHERNET = YES
# USE_USB = YES
