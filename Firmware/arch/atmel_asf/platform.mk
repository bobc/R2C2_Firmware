#
# For any Atmel SAMxxx target supported by ASF
#

ARCHLIBDIR = arch/atmel_asf/asf

ARCHLIBINC = \
    $(ARCHLIBDIR)/common/services/clock                              \
    $(ARCHLIBDIR)/common/utils                                       \
	$(ARCHLIBDIR)/sam/drivers/adc \
	$(ARCHLIBDIR)/sam/drivers/pio \
    $(ARCHLIBDIR)/sam/drivers/pmc \
    $(ARCHLIBDIR)/sam/drivers/tc \
    $(ARCHLIBDIR)/sam/drivers/uart \
    $(ARCHLIBDIR)/sam/drivers/usart \
    $(ARCHLIBDIR)/sam                                                \
    $(ARCHLIBDIR)/sam/utils                                          \
    $(ARCHLIBDIR)/sam/utils/cmsis/sam3x/include                      \
    $(ARCHLIBDIR)/sam/utils/cmsis/sam3x/source/templates             \
    $(ARCHLIBDIR)/sam/utils/header_files                             \
    $(ARCHLIBDIR)/sam/utils/preprocessor                             \
	$(ARCHLIBDIR)/thirdparty/CMSIS/Include \
	arch/atmel_asf/hal


ARCHLIBSRC = \
    $(ARCHLIBDIR)/common/services/clock/sam3x/sysclk.c               \
	$(ARCHLIBDIR)/sam/drivers/adc/adc.c                              \
	$(ARCHLIBDIR)/sam/drivers/pio/pio.c                              \
    $(ARCHLIBDIR)/sam/drivers/pio/pio_handler.c                      \
    $(ARCHLIBDIR)/sam/drivers/pmc/pmc.c                              \
    $(ARCHLIBDIR)/sam/drivers/tc/tc.c                                \
    $(ARCHLIBDIR)/sam/drivers/uart/uart.c                            \
    $(ARCHLIBDIR)/sam/drivers/usart/usart.c                          \
    $(ARCHLIBDIR)/sam/utils/cmsis/sam3x/source/templates/exceptions.c \
    $(ARCHLIBDIR)/sam/utils/cmsis/sam3x/source/templates/gcc/startup_sam3x.c \
    $(ARCHLIBDIR)/sam/utils/cmsis/sam3x/source/templates/system_sam3x.c \
	$(wildcard arch/atmel_asf/hal/*.c)

#ARCHLIB_ASRC = startup.S

ifeq ($(USE_USB),YES)
#ARCHLIBINC += $(ARCHLIBDIR)/LPCUSB/inc
#ARCHLIBSRC += $(wildcard $(ARCHLIBDIR)/LPCUSB/src/*.c)
endif
