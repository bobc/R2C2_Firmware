ARCHLIBDIR = arch/lpc17xx

ARCHLIBINC = \
	$(ARCHLIBDIR)/CMSISv1p30/inc \
	$(ARCHLIBDIR)/drivers/include \
	$(ARCHLIBDIR)/LPCUSB/inc

ARCHLIBSRC = \
	$(ARCHLIBDIR)/CMSISv1p30/src/system_LPC17xx.c \
	$(wildcard $(ARCHLIBDIR)/Drivers/source/lpc17xx_*.c) \
	$(wildcard $(ARCHLIBDIR)/hal/*.c) \
	$(wildcard $(ARCHLIBDIR)/LPCUSB/src/*.c)
