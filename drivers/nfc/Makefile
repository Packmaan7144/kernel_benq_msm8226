#
# Makefile for nfc devices
#
ifneq ($(CONFIG_HK_VARIANT), yes)
ifneq ($(CONFIG_MY1_VARIANT), yes)
ifneq ($(CONFIG_MY2_VARIANT), yes)
obj-$(CONFIG_PN544_NFC)		+= pn544.o
# Jen Chang add for factory test scripts
obj-$(CONFIG_PN544_TEST)	+= pn544_test.o
# Jen Chang add 20130808
endif
endif
endif

obj-$(CONFIG_NFC_PN533)		+= pn533.o
obj-$(CONFIG_NFC_WILINK)	+= nfcwilink.o
obj-$(CONFIG_NFC_QNCI)		+= nfc-nci.o

ccflags-$(CONFIG_NFC_DEBUG) := -DDEBUG
