BOARD ?= native

INCLUDES += -I$(APPDIR)

include ../Makefile.drivers_common

USEMODULE += motor_driver
USEMODULE += shell_cmds_default
USEMODULE += xtimer

CFLAGS += -DLOG_LEVEL=LOG_DEBUG
CFLAGS += -DDEBUG_ASSERT_VERBOSE

include $(RIOTBASE)/Makefile.include
