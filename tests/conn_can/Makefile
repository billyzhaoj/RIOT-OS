export APPLICATION = can
include ../Makefile.tests_common

BOARD_INSUFFICIENT_MEMORY := chronos msb-430 msb-430h nucleo32-f031 nucleo32-f042 \
                             nucleo32-f303 nucleo32-l031 nucleo-f030 nucleo-f070 \
                             nucleo-f072 nucleo-f302 nucleo-f303 nucleo-f334 \
                             nucleo-l053 stm32f0discovery telosb weio wsn430-v1_3b \
                             wsn430-v1_4 z1


CFLAGS += -DDEVELHELP
CFLAGS += -DLOG_LEVEL=LOG_ALL

USEMODULE += shell
USEMODULE += shell_commands
USEMODULE += ps

USEMODULE += conn_can
USEMODULE += can_isotp
USEMODULE += conn_can_isotp_multi
USEMODULE += can_pm
USEMODULE += can_trx

include $(RIOTBASE)/Makefile.include
