PKG_NAME=tinydtls
PKG_URL=https://github.com/eclipse/tinydtls.git
PKG_VERSION=95d02ee28ea745541d6c55dba49f3b3ab8b71a43
PKG_LICENSE=EPL-1.0,EDL-1.0

include $(RIOTBASE)/pkg/pkg.mk

# tinyDTLS custom logging functions pass non-literals to printf(), so disable them
ifneq (,$(filter arch_avr8,$(FEATURES_USED)))
  CFLAGS += -DNDEBUG
endif

all:
	$(QQ)"$(MAKE)" -C $(PKG_SOURCE_DIR) -f $(PKG_SOURCE_DIR)/Makefile.riot
	$(QQ)"$(MAKE)" -C $(PKG_SOURCE_DIR)/aes -f $(PKG_SOURCE_DIR)/aes/Makefile.riot
	$(QQ)"$(MAKE)" -C $(PKG_SOURCE_DIR)/ecc -f $(PKG_SOURCE_DIR)/ecc/Makefile.riot

ifeq (llvm,$(TOOLCHAIN))
  CFLAGS += -Wno-format-nonliteral
endif

# TinyDTLS emits several messages during connection establishment at the info
# level; this is way more verbose than common in RIOT.
TINYDTLS_LOG_LEVEL ?= LOG_WARNING
CFLAGS += -DLOG_LEVEL=$(TINYDTLS_LOG_LEVEL)
