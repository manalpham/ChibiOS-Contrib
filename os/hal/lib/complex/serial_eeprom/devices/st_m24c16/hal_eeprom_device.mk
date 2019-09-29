EEPROMDIR  := $(CHIBIOS_CONTRIB)/os/hal/lib/peripherals/eeprom
SEEPROMDIR := $(CHIBIOS_CONTRIB)/os/hal/lib/complex/serial_eeprom
M24C16DIR  := $(SEEPROMDIR)/devices/st_m24c16

# List of all the ST M24CXX device files.
SEEPROMSRC := $(EEPROMDIR)/hal_base_eeprom.c          \
              $(SEEPROMDIR)/hal_serial_eeprom.c       \
              $(M24C16DIR)/hal_eeprom_device.c

# Required include directories
SEEPROMINC := $(EEPROMDIR)  \
              $(SEEPROMDIR) \
              $(M24C16DIR)

# Shared variables
ALLCSRC += $(SEEPROMSRC)
ALLINC  += $(SEEPROMINC)
