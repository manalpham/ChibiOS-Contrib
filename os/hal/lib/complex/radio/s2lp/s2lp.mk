RADIO_DIR 	:=  $(CHIBIOS_CONTRIB)/os/hal/lib/peripherals/radio
S2LP_DIR	:=  $(CHIBIOS_CONTRIB)/os/hal/lib/complex/radio/s2lp
ST_S2LP_DIR	:=  $(S2LP_DIR)/st_s2lp_lib

# List of all the S2LP device files.
S2LPSRC 	:= 	$(ST_S2LP_DIR)/Src/S2LP_Commands.c 						\
				$(ST_S2LP_DIR)/Src/S2LP_Csma.c 							\
				$(ST_S2LP_DIR)/Src/S2LP_Fifo.c 							\
				$(ST_S2LP_DIR)/Src/S2LP_General.c 						\
				$(ST_S2LP_DIR)/Src/S2LP_Gpio.c 							\
				$(ST_S2LP_DIR)/Src/S2LP_PacketHandler.c 				\
				$(ST_S2LP_DIR)/Src/S2LP_PktBasic.c 						\
				$(ST_S2LP_DIR)/Src/S2LP_PktStack.c 						\
				$(ST_S2LP_DIR)/Src/S2LP_PktWMbus.c 						\
				$(ST_S2LP_DIR)/Src/S2LP_Qi.c 							\
				$(ST_S2LP_DIR)/Src/S2LP_Radio.c 						\
				$(ST_S2LP_DIR)/Src/S2LP_Timer_ex.c 						\
				$(ST_S2LP_DIR)/Src/S2LP_Timer.c 						\
				$(ST_S2LP_DIR)/Src/S2LP_Types.c							\
			  	$(S2LP_DIR)/s2lp.c

# Required include directories
S2LPINC 	:=  $(RADIO_DIR)											\
           	    $(S2LP_DIR)												\
           	    $(ST_S2LP_DIR)/Inc

# Shared variables
ALLCSRC 	+=  $(S2LPSRC)
ALLINC  	+=  $(S2LPINC)