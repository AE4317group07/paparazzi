#
# $Id: booz2_autopilot.makefile 4827 2010-04-21 08:02:18Z poine $
#
# Copyright (C) 2008 Antoine Drouin
#
# This file is part of paparazzi.
#tin
# paparazzi is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2, or (at your option)
# any later version.
#
# paparazzi is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with paparazzi; see the file COPYING.  If not, write to
# the Free Software Foundation, 59 Temple Place - Suite 330,
# Boston, MA 02111-1307, USA.
#
#

######################################################################
##
## COMMON FIXEDWING ALL TARGETS (SIM + AP + FBW ...)
##

#
# Board config + Include paths
#

$(TARGET).CFLAGS 	+= -DBOARD_CONFIG=$(BOARD_CFG)
$(TARGET).CFLAGS 	+= $(FIXEDWING_INC)

#
# Common Options
#

ifeq ($(OPTIONS), minimal)
else
  $(TARGET).CFLAGS 	+= -DWIND_INFO
endif

$(TARGET).CFLAGS 	+= -DTRAFFIC_INFO

#
# LEDs
#

$(TARGET).CFLAGS 	+= -DLED
ifneq ($(ARCHI), arm7)
  ifneq ($(ARCHI), jsbsim)
    $(TARGET).srcs 	+= $(SRC_ARCH)/led_hw.c
  endif
endif

#
# Sys-time
#

$(TARGET).srcs 		+= sys_time.c

#
# InterMCU & Commands
#

$(TARGET).CFLAGS 	+= -DINTER_MCU
$(TARGET).srcs 		+= $(SRC_FIXEDWING)/inter_mcu.c

######################################################################
##
## COMMON FOR ALL NON-SIMULATION TARGETS
##

#
# Interrupt Vectors
#

ifeq ($(ARCHI), arm7)
  ns_srcs 		+= $(SRC_ARCH)/armVIC.c
else ifeq ($(ARCHI), stm32)
  ns_srcs 		+= $(SRC_ARCH)/stm32_exceptions.c
  ns_srcs 		+= $(SRC_ARCH)/stm32_vector_table.c
  ns_CFLAGS 		+= -DPERIPHERALS_AUTO_INIT
endif

ifeq ($(ARCHI), stm32)
  ns_srcs 		+= lisa/plug_sys.c
endif


#
# Main
#

ns_srcs	   	+= $(SRC_FIXEDWING)/main.c

#
# LEDs
#

ns_CFLAGS 		+= -DUSE_LED
ifeq ($(ARCHI), stm32)
  ns_CFLAGS 	+= -DSYS_TIME_LED=1
else
  ns_CFLAGS 	+= -DTIME_LED=1
endif

#
# Sys-time
#

ns_CFLAGS 		+= -DUSE_SYS_TIME
ns_CFLAGS 		+= -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC((1./60.))'
ns_srcs 		+= $(SRC_ARCH)/sys_time_hw.c


#
# UARTS
#

ns_srcs 		+= $(SRC_ARCH)/uart_hw.c

#
# ANALOG
#

ifeq ($(ARCHI), arm7)
  ns_CFLAGS 		+= -DADC
  ns_srcs 		+= $(SRC_ARCH)/adc_hw.c
else ifeq ($(ARCHI), stm32)
  ns_srcs 		+= lisa/lisa_analog_plug.c
endif


######################################################################
##
## FLY BY WIRE THREAD SPECIFIC
##

fbw_CFLAGS		+= -DFBW
fbw_srcs 		+= $(SRC_FIXEDWING)/main_fbw.c
fbw_srcs 		+= $(SRC_FIXEDWING)/commands.c

######################################################################
##
## AUTOPILOT THREAD SPECIFIC
##

ap_CFLAGS 		+= -DAP
ap_srcs 		+= $(SRC_FIXEDWING)/main_ap.c
ap_srcs 		+= $(SRC_FIXEDWING)/estimator.c


######################################################################
##
## SIMULATOR THREAD SPECIFIC
##

sim.CFLAGS 		+= $(fbw_CFLAGS) $(ap_CFLAGS)
sim.srcs 		+= $(fbw_srcs) $(ap_srcs)

sim.CFLAGS 		+= -DSITL
sim.srcs 		+= $(SRC_ARCH)/sim_ap.c

sim.CFLAGS 		+= -DDOWNLINK -DDOWNLINK_TRANSPORT=IvyTransport
sim.srcs 		+= downlink.c datalink.c $(SRC_ARCH)/sim_gps.c $(SRC_ARCH)/ivy_transport.c $(SRC_ARCH)/sim_adc_generic.c

######################################################################
##
## JSBSIM THREAD SPECIFIC
##

JSBSIM_ROOT = /opt/jsbsim
JSBSIM_INC = $(JSBSIM_ROOT)/include/JSBSim
JSBSIM_LIB = $(JSBSIM_ROOT)/lib

jsbsim.CFLAGS 		+= $(fbw_CFLAGS) $(ap_CFLAGS)
jsbsim.srcs 		+= $(fbw_srcs) $(ap_srcs)

jsbsim.CFLAGS 		+= -DSITL
jsbsim.srcs 		+= $(SIMDIR)/sim_ac_jsbsim.c $(SIMDIR)/sim_ac_fw.c

# external libraries
jsbsim.CFLAGS 		+= -I$(SIMDIR) -I/usr/include -I$(JSBSIM_INC) `pkg-config glib-2.0 --cflags`
jsbsim.LDFLAGS		+= `pkg-config glib-2.0 --libs` -lm -lpcre -lglibivy -L/usr/lib -L$(JSBSIM_LIB) -lJSBSim

jsbsim.CFLAGS 		+= -DDOWNLINK -DDOWNLINK_TRANSPORT=IvyTransport
jsbsim.srcs 		+= downlink.c datalink.c $(SRC_ARCH)/jsbsim_hw.c $(SRC_ARCH)/jsbsim_gps.c $(SRC_ARCH)/ivy_transport.c $(SRC_ARCH)/jsbsim_transport.c

######################################################################
##
## Final Target Allocations
##

#
# SINGLE MCU / DUAL MCU
#

ifeq ($(BOARD),classix)
  fbw.CFLAGS 		+= -DMCU_SPI_LINK -DUSE_SPI -DSPI_SLAVE
  fbw.srcs 		+= $(SRC_FIXEDWING)/link_mcu.c $(SRC_FIXEDWING)/spi.c $(SRC_ARCH)/spi_hw.c
  ap.CFLAGS 		+= -DMCU_SPI_LINK -DUSE_SPI -DSPI_MASTER -DUSE_SPI_SLAVE0
  ap.srcs 		+= $(SRC_FIXEDWING)/link_mcu.c $(SRC_FIXEDWING)/spi.c $(SRC_ARCH)/spi_hw.c
else
  # Single MCU's run both
  ap.CFLAGS 		+= $(fbw_CFLAGS)
  ap.srcs 		+= $(fbw_srcs)
endif

#
# No-Sim parameters
#

fbw.CFLAGS 		+= $(fbw_CFLAGS) $(ns_CFLAGS)
fbw.srcs 		+= $(fbw_srcs) $(ns_srcs)

ap.CFLAGS 		+= $(ap_CFLAGS) $(ns_CFLAGS)
ap.srcs 		+= $(ap_srcs) $(ns_srcs)
