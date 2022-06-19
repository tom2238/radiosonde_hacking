##
## This file is part of the libopencm3 project.
##
## Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
##
## This library is free software: you can redistribute it and/or modify
## it under the terms of the GNU Lesser General Public License as published by
## the Free Software Foundation, either version 3 of the License, or
## (at your option) any later version.
##
## This library is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU Lesser General Public License for more details.
##
## You should have received a copy of the GNU Lesser General Public License
## along with this library.  If not, see <http://www.gnu.org/licenses/>.
##

# Output program name
BINARY = main
# Include library path
LIBRARY_DIR = ../libraries
DEFS += -I$(LIBRARY_DIR)
# Add library objects
OBJS += $(LIBRARY_DIR)/utils.o
OBJS += $(LIBRARY_DIR)/frame.o
OBJS += $(LIBRARY_DIR)/si4032.o
OBJS += $(LIBRARY_DIR)/ublox6.o
# Add project custom objects
OBJS += init.o ptu_measure.o
# Change maximum frame length in bytes
DEFS += -DFRAME_LEN_MAX=70
# libOpenCM dir
OPENCM3_DIR = ../../libopencm3
LDSCRIPT = ../../linker/stm32f100x8.ld
# Include common rules
include ../../common.f1.mk

