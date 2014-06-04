include CONFIG

#
# Xilinx ISE Environment
#
PLATFORM = $(shell uname -m)

ifeq ($(PLATFORM),x86_64)
	ISE=source /dls_sw/apps/FPGA/Xilinx/14.7/ISE_DS/settings64.sh > /dev/null
endif

ifeq ($(PLATFORM),i686)
	ISE=source /dls_sw/apps/FPGA/Xilinx/14.7/ISE_DS/settings32.sh > /dev/null
endif

#
# Print the names of unlocked (unconstrainted) IOs
#
export XIL_PAR_DESIGN_CHECK_VERBOSE=1
#export XIL_PAR_ALLOW_LVDS_LOC_OVERRIDE

#
# Name of the PC to be used for programming the FPGA
#
JTAG_PC=pc0044

#
# Hardware Platform Settings
#
CONFIG_FILE = rtl/spec_pkg/vhdl/spec_defines.vhd

main: $(CONFIG_FILE)
	$(ISE) && make -C syn/run -f ../Makefile bits

download:
	$(ISE) && make -C syn/run -f ../Makefile JTAG_PC=$(JTAG_PC) download

program:
	$(ISE) && make -C syn/run -f ../Makefile JTAG_PC=$(JTAG_PC) program

hwclean:
	$(ISE) && make -C syn/run -f ../Makefile hwclean
	$(ISE) && make -C syn/run -f ../Makefile clean

$(CONFIG_FILE): CONFIG
	rm -f $@
	echo 'library ieee;' >>$@
	echo 'use ieee.std_logic_1164.all;' >>$@
	echo 'package spec_defines is' >>$@
	echo -n 'constant FPGA_VERSION: std_logic_vector(15 downto 0)' \ >>$@
	echo ' := X"$(VERSION)";' >>$@
	echo 'end spec_defines;' >>$@

clean:
	rm -f $(CONFIG_FILE)
	$(ISE) && make -C syn/run -f ../Makefile clean


