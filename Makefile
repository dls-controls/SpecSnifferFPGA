include CONFIG

#
# Xilinx ISE Environment
#
PLATFORM = $(shell uname -m)
# Path to Xilinx ISE tools
ISE_PATH=/dls_sw/apps/FPGA/Xilinx/14.7/ISE_DS

ifeq ($(PLATFORM),x86_64)
	ISE=source $(ISE_PATH)/settings64.sh > /dev/null
endif

ifeq ($(PLATFORM),i686)
	ISE=source $(ISE_PATH)/settings32.sh > /dev/null
endif

#
# Print the names of unlocked (unconstrainted) IOs
#
export XIL_PAR_DESIGN_CHECK_VERBOSE=1
#export XIL_PAR_ALLOW_LVDS_LOC_OVERRIDE

#
# Hardware Platform Settings
#
CONFIG_FILE = rtl/spec_pkg/vhdl/spec_defines.vhd

main: $(CONFIG_FILE)
	$(ISE) && make -C syn/run -f ../Makefile bits

program: $(CONFIG_FILE)
	$(ISE) && make -C syn/run -f ../Makefile program

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

wbone:
	toolkit/wbgen2 -l vhdl -V rtl/spec_wishbone/vhdl/dma_controller_wb_slave.vhd -C dma_controller_wb_slave.h rtl/spec_wishbone/wb_gen/dma_controller_wb_slave.wb
	toolkit/wbgen2 -l vhdl -V rtl/spec_wishbone/vhdl/syscsr_wb_slave.vhd -C syscsr_wb_slave.h rtl/spec_wishbone/wb_gen/syscsr_wb_slave.wb

clean:
	rm -f $(CONFIG_FILE)
	$(ISE) && make -C syn/run -f ../Makefile clean

