include ../CONFIG

FAMILY = spartan6

all:
	@echo "----------------------------------------"
	@echo "Makefile to build UDP Sniffer firmware :"
	@echo "----------------------------------------"
	@echo "Run make with any of the following targets"
	@echo " "
	@echo " netlist  : Generates the netlist for the project for given target board "
	@echo " bits     : Run implementation tools to generate the bitstream
	@echo " download : Downloads the bitstream onto the board"
	@echo " program  : Generate mcs file from bitstream and program the PROM"

netlist: $(POSTSYN_NETLIST)

bits : $(BIT_FILE)

program : $(PROM_FILE)

$(NETLIST_DIR):
	mkdir $(NETLIST_DIR)

$(COREGEN_NGC_FILES):
	coregen -b $(CC_IP_DIR)/fofb_cc_rx_fifo/coregen/$(FAMILY)/fofb_cc_rx_fifo.xco
	coregen -b $(CC_IP_DIR)/fofb_cc_tx_fifo/coregen/$(FAMILY)/fofb_cc_tx_fifo.xco


$(LIST_FILE):
	cat ../xilinx/fofb_cc_top.lst >> $(LIST_FILE)
	cat ../xilinx/gn4124core.lst >> $(LIST_FILE)
	cat ../xilinx/spec_top.lst >> $(LIST_FILE)

$(SCR_FILE):
	cat ../xilinx/spec_top.scr >> $(SCR_FILE)

$(POSTSYN_NETLIST): $(NETLIST_DIR) $(COREGEN_NGC_FILES) $(LIST_FILE) $(SCR_FILE)
	xst -ifn $(SCR_FILE)

$(UCF_FILE):
	cat ../constr/spec_top.ucf > $(UCF_FILE)

NGDBUILD_FLAGS = -sd $(NETLIST_DIR) -sd $(OHWR_IP_DIR)
MAP_FLAGS      = -ol high -xe n -w
PAR_FLAGS      = -ol high -xe n -w

$(BIT_FILE): $(POSTSYN_NETLIST) $(UCF_FILE)
	ngdbuild -uc $(UCF_FILE) $(POSTSYN_NETLIST) $(NGDBUILD_FLAGS)
	map $(MAP_FLAGS) $(NGD_FILE) -o $(MAPPED_NCD_FILE) $(PCF_FILE)
	par $(PAR_FLAGS) $(MAPPED_NCD_FILE) $(ROUTED_NCD_FILE) $(PCF_FILE)
	trce -e 3 -l 3 $(ROUTED_NCD_FILE) $(PCF_FILE)
	bitgen -f ../xilinx/bitgen_options.ut $(ROUTED_NCD_FILE)

smartxplorer : $(POSTSYN_NETLIST) $(UCF_FILE)
	cp $(OHWR_IP_DIR)/*.ngc ../implementation
	smartxplorer -p xc6slx45t-3fgg484 -uc spec_top.ucf -wd smartxplorer_results -sd ../implementation -m 12 -ra spec_top.ngc -l ../xilinx/hosts.lst -rcmd ssh

download: $(BIT_FILE)
	ssh iu42@$(JTAG_PC) 'cd $(CURDIR);$(XILINX)/bin/lin64/impact -batch ../xilinx/loadFPGA.cmd'

$(PROM_FILE): $(BIT_FILE)
	promgen -w -p mcs -o $(PROM_FILE) -x xcf128x -data_width 16 -u 0 $(BIT_FILE)
	ssh iu42@$(JTAG_PC) 'cd $(CURDIR);$(XILINX)/bin/lin64/impact -batch ../xilinx/loadPROM.cmd'

hwclean:
	rm -rf $(NETLIST_DIR)

clean:
	rm -rf ../run/*
