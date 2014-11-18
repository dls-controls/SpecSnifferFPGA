if [file exists work] {
    vdel -all -lib work
}

vlib work
vmap work

vcom -quiet -f fofb_cc_top.lst

vcom -quiet "../../../rtl/spec_pkg/vhdl/spec_defines.vhd"

vcom -quiet "../../../rtl/ip_cores/gn4124-core/hdl/spec/sim/testbench/util.vhd"
vcom -quiet "../../../rtl/ip_cores/gn4124-core/hdl/spec/sim/testbench/textutil.vhd"
vcom -quiet -87 "../../../rtl/ip_cores/gn4124-core/hdl/spec/sim/testbench/mem_model.vhd"
vcom -quiet "../../../rtl/ip_cores/gn4124-core/hdl/spec/sim/testbench/cmd_router1.vhd"
vcom -quiet "../../../rtl/ip_cores/gn4124-core/hdl/spec/sim/testbench/gn412x_bfm.vhd"
vcom -quiet "../../../rtl/ip_cores/gn4124-core/hdl/spec/sim/testbench/cmd_router.vhd"

vcom -work work -quiet ../../../rtl/ip_cores/general-cores/modules/genrams/genram_pkg.vhd
vcom -work work -quiet ../../../rtl/ip_cores/general-cores/modules/genrams/memory_loader_pkg.vhd
vcom -work work -quiet ../../../rtl/ip_cores/general-cores/modules/genrams/inferred_async_fifo.vhd
vcom -work work -quiet ../../../rtl/ip_cores/general-cores/modules/genrams/generic/generic_async_fifo.vhd
vcom -work work -quiet ../../../rtl/ip_cores/general-cores/modules/genrams/xilinx/generic_dpram.vhd
vcom -work work -quiet ../../../rtl/ip_cores/general-cores/modules/genrams/xilinx/generic_spram.vhd
vcom -work work -quiet ../../../rtl/ip_cores/general-cores/modules/genrams/xilinx/generic_dpram_dualclock.vhd

vcom -quiet ../../../rtl/ip_cores/general-cores/modules/wishbone/wbgen2/wbgen2_pkg.vhd
vcom -quiet ../../../rtl/ip_cores/general-cores/modules/wishbone/wbgen2/wbgen2_dpssram.vhd

vcom -quiet "../../../rtl/spec_wishbone/vhdl/syscsr_wb_slave.vhd"
vcom -quiet "../../../rtl/spec_wishbone/vhdl/dma_controller_wb_slave.vhd"
vcom -quiet "../../../rtl/spec_wishbone/vhdl/dma_wb_slave_wrapper.vhd"

vcom -quiet ../../../rtl/ip_cores/gn4124-core/hdl/gn4124core/rtl/spartan6/gn4124_core_pkg.vhd
vcom -quiet ../../../rtl/ip_cores/gn4124-core/hdl/gn4124core/rtl/dma_controller.vhd
vcom -quiet ../../../rtl/ip_cores/gn4124-core/hdl/gn4124core/rtl/l2p_arbiter.vhd
vcom -quiet ../../../rtl/ip_cores/gn4124-core/hdl/gn4124core/rtl/l2p_dma_master.vhd
vcom -quiet ../../../rtl/ip_cores/gn4124-core/hdl/gn4124core/rtl/p2l_decode32.vhd
vcom -quiet ../../../rtl/ip_cores/gn4124-core/hdl/gn4124core/rtl/p2l_dma_master.vhd
vcom -quiet ../../../rtl/ip_cores/gn4124-core/hdl/gn4124core/rtl/wbmaster32.vhd
vcom -quiet ../../../rtl/ip_cores/gn4124-core/hdl/gn4124core/rtl/spartan6/l2p_ser.vhd
vcom -quiet ../../../rtl/ip_cores/gn4124-core/hdl/gn4124core/rtl/spartan6/p2l_des.vhd
vcom -quiet ../../../rtl/ip_cores/gn4124-core/hdl/gn4124core/rtl/spartan6/pulse_sync_rtl.vhd
vcom -quiet ../../../rtl/ip_cores/gn4124-core/hdl/gn4124core/rtl/spartan6/serdes_1_to_n_clk_pll_s2_diff.vhd
vcom -quiet ../../../rtl/ip_cores/gn4124-core/hdl/gn4124core/rtl/spartan6/serdes_1_to_n_data_s2_se.vhd
vcom -quiet ../../../rtl/ip_cores/gn4124-core/hdl/gn4124core/rtl/spartan6/serdes_n_to_1_s2_diff.vhd
vcom -quiet ../../../rtl/ip_cores/gn4124-core/hdl/gn4124core/rtl/spartan6/serdes_n_to_1_s2_se.vhd


vcom -quiet "../../../rtl/spec_si57x/vhdl/pulse2pulse.vhd"
vcom -quiet "../../../rtl/spec_si57x/vhdl/i2c_master_stellar_cmd.vhd"
vcom -quiet "../../../rtl/spec_si57x/vhdl/i2c_master_bit_ctrl.vhd"
vcom -quiet "../../../rtl/spec_si57x/vhdl/i2c_master_byte_ctrl.vhd"
vcom -quiet "../../../rtl/spec_si57x/vhdl/i2c_master.vhd"
vcom -quiet "../../../rtl/spec_si57x/vhdl/i2c_master_top.vhd"
vcom -quiet "../../../rtl/spec_si57x/vhdl/spec_si57x.vhd"
vcom -quiet "../../../rtl/spec_clk_if/vhdl/spec_clk_if.vhd"
vcom -quiet "../../../rtl/spec_wishbone/vhdl/wb_addr_decoder.vhd"
vcom -quiet "../../../rtl/spec_top/vhdl/fofb_cc_top_wrapper.vhd"

vlog -quiet "../../../rtl/spec_top/verilog/BMD_64_RWDMA_FSM.v"
vcom -quiet "../../../rtl/spec_top/vhdl/dma_controller.vhd"
vcom -quiet "../../../rtl/spec_top/vhdl/gn4124_core.vhd"

vcom -quiet "../../../rtl/spec_top/vhdl/freq_cnt16.vhd"
vcom -quiet "../../../rtl/spec_top/vhdl/spec_top.vhd"


#
# COMMUNICATION CONTROLLER TESTER
#
vcom -quiet ../../../rtl/spec_commsctrl/sim/fofb_cc_top/bench/test_interface.vhd
vcom -quiet ../../../rtl/spec_commsctrl/sim/fofb_cc_top/bench/fofb_cc_usrapp_tx.vhd
vcom -quiet ../../../rtl/spec_commsctrl/sim/fofb_cc_top/bench/fofb_cc_usrapp_rx.vhd
vcom -quiet ../../../rtl/spec_commsctrl/sim/fofb_cc_top/bench/fofb_cc_usrapp_checker.vhd
vcom -quiet ../../../rtl/spec_commsctrl/sim/fofb_cc_top/bench/sim_reset_mgt_model.vhd

vcom -quiet ../bench/fofb_cc_top_tester.vhd


#
# TOP LEVEL TESTBENCH
#
vcom -quiet "../bench/spec_top_tb.vhd"

vsim -novopt -t 1ps spec_top_tb

view wave
add wave -group "Top Testbench" \
    "sim:/spec_top_tb/*"

add wave -group "SPEC TOP"      \
    "sim:/spec_top_tb/spec_top_inst/*"

add wave -group "GN4124 Core"   \
sim:/spec_top_tb/spec_top_inst/gn4124_core_inst/*

add wave -group "DMA State Machine" \
sim:/spec_top_tb/spec_top_inst/gn4124_core_inst/cmp_dma_controller/dma_fsm_inst/*

add wave -group "DMA Controller" \
    "sim:/spec_top_tb/spec_top_inst/gn4124_core_inst/cmp_dma_controller/*"

add wave -group "L2P Master"     \
sim:/spec_top_tb/spec_top_inst/gn4124_core_inst/cmp_l2p_dma_master/*

run 500 us
