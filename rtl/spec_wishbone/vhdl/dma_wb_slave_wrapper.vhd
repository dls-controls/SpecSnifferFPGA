library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity dma_wb_slave_wrapper is
port (
    rst_n_i                     : in     std_logic;
    clk_sys_i                   : in     std_logic;
    wb_adr_i                    : in     std_logic_vector(5 downto 0);
    wb_dat_i                    : in     std_logic_vector(31 downto 0);
    wb_dat_o                    : out    std_logic_vector(31 downto 0);
    wb_cyc_i                    : in     std_logic;
    wb_sel_i                    : in     std_logic_vector(3 downto 0);
    wb_stb_i                    : in     std_logic;
    wb_we_i                     : in     std_logic;
    wb_ack_o                    : out    std_logic;
    wb_stall_o                  : out    std_logic;
    clk_i                       : in     std_logic;
    fai_clk_i                   : in     std_logic;

    -- DMA Engine Control
    dma_dcsr_o                  : out    std_logic_vector(31 downto 0);
    dma_dcsr_i                  : in     std_logic_vector(31 downto 0);
    dma_dcsr_load_o             : out    std_logic;
    dma_ddmacsr_o               : out    std_logic_vector(31 downto 0);
    dma_ddmacsr_load_o          : out    std_logic;
    dma_wdmatlpa_o              : out    std_logic_vector(31 downto 0);
    dma_wdmatlpa_load_o         : out    std_logic;
    dma_wdmatlps_o              : out    std_logic_vector(31 downto 0);
    dma_wdmatlps_load_o         : out    std_logic;
    dma_wdmatlpc_o              : out    std_logic_vector(31 downto 0);
    dma_wdmatlpc_load_o         : out    std_logic;
    dma_wdmatlpp_o              : out    std_logic_vector(31 downto 0);
    dma_wdmatlpp_load_o         : out    std_logic;

    -- CC Control and Monitor
    dma_ccfaicfgval_o           : out    std_logic_vector(31 downto 0);
    dma_wdmastatus_i            : in     std_logic_vector(31 downto 0);
    dma_linkstatus_i            : in     std_logic_vector(31 downto 0);
    dma_frameerrcnt_i           : in     std_logic_vector(31 downto 0);
    dma_softerrcnt_i            : in     std_logic_vector(31 downto 0);
    dma_harderrcnt_i            : in     std_logic_vector(31 downto 0);
    dma_nodeid_o                : out    std_logic_vector(31 downto 0);
    dma_timeframelen_o          : out    std_logic_vector(31 downto 0)
);
end dma_wb_slave_wrapper;

architecture rtl of dma_wb_slave_wrapper is

begin

dma_controller_wb_slave_inst : entity work.dma_controller_wb_slave
port map (
    rst_n_i                   => rst_n_i,
    clk_sys_i                 => clk_sys_i,
    wb_adr_i                  => wb_adr_i,
    wb_dat_i                  => wb_dat_i,
    wb_dat_o                  => wb_dat_o,
    wb_cyc_i                  => wb_cyc_i,
    wb_sel_i                  => wb_sel_i,
    wb_stb_i                  => wb_stb_i,
    wb_we_i                   => wb_we_i,
    wb_ack_o                  => wb_ack_o,
    wb_stall_o                => wb_stall_o,
    clk_i                     => clk_i,
    fai_clk_i                 => fai_clk_i,

    dma_dcsr_o                => dma_dcsr_o,
    dma_dcsr_i                => dma_dcsr_i,
    dma_dcsr_load_o           => dma_dcsr_load_o,

    dma_ddmacsr_o             => dma_ddmacsr_o,
    dma_ddmacsr_i             => X"00000000",
    dma_ddmacsr_load_o        => dma_ddmacsr_load_o,

    dma_wdmatlpa_o            => dma_wdmatlpa_o,
    dma_wdmatlpa_i            => X"00000000",
    dma_wdmatlpa_load_o       => dma_wdmatlpa_load_o,

    dma_wdmatlps_o            => dma_wdmatlps_o,
    dma_wdmatlps_i            => X"00000000",
    dma_wdmatlps_load_o       => dma_wdmatlps_load_o,

    dma_wdmatlpc_o            => dma_wdmatlpc_o,
    dma_wdmatlpc_i            => X"00000000",
    dma_wdmatlpc_load_o       => dma_wdmatlpc_load_o,

    dma_wdmatlpp_o            => dma_wdmatlpp_o,
    dma_wdmatlpp_i            => X"00000000",
    dma_wdmatlpp_load_o       => dma_wdmatlpp_load_o,

    dma_rdmatlpp_o            => open,
    dma_rdmatlpa_o            => open,
    dma_rdmatlps_o            => open,
    dma_rdmatlpc_o            => open,
    dma_wdmaperf_o            => open,
    dma_rdmaperf_o            => open,
    dma_rdmastat_o            => open,
    dma_nrdcomp_o             => open,
    dma_rcompdsizw_o          => open,
    dma_dlwstat_o             => open,
    dma_dltrsstat_o           => open,
    dma_dmisccont_o           => open,
    dma_ccfaiirqclr_o         => open,
    dma_na0_o                 => open,
    dma_na1_o                 => open,
    dma_na2_o                 => open,
    dma_na3_o                 => open,
    dma_na4_o                 => open,
    dma_na5_o                 => open,
    dma_na6_o                 => open,
    dma_na7_o                 => open,
    dma_na8_o                 => open,
    dma_na9_o                 => open,
    dma_na10_o                => open,
    dma_na11_o                => open,
    dma_na12_o                => open,

    dma_ccfaicfgval_o         => dma_ccfaicfgval_o,
    dma_wdmastatus_i          => dma_wdmastatus_i,
    dma_linkstatus_i          => dma_linkstatus_i,
    dma_frameerrcnt_i         => dma_frameerrcnt_i,
    dma_softerrcnt_i          => dma_softerrcnt_i,
    dma_harderrcnt_i          => dma_harderrcnt_i,
    dma_nodeid_o              => dma_nodeid_o,
    dma_timeframelen_o        => dma_timeframelen_o
  );

end rtl;

