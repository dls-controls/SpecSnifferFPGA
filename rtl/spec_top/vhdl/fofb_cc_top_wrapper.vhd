library ieee;
use ieee.std_logic_1164.all;

library work;
use work.fofb_cc_pkg.all;

-----------------------------------------------
--  Entity declaration
-----------------------------------------------
entity fofb_cc_top_wrapper is
    generic (
        ID                      : integer := 0;
        SIM_GTPRESET_SPEEDUP    : integer := 0;
        LANE_COUNT              : integer := 2
    );
    port (
        -- differential MGT/GTP clock inputs
        refclk_p_i              : in  std_logic;
        refclk_n_i              : in  std_logic;
        -- clock and reset interface
        sysclk_i                : in  std_logic;
        sysreset_n_i            : in  std_logic;
        -- FOFB communication controller configuration interface
        fai_cfg_clk_o           : out std_logic;
        -- serial I/Os for eight RocketIOs on the Libera
        fai_rio_rdp_i           : in  std_logic_vector(LANE_COUNT-1 downto 0);
        fai_rio_rdn_i           : in  std_logic_vector(LANE_COUNT-1 downto 0);
        fai_rio_tdp_o           : out std_logic_vector(LANE_COUNT-1 downto 0);
        fai_rio_tdn_o           : out std_logic_vector(LANE_COUNT-1 downto 0);
        -- PMC-SFP module interface
        xy_buf_addr_i           : in  std_logic_vector(9 downto 0);
        xy_buf_dat_o            : out std_logic_vector(63 downto 0);
        xy_buf_rstb_i           : in  std_logic;
        timeframe_end_rise_o    : out std_logic;
        -- Higher-level integration interface (used for PMC)
        fofb_dma_ok_i           : in  std_logic;
        fofb_node_mask_o        : out std_logic_vector(NodeNum-1 downto 0);
        --
        dma_ccfaicfgval_i       : in  std_logic_vector(31 downto 0);
        dma_linkstatus_o        : out std_logic_vector(31 downto 0);
        dma_nodeid_i            : in  std_logic_vector(31 downto 0);
        dma_timeframelen_i      : in  std_logic_vector(31 downto 0)
);
end fofb_cc_top_wrapper;

architecture structure of fofb_cc_top_wrapper is

signal fofb_rxlink_up          : std_logic_vector(LANE_COUNT-1 downto 0);
signal fofb_rxlink_partner     : std_logic_2d_10(LANE_COUNT-1 downto 0);
signal fofb_timestamp_val      : std_logic_vector(31 downto 0);
signal harderror_cnt           : std_logic_2d_16(LANE_COUNT-1 downto 0);
signal softerror_cnt           : std_logic_2d_16(LANE_COUNT-1 downto 0);
signal frameerror_cnt          : std_logic_2d_16(LANE_COUNT-1 downto 0);

begin

fofb_cc_top : entity work.fofb_cc_top
    generic map (
        ID                      => ID,
        DEVICE                  => SNIFFER,
        INTERLEAVED             => true,
        SIM_GTPRESET_SPEEDUP    => SIM_GTPRESET_SPEEDUP,
        LANE_COUNT              => LANE_COUNT
    )
    port map (
        refclk_p_i              => refclk_p_i,
        refclk_n_i              => refclk_n_i,

        adcclk_i                => '0',
        adcreset_i              => '0',
        sysclk_i                => sysclk_i,
        sysreset_n_i            => sysreset_n_i,

        fai_fa_block_start_i    => '0',
        fai_fa_data_valid_i     => '0',
        fai_fa_d_i              => (others => '0'),

        fai_cfg_a_o             => open,
        fai_cfg_d_o             => open,
        fai_cfg_d_i             => (others => '0'),
        fai_cfg_we_o            => open,
        fai_cfg_clk_o           => fai_cfg_clk_o,
        fai_cfg_val_i           => dma_ccfaicfgval_i,

        toa_rstb_i              => '0',
        toa_rden_i              => '0',
        toa_dat_o               => open,
        rcb_rstb_i              => '0',
        rcb_rden_i              => '0',
        rcb_dat_o               => open,
        fai_rxfifo_clear        => '0',
        fai_txfifo_clear        => '0',

        fai_rio_rdp_i           => fai_rio_rdp_i,
        fai_rio_rdn_i           => fai_rio_rdn_i,
        fai_rio_tdp_o           => fai_rio_tdp_o,
        fai_rio_tdn_o           => fai_rio_tdn_o,
        fai_rio_tdis_o          => open,

        coeff_x_addr_i          => (others => '0'),
        coeff_x_dat_o           => open,
        coeff_y_addr_i          => (others => '0'),
        coeff_y_dat_o           => open,

        xy_buf_addr_i           => xy_buf_addr_i,
        xy_buf_dat_o            => xy_buf_dat_o,
        xy_buf_rstb_i           => xy_buf_rstb_i,
        timeframe_start_o       => open,
        timeframe_end_o         => timeframe_end_rise_o,
        fofb_watchdog_i         => (others => '0'),
        fofb_event_i            => (others => '0'),
        fofb_process_time_o     => open,
        fofb_bpm_count_o        => open,
        fofb_dma_ok_i           => fofb_dma_ok_i,
        fofb_node_mask_o        => fofb_node_mask_o,
        fofb_timestamp_val_o    => open,
        fofb_link_status_o      => dma_linkstatus_o
    );
end structure;

