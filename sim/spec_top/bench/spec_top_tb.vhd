library ieee;
use ieee.std_logic_1164.all;
use ieee.NUMERIC_STD.all;
use std.textio.all;

use work.util.all;
use work.textutil.all;

library modelsim_lib;
use modelsim_lib.util.all;

-- DLS FOFB packages
library work;
use work.fofb_cc_pkg.all;
use work.test_interface.all;

entity spec_top_tb is
    generic (
        test_selector       : in string  := String'("fofb_cc_smoke_test");
        -- Configure CC design under test
        DEVICE              : device_t := BPM;
        TX_IDLE_NUM         : natural := 10;
        RX_IDLE_NUM         : natural := 8;
        SEND_ID_NUM         : natural := 14;

        BPMS                : integer:= 4;
        FAI_DW              : integer := 32;
        DMUX                : integer := 1;
        --
        TEST_DURATION       : integer := 5        -- # of time frames to be simulated
    );
end spec_top_tb;

architecture rtl of spec_top_tb is

---------------------------------------------------------------------
-- Constants
-- Number of Models receiving commands
---------------------------------------------------------------------
constant N_BFM      : integer := 2;   -- 0 : GN412X_BFM in Model Mode
--                                    -- 1 : GN412X_BFM in DUT mode
-- Number of files to feed BFMs
constant N_FILES    : integer := 2;
--
-- Depth of the command FIFO for each model
constant FIFO_DEPTH : integer := 16;
--
-- Maximum width of a command string
constant STRING_MAX : integer := 256;

-----------------------------------------------------------------------------
-- Command Router Signals
-----------------------------------------------------------------------------
signal CMD                  : string(1 to STRING_MAX);
signal CMD_REQ              : bit_vector(N_BFM-1 downto 0);
signal CMD_ACK              : bit_vector(N_BFM-1 downto 0);
signal CMD_ERR              : bit_vector(N_BFM-1 downto 0);
signal CMD_CLOCK_EN         : boolean;

-----------------------------------------------------------------------------
-- GN412x BFM Signals
-----------------------------------------------------------------------------
-- System signals
signal si57x_clk_p          : std_logic := '0';
signal si57x_clk_n          : std_logic;
signal fpga_clk_p           : std_logic := '0';
signal fpga_clk_n           : std_logic;

-- GN4124 interface
signal RSTOUT18n            : std_logic;
signal RSTOUT33n            : std_logic;
signal LCLK, LCLKn          : std_logic;
signal L2P_CLKp, L2P_CLKn   : std_logic;
signal L2P_DATA             : std_logic_vector(15 downto 0);
signal L2P_DATA_32          : std_logic_vector(31 downto 0);  -- For monitoring use
signal L2P_DFRAME           : std_logic;
signal L2P_VALID            : std_logic;
signal L2P_EDB              : std_logic;
signal L_WR_RDY             : std_logic_vector(1 downto 0);
signal P_RD_D_RDY           : std_logic_vector(1 downto 0);
signal L2P_RDY              : std_logic;
signal TX_ERROR             : std_logic;
signal P2L_CLKp, P2L_CLKn   : std_logic;
signal P2L_DATA             : std_logic_vector(15 downto 0);
signal P2L_DATA_32          : std_logic_vector(31 downto 0);  -- For monitoring use
signal P2L_DFRAME           : std_logic;
signal P2L_VALID            : std_logic;
signal P2L_RDY              : std_logic;
signal P_WR_REQ             : std_logic_vector(1 downto 0);
signal P_WR_RDY             : std_logic_vector(1 downto 0);
signal RX_ERROR             : std_logic;
signal VC_RDY               : std_logic_vector(1 downto 0);
signal GPIO                 : std_logic_vector(15 downto 0) := X"0000";

-----------------------------------------------------------------------------
-- Bus Monitor Signals
-----------------------------------------------------------------------------
signal Q_P2L_DFRAME         : std_logic;
signal SIMPLE_TEST          : std_logic_vector(15 downto 0);

signal sysclk               : std_logic := '0';
signal adcclk               : std_logic := '0';
signal rxp                  : std_logic_vector(1 downto 0);
signal rxn                  : std_logic_vector(1 downto 0);
signal txp                  : std_logic_vector(1 downto 0);
signal txn                  : std_logic_vector(1 downto 0);
signal mgtreset             : std_logic;
signal powerdown            : std_logic := '0';
signal refclk               : std_logic;
signal userclk              : std_logic;
signal userclk_2x           : std_logic;
signal gtreset_in           : std_logic;
signal gtreset              : std_logic;
signal txoutclk             : std_logic;
signal plllkdet             : std_logic;

signal spy_timeframe_end    : std_logic;
signal spy_dma_rstb         : std_logic;
signal spy_dma_rstb_prev    : std_logic;
signal spy_dma_dat_i        : std_logic_vector(63 downto 0);
signal spy_dma_adr          : std_logic_vector(31 downto 0);
signal spy_fpga_clk         : std_logic;

file   dmafile              : text open write_mode is "dmafile.dat";

begin

-----------------------------------------------------------------------------
-- MODEL Component
-----------------------------------------------------------------------------
CMD_ERR <= (others => '0');

UC : entity work.cmd_router
generic map(
    N_BFM               => N_BFM,
    N_FILES             => N_FILES,
    FIFO_DEPTH          => FIFO_DEPTH,
    STRING_MAX          => STRING_MAX
)
port map(
    CMD                 => CMD,
    CMD_REQ             => CMD_REQ,
    CMD_ACK             => CMD_ACK,
    CMD_ERR             => CMD_ERR,
    CMD_CLOCK_EN        => CMD_CLOCK_EN
);

-----------------------------------------------------------------------------
-- GN412x BFM - PRIMARY
-----------------------------------------------------------------------------
gn412x_bfm_inst : entity work.gn412x_bfm
generic map (
    STRING_MAX          => STRING_MAX,
    T_LCLK              => 10 ns,
    T_P2L_CLK_DLY       => 2 ns,
    INSTANCE_LABEL      => "gn412x_bfm_inst",
    MODE_PRIMARY        => true
)
port map (
    -- CMD_ROUTER Interface
    CMD                 => CMD,
    CMD_REQ             => CMD_REQ(0),
    CMD_ACK             => CMD_ACK(0),
    CMD_CLOCK_EN        => CMD_CLOCK_EN,
    -- GN412x Signal I/O
    RSTINn              => '0',     -- not used in the model
    -- Reset outputs to DUT
    RSTOUT18n           => RSTOUT18n,
    RSTOUT33n           => RSTOUT33n,
    ----------------- Local Bus Clock ---------------------------
    LCLK                => LCLK,
    LCLKn               => LCLKn,
    ----------------- Local-to-PCI Dataflow ---------------------
    -- Transmitter Source Synchronous Clock.
    L2P_CLKp            => L2P_CLKp,
    L2P_CLKn            => L2P_CLKn,
    -- L2P DDR Link
    L2P_DATA            => L2P_DATA,
    L2P_DFRAME          => L2P_DFRAME,
    L2P_VALID           => L2P_VALID,
    L2P_EDB             => L2P_EDB,
    -- L2P SDR Controls
    L_WR_RDY            => L_WR_RDY,
    P_RD_D_RDY          => P_RD_D_RDY,
    L2P_RDY             => L2P_RDY,
    TX_ERROR            => TX_ERROR,
    ----------------- PCIe-to-Local Dataflow ---------------------
    -- Transmitter Source Synchronous Clock.
    P2L_CLKp            => P2L_CLKp,
    P2L_CLKn            => P2L_CLKn,
    -- P2L DDR Link
    P2L_DATA            => P2L_DATA,
    P2L_DFRAME          => P2L_DFRAME,
    P2L_VALID           => P2L_VALID,
    -- P2L SDR Controls
    P2L_RDY             => P2L_RDY,
    P_WR_REQ            => P_WR_REQ,
    P_WR_RDY            => P_WR_RDY,
    RX_ERROR            => RX_ERROR,
    VC_RDY              => VC_RDY,
    GPIO                => gpio
);

-----------------------------------------------------------------------------
-- SPEC Sniffer Instantiation
-----------------------------------------------------------------------------
spec_top_inst : entity work.spec_top
generic map (
    SIM_GTPRESET_SPEEDUP=> 1
)
port map (
    si57x_clk_p         => si57x_clk_p,
    si57x_clk_n         => si57x_clk_n,
--    l_clkp              => LCLK,
--    l_clkn              => LCLKn,
    fpga_clk_p          => fpga_clk_p,
    fpga_clk_n          => fpga_clk_n,
    sfp_rxn             => rxn,
    sfp_rxp             => rxp,
    sfp_txn             => txn,
    sfp_txp             => txp,
    -- GN4124 interface
    l_rst_n             => RSTOUT18n,
    p2l_rdy             => P2L_RDY,
    p2l_clkn            => P2L_CLKn,
    p2l_clkp            => P2L_CLKp,
    p2l_data            => P2L_DATA,
    p2l_dframe          => P2L_DFRAME,
    p2l_valid           => P2L_VALID,
    p_wr_req            => P_WR_REQ,
    p_wr_rdy            => P_WR_RDY,
    rx_error            => RX_ERROR,
    l2p_data            => L2P_DATA,
    l2p_dframe          => L2P_DFRAME,
    l2p_valid           => L2P_VALID,
    l2p_clkn            => L2P_CLKn,
    l2p_clkp            => L2P_CLKp,
    l2p_edb             => L2P_EDB,
    l2p_rdy             => L2P_RDY,
    l_wr_rdy            => L_WR_RDY,
    p_rd_d_rdy          => P_RD_D_RDY,
    tx_error            => TX_ERROR,
    vc_rdy              => VC_RDY,
    gpio                => GPIO(9 downto 8)
);


process
    variable vP2L_DATA_LOW : std_logic_vector(P2L_DATA'range);
begin
    wait until(P2L_CLKp'event and (P2L_CLKp = '1'));
    vP2L_DATA_LOW := P2L_DATA;
    loop
        wait on P2L_DATA, P2L_CLKp;
        P2L_DATA_32 <= P2L_DATA & vP2L_DATA_LOW;
        if(P2L_CLKp = '0') then
            exit;
        end if;
    end loop;
end process;

-- CC Clock at 106.25MHz
si57x_clk_p <= not si57x_clk_p after 4700 ps;
si57x_clk_n <= not si57x_clk_p;

-- System Clock at 125MHz
fpga_clk_p  <= not fpga_clk_p after 4000 ps;
fpga_clk_n  <= not fpga_clk_p;

-----------------------------------------------------------------------------
-- Communication Controller Tester
-----------------------------------------------------------------------------
fofb_cc_top_tester_inst : entity work.fofb_cc_top_tester
    generic map (
        test_selector           => test_selector,
        --
        DEVICE                  => DEVICE,
        BPMS                    => BPMS,
        FAI_DW                  => FAI_DW,
        DMUX                    => DMUX,
        --
        TX_IDLE_NUM             => TX_IDLE_NUM,
        RX_IDLE_NUM             => RX_IDLE_NUM,
        SEND_ID_NUM             => SEND_ID_NUM,
        --
        TEST_DURATION           => TEST_DURATION
    )
    port map (
        refclk_i                => refclk,
        sysclk_i                => sysclk,

        userclk_i               => userclk,
        userclk_2x_i            => userclk_2x,

        gtreset_i               => gtreset,
        mgtreset_i              => mgtreset,
        adcclk_i                => adcclk,

        txoutclk_o              => txoutclk,
        plllkdet_o              => plllkdet,

        fai_cfg_a_i             => (others => '0'),
        fai_cfg_do_i            => (others => '0'),
        fai_cfg_di_o            => open,
        fai_cfg_we_i            => '0',
        fai_cfg_clk_i           => '0',
        fai_cfg_val_o           => open,

        fai_fa_block_start_o    => open,
        fai_fa_data_valid_o     => open,
        fai_fa_d_o              => open,

        xy_buf_dat_i            => (others => '0'),
        xy_buf_addr_o           => open,
        xy_buf_rstb_o           => open,
        timeframe_start_i       => '0',
        timeframe_end_i         => '0',

        rxn_i                   => txn,
        rxp_i                   => txp,
        txn_o                   => rxn,
        txp_o                   => rxp
    );

gtreset_in <= not RSTOUT18n;

tester_clkgen : entity work.fofb_cc_clk_if
port map (
    refclk_n_i              => si57x_clk_n,
    refclk_p_i              => si57x_clk_p,

    gtreset_i               => gtreset_in,
    txoutclk_i              => txoutclk,
    plllkdet_i              => plllkdet,

    initclk_o               => open,
    refclk_o                => refclk,
    mgtreset_o              => mgtreset,
    gtreset_o               => gtreset,

    userclk_o               => userclk,
    userclk_2x_o            => userclk_2x
);

--------------------------------------------------------------------
-- Spy Signals
--------------------------------------------------------------------
spy_process : process
begin
    init_signal_spy("spec_top_tb/spec_top_inst/timeframe_end", "spec_top_tb/spy_timeframe_end", 1, -1);
    init_signal_spy("spec_top_tb/spec_top_inst/dma_rstb", "spec_top_tb/spy_dma_rstb", 1, -1);
    init_signal_spy("spec_top_tb/spec_top_inst/dma_dat_i", "spec_top_tb/spy_dma_dat_i", 1, -1);
    init_signal_spy("spec_top_tb/spec_top_inst/dma_adr", "spec_top_tb/spy_dma_adr", 1, -1);
    init_signal_spy("spec_top_tb/spec_top_inst/fpga_clk", "spec_top_tb/spy_fpga_clk", 1, -1);
    wait;
end process;

process(spy_fpga_clk)
    variable outline    : line;
    variable odat       : integer;
begin
    if rising_edge(spy_fpga_clk) then
        spy_dma_rstb_prev <= spy_dma_rstb;

        if (spy_dma_rstb_prev = '1') then
            odat := to_integer(signed(spy_dma_dat_i(31 downto 0)));
            write(outline, odat);
            writeline(dmafile, outline);
        end if;
    end if;
end process;


end rtl;
