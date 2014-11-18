library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library unisim;
use unisim.vcomponents.all;

library work;
use work.fofb_cc_pkg.all;

entity spec_top is
generic (
    DCC_INST                : boolean := true;
    SIM_GTPRESET_SPEEDUP    : integer := 0
);
port (
    -----------------------------------------------------------------------
    -- CC differential clock from Si570
    -----------------------------------------------------------------------
    si57x_clk_p             : in  std_logic;
    si57x_clk_n             : in  std_logic;

    -------------------------------------------------------------------------
    -- Interface with the Gennum GN4124
    -------------------------------------------------------------------------
    -- Reset from Gennum GN4124 (RSTOUT18_N)
    L_CLKp                  : in std_logic;
    L_CLKn                  : in std_logic;
    L_RST_N                 : in std_logic;

    -- Gennum GN4124 GPIO Interface
    GPIO                    : inout std_logic_vector(1 downto 0);

    -- PCIe to Local [Inbound Data] - RX
    P2L_RDY                 : out std_logic;
    P2L_CLKn                : in  std_logic;
    P2L_CLKp                : in  std_logic;
    P2L_DATA                : in  std_logic_vector(15 downto 0);
    P2L_DFRAME              : in  std_logic;
    P2L_VALID               : in  std_logic;

    -- Inbound Buffer Request/Status
    P_WR_REQ                : in  std_logic_vector(1 downto 0);
    P_WR_RDY                : out std_logic_vector(1 downto 0);
    RX_ERROR                : out std_logic;

    -- Local to Parallel [Outbound Data] - TX
    L2P_DATA                : out std_logic_vector(15 downto 0);
    L2P_DFRAME              : out std_logic;
    L2P_VALID               : out std_logic;
    L2P_CLKn                : out std_logic;
    L2P_CLKp                : out std_logic;
    L2P_EDB                 : out std_logic;

    -- Outbound Buffer Status
    L2P_RDY                 : in std_logic;
    L_WR_RDY                : in std_logic_vector(1 downto 0);
    P_RD_D_RDY              : in std_logic_vector(1 downto 0);
    TX_ERROR                : in std_logic;
    VC_RDY                  : in std_logic_vector(1 downto 0);

    -----------------------------------------------------------------------
    -- Si570 I2C interface
    -----------------------------------------------------------------------
    si57x_oe                : out std_logic;
    si57x_sda               : inout std_logic;
    si57x_scl               : inout std_logic;

    -----------------------------------------------------------------------
    -- SFP Interface
    -----------------------------------------------------------------------
    sfp_ena                 : out std_logic;
    sfp_txp                 : out std_logic_vector(1 downto 0);
    sfp_txn                 : out std_logic_vector(1 downto 0);
    sfp_rxp                 : in  std_logic_vector(1 downto 0);
    sfp_rxn                 : in  std_logic_vector(1 downto 0)
);
end entity spec_top;

architecture rtl of spec_top is

--------------------------------------------------------------------
-- Component Definitions
--------------------------------------------------------------------
component icon
    port (
        control0    : out std_logic_vector(35 downto 0)
    );
end component;

component ila
    port (
        control     : in  std_logic_vector(35 downto 0);
        clk         : in  std_logic;
        data        : in  std_logic_vector(127 downto 0);
        trig0       : in  std_logic_vector(7 downto 0)
     );
end component;

--------------------------------------------------------------------
-- Constants Declaration
--------------------------------------------------------------------
constant c_BAR0_APERTURE    : integer := 18;  -- nb of bits
constant c_CSR_WB_SLAVES_NB : integer := 11;

--------------------------------------------------------------------
-- Design Signals
--------------------------------------------------------------------
signal control              : std_logic_vector(35 downto 0);
signal trig0                : std_logic_vector(7 downto 0);
signal data                 : std_logic_vector(127 downto 0);

signal l_clk                : std_logic;
signal si57x_clk            : std_logic;
signal sys_clk              : std_logic;

signal freq_counter         : unsigned(31 downto 0);
signal i2c_done             : std_logic;

signal reset_n              : std_logic;
signal reset                : std_logic := '0';

-- CSR wishbone bus (master)
signal wbm_adr              : std_logic_vector(31 downto 0);
signal wbm_dati             : std_logic_vector(31 downto 0);
signal wbm_dato             : std_logic_vector(31 downto 0);
signal wbm_sel              : std_logic_vector(3 downto 0);
signal wbm_cyc              : std_logic;
signal wbm_stb              : std_logic;
signal wbm_we               : std_logic;
signal wbm_ack              : std_logic;
signal wbm_stall            : std_logic;

-- CSR wishbone bus (slaves)
signal wb_adr               : std_logic_vector(31 downto 0);
signal wb_dati              : std_logic_vector((32*c_CSR_WB_SLAVES_NB)-1 downto 0);
signal wb_dato              : std_logic_vector(31 downto 0);
signal wb_sel               : std_logic_vector(3 downto 0);
signal wb_cyc               : std_logic_vector(c_CSR_WB_SLAVES_NB-1 downto 0);
signal wb_stb               : std_logic;
signal wb_we                : std_logic;
signal wb_ack               : std_logic_vector(c_CSR_WB_SLAVES_NB-1 downto 0);
signal wb_stall             : std_logic_vector(c_CSR_WB_SLAVES_NB-1 downto 0);

signal fai_cfg_clk          : std_logic;
signal fai_cfg_val          : std_logic_vector(31 downto 0);

-- DMA wishbone bus
signal dma_adr              : std_logic_vector(31 downto 0);
signal dma_dat_i            : std_logic_vector(31 downto 0);
signal dma_dat_o            : std_logic_vector(31 downto 0);
signal dma_sel              : std_logic_vector(3 downto 0);
signal dma_cyc              : std_logic;
signal dma_stb              : std_logic;
signal dma_we               : std_logic;
signal dma_ack              : std_logic;
signal dma_stall            : std_logic;
signal dma_err              : std_logic;
signal dma_rty              : std_logic;
signal dma_int              : std_logic;
signal dma_rstb             : std_logic;

-- Interrupts stuff
signal irq_sources          : std_logic_vector(1 downto 0);
signal irq_to_gn4124        : std_logic;

-- Communication Controllers signals
signal dma_ccfaicfgval      : std_logic_vector(31 downto 0);
signal dma_linkstatus       : std_logic_vector(31 downto 0);
signal dma_nodeid           : std_logic_vector(31 downto 0);
signal dma_timeframelen     : std_logic_vector(31 downto 0);

signal timeframe_end        : std_logic;
signal timeframe_end_synced : std_logic;
signal sysreset_n           : std_logic;

signal irq_out              : std_logic;

signal sys_clksel           : std_logic_vector(31 downto 0);
signal sys_clkcnt           : std_logic_vector(31 downto 0);
signal test_clocks          : std_logic_vector(15 downto 0);

signal timeframe_cntr       : unsigned(31 downto 0) := (others => '0');
signal gn4124_core_status   : std_logic_vector(31 downto 0);

begin

GPIO(0) <= irq_out;
GPIO(1) <= '0';

--------------------------------------------------------------------
-- Fixed output ports
--------------------------------------------------------------------
si57x_oe <= '1';    -- Output Enable to Si57x
sfp_ena <= '0';     -- Output Enable to SFP

--------------------------------------------------------------------
-- Local clock from gennum LCLK
-- Must be set to 100MHz on GN4124
--------------------------------------------------------------------
l_clk_buf : IBUFDS
generic map (
    DIFF_TERM    => false,
    IBUF_LOW_PWR => true,
    IOSTANDARD   => "DEFAULT"
)
port map (
    O  => sys_clk,
    I  => L_CLKp,
    IB => L_CLKn
);

--------------------------------------------------------------------
-- Initial reset
--------------------------------------------------------------------
process(sys_clk)
    variable counter : unsigned(3 downto 0) := "0000";
begin
    if rising_edge(sys_clk) then
        if (counter = "1111") then
            reset <= '0';
        else
            reset <= '1';
            counter := counter + 1;
        end if;
    end if;
end process;

reset_n <= L_RST_N;

--------------------------------------------------------------------
-- I2C module for configuring Si570 to generate 106.25MHz GTPA clock
--------------------------------------------------------------------
spec_si57x_inst : entity work.spec_si57x
port map (
    clk                     => sys_clk,
    rst                     => reset,
    i2c_scl                 => si57x_scl,
    i2c_sda                 => si57x_sda,
    i2c_done                => i2c_done
);

sysreset_n <= i2c_done when (SIM_GTPRESET_SPEEDUP = 0) else '1';

-------------------------------------------------------------------
-- Communication Controller instantiation
--------------------------------------------------------------------
fofb_cc_top_wrapper_inst : entity work.fofb_cc_top_wrapper
generic map (
    SIM_GTPRESET_SPEEDUP        => SIM_GTPRESET_SPEEDUP
)
port map (
    refclk_p_i                  => si57x_clk_p,
    refclk_n_i                  => si57x_clk_n,

    sysclk_i                    => sys_clk,
    sysreset_n_i                => sysreset_n,

    fai_cfg_clk_o               => fai_cfg_clk,

    fai_rio_rdp_i               => sfp_rxp,
    fai_rio_rdn_i               => sfp_rxn,
    fai_rio_tdp_o               => sfp_txp,
    fai_rio_tdn_o               => sfp_txn,

    xy_buf_addr_i               => dma_adr(9 downto 0),
    xy_buf_dat_o(63 downto 32)  => open,
    xy_buf_dat_o(31 downto 0)   => dma_dat_i,
    xy_buf_rstb_i               => dma_rstb,

    fofb_dma_ok_i               => '1',

    dma_ccfaicfgval_i           => dma_ccfaicfgval,
    dma_linkstatus_o            => dma_linkstatus,
    dma_nodeid_i                => dma_nodeid,
    dma_timeframelen_i          => dma_timeframelen,
    timeframe_end_rise_o        => timeframe_end
);

--------------------------------------------------------------------
-- Gennum 4124 Core Instantiation

gn4124_core_inst : entity work.gn4124_core
port map (
    ---------------------------------------------------------
    -- Control and status
    ---------------------------------------------------------
    rst_n_a_i               => reset_n,
    status_o                => gn4124_core_status,

    ---------------------------------------------------------
    -- P2L Direction
    ---------------------------------------------------------
    -- Source Sync DDR related signals
    p2l_clk_p_i             => P2L_CLKp,
    p2l_clk_n_i             => P2L_CLKn,
    p2l_data_i              => P2L_DATA,
    p2l_dframe_i            => P2L_DFRAME,
    p2l_valid_i             => P2L_VALID,
    -- P2L Control
    p2l_rdy_o               => P2L_RDY,
    p_wr_req_i              => P_WR_REQ,
    p_wr_rdy_o              => P_WR_RDY,
    rx_error_o              => RX_ERROR,
    vc_rdy_i                => VC_RDY,

    ---------------------------------------------------------
    -- L2P Direction
    ---------------------------------------------------------
    -- Source Sync DDR related signals
    l2p_clk_p_o             => L2P_CLKp,
    l2p_clk_n_o             => L2P_CLKn,
    l2p_data_o              => L2P_DATA,
    l2p_dframe_o            => L2P_DFRAME,
    l2p_valid_o             => L2P_VALID,
    -- L2P Control
    l2p_edb_o               => L2P_EDB,
    l2p_rdy_i               => L2P_RDY,
    l_wr_rdy_i              => L_WR_RDY,
    p_rd_d_rdy_i            => P_RD_D_RDY,
    tx_error_i              => TX_ERROR,

    ---------------------------------------------------------
    -- Interrupt interface
    ---------------------------------------------------------
    dma_irq_o               => irq_sources,
    irq_p_i                 => irq_to_gn4124,
    irq_p_o                 => irq_out,

    ---------------------------------------------------------
    -- DMA registers wishbone interface (slave classic)
    ---------------------------------------------------------
    dma_reg_clk_i           => sys_clk,
    dma_reg_adr_i           => wb_adr,
    dma_reg_dat_i           => wb_dato,
    dma_reg_sel_i           => wb_sel,
    dma_reg_stb_i           => wb_stb,
    dma_reg_we_i            => wb_we,
    dma_reg_cyc_i           => wb_cyc(0),
    dma_reg_dat_o           => wb_dati(31 downto 0),
    dma_reg_ack_o           => wb_ack(0),
    dma_reg_stall_o         => wb_stall(0),

    ---------------------------------------------------------
    -- CSR wishbone interface (master pipelined)
    ---------------------------------------------------------
    csr_clk_i               => sys_clk,
    csr_adr_o               => wbm_adr,
    csr_dat_o               => wbm_dato,
    csr_sel_o               => wbm_sel,
    csr_stb_o               => wbm_stb,
    csr_we_o                => wbm_we,
    csr_cyc_o               => wbm_cyc,
    csr_dat_i               => wbm_dati,
    csr_ack_i               => wbm_ack,
    csr_stall_i             => wbm_stall,
    csr_err_i               => '0',
    csr_rty_i               => '0',
    csr_int_i               => '0',

    ---------------------------------------------------------
    -- DMA wishbone interface (master pipelined)
    ---------------------------------------------------------
    dma_clk_i               => sys_clk,
    dma_adr_o               => dma_adr,
    dma_dat_o               => dma_dat_o,
    dma_sel_o               => dma_sel,
    dma_stb_o               => dma_stb,
    dma_we_o                => dma_we,
    dma_cyc_o               => dma_cyc,
    dma_dat_i               => dma_dat_i,
    dma_ack_i               => dma_ack,
    dma_stall_i             => dma_stall,
    dma_err_i               => dma_err,
    dma_rty_i               => dma_rty,
    dma_int_i               => dma_int,
    dma_clk_o               => l_clk,

    ---------------------------------------------------------
    -- DLS Communication Controller interface
    ---------------------------------------------------------
    fai_clk_i               => fai_cfg_clk,
    dma_ccfaicfgval_o       => dma_ccfaicfgval,
    dma_linkstatus_i        => dma_linkstatus,
    dma_nodeid_o            => dma_nodeid,
    dma_timeframelen_o      => dma_timeframelen,
    timeframe_end_i         => timeframe_end
);

--------------------------------------------------------------------
-- CSR wishbone address decoder
--  0x00000 : DMA configuration registers
--  0x10000 : TOP configuration registers
--------------------------------------------------------------------
wb_addr_decoder_inst : entity work.wb_addr_decoder
generic map (
    g_WINDOW_SIZE           => c_BAR0_APERTURE,
    g_WB_SLAVES_NB          => c_CSR_WB_SLAVES_NB
)
port map (
    -- GN4124 core clock and reset
    clk_i                   => sys_clk,
    rst_n_i                 => reset_n,

    -- Wishbone master interface
    wbm_adr_i               => wbm_adr,
    wbm_dat_i               => wbm_dato,
    wbm_sel_i               => wbm_sel,
    wbm_stb_i               => wbm_stb,
    wbm_we_i                => wbm_we,
    wbm_cyc_i               => wbm_cyc,
    wbm_dat_o               => wbm_dati,
    wbm_ack_o               => wbm_ack,
    wbm_stall_o             => wbm_stall,

    -- Wishbone slaves interface
    wb_adr_o                => wb_adr,
    wb_dat_o                => wb_dato,
    wb_sel_o                => wb_sel,
    wb_stb_o                => wb_stb,
    wb_we_o                 => wb_we,
    wb_cyc_o                => wb_cyc,
    wb_dat_i                => wb_dati,
    wb_ack_i                => wb_ack,
    wb_stall_i              => wb_stall
);


--------------------------------------------------------------------
-- SLAVE-1 is System Status
syscsr_wb_slave_inst : entity work.syscsr_wb_slave
port map (
    rst_n_i                 => reset_n,
    clk_sys_i               => sys_clk,
    wb_adr_i                => wb_adr(1 downto 0),
    wb_dat_i                => wb_dato,
    wb_dat_o                => wb_dati(63 downto 32),
    wb_cyc_i                => wb_cyc(1),
    wb_sel_i                => wb_sel,
    wb_stb_i                => wb_stb,
    wb_we_i                 => wb_we,
    wb_ack_o                => wb_ack(1),
    wb_stall_o              => wb_stall(1),

    sys_status_i            => gn4124_core_status,
    sys_clksel_o            => sys_clksel,
    sys_clkcnt_i            => sys_clkcnt
);

--------------------------------------------------------------------
-- Measure on board clock(s)

freq_cnt16_inst : entity work.freq_cnt16
port map (
    reset                   => '0',
    reference_clock         => sys_clk,
    test_clocks             => test_clocks,
    clock_sel               => sys_clksel(3 downto 0),
    clock_cnt_out           => sys_clkcnt(15 downto 0)
);

test_clocks <= X"000" & "000" &  fai_cfg_clk;

------------------------------------------------------------------------------
-- Interrupt Generation and Handling
-- Forward irq pulses for to GN4124
------------------------------------------------------------------------------
irq_to_gn4124 <= irq_sources(1) or irq_sources(0);

--------------------------------------------------------------------
-- DMA wishbone bus connected to a DPRAM
--------------------------------------------------------------------
process (sys_clk, reset_n)
begin
    if (reset_n = '0') then
        dma_ack <= '0';
    elsif rising_edge(sys_clk) then
        if (dma_cyc = '1' and dma_stb = '1') then
            dma_ack <= '1';
        else
            dma_ack <= '0';
        end if;
    end if;
end process;

dma_stall <= '0';
dma_err   <= '0';
dma_rty   <= '0';
dma_int   <= '0';
dma_rstb  <= '0'; -- Not bothered to use
-- dma_rstb <= not dma_we and dma_cyc and dma_stb;

--------------------------------------------------------------------
-- Test "timeframe_end" generator

--fai_cfg_clk <= sys_clk;
--
--process(sys_clk)
--    variable counter  : integer := 0;
--begin
--    if rising_edge(sys_clk) then
--        if (counter = 10000) then
--            timeframe_end <= '1';
--            counter := 0;
--            timeframe_cntr <= timeframe_cntr + 1;
--        else
--            timeframe_end <= '0';
--            counter := counter + 1;
--        end if;
--
--        dma_dat_i <= std_logic_vector(resize(unsigned(dma_adr(9 downto 0)),32) + timeframe_cntr);
--
--    end if;
--end process;

---- Test ROM for X/Y Buffer
--cmp_test_rom : entity work.generic_sprom
--port map(
--    clk_i       => sys_clk,
--    addr_i      => dma_adr(9 downto 0),
--    dat_o       => open --dma_dat_i(31 downto 0)
--);


--------------------------------------------------------------------
-- Chipscope Instantiation

--trig0(0)            <= irq_out;
--
--data(0)             <= irq_out;
--
--ila_inst : ila
--port map (
--    control         => control,
--    clk             => sys_clk,
--    data            => data,
--    trig0           => trig0
--);
--
--icon_inst : icon
--port map (
--    control0        => control
--);

end architecture rtl;
