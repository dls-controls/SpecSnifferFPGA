--------------------------------------------------------------------------------
--                                                                            --
-- CERN BE-CO-HT         GN4124 core for PCIe FMC carrier                     --
--                       http://www.ohwr.org/projects/gn4124-core             --
--------------------------------------------------------------------------------
--
-- unit name: DMA controller (dma_controller.vhd)
--
-- authors: Simon Deprez (simon.deprez@cern.ch)
--          Matthieu Cattin (matthieu.cattin@cern.ch)
--
-- date: 31-08-2010
--
-- version: 0.2
--
-- description: Manages the DMA transfers.
--
--
-- dependencies:
--
--------------------------------------------------------------------------------
-- GNU LESSER GENERAL PUBLIC LICENSE
--------------------------------------------------------------------------------
-- This source file is free software; you can redistribute it and/or modify it
-- under the terms of the GNU Lesser General Public License as published by the
-- Free Software Foundation; either version 2.1 of the License, or (at your
-- option) any later version. This source is distributed in the hope that it
-- will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty
-- of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
-- See the GNU Lesser General Public License for more details. You should have
-- received a copy of the GNU Lesser General Public License along with this
-- source; if not, download it from http://www.gnu.org/licenses/lgpl-2.1.html
--------------------------------------------------------------------------------
-- last changes: 30-09-2010 (mcattin) Add status, error and abort
--------------------------------------------------------------------------------

library ieee;
use ieee.STD_LOGIC_1164.all;
use ieee.NUMERIC_STD.all;

library work;
use work.gn4124_core_pkg.all;
use work.spec_defines.all;

entity dma_controller is
port (
    ---------------------------------------------------------
    -- GN4124 core clock and reset
    clk_i                       : in std_logic;
    rst_n_i                     : in std_logic;

    ---------------------------------------------------------
    -- Interrupt request
    dma_ctrl_irq_o              : out std_logic_vector(1 downto 0);

    ---------------------------------------------------------
    -- To the L2P DMA master and P2L DMA master
    dma_ctrl_carrier_addr_o     : out std_logic_vector(31 downto 0);
    dma_ctrl_host_addr_h_o      : out std_logic_vector(31 downto 0);
    dma_ctrl_host_addr_l_o      : out std_logic_vector(31 downto 0);
    dma_ctrl_len_o              : out std_logic_vector(31 downto 0);
    dma_ctrl_start_l2p_o        : out std_logic;  -- To the L2P DMA master
    dma_ctrl_start_p2l_o        : out std_logic;  -- To the P2L DMA master
    dma_ctrl_start_next_o       : out std_logic;  -- To the P2L DMA master
    dma_ctrl_byte_swap_o        : out std_logic_vector(1 downto 0);
    dma_ctrl_abort_o            : out std_logic;
    dma_ctrl_done_i             : in  std_logic;
    dma_ctrl_error_i            : in  std_logic;

    ---------------------------------------------------------
    -- From P2L DMA master
    next_item_carrier_addr_i    : in std_logic_vector(31 downto 0);
    next_item_host_addr_h_i     : in std_logic_vector(31 downto 0);
    next_item_host_addr_l_i     : in std_logic_vector(31 downto 0);
    next_item_len_i             : in std_logic_vector(31 downto 0);
    next_item_next_l_i          : in std_logic_vector(31 downto 0);
    next_item_next_h_i          : in std_logic_vector(31 downto 0);
    next_item_attrib_i          : in std_logic_vector(31 downto 0);
    next_item_valid_i           : in std_logic;

    ---------------------------------------------------------
    -- Wishbone slave interface
    wb_clk_i                    : in  std_logic;
    wb_adr_i                    : in  std_logic_vector(5 downto 0);
    wb_dat_o                    : out std_logic_vector(31 downto 0);
    wb_dat_i                    : in  std_logic_vector(31 downto 0);
    wb_sel_i                    : in  std_logic_vector(3 downto 0);
    wb_cyc_i                    : in  std_logic;
    wb_stb_i                    : in  std_logic;
    wb_we_i                     : in  std_logic;
    wb_ack_o                    : out std_logic;
    ---------------------------------------------------------
    -- DLS Communication Controller interface
    fai_clk_i                   : in  std_logic;
    dma_ccfaicfgval_o           : out std_logic_vector(31 downto 0);
    dma_linkstatus_i            : in  std_logic_vector(31 downto 0);
    dma_nodeid_o                : out std_logic_vector(31 downto 0);
    dma_timeframelen_o          : out std_logic_vector(31 downto 0);
    timeframe_end_i             : in  std_logic
);
end dma_controller;


architecture behaviour of dma_controller is

------------------------------------------------------------------------------
-- Constants declaration
------------------------------------------------------------------------------
-- Waiting in IDLE statre
constant c_IDLE         : std_logic_vector(3 downto 0) := "0000";
-- DMA transfer error
constant c_DMAERR       : std_logic_vector(3 downto 0) := "1111";
-- No DMA taken place, next dma is not valid
constant c_FIRSTADRERR  : std_logic_vector(3 downto 0) := "0010";
-- DMA block finished, but next dma is not valid
constant c_NEXTADRERR   : std_logic_vector(3 downto 0) := "0011";
-- User intervention for stop
constant c_ABORT        : std_logic_vector(3 downto 0) := "0100";
-- Communication Controller link is down
constant c_DCCERR       : std_logic_vector(3 downto 0) := "1000";
-- Completed DMA succesfully and next DMA is in progress
constant c_BUSY         : std_logic_vector(3 downto 0) := "0001";

-- DMA controller FSM
type dma_ctrl_state_type is (DMA_IDLE, DMA_START_TRANSFER, DMA_TRANSFER,
                             DMA_ERROR);
------------------------------------------------------------------------------
-- Signals declaration
------------------------------------------------------------------------------
signal dma_ctrl_current_state   : dma_ctrl_state_type;
signal dma_dcsr                 : std_logic_vector(31 downto 0);
signal dma_dcsr_reg             : std_logic_vector(31 downto 0);
signal dma_dcsr_load            : std_logic;
signal dma_ddmacsr              : std_logic_vector(31 downto 0);
signal dma_ddmacsr_load         : std_logic;
signal dma_wdmatlpa             : std_logic_vector(31 downto 0);
signal dma_wdmatlpa_load        : std_logic;
signal dma_wdmatlps             : std_logic_vector(31 downto 0);
signal dma_wdmatlps_load        : std_logic;
signal dma_wdmatlpc             : std_logic_vector(31 downto 0);
signal dma_wdmatlpc_load        : std_logic;
signal dma_wdmatlpp             : std_logic_vector(31 downto 0);
signal dma_wdmatlpp_load        : std_logic;
signal dma_wdmastat_reg         : std_logic_vector(31 downto 0);
signal dma_linkstatus_reg       : std_logic_vector(31 downto 0);

signal dma_host_addr            : std_logic_vector(39 downto 0);

signal wdma_start               : std_logic;
signal wdma_stop                : std_logic;

signal timeframe_end            : std_logic;
signal mwr_addr                 : std_logic_vector(31 downto 0);
signal mwr_up_addr              : std_logic_vector(7 downto 0);
signal next_wdma_valid          : std_logic;
signal addr_val_lower           : std_logic;
signal addr_val_upper           : std_logic;
signal wdma_frame_len           : std_logic_vector(15 downto 0);
signal wdma_buf_ptr             : std_logic_vector(15 downto 0);
signal wdma_status              : std_logic_vector(3 downto 0);
signal cc_timeout               : std_logic;
signal init_reset               : std_logic;
signal wdma_len                 : std_logic_vector(9 downto 0);
signal wdma_count               : std_logic_vector(15 downto 0);
signal wdma_enabled             : std_logic;
signal dma_ctrl_start_l2p       : std_logic;
signal dma_ctrl_irq             : std_logic_vector(1 downto 0);

signal control                  : std_logic_vector(35 downto 0);
signal trig0                    : std_logic_vector(7 downto 0);
signal data                     : std_logic_vector(127 downto 0);

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


begin

-- Fixed outputs
dma_ctrl_start_p2l_o    <= '0';             -- Master Write only.
dma_ctrl_start_next_o   <= '0';             -- No SG DMA support.
dma_ctrl_carrier_addr_o <= (others => '0'); -- CC buffer addr starts at 0.
dma_ctrl_byte_swap_o    <= (others => '0');

-----------------------------------------------------------------
-- Wishbone slave instanciation
-----------------------------------------------------------------
dma_controller_wb_slave_0 : entity work.dma_wb_slave_wrapper
port map (
    rst_n_i                 => rst_n_i,
    clk_sys_i               => wb_clk_i,
    wb_adr_i                => wb_adr_i,
    wb_dat_i                => wb_dat_i,
    wb_dat_o                => wb_dat_o,
    wb_cyc_i                => wb_cyc_i,
    wb_sel_i                => wb_sel_i,
    wb_stb_i                => wb_stb_i,
    wb_we_i                 => wb_we_i,
    wb_ack_o                => wb_ack_o,
    clk_i                   => clk_i,
    fai_clk_i               => fai_clk_i,

    dma_dcsr_o              => dma_dcsr,
    dma_dcsr_i              => dma_dcsr_reg,
    dma_dcsr_load_o         => dma_dcsr_load,
    dma_ddmacsr_o           => dma_ddmacsr,
    dma_ddmacsr_load_o      => dma_ddmacsr_load,
    dma_wdmatlpa_o          => dma_wdmatlpa,
    dma_wdmatlpa_load_o     => dma_wdmatlpa_load,
    dma_wdmatlps_o          => dma_wdmatlps,
    dma_wdmatlps_load_o     => dma_wdmatlps_load,
    dma_wdmatlpc_o          => dma_wdmatlpc,
    dma_wdmatlpc_load_o     => dma_wdmatlpc_load,
    dma_wdmatlpp_o          => dma_wdmatlpp,
    dma_wdmatlpp_load_o     => dma_wdmatlpp_load,
    dma_ccfaicfgval_o       => dma_ccfaicfgval_o,
    dma_wdmastatus_i        => dma_wdmastat_reg,
    dma_linkstatus_i        => dma_linkstatus_reg,
    dma_frameerrcnt_i       => X"00000000",
    dma_softerrcnt_i        => X"00000000",
    dma_harderrcnt_i        => X"00000000",
    dma_nodeid_o            => dma_nodeid_o,
    dma_timeframelen_o      => dma_timeframelen_o
);

dma_linkstatus_reg <= "00" & X"000" & dma_linkstatus_i(17 downto 8) & "000000" & cc_timeout & dma_linkstatus_i(0);

-------------------------------------------------------
-- DMA Controller Configuration
next_wdma_valid <= addr_val_lower and addr_val_upper;

p_fsm : process (clk_i, rst_n_i)
begin
    if(rst_n_i = c_RST_ACTIVE) then

    elsif rising_edge(clk_i) then
        -- Initial Reset
        init_reset <= '0';
        if (dma_dcsr_load = '1') then
            init_reset <= dma_dcsr(0);
        end if;

        -- Lower DMA Address
        if (dma_wdmatlpa_load = '1') then
            mwr_addr <= dma_wdmatlpa;
        end if;

        -- Upper DMA Address
        if (dma_wdmatlps_load = '1') then
            mwr_up_addr <= dma_wdmatlps(31 downto 24);
        end if;

        -- DMA Address Valid
        if (addr_val_lower = '1' and addr_val_upper = '1') then
            addr_val_lower <= '0';
        elsif (dma_wdmatlpa_load = '1') then
            addr_val_lower <= '1';
        end if;

        if (addr_val_lower = '1' and addr_val_upper = '1') then
            addr_val_upper <= '0';
        elsif (dma_wdmatlps_load = '1') then
            addr_val_upper <= '1';
        end if;

        -- DMA Start
        wdma_start <= '0';
        if (dma_ddmacsr_load = '1') then
            wdma_start <= dma_ddmacsr(0);
        end if;

        -- DMA Stop
        wdma_stop <= '0';
        if (dma_ddmacsr_load = '1') then
            wdma_stop <= dma_ddmacsr(1);
        end if;

        -- DMA Block Size
        if (dma_wdmatlpp_load = '1') then
            wdma_frame_len <= dma_wdmatlpp(15 downto 0);
        end if;

        -- Status Register
        dma_dcsr_reg <= X"0000" & FPGA_VERSION;

        -- DMA Status Readback
        dma_wdmastat_reg <= X"00" &
                            wdma_buf_ptr &
                            "0000" & wdma_status;
    end if;
end process;

dma_ctrl_irq_o <= dma_ctrl_irq;
dma_ctrl_start_l2p_o <= dma_ctrl_start_l2p;
dma_ctrl_abort_o <= '0';
dma_ctrl_host_addr_h_o <= X"000000" & dma_host_addr(39 downto 32);
dma_ctrl_host_addr_l_o <= dma_host_addr(31 downto 0);
dma_ctrl_len_o <= X"00000800";      -- 2KBytes
dma_ctrl_irq(1) <= '0';           -- not used

-------------------------------------------------------------------
-- DMA controller FSM
-------------------------------------------------------------------
wdma_len   <= std_logic_vector(to_unsigned(32, 10)); -- TLP size in DWords
wdma_count <= std_logic_vector(to_unsigned(16,16));  -- For 2K we need 16 TLPs

dma_fsm_inst : entity work.BMD_64_RWDMA_FSM
port map (
    clk                 => clk_i,
    rst_n               => rst_n_i,
    init_rst_i          => init_reset,
    wdma_rst_o          => open,

    mwr_len_i           => wdma_len,
    mwr_count_i         => wdma_count,

    wdma_start_o        => dma_ctrl_start_l2p,
    wdma_addr_o         => dma_host_addr,
    wdma_done_i         => dma_ctrl_done_i,
    wdma_irq_o          => dma_ctrl_irq(0),

    next_wdma_addr_i    => mwr_addr,
    next_wdma_up_addr_i => mwr_up_addr,
    next_wdma_valid_i   => next_wdma_valid,
    wdma_start_i        => wdma_start,
    wdma_stop_i         => wdma_stop,
    wdma_running_o      => open,
    wdma_frame_len_i    => wdma_frame_len,

    timeframe_end_i     => timeframe_end,

    wdma_buf_ptr_o      => wdma_buf_ptr,
    wdma_status_o       => wdma_status,
    cc_timeout_o        => cc_timeout
);

timeframe_end_p2p : entity work.fofb_cc_p2p
port map (
    in_clk              => fai_clk_i,
    out_clk             => clk_i,
    rst                 => '0',
    pulsein             => timeframe_end_i,
    inbusy              => open,
    pulseout            => timeframe_end
);

--wdma_status <= "0001";
--
--process(clk_i)
--    variable counter    : integer := 0;
--begin
--    if rising_edge(clk_i) then
--        if (wdma_start = '1') then
--            wdma_enabled <= '1';
--        elsif (wdma_stop = '1') then
--            wdma_enabled <= '0';
--        end if;
--
--        if (wdma_enabled = '1') then
--            if (counter = 256 * 10000) then
--                dma_ctrl_irq(0) <= '1';
--                counter := 0;
--            else
--                dma_ctrl_irq(0) <= '0';
--                counter := counter + 1;
--            end if;
--        else
--            counter := 0;
--            dma_ctrl_irq(0) <= '0';
--        end if;
--    end if;
--end process;

------------------------------------------------
-- Chipscope

--trig0(0) <= dma_ctrl_start_l2p;
--trig0(1) <= dma_ctrl_error_i;
--
--data(0) <= dma_ctrl_start_l2p;
--data(1) <= dma_ctrl_done_i;
--data(2) <= dma_ctrl_error_i;
--
--ila_inst : ila
--port map (
--    control         => control,
--    clk             => clk_i,
--    data            => data,
--    trig0           => trig0
--);
--
--icon_inst : icon
--port map (
--    control0        => control
--);

end behaviour;

