library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library unisim;
use unisim.vcomponents.all;

library work;

entity spec_top is
port (
    -- CC differential clock
    si57x_clk_p                   : in  std_logic;
    si57x_clk_n                   : in  std_logic;
    -- 125MHz FPGA Clock inputs
    fpga_clk_p                  : in  std_logic;
    fpga_clk_n                  : in  std_logic;
    -- Si570 I2C interface
    si57x_oe                    : out std_logic;
    si57x_sda                   : inout std_logic;
    si57x_scl                   : inout std_logic;
    -- SFP Interface
    sfp_ena                     : out std_logic;
    sfp_txp                     : out std_logic_vector(1 downto 0);
    sfp_txn                     : out std_logic_vector(1 downto 0);
    sfp_rxp                     : in  std_logic_vector(1 downto 0);
    sfp_rxn                     : in  std_logic_vector(1 downto 0)
);
end entity spec_top;

architecture rtl of spec_top is

--------------------------------------------------------------------
-- Chipscope Definitions
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
-- Design Signals
--------------------------------------------------------------------
signal control              : std_logic_vector(35 downto 0);
signal trig0                : std_logic_vector(7 downto 0);
signal data                 : std_logic_vector(127 downto 0);

signal fpga_clk             : std_logic;
signal si57x_clk            : std_logic;
signal reset                : std_logic;

signal freq_counter         : unsigned(31 downto 0);
signal i2c_done             : std_logic;

begin

--------------------------------------------------------------------
-- Fixed output ports
--------------------------------------------------------------------
si57x_oe <= '1';    -- Output Enable to Si57x
sfp_ena <= '0';     -- Output Enable to SFP

--------------------------------------------------------------------
-- Differential clock input buffer from Si570 125MHz fixed reference
--------------------------------------------------------------------
fpga_clk_buf : IBUFGDS
generic map (
    DIFF_TERM       => TRUE,
    IBUF_LOW_PWR    => TRUE,
    IOSTANDARD      => "DEFAULT"
)
port map (
    O               => fpga_clk,
    I               => fpga_clk_p,
    IB              => fpga_clk_n
);

--------------------------------------------------------------------
-- Power-on Reset on fpga_clk domain
--------------------------------------------------------------------
process(fpga_clk)
    variable counter : unsigned(3 downto 0) := "0000";
begin
    if rising_edge(fpga_clk) then
        if (counter = "1111") then
            reset <= '0';
        else
            reset <= '1';
            counter := counter + 1;
        end if;
    end if;
end process;

--------------------------------------------------------------------
-- I2C module for configuring Si570 to generate 106.25MHz GTPA clock
--------------------------------------------------------------------
spec_si57x_inst : entity work.spec_si57x
port map (
    clk             => fpga_clk,
    rst             => reset,
    i2c_scl         => si57x_scl,
    i2c_sda         => si57x_sda,
    i2c_done        => i2c_done
);

-------------------------------------------------------------------
-- Communication Controller instantiation
--------------------------------------------------------------------
fofb_cc_top_wrapper : entity work.fofb_cc_top_wrapper
port map (
    refclk_p_i             => si57x_clk_p,
    refclk_n_i             => si57x_clk_n,

    sysclk_i               => '0',
    sysreset_n_i           => i2c_done,

    fai_cfg_a_o            => open,
    fai_cfg_d_o            => open,
    fai_cfg_d_i            => X"00000000",
    fai_cfg_we_o           => open,
    fai_cfg_clk_o          => open,
    fai_cfg_val_i          => X"00000008",


    fai_rio_rdp_i          => sfp_rxp,
    fai_rio_rdn_i          => sfp_rxn,
    fai_rio_tdp_o          => sfp_txp,
    fai_rio_tdn_o          => sfp_txn,

    xy_buf_addr_i          => (others => '0'),
    xy_buf_dat_o           => open,
    xy_buf_rstb_i          => '0',
    timeframe_end_rise_o   => open,

    fofb_dma_ok_i          => '1'
);

--------------------------------------------------------------------
-- Chipscope Instantiation
--------------------------------------------------------------------
----trig0(0)          <= sec_pulse_synced;
----
----data(31 downto 0) <= std_logic_vector(freq_counter);
----data(32)          <= sec_pulse_synced;
----
----ila_inst : ila
----port map (
----    control         => control,
----    clk             => si57x_clk,
----    data            => data,
----    trig0           => trig0
----);
----
----icon_inst : icon
----port map (
----    control0        => control
----);


end architecture rtl;
