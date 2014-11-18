library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library unisim;
use unisim.vcomponents.all;

entity spec_clk_if is
port (
    clk_p_i         : in  std_logic;
    clk_n_i         : in  std_logic;
    clk_o           : out std_logic;
    pcie_rstn_i     : in  std_logic;
    rst_n_o         : out std_logic
);
end spec_clk_if;

architecture rtl of spec_clk_if is

signal pllclkin         : std_logic;
signal pllclkfb         : std_logic;
signal pllclkout0       : std_logic;
signal clk              : std_logic;

begin
fpga_clk_buf : IBUFGDS
generic map (
    DIFF_TERM               => TRUE,
    IBUF_LOW_PWR            => TRUE,
    IOSTANDARD              => "DEFAULT"
)
port map (
    O                       => clk_o,
    I                       => clk_p_i,
    IB                      => clk_n_i
);

rst_n_o <= pcie_rstn_i;


---- Output system clock
--clk_o <= clk;
--
----------------------------------------------------------------------
---- Differential clock input buffer from Si570 125MHz fixed reference
----------------------------------------------------------------------
--fpga_clk_buf : IBUFGDS
--generic map (
--    DIFF_TERM               => TRUE,
--    IBUF_LOW_PWR            => TRUE,
--    IOSTANDARD              => "DEFAULT"
--)
--port map (
--    O                       => pllclkin,
--    I                       => clk_p_i,
--    IB                      => clk_n_i
--);
--
--------------------------------------------------------------------------------
---- PLL System clock generator (62.5 MHz Output from 125 MHz input) 
--------------------------------------------------------------------------------
--cmp_sys_clk_pll : PLL_BASE
--generic map (
--    BANDWIDTH               => "OPTIMIZED",
--    CLK_FEEDBACK            => "CLKFBOUT",
--    COMPENSATION            => "INTERNAL",
--    DIVCLK_DIVIDE           => 1,
--    CLKFBOUT_MULT           => 8,
--    CLKFBOUT_PHASE          => 0.000,
--    CLKOUT0_DIVIDE          => 16,         -- 62.5 MHz
--    CLKOUT0_PHASE           => 0.000,
--    CLKOUT0_DUTY_CYCLE      => 0.500,
--    CLKOUT1_DIVIDE          => 16,
--    CLKOUT1_PHASE           => 0.000,
--    CLKOUT1_DUTY_CYCLE      => 0.500,
--    CLKOUT2_DIVIDE          => 16,
--    CLKOUT2_PHASE           => 0.000,
--    CLKOUT2_DUTY_CYCLE      => 0.500,
--    CLKIN_PERIOD            => 8.0,
--    REF_JITTER              => 0.016)
--port map (
--    CLKFBOUT                => pllclkfb,
--    CLKOUT0                 => pllclkout0,
--    CLKOUT1                 => open,
--    CLKOUT2                 => open,
--    CLKOUT3                 => open,
--    CLKOUT4                 => open,
--    CLKOUT5                 => open,
--    LOCKED                  => rst_n_o,
--    RST                     => not pcie_rstn_i,
--    CLKFBIN                 => pllclkfb,
--    CLKIN                   => pllclkin
--);
--
--cmp_clk_sys_buf : BUFG
--port map (
--    O                       => clk,
--    I                       => pllclkout0
--);

end rtl;

