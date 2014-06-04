--------------------------------------------------------------------------------
-- Company: 
-- Engineer:
--
-- Create Date:   17:57:45 05/31/2014
-- Design Name:   
-- Module Name:   /home/iu42/hardware/trunk/FPGA/spec_sniffer/sim/spec_si57x/bench/spec_si57x_tb.vhd
-- Project Name:  spec_si57x
-- Target Device:  
-- Tool versions:  
-- Description:   
-- 
-- VHDL Test Bench Created by ISE for module: spec_si57x
-- 
-- Dependencies:
-- 
-- Revision:
-- Revision 0.01 - File Created
-- Additional Comments:
--
-- Notes: 
-- This testbench has been automatically generated using types std_logic and
-- std_logic_vector for the ports of the unit under test.  Xilinx recommends
-- that these types always be used for the top-level I/O of a design in order
-- to guarantee that the testbench will bind correctly to the post-implementation 
-- simulation model.
--------------------------------------------------------------------------------
LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
use ieee.numeric_std.all;
-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
--USE ieee.numeric_std.ALL;
 
ENTITY spec_si57x_tb IS
END spec_si57x_tb;
 
ARCHITECTURE behavior OF spec_si57x_tb IS 

signal clk          : std_logic := '0';
signal reset        : std_logic;
signal i2c_scl      : std_logic;
signal i2c_sda      : std_logic;
signal i2c_done     : std_logic;

procedure CLK_EAT  (
    clock_count             : in integer;
    signal trn_clk          : in std_logic
) is
    variable i  : integer;
begin
    for i in 0 to (clock_count - 1) loop
        wait until (trn_clk'event and trn_clk = '1');
    end loop;
end CLK_EAT;


BEGIN

clk <= not clk after 4 ns;

--------------------------------------------------------------------
-- Initial Reset
--------------------------------------------------------------------
process(clk)
    variable counter : unsigned(3 downto 0) := "0000";
begin
    if rising_edge(clk) then
        if (counter = "1111") then
            reset <= '0';
        else
            reset <= '1';
            counter := counter + 1;
        end if;
    end if;
end process;

uut: entity work.spec_si57x
PORT MAP (
    clk         => clk,
    rst         => reset,
    i2c_done    => i2c_done,
    i2c_scl     => i2c_scl,
    i2c_sda     => i2c_sda
);

adt7411: entity work.i2c_slave_model
generic map (
    I2C_ADR => "1010101"
)
port map (
    scl => i2c_scl,
    sda => i2c_sda
);

i2c_scl  <= 'H';
i2c_sda  <= 'H';

end;
