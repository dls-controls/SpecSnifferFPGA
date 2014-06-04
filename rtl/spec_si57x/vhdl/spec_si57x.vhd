--------------------------------------------------------------------
-- This module programs Si57x over I2C bus to generate 106.25MHz
-- reference clock to Spartan-6 GTPA block.
-- It uses Wishbone-based code from 4DSP.
--------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library unisim;
use unisim.vcomponents.all;

entity spec_si57x is
generic (
    -- I2C R/W Trigger Clock in usec
    I2C_TRIG_PERIOD     : integer := 1000
);
port (
    clk                 : in  std_logic;
    rst                 : in  std_logic;
    i2c_done            : out std_logic;
    i2c_scl             : inout std_logic;
    i2c_sda             : inout std_logic
);
end spec_si57x;

architecture rtl of spec_si57x is

-- Convert I2C R/W Trigger Clock to ticks
constant I2C_TRIG_PRD   : integer := 125 * I2C_TRIG_PERIOD;
-- Play Memory depth
constant PLAY_SIZE      : integer := 9;

type i2c_type is array (natural range <>) of std_logic_vector(15 downto 0);

--------------------------------------------------------------------
-- I2C Read/Write operations are played from memory,
-- Memory Content stores address and value in 16-bit format.
-- bits[15:8] => Register address
-- bits[ 7:0] => Register value to be written
--------------------------------------------------------------------
constant i2c_mem : i2c_type(0 to PLAY_SIZE-1) := (
                                            X"8910",
                                            X"0741",
                                            X"08C2",
                                            X"09CA",
                                            X"0A01",
                                            X"0BCD",
                                            X"0C25",
                                            X"8900",
                                            X"8740");

signal cmdin            : std_logic_vector(63 downto 0);
signal cmdin_val        : std_logic;
signal cmdout           : std_logic_vector(63 downto 0);
signal cmdout_val       : std_logic;
signal i2c_trig         : std_logic;

signal scl_i            : std_logic;
signal scl_o            : std_logic;
signal scl_oe           : std_logic;
signal sda_i            : std_logic;
signal sda_o            : std_logic;
signal sda_oe           : std_logic;

signal i2c_play         : std_logic;
signal i2c_addr         : integer range 0 to PLAY_SIZE-1;
signal i2c_done_n       : std_logic;

begin

--------------------------------------------------------------------
-- Wishbone-based I2C Core from 4DSP
--------------------------------------------------------------------
i2c_master_inst : entity work.i2c_master
port map (
  rst             => rst,
  clk_cmd         => clk,
  in_cmd_val      => cmdin_val,
  in_cmd          => cmdin,
  out_cmd_val     => cmdout_val,
  out_cmd         => cmdout,
  in_cmd_busy     => open,

  scl_i           => scl_i,
  scl_o           => scl_o,
  scl_oe          => scl_oe,
  sda_i           => sda_i,
  sda_o           => sda_o,
  sda_oe          => sda_oe
);

--------------------------------------------------------------------
-- 3-state I2C I/O Buffers
--------------------------------------------------------------------
iobuf_scl : iobuf
port map (
  I  => '0',
  O  => scl_i,
  IO => i2c_scl,
  T  => scl_oe
);

iobuf_sda : iobuf
port map (
  I  => '0',
  O  => sda_i,
  IO => i2c_sda,
  T  => sda_oe
);

--------------------------------------------------------------------
-- I2C Read/Write Trigger Generator
--------------------------------------------------------------------
process(clk)
    variable counter : integer range 0 to I2C_TRIG_PRD-1;
begin
    if rising_edge(clk) then
        if (rst = '1') then
            i2c_trig <= '0';
            counter := 0;
        else
            if (counter = I2C_TRIG_PRD-1) then
                i2c_trig <= '1';
                counter := 0;
            else
                i2c_trig <= '0';
                counter := counter + 1;
            end if;
        end if;
    end if;
end process;

--------------------------------------------------------------------
-- Si570 I2C CONFIG Memory Address Generator
-- Play enable store is generated only once to prevent
-- continious R/W operation on I2C
--------------------------------------------------------------------
process(clk)
begin
    if rising_edge(clk) then
        if (rst = '1') then
            i2c_addr <= 0;
            i2c_play <= '1';
        else
            if (i2c_trig = '1') then
                if (i2c_addr = PLAY_SIZE-1) then
                    i2c_addr <= i2c_addr;
                    i2c_play <= '0';
                else
                    i2c_addr <= i2c_addr + 1;
                end if;
            end if;
        end if;
    end if;
end process;

--------------------------------------------------------------------
-- Construct CMD data to i2c_master block
-- [63..60] = command 1/W, 2/R
-- [59..32] = address
-- [31.. 0] = data
--------------------------------------------------------------------
cmdin_val <= i2c_trig and i2c_play;

cmdin <= X"1"      &
         X"00055"  & i2c_mem(i2c_addr)(15 downto 8) &
         X"000000" & i2c_mem(i2c_addr)(7 downto 0);

--------------------------------------------------------------------
-- Assert Done Output After ~15ms as required by si57x spec
--------------------------------------------------------------------
i2c_done_delay : SRL16E
generic map (
    INIT    => X"FFFF"
)
port map (
    D       => i2c_play,
    CE      => i2c_trig,
    CLK     => clk,
    A0      => '1',
    A1      => '1',
    A2      => '1',
    A3      => '1',
    Q       => i2c_done_n
);

i2c_done <= not i2c_done_n;

end rtl;
