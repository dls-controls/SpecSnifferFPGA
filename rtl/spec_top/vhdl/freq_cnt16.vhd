-----------------------------------------------------------------
-- Entity freq_cnt16
----------------------------------------------------------------

library ieee;
  use ieee.std_logic_unsigned.all;
  use ieee.std_logic_misc.all;
  use ieee.std_logic_arith.all;
  use ieee.math_real.all;
  use ieee.std_logic_1164.all;

entity freq_cnt16 is
generic (
    NB_CNTR         : integer := 16
);
port (
    -- Clocks Interface
    reset           : in  std_logic;
    reference_clock : in  std_logic;
    test_clocks     : in  std_logic_vector(15 downto 0);
    clock_sel       : in  std_logic_vector(3 downto 0);
    clock_cnt_out   : out std_logic_vector(NB_CNTR - 1 downto 0)
);
end freq_cnt16;

architecture freq_cnt16_syn of freq_cnt16 is

type std2d_nb_cntrb is array(natural range <>) of std_logic_vector(NB_CNTR - 1  downto 0);

signal ref_cntr        : integer range 2**13-1 downto 0;
signal ref_trigger     : std_logic;
signal trigger         : std_logic_vector(NB_CNTR - 1 downto 0);

signal clk_cntr        : std2d_nb_cntrb(NB_CNTR - 1 downto 0);
signal clk_cnt_reg     : std2d_nb_cntrb(NB_CNTR - 1 downto 0);

signal cmd_rst          : std_logic;

component pulse2pulse
port (
  in_clk   : in  std_logic;
  out_clk  : in  std_logic;
  rst      : in  std_logic;
  pulsein  : in  std_logic;
  inbusy   : out std_logic;
  pulseout : out std_logic
);
end component;


begin

-- Reference Counter
process(reset, reference_clock)
begin
  if (reset = '1') then

    ref_cntr    <= 0;
    ref_trigger <= '1';

  elsif (rising_edge(reference_clock)) then

    if (ref_cntr = 2**13-1) then
      ref_cntr    <= 0;
      ref_trigger <= '1';
    else
      ref_cntr    <= ref_cntr + 1;
      ref_trigger <= '0';
    end if;

  end if;
end process;

-- Clock counters

CNTR_GEN : for i in 0 to NB_CNTR - 1 generate

  p2p_trigger_inst : pulse2pulse
  port map (
    in_clk   => reference_clock,
    out_clk  => test_clocks(i),
    rst      => reset,
    pulsein  => ref_trigger,
    inbusy   => open,
    pulseout => trigger(i)
  );

  process(reset, cmd_rst, test_clocks(i))
  begin
    if (reset = '1' or cmd_rst = '1') then

      clk_cntr(i)    <= (others=>'0');
      clk_cnt_reg(i) <= (others=>'0');

    elsif (rising_edge(test_clocks(i))) then

      if (trigger(i) = '1') then
        clk_cntr(i)    <= (others=>'0');
        clk_cnt_reg(i) <= clk_cntr(i);
      else
        clk_cntr(i)    <= clk_cntr(i) + conv_std_logic_vector(1, NB_CNTR);
        clk_cnt_reg(i) <= clk_cnt_reg(i);
      end if;

    end if;
  end process;

end generate;

-- Output MUX
process(clock_sel, clk_cnt_reg)
begin
  clock_cnt_out <= clk_cnt_reg(conv_integer(clock_sel));
end process;

end freq_cnt16_syn;
