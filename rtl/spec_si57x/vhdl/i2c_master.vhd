----------------------------------------------------------------------------------------------------
library ieee;
  use ieee.std_logic_1164.all;
  use ieee.std_logic_arith.all;
  use ieee.std_logic_misc.all;
  use ieee.std_logic_unsigned.all;

entity i2c_master is
generic (
  START_ADDR      : std_logic_vector(27 downto 0) := x"0000000";
  STOP_ADDR       : std_logic_vector(27 downto 0) := x"000FFFF"
);
port (
  rst             : in  std_logic;
  -- Command Interface
  clk_cmd         : in  std_logic;
  in_cmd_val      : in  std_logic;
  in_cmd          : in  std_logic_vector(63 downto 0);
  out_cmd_val     : out std_logic;
  out_cmd         : out std_logic_vector(63 downto 0);
  in_cmd_busy     : out std_logic;
  -- SPI control
  scl_i           : in  std_logic;
  scl_o           : out std_logic;
  scl_oe          : out std_logic;
  sda_i           : in  std_logic;
  sda_o           : out std_logic;
  sda_oe          : out std_logic
);
end i2c_master;

architecture i2c_master_syn of i2c_master is

component i2c_master_stellar_cmd is
generic
(
  START_ADDR           : std_logic_vector(27 downto 0) := x"0000000";
  STOP_ADDR            : std_logic_vector(27 downto 0) := x"00000FF"
);
port
(
  reset                : in  std_logic;
  -- Command Interface
  clk_cmd              : in  std_logic;                     --cmd_in and cmd_out are synchronous to this clock;
  out_cmd              : out std_logic_vector(63 downto 0);
  out_cmd_val          : out std_logic;
  in_cmd               : in  std_logic_vector(63 downto 0);
  in_cmd_val           : in  std_logic;
  -- Register interface
  clk_reg              : in  std_logic;                     --register interface is synchronous to this clock
  out_reg              : out std_logic_vector(31 downto 0); --caries the out register data
  out_reg_val          : out std_logic;                     --the out_reg has valid data  (pulse)
  out_reg_addr         : out std_logic_vector(27 downto 0); --out register address
  in_reg               : in  std_logic_vector(31 downto 0); --requested register data is placed on this bus
  in_reg_val           : in  std_logic;                     --pulse to indicate requested register is valid
  in_reg_req           : out std_logic;                     --pulse to request data
  in_reg_addr          : out std_logic_vector(27 downto 0);  --requested address
  --mailbox interface
  mbx_in_reg           : in  std_logic_vector(31 downto 0); --value of the mailbox to send
  mbx_in_val           : in  std_logic                      --pulse to indicate mailbox is valid
);
end component;

component i2c_master_top is
generic (
  ARST_LVL : std_logic := '0'                       -- asynchronous reset level
);
port (
  -- wishbone signals
  wb_clk_i      : in  std_logic;                    -- master clock input
  wb_rst_i      : in  std_logic := '0';             -- synchronous active high reset
  arst_i        : in  std_logic := not ARST_LVL;    -- asynchronous reset
  wb_adr_i      : in  std_logic_vector(2 downto 0); -- lower address bits
  wb_dat_i      : in  std_logic_vector(7 downto 0); -- Databus input
  wb_dat_o      : out std_logic_vector(7 downto 0); -- Databus output
  wb_we_i       : in  std_logic;                    -- Write enable input
  wb_stb_i      : in  std_logic;                    -- Strobe signals / core select signal
  wb_cyc_i      : in  std_logic;                    -- Valid bus cycle input
  wb_ack_o      : out std_logic;                    -- Bus cycle acknowledge output
  wb_inta_o     : out std_logic;                    -- interrupt request output signal

  -- i2c lines
  scl_pad_i     : in  std_logic;                    -- i2c clock line input
  scl_pad_o     : out std_logic;                    -- i2c clock line output
  scl_padoen_o  : out std_logic;                    -- i2c clock line output enable, active low
  sda_pad_i     : in  std_logic;                    -- i2c data line input
  sda_pad_o     : out std_logic;                    -- i2c data line output
  sda_padoen_o  : out std_logic                     -- i2c data line output enable, active low
);
end component i2c_master_top;

-----------------------------------------------------------------------------------
-- Constant declarations
-----------------------------------------------------------------------------------
constant ADDR_PRER         : std_logic_vector(27 downto 0) := x"0000000";
constant ADDR_BYTE         : std_logic_vector(27 downto 0) := x"0000001";

-- I2C to SPI bridge: Slave address
--constant I2C_SLAVE_ADDR    : std_logic_vector(7 downto 1) := "1010" & A;

-- I2C core: Register Map
constant I2S_PRERLO_ADR    : std_logic_vector(2 downto 0) := "000"; -- RW
constant I2S_PRERHI_ADR    : std_logic_vector(2 downto 0) := "001"; -- RW
constant I2S_CTR_ADR       : std_logic_vector(2 downto 0) := "010"; -- RW
constant I2S_TXR_ADR       : std_logic_vector(2 downto 0) := "011"; -- W
constant I2S_RXR_ADR       : std_logic_vector(2 downto 0) := "011"; -- R
constant I2S_CR_ADR        : std_logic_vector(2 downto 0) := "100"; -- W
constant I2S_SR_ADR        : std_logic_vector(2 downto 0) := "100"; -- R

-- I2C core: Control register bit map
constant I2S_CTR_EN        : integer := 7; -- I2C core enable
constant I2S_CTR_IEN       : integer := 6; -- I2C core interrupt enable

-- I2C core: Command register bit map
constant I2S_CR_STA        : integer := 7; -- Generate (repeated) start condition
constant I2S_CR_STO        : integer := 6; -- Generate stop condition
constant I2S_CR_RD         : integer := 5; -- Read from slave
constant I2S_CR_WR         : integer := 4; -- Write to slave
constant I2S_CR_ACK        : integer := 3; -- When a receiver, sent ACK (ACK=0) or NACK (ACK=1)
constant I2S_CR_IACK       : integer := 0; -- Interrupt acknowledge. When set, clears a pending interrupt.

-- I2C core: Status register bit map
constant I2S_SR_RXACK      : integer := 7; -- Received acknowledge from slave
constant I2S_SR_BUSY       : integer := 6; -- I2C bus busy
constant I2S_SR_AL         : integer := 5; -- Arbitration lost
constant I2S_SR_TIP        : integer := 1; -- Transfer in progress
constant I2S_SR_IF         : integer := 0; -- Interrupt Flag

-- I2C core: Most common command register combinations
constant I2S_START_RD      : std_logic_vector(7 downto 0) := x"69";
constant I2S_START_ADR_WR  : std_logic_vector(7 downto 0) := x"91";
constant I2S_START_SUB_WR  : std_logic_vector(7 downto 0) := x"11";
constant I2S_START_SUB_RD  : std_logic_vector(7 downto 0) := x"51";
constant I2S_START_DATA_WR : std_logic_vector(7 downto 0) := x"51";

-- I2C core: Default power up PRER register value (I2C = 100kHz)
constant DEFAULT_PRER : std_logic_vector(15 downto 0) := x"00F9";

-----------------------------------------------------------------------------------
-- Signal declarations
-----------------------------------------------------------------------------------

type wb_states is (
  init_prerlo,
  init_prerhi,
  init_ctr,
  idle,
  wr_sl_adr,
  wr_sl_adr_ack,
  wr_func_id,
  wr_func_id_ack,
  wr_data,
  wr_data_ack,
  wr_data_stop,
  rd_sl_adr,
  rd_sl_adr_ack,
  rd_data,
  rd_data_ack,
  rd_data_stop,
  rd_data_val,
  nop,
  wait_irq,
  wait_irq_l,
  ack_received
);
signal wb_state       : wb_states;
signal st_state       : wb_states; -- stored state

signal out_reg_val    : std_logic;
signal out_reg_addr   : std_logic_vector(27 downto 0);
signal out_reg        : std_logic_vector(31 downto 0);

signal in_reg_req     : std_logic;
signal in_reg_addr    : std_logic_vector(27 downto 0);
signal in_reg_val     : std_logic;
signal in_reg         : std_logic_vector(31 downto 0);

signal inst_val       : std_logic;
signal inst_rw        : std_logic;
signal slave_addr     : std_logic_vector(6 downto 0);
signal func_id        : std_logic_vector(7 downto 0);

signal data_cnt       : integer range 0 to 7;
signal byte_cnt       : integer range 0 to 7;
type data_array is array(7 downto 0) of std_logic_vector(7 downto 0);
signal data_reg       : data_array;
signal read_reg       : std_logic_vector(31 downto 0);

signal init           : std_logic;
signal prer           : std_logic_vector(15 downto 0);
signal byte           : std_logic_vector(1 downto 0);
signal mask           : std_logic_vector(31 downto 0);

signal wb_adr_i       : std_logic_vector(2 downto 0);
signal wb_dat_i       : std_logic_vector(7 downto 0);
signal wb_dat_o       : std_logic_vector(7 downto 0);
signal wb_we          : std_logic;
signal wb_stb         : std_logic;
signal wb_cyc         : std_logic;
signal wb_ack         : std_logic;
signal wb_inta        : std_logic;

begin

----------------------------------------------------------------------------------------------------
-- Stellar Command Interface
----------------------------------------------------------------------------------------------------
i2c_master_stellar_cmd_inst : i2c_master_stellar_cmd
generic map
(
  START_ADDR   => START_ADDR,
  STOP_ADDR    => STOP_ADDR
)
port map
(
  reset        => rst,

  clk_cmd      => clk_cmd,
  in_cmd_val   => in_cmd_val,
  in_cmd       => in_cmd,
  out_cmd_val  => out_cmd_val,
  out_cmd      => out_cmd,

  clk_reg      => clk_cmd,
  out_reg_val  => out_reg_val,
  out_reg_addr => out_reg_addr,
  out_reg      => out_reg,

  in_reg_req   => in_reg_req,
  in_reg_addr  => in_reg_addr,
  in_reg_val   => in_reg_val,
  in_reg       => in_reg,

  mbx_in_val   => '0',
  mbx_in_reg   => (others => '0')
);

----------------------------------------------------------------------------------------------------
-- Shoot commands to the state machine
----------------------------------------------------------------------------------------------------
process (rst, clk_cmd)
begin
  if (rst = '1') then

    init           <= '0';
    prer           <= DEFAULT_PRER;
    byte           <= "00";
    inst_val       <= '0';
    inst_rw        <= '0';
    slave_addr     <= (others => '0');
    func_id        <= (others => '0');
    data_reg       <= (others => (others => '0'));
    read_reg       <= (others => '0');
    in_reg_val     <= '0';
    in_reg         <= (others => '0');

  elsif (rising_edge(clk_cmd)) then

    --Write access to the I2C master
    if (out_reg_val = '1' and out_reg_addr = ADDR_PRER) then
      init <= '1';
      prer <= out_reg(15 downto 0);
    else
      init <= '0';
      prer <= prer;
    end if;

    --Write access to the I2C master
    if (out_reg_val = '1' and out_reg_addr = ADDR_BYTE) then
      byte <= out_reg(1 downto 0);
    end if;

    --Write access to I2C
    if (out_reg_val = '1' and out_reg_addr > ADDR_BYTE) then
      inst_val    <= '1';
      inst_rw     <= '0';
      data_cnt    <= conv_integer(byte)+1;
      slave_addr  <= out_reg_addr(14 downto 8);
      func_id     <= out_reg_addr(7 downto 0);
      data_reg(0) <= out_reg(07 downto 00);
      data_reg(1) <= out_reg(15 downto 08);
      data_reg(2) <= out_reg(23 downto 16);
      data_reg(3) <= out_reg(31 downto 24);
    --Read access to I2C
    elsif (in_reg_req = '1' and in_reg_addr > ADDR_BYTE) then
      inst_val    <= '1';
      inst_rw     <= '1';
      data_cnt    <= conv_integer(byte)+1;
      slave_addr  <= in_reg_addr(14 downto 8);
      func_id     <= in_reg_addr(7 downto 0);
      data_reg(0) <= x"FF";
      data_reg(1) <= x"FF";
      data_reg(2) <= x"FF";
      data_reg(3) <= x"FF";
    else
      inst_val    <= '0';
    end if;

    --Shift data register every byte coming from I2C
    if (wb_adr_i = I2S_RXR_ADR and wb_we = '1' and wb_ack = '1') then
      read_reg <= read_reg(23 downto 0) & wb_dat_o;
    end if;

    --Read response
    if (wb_state = rd_data_val) then
      in_reg_val <= '1';
    else
      in_reg_val <= '0';
    end if;

    if (in_reg_addr = ADDR_PRER) then
      in_reg <= x"00000000" + prer;
    elsif (in_reg_addr = ADDR_BYTE) then
      in_reg <= x"00000000" + byte;
    else
      in_reg <= read_reg and mask;
    end if;

  end if;
end process;

mask <= x"FFFFFFFF" when byte = "11" else
        x"00FFFFFF" when byte = "10" else
        x"0000FFFF" when byte = "01" else
        x"000000FF";

----------------------------------------------------------------------------------------------------
-- Serial interface state-machine
----------------------------------------------------------------------------------------------------
process (rst, clk_cmd)
begin
  if (rst = '1') then

    wb_state <= init_prerlo;
    st_state <= init_prerlo;

  elsif (rising_edge(clk_cmd)) then

    -- Main state machine
    case wb_state is

      when init_prerlo =>
        if (wb_ack = '1') then
          wb_state <= nop;
        end if;
        st_state <= init_prerhi;

      when init_prerhi =>
        if (wb_ack = '1') then
          wb_state <= nop;
        end if;
        st_state <= init_ctr;

      when init_ctr =>
        if (wb_ack = '1') then
          wb_state <= nop;
        end if;
        st_state <= idle;

      when idle =>
        byte_cnt <= 0;
        if (init = '1') then
          wb_state <= init_prerlo;
        elsif (inst_val = '1' and inst_rw = '0') then
          wb_state <= wr_sl_adr;
        -- on a read access to a SPI slave is executed as a I2C write, followed by a I2C read
        elsif (inst_val = '1' and inst_rw = '1') then
          wb_state <= wr_sl_adr;
        end if;
        st_state <= idle;

      --Writing
      when wr_sl_adr =>
        if (wb_ack = '1') then
          wb_state <= nop;
        end if;
        st_state <= wr_sl_adr_ack;

      when wr_sl_adr_ack =>
        if (wb_ack = '1') then
          wb_state <= wait_irq;
        end if;
        st_state <= wr_func_id;

      when wr_func_id =>
        if (wb_ack = '1') then
          wb_state <= nop;
        end if;
        st_state <= wr_func_id_ack;

      when wr_func_id_ack =>
        if (wb_ack = '1') then
          wb_state <= wait_irq;
        end if;
        if (inst_rw = '0') then
          st_state <= wr_data;
        else
          st_state <= rd_sl_adr;
        end if;

      when wr_data =>
        if (wb_ack = '1') then
          byte_cnt <= byte_cnt + 1;
          if (byte_cnt = data_cnt - 1) then
            wb_state <= nop;
            st_state <= wr_data_stop;
          else
            wb_state <= nop;
            st_state <= wr_data_ack;
          end if;
        end if;

      when wr_data_ack =>
        if (wb_ack = '1') then
          wb_state <= wait_irq;
        end if;
        st_state <= wr_data;

      when wr_data_stop =>
        byte_cnt <= 0;
        if (wb_ack = '1') then
          wb_state <= wait_irq;
        end if;
        st_state <= idle;

      --Reading
      when rd_sl_adr =>
        if (wb_ack = '1') then
          wb_state <= nop;
        end if;
        st_state <= rd_sl_adr_ack;

      when rd_sl_adr_ack =>
        if (wb_ack = '1') then
          wb_state <= wait_irq;
        end if;
        st_state <= rd_data;

      when rd_data =>
        if (wb_ack = '1') then
          byte_cnt <= byte_cnt + 1;
          if (byte_cnt = data_cnt) then
            wb_state <= rd_data_val;
            st_state <= idle;
          elsif (byte_cnt = data_cnt - 1) then
            wb_state <= nop;
            st_state <= rd_data_stop;
          else
            wb_state <= nop;
            st_state <= rd_data_ack;
          end if;
         end if;

      when rd_data_ack =>
        if (wb_ack = '1') then
          wb_state <= wait_irq;
        end if;
        st_state <= rd_data;

      when rd_data_stop =>
        if (wb_ack = '1') then
          wb_state <= wait_irq;
        end if;
        st_state <= rd_data;

      when rd_data_val =>
        wb_state <= st_state;

      when nop =>
        wb_state <= st_state;

      when wait_irq =>
        --if (wb_dat_o(I2S_SR_TIP) = '1') then
        if (wb_inta = '0') then
          wb_state <= wait_irq_l;
        end if;

      when wait_irq_l =>
        --if (wb_dat_o(I2S_SR_TIP) = '0') then
        if (wb_inta = '1') then
          wb_state <= ack_received;
        end if;

      -- Check if ACK has been received from slave
      when ack_received =>
        --if ack is received go to the stored (next) state
        if (wb_dat_o(I2S_SR_RXACK) = '0') then
          wb_state <= st_state;
        --if no ack is received after sending the slave address, retry (ready polling)
        elsif (st_state = wr_func_id) then
          wb_state <= wr_sl_adr_ack;
        --if no ack is received after sending the slave address, retry (ready polling)
        elsif (st_state = rd_data) then
          if (byte_cnt = 0) then
            wb_state <= rd_sl_adr_ack;
          elsif (byte_cnt = data_cnt) then -- last byte is never acked
            wb_state <= st_state;
          end if;
        --if no ack is received and polling is not applicable got idle (=error)
        else
          wb_state <= idle;
        end if;

      when others =>
        wb_state <= nop;
        st_state <= init_prerlo;

    end case;

  end if;
end process;

in_cmd_busy <= '0' when (wb_state = idle) else '1';

----------------------------------------------------------------------------------------------------
-- Serial interface state-machine
----------------------------------------------------------------------------------------------------
process (rst, clk_cmd)

  variable wb_adr : std_logic_vector(2 downto 0);
  variable wb_dat : std_logic_vector(7 downto 0);

begin
  if (rst = '1') then

    wb_adr_i <= (others => '0');
    wb_dat_i <= (others => '0');
    wb_we    <= '0';
    wb_stb   <= '0';
    wb_cyc   <= '0';

  elsif (rising_edge(clk_cmd)) then

    wb_adr := (others => '0');
    wb_dat := (others => '0');

    -- Main state machine
    case wb_state is

      --Init
      when init_prerlo =>
                                  wb_cyc              <= not wb_ack;
                                  wb_stb              <= not wb_ack;
                                  wb_we               <= not wb_ack;
                                  wb_adr              := I2S_PRERLO_ADR;
                                  wb_dat              := prer(7 downto 0);

      when init_prerhi =>
                                  wb_cyc              <= not wb_ack;
                                  wb_stb              <= not wb_ack;
                                  wb_we               <= not wb_ack;
                                  wb_adr              := I2S_PRERHI_ADR;
                                  wb_dat              := prer(15 downto 8);

      when init_ctr =>
                                  wb_cyc              <= not wb_ack;
                                  wb_stb              <= not wb_ack;
                                  wb_we               <= not wb_ack;
                                  wb_adr              := I2S_CTR_ADR;
                                  wb_dat(I2S_CTR_EN)  := '1';
                                  wb_dat(I2S_CTR_IEN) := '1';
      --Writing
      when wr_sl_adr =>
                                  wb_cyc              <= not wb_ack;
                                  wb_stb              <= not wb_ack;
                                  wb_we               <= not wb_ack;
                                  wb_adr              := I2S_TXR_ADR;
                                  wb_dat              := slave_addr & '0';

      when wr_sl_adr_ack =>
                                  wb_cyc              <= not wb_ack;
                                  wb_stb              <= not wb_ack;
                                  wb_we               <= not wb_ack;
                                  wb_adr              := I2S_CR_ADR;
                                  wb_dat(I2S_CR_STA)  := '1';
                                  wb_dat(I2S_CR_STO)  := '0';
                                  wb_dat(I2S_CR_RD)   := '0';
                                  wb_dat(I2S_CR_WR)   := '1';
                                  wb_dat(I2S_CR_ACK)  := '0';
                                  wb_dat(I2S_CR_IACK) := '1';

      when wr_func_id =>
                                  wb_cyc              <= not wb_ack;
                                  wb_stb              <= not wb_ack;
                                  wb_we               <= not wb_ack;
                                  wb_adr              := I2S_TXR_ADR;
                                  wb_dat              := func_id;

      when wr_func_id_ack =>
                                  wb_cyc              <= not wb_ack;
                                  wb_stb              <= not wb_ack;
                                  wb_we               <= not wb_ack;
                                  wb_adr              := I2S_CR_ADR;
                                  wb_dat(I2S_CR_STA)  := '0';
                                  wb_dat(I2S_CR_STO)  := '0';
                                  wb_dat(I2S_CR_RD)   := '0';
                                  wb_dat(I2S_CR_WR)   := '1';
                                  wb_dat(I2S_CR_ACK)  := '0';
                                  wb_dat(I2S_CR_IACK) := '1';

      when wr_data =>
                                  wb_cyc              <= not wb_ack;
                                  wb_stb              <= not wb_ack;
                                  wb_we               <= not wb_ack;
                                  wb_adr              := I2S_TXR_ADR;
                                  wb_dat              := data_reg(byte_cnt);

      when wr_data_ack =>
                                  wb_cyc              <= not wb_ack;
                                  wb_stb              <= not wb_ack;
                                  wb_we               <= not wb_ack;
                                  wb_adr              := I2S_CR_ADR;
                                  wb_dat(I2S_CR_STA)  := '0';
                                  wb_dat(I2S_CR_STO)  := '0';
                                  wb_dat(I2S_CR_RD)   := '0';
                                  wb_dat(I2S_CR_WR)   := '1';
                                  wb_dat(I2S_CR_ACK)  := '0';
                                  wb_dat(I2S_CR_IACK) := '1';

      when wr_data_stop =>
                                  wb_cyc              <= not wb_ack;
                                  wb_stb              <= not wb_ack;
                                  wb_we               <= not wb_ack;
                                  wb_adr              := I2S_CR_ADR;
                                  wb_dat(I2S_CR_STA)  := '0';
                                  wb_dat(I2S_CR_STO)  := '1';
                                  wb_dat(I2S_CR_RD)   := '0';
                                  wb_dat(I2S_CR_WR)   := '1';
                                  wb_dat(I2S_CR_ACK)  := '0';
                                  wb_dat(I2S_CR_IACK) := '1';
      --Reading
      when rd_sl_adr =>
                                  wb_cyc              <= not wb_ack;
                                  wb_stb              <= not wb_ack;
                                  wb_we               <= not wb_ack;
                                  wb_adr              := I2S_TXR_ADR;
                                  wb_dat              := slave_addr & '1';

      when rd_sl_adr_ack =>
                                  wb_cyc              <= not wb_ack;
                                  wb_stb              <= not wb_ack;
                                  wb_we               <= not wb_ack;
                                  wb_adr              := I2S_CR_ADR;
                                  wb_dat(I2S_CR_STA)  := '1';
                                  wb_dat(I2S_CR_STO)  := '0';
                                  wb_dat(I2S_CR_RD)   := '0';
                                  wb_dat(I2S_CR_WR)   := '1';
                                  wb_dat(I2S_CR_ACK)  := '0';
                                  wb_dat(I2S_CR_IACK) := '1';

      when rd_data =>
                                  wb_cyc              <= not wb_ack;
                                  wb_stb              <= not wb_ack;
                                  wb_we               <= not wb_ack;
                                  wb_adr              := I2S_RXR_ADR;
                                  wb_dat              := x"00";

      when rd_data_ack =>
                                  wb_cyc              <= not wb_ack;
                                  wb_stb              <= not wb_ack;
                                  wb_we               <= not wb_ack;
                                  wb_adr              := I2S_CR_ADR;
                                  wb_dat(I2S_CR_STA)  := '0';
                                  wb_dat(I2S_CR_STO)  := '0';
                                  wb_dat(I2S_CR_RD)   := '1';
                                  wb_dat(I2S_CR_WR)   := '0';
                                  wb_dat(I2S_CR_ACK)  := '0';
                                  wb_dat(I2S_CR_IACK) := '1';

      when rd_data_stop =>
                                  wb_cyc              <= not wb_ack;
                                  wb_stb              <= not wb_ack;
                                  wb_we               <= not wb_ack;
                                  wb_adr              := I2S_CR_ADR;
                                  wb_dat(I2S_CR_STA)  := '0';
                                  wb_dat(I2S_CR_STO)  := '1';
                                  wb_dat(I2S_CR_RD)   := '1';
                                  wb_dat(I2S_CR_WR)   := '0';
                                  wb_dat(I2S_CR_ACK)  := '1'; --no ACK
                                  wb_dat(I2S_CR_IACK) := '1';
      --Other states
      when others =>
                                  wb_cyc              <= '0';
                                  wb_stb              <= '0';
                                  wb_we               <= '0';
                                  wb_adr              := I2S_SR_ADR;

    end case;

    wb_adr_i <= wb_adr;
    wb_dat_i <= wb_dat;

  end if;
end process;

----------------------------------------------------------------------------------------------------
-- Wishbone I2C core
----------------------------------------------------------------------------------------------------
i2c_master_top_inst : i2c_master_top
generic map (
  ARST_LVL      => '1'
)
port map (
  wb_clk_i      => clk_cmd     , --: in  std_logic;                    -- master clock input
  wb_rst_i      => '0'         , --: in  std_logic := '0';             -- synchronous active high reset
  arst_i        => rst         , --: in  std_logic := not ARST_LVL;    -- asynchronous reset
  wb_adr_i      => wb_adr_i    , --: in  std_logic_vector(2 downto 0); -- lower address bits
  wb_dat_i      => wb_dat_i    , --: in  std_logic_vector(7 downto 0); -- Databus input
  wb_dat_o      => wb_dat_o    , --: out std_logic_vector(7 downto 0); -- Databus output
  wb_we_i       => wb_we       , --: in  std_logic;                    -- Write enable input
  wb_stb_i      => wb_stb      , --: in  std_logic;                    -- Strobe signals / core select signal
  wb_cyc_i      => wb_cyc      , --: in  std_logic;                    -- Valid bus cycle input
  wb_ack_o      => wb_ack      , --: out std_logic;                    -- Bus cycle acknowledge output
  wb_inta_o     => wb_inta     , --: out std_logic;                    -- interrupt request output signal
  scl_pad_i     => scl_i       , --: in  std_logic;                    -- i2c clock line input
  scl_pad_o     => scl_o       , --: out std_logic;                    -- i2c clock line output
  scl_padoen_o  => scl_oe      , --: out std_logic;                    -- i2c clock line output enable, active low
  sda_pad_i     => sda_i       , --: in  std_logic;                    -- i2c data line input
  sda_pad_o     => sda_o       , --: out std_logic;                    -- i2c data line output
  sda_padoen_o  => sda_oe        --: out std_logic                     -- i2c data line output enable, active low
);

----------------------------------------------------------------------------------------------------
-- End
----------------------------------------------------------------------------------------------------

end i2c_master_syn;
