----------------------------------------------------------------------------------
-- Company:        University of Queensland
-- Engineer:       Kayla Bettini
--
-- Module Name:    angle_uart_reporter
-- Project Name:   Implementation of the MUSIC algorithm on an FPGA
-- Target Devices: Nexys 4 (XC7A100T)
-- Description:    Sends the HFMA angle estimate out of the Nexys 4 USB-UART as
--                 one ASCII line per report, e.g. "-60\r\n", "7\r\n", "30\r\n".
--
--                 - The angle is sampled at a fixed rate (REPORT_HZ, default
--                   50 Hz = every 20 ms), so every trial gives a steady stream
--                   of readings for statistics.
--                 - 8N1 serial, default 115200 baud. A line is at most 6 bytes
--                   (about 0.5 ms), so 50 Hz uses under 3 % of the link.
--                 - Signed integer degrees in, ANGLE_W <= 10 bits (+/-511).
--
--                 Clocking: run this in the same clock domain as the signal
--                 connected to `angle` (e.g. the 10 MHz core clock clk_out3) and
--                 set CLK_HZ to match. At 10 MHz / 115200 the bit divider is 87
--                 (0.2 % baud error, well within UART tolerance).
----------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity angle_uart_reporter is
  generic (
    CLK_HZ    : positive := 10_000_000;
    BAUD      : positive := 115_200;
    REPORT_HZ : positive := 50;
    ANGLE_W   : positive := 8
  );
  port (
    clk   : in  std_logic;
    rst   : in  std_logic;                      -- Synchronous, active high
    angle : in  signed(ANGLE_W-1 downto 0);     -- Estimate in whole degrees
    tx    : out std_logic                       -- To UART_RXD_OUT (FPGA -> PC)
  );
end entity;

architecture rtl of angle_uart_reporter is

  --============================================================================
  -- Constants and signals
  --============================================================================
  constant BIT_DIV : positive := (CLK_HZ + BAUD/2) / BAUD;
  constant REP_DIV : positive := CLK_HZ / REPORT_HZ;

  -- Report timer
  signal rep_cnt  : integer range 0 to REP_DIV-1 := 0;
  signal rep_tick : std_logic := '0';

  -- Formatter
  type fstate_t is (F_IDLE, F_HUND, F_TENS, F_SEND, F_WAIT1, F_WAIT);
  signal fs    : fstate_t := F_IDLE;
  -- Character position: 0 sign, 1 hundreds, 2 tens, 3 ones, 4 CR, 5 LF
  -- (characters are generated on the fly, so no line buffer is needed)
  signal pos   : integer range 0 to 5 := 0;
  signal mag   : integer range 0 to 2**(ANGLE_W-1) := 0;
  signal hund  : integer range 0 to 9 := 0;
  signal tens  : integer range 0 to 9 := 0;
  signal neg   : std_logic := '0';

  -- UART transmitter
  signal tx_start : std_logic := '0';
  signal tx_data  : std_logic_vector(7 downto 0) := (others => '0');
  signal tx_busy  : std_logic := '0';
  signal shreg    : std_logic_vector(9 downto 0) := (others => '1');
  signal bit_cnt  : integer range 0 to 9 := 0;
  signal baud_cnt : integer range 0 to BIT_DIV-1 := 0;

  -- Digit 0..9 to ASCII ('0' + d)
  function digit(d : integer) return std_logic_vector is
  begin
    return std_logic_vector(to_unsigned(48 + d, 8));
  end function;

begin

  assert ANGLE_W <= 10
    report "angle_uart_reporter: ANGLE_W must be <= 10 (max 3 digits)"
    severity failure;

  --============================================================================
  -- Report timer: one tick every 1/REPORT_HZ seconds
  --============================================================================
  p_tick : process (clk)
  begin
    if rising_edge(clk) then
      rep_tick <= '0';
      if rst = '1' then
        rep_cnt <= 0;
      elsif rep_cnt = REP_DIV-1 then
        rep_cnt  <= 0;
        rep_tick <= '1';
      else
        rep_cnt <= rep_cnt + 1;
      end if;
    end if;
  end process;

  --============================================================================
  -- Formatter: sample the angle, convert to decimal ASCII, send byte by byte
  --============================================================================
  p_fmt : process (clk)
  begin
    if rising_edge(clk) then
      tx_start <= '0';
      if rst = '1' then
        fs <= F_IDLE;
      else
        case fs is
          -- Wait for the report tick, then take the sign and magnitude
          when F_IDLE =>
            if rep_tick = '1' then
              if angle < 0 then
                neg <= '1';
                mag <= to_integer(-resize(angle, ANGLE_W+1));
              else
                neg <= '0';
                mag <= to_integer(angle);
              end if;
              hund <= 0;
              tens <= 0;
              fs   <= F_HUND;
            end if;

          -- Hundreds by repeated subtraction
          when F_HUND =>
            if mag >= 100 then
              mag  <= mag - 100;
              hund <= hund + 1;
            else
              fs <= F_TENS;
            end if;

          -- Tens by repeated subtraction (the remainder is the ones digit)
          when F_TENS =>
            if mag >= 10 then
              mag  <= mag - 10;
              tens <= tens + 1;
            else
              pos <= 0;
              fs  <= F_SEND;
            end if;

          -- Send the next character, skipping unused positions (no leading zeros)
          when F_SEND =>
            if (pos = 0 and neg = '0') or
               (pos = 1 and hund = 0) or
               (pos = 2 and hund = 0 and tens = 0) then
              pos <= pos + 1;
            elsif tx_busy = '0' then
              case pos is
                when 0      => tx_data <= x"2D";        -- '-'
                when 1      => tx_data <= digit(hund);
                when 2      => tx_data <= digit(tens);
                when 3      => tx_data <= digit(mag);   -- Ones
                when 4      => tx_data <= x"0D";        -- CR
                when others => tx_data <= x"0A";        -- LF
              end case;
              tx_start <= '1';
              fs       <= F_WAIT1;
            end if;

          -- Let tx_busy rise
          when F_WAIT1 =>
            fs <= F_WAIT;

          -- Wait for the byte to finish, then move to the next position
          when F_WAIT =>
            if tx_busy = '0' then
              if pos = 5 then
                fs <= F_IDLE;
              else
                pos <= pos + 1;
                fs  <= F_SEND;
              end if;
            end if;
        end case;
      end if;
    end if;
  end process;

  --============================================================================
  -- 8N1 transmitter: shreg idles at all '1', LSB first
  --============================================================================
  p_tx : process (clk)
  begin
    if rising_edge(clk) then
      if rst = '1' then
        shreg    <= (others => '1');
        tx_busy  <= '0';
        bit_cnt  <= 0;
        baud_cnt <= 0;
      elsif tx_busy = '0' then
        if tx_start = '1' then
          shreg    <= '1' & tx_data & '0';     -- Stop, data, start
          tx_busy  <= '1';
          bit_cnt  <= 0;
          baud_cnt <= 0;
        end if;
      elsif baud_cnt = BIT_DIV-1 then
        baud_cnt <= 0;
        shreg    <= '1' & shreg(9 downto 1);
        if bit_cnt = 9 then
          tx_busy <= '0';
        else
          bit_cnt <= bit_cnt + 1;
        end if;
      else
        baud_cnt <= baud_cnt + 1;
      end if;
    end if;
  end process;

  tx <= shreg(0);   -- Straight from a register: glitch-free

end architecture;