----------------------------------------------------------------------------------
-- Company:        University of Queensland
-- Engineer:       Kayla Bettini
--
-- Module Name:    main
-- Project Name:   Implementation of the MUSIC algorithm on an FPGA
-- Target Devices: Nexys 4 (XC7A100T)
-- Description:    Top level. Receives 4-channel audio over Ethernet (UDP), feeds
--                 it to the HFMA DOA core (hfma_core_0) and reports the angle on
--                 the 7-segment display and over the USB-UART.
--
--                 - clk_wiz_0 clk_out3 = 10 MHz core clock (clk_HFMA)
--                 - Parser frames (clk_mac) -> xpm_fifo_async (512 deep, LUT RAM)
--                   -> clk_HFMA domain
--                 - 8 kHz tick: pop one 4-mic frame, then 64 enabled clocks
--                   (HDL Coder Oversampling = 64)
--                 - DOA (sfix18_En10) shown on the 7-segment display in degrees
--                 - DOA also sent over the USB-UART (UART_TXD, 115200 8N1) 50 times
--                   a second as "<deg>\r\n" by angle_uart_reporter
----------------------------------------------------------------------------------

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;
library xpm;
use xpm.vcomponents.all;

entity main is
    Port (
        -- Clock and reset
        CLK100MHZ : in  STD_LOGIC;
        RESET     : in  STD_LOGIC;                     -- Reset button, active low

        -- Ethernet RMII
        RMII_RX_DATA : inout STD_LOGIC_VECTOR(1 downto 0);
        RMII_CRS_DV  : inout STD_LOGIC;
        RMII_RX_ERR  : inout STD_LOGIC;
        RMII_TX_EN   : out STD_LOGIC;
        RMII_TX_DATA : out STD_LOGIC_VECTOR(1 downto 0);

        -- Ethernet PHY management, clock, reset and interrupt
        ETH_MDC   : out STD_LOGIC;
        ETH_MDIO  : inout STD_LOGIC;
        ETH_CLKIN : out STD_LOGIC;
        ETH_RSTN  : out STD_LOGIC;
        ETH_INTN  : inout STD_LOGIC;
        
        -- 7-segment display and LEDs
        SSD_CATHODE : out STD_LOGIC_VECTOR(7 downto 0);
        SSD_ANODE   : out STD_LOGIC_VECTOR(7 downto 0);
        LED         : out STD_LOGIC_VECTOR(15 downto 0);

        -- USB-UART (FPGA -> PC)
        UART_TXD : out STD_LOGIC
    );
end main;

architecture Structural of main is

--==============================================================================
-- Constants
--==============================================================================
-- Microphone channels and sample resolution
constant CHANNELS_C   : integer := 4;
constant RESOLUTION_C : integer := 16;

-- HFMA core timing
constant CLKS_PER_SAMPLE : integer := 1250;   -- 10 MHz / 8 kHz
constant OVERSAMPLE      : integer := 64;     -- Same as the HDL Coder Oversampling
constant PRIME_FRAMES    : integer := 128;    -- 16 ms buffered before starting

--==============================================================================
-- Components
--==============================================================================
-- Clock wizard: Ethernet MAC and PHY clocks plus the 10 MHz HFMA clock
component clk_wiz_0
    port(
        reset    : in  STD_LOGIC;   -- Reset
        clk_in1  : in  STD_LOGIC;   -- 100 MHz board clock
        clk_out1 : out STD_LOGIC;   -- Ethernet MAC clock
        clk_out2 : out STD_LOGIC;   -- Ethernet PHY clock
        clk_out3 : out STD_LOGIC;   -- 10 MHz HFMA core clock
        locked   : out STD_LOGIC
    );
end component;

-- Ethernet MAC (RMII)
component eth_mac
    generic (AXIS_INTERFACES : integer := 1);
    port (
        clk_mac : in STD_LOGIC; 
        clk_phy : in STD_LOGIC;
        rst_n : in STD_LOGIC; 
        mode_straps : in STD_LOGIC_VECTOR(2 downto 0);
        
        eth_mdc : out STD_LOGIC; 
        eth_mdio : inout STD_LOGIC;
        eth_rstn : out STD_LOGIC; 
        eth_crsdv : inout STD_LOGIC;
        eth_rxerr : inout STD_LOGIC; 
        eth_rxd : inout STD_LOGIC_VECTOR(1 downto 0);
        eth_txen : out STD_LOGIC; 
        eth_txd : out STD_LOGIC_VECTOR(1 downto 0);
        eth_clkin : out STD_LOGIC; 
        eth_intn : inout STD_LOGIC;
        
        rx_vld : out STD_LOGIC; 
        rx_dat : out STD_LOGIC_VECTOR(7 downto 0);
        rx_sof : out STD_LOGIC;
        rx_eof : out STD_LOGIC; 
        rx_err : out STD_LOGIC;
        
        rx_axis_mac_tdata  : out STD_LOGIC_VECTOR(7 downto 0);
        rx_axis_mac_tvalid : out STD_LOGIC;
        rx_axis_mac_tlast  : out STD_LOGIC;
        rx_axis_mac_tuser  : out STD_LOGIC;
        
        tx_vld : in STD_LOGIC; 
        tx_dat : in STD_LOGIC_VECTOR(7 downto 0);
        tx_sof : in STD_LOGIC; 
        tx_eof : in STD_LOGIC; 
        tx_ack : out STD_LOGIC;
        
        tx_axis_mac_tdata  : in STD_LOGIC_VECTOR(7 downto 0);
        tx_axis_mac_tvalid : in STD_LOGIC; 
        tx_axis_mac_tlast  : in STD_LOGIC;
        tx_axis_mac_tready : out STD_LOGIC;
        
        reg_vld : in STD_LOGIC := '0'; 
        reg_addr : in STD_LOGIC_VECTOR(4 downto 0);
        reg_write : in STD_LOGIC; 
        reg_wval : in STD_LOGIC_VECTOR(15 downto 0);
        reg_rval : out STD_LOGIC_VECTOR(15 downto 0); 
        reg_ack : out STD_LOGIC;
        
        speed_100 : out STD_LOGIC; 
        full_duplex : out STD_LOGIC;
        link_up : out STD_LOGIC; 
        remote_fault : out STD_LOGIC;
        auto_neg_done : out STD_LOGIC
    );
end component;

-- Ethernet packet parser: UDP payload -> 4-channel audio frames
component ethernet_audio_parser
    Generic (
        CHANNELS : integer := 4; 
        RESOLUTION : integer := 16; 
        FRAMES_PER_PACKET : integer := 64);
    Port (
        CLK : in STD_LOGIC; 
        RESET : in STD_LOGIC;
        RX_DATA : in STD_LOGIC_VECTOR(7 downto 0);
        RX_VALID : in STD_LOGIC; 
        RX_LAST : in STD_LOGIC;
        AUDIO_DATA_OUT     : out STD_LOGIC_VECTOR(7 downto 0);
        AUDIO_BYTE_VALID   : out STD_LOGIC;
        AUDIO_CHANNELS_OUT : out STD_LOGIC_VECTOR((RESOLUTION*CHANNELS)-1 downto 0);
        AUDIO_FRAME_VALID  : out STD_LOGIC;
        SEQ_NUM_OUT    : out STD_LOGIC_VECTOR(31 downto 0);
        FRAG_ID_OUT    : out STD_LOGIC_VECTOR(7 downto 0);
        FRAG_TOTAL_OUT : out STD_LOGIC_VECTOR(7 downto 0);
        HEADER_OUT     : out STD_LOGIC_VECTOR(31 downto 0);
        PACKET_READY   : out STD_LOGIC;
        DEBUG_STATE    : out STD_LOGIC_VECTOR(2 downto 0)
    );
end component;

-- Debug LEDs
component led_driver
    Port (
        reset : in STD_LOGIC;
        SIG0  : in STD_LOGIC; 
        SIG1  : in STD_LOGIC; 
        SIG2  : in STD_LOGIC;
        SIG3  : in STD_LOGIC; 
        SIG4  : in STD_LOGIC; 
        SIG5  : in STD_LOGIC;
        SIG6  : in STD_LOGIC; 
        SIG7  : in STD_LOGIC; 
        SIG8  : in STD_LOGIC;
        SIG9  : in STD_LOGIC; 
        SIG10 : in STD_LOGIC; 
        SIG11 : in STD_LOGIC;
        SIG12 : in STD_LOGIC; 
        SIG13 : in STD_LOGIC; 
        SIG14 : in STD_LOGIC;
        SIG15 : in STD_LOGIC; 
        LEDS  : out STD_LOGIC_VECTOR(15 downto 0)
    );
end component;

-- HFMA core (IP packaged from the HDL Coder output; ports as in Subsystem.vhd)
component hfma_core_0
    port (
        clk        : in  STD_LOGIC;
        clk_enable : in  STD_LOGIC;
        Mic1       : in  STD_LOGIC_VECTOR(15 downto 0);
        Mic2       : in  STD_LOGIC_VECTOR(15 downto 0);
        Mic3       : in  STD_LOGIC_VECTOR(15 downto 0);
        Mic4       : in  STD_LOGIC_VECTOR(15 downto 0);
        RESET_1    : in  STD_LOGIC;
        ce_out     : out STD_LOGIC;
        DOA        : out STD_LOGIC_VECTOR(17 downto 0)    -- sfix18_En10 (degrees)
    );
end component;

--==============================================================================
-- Signals
--==============================================================================
-- Clocks and reset
signal clock_locked : STD_LOGIC;
signal rst          : STD_LOGIC;
signal clk_mac      : STD_LOGIC;
signal clk_phy      : STD_LOGIC;
signal clk_HFMA     : STD_LOGIC;

-- PHY reset delay
signal phy_reset_counter : unsigned(24 downto 0) := (others => '0');
signal phy_rstn_reg      : STD_LOGIC := '0';

-- Ethernet MAC receive stream
signal link_up        : STD_LOGIC;
signal rx_bad_frame   : STD_LOGIC;
signal rx_axis_tdata  : STD_LOGIC_VECTOR(7 downto 0);
signal rx_axis_tvalid : STD_LOGIC;
signal rx_axis_tlast  : STD_LOGIC;

-- Parser outputs
signal seq_num_display    : STD_LOGIC_VECTOR(31 downto 0);
signal frag_id_display    : STD_LOGIC_VECTOR(7 downto 0);
signal frag_total_display : STD_LOGIC_VECTOR(7 downto 0);
signal header_display     : STD_LOGIC_VECTOR(31 downto 0);
signal packet_ready_sig   : STD_LOGIC;
signal debug_state        : STD_LOGIC_VECTOR(2 downto 0);

signal clean_audio_valid     : STD_LOGIC;
signal audio_channels_sig    : STD_LOGIC_VECTOR((RESOLUTION_C*CHANNELS_C)-1 downto 0);
signal audio_frame_valid_sig : STD_LOGIC;

-- Diagnostics
signal pkt_ready_sync1       : STD_LOGIC := '0';
signal pkt_ready_sync2       : STD_LOGIC := '0';
signal header_display_stable : STD_LOGIC_VECTOR(31 downto 0) := (others => '0');

signal packet_count : STD_LOGIC_VECTOR(31 downto 0) := (others => '0');

signal LED_signal   : STD_LOGIC := '0';
signal diag_counter : integer range 0 to 50000000 := 0;

signal rx_seen_sticky     : STD_LOGIC := '0';
signal packet_seen_sticky : STD_LOGIC := '0';
signal audio_seen_sticky  : STD_LOGIC := '0';

-- HFMA: clk_mac side of the clock-crossing FIFO
signal cdc_wr_en         : STD_LOGIC;
signal cdc_full          : STD_LOGIC;
signal cdc_wr_busy       : STD_LOGIC;
signal cdc_overflow_seen : STD_LOGIC := '0';

-- HFMA: clk_HFMA side
signal hfma_rst_meta, hfma_rst : STD_LOGIC := '1';
signal cdc_dout       : STD_LOGIC_VECTOR(63 downto 0);
signal cdc_empty      : STD_LOGIC;
signal cdc_rd_en      : STD_LOGIC := '0';
signal cdc_rd_count   : STD_LOGIC_VECTOR(9 downto 0);
signal cdc_rd_busy    : STD_LOGIC;
signal primed         : STD_LOGIC := '0';
signal underflow_seen : STD_LOGIC := '0';

signal samp_cnt   : integer range 0 to CLKS_PER_SAMPLE-1 := 0;
signal tick       : STD_LOGIC := '0';
signal latch_now  : STD_LOGIC := '0';
signal mic1_r, mic2_r, mic3_r, mic4_r : STD_LOGIC_VECTOR(15 downto 0) := (others => '0');
signal burst_cnt  : integer range 0 to OVERSAMPLE := 0;
signal core_ce    : STD_LOGIC;
signal core_reset : STD_LOGIC := '1';
signal core_ce_out: STD_LOGIC;
signal core_doa   : STD_LOGIC_VECTOR(17 downto 0);
signal doa_deg    : integer range -128 to 127 := 0;
signal doa_count  : unsigned(12 downto 0) := (others => '0');
signal doa_deg_s  : signed(7 downto 0);                        -- doa_deg for the UART

-- 7-segment display
signal scan_cnt : unsigned(13 downto 0) := (others => '0');
type dig_arr is array (0 to 7) of integer range 0 to 11;      -- 10 = '-', 11 = blank
signal digits   : dig_arr := (others => 11);

-- Digit to segments, active high "abcdefg"
function seg7 (d : integer) return STD_LOGIC_VECTOR is
begin
    case d is
        when 0 => return "1111110";  when 1 => return "0110000";
        when 2 => return "1101101";  when 3 => return "1111001";
        when 4 => return "0110011";  when 5 => return "1011011";
        when 6 => return "1011111";  when 7 => return "1110000";
        when 8 => return "1111111";  when 9 => return "1111011";
        when 10 => return "0000001";
        when others => return "0000000";
    end case;
end function;

begin

--==============================================================================
-- Clocks and reset
--==============================================================================
rst <= not RESET;

-- Hold the PHY in reset for 20,000,000 clocks (200 ms) after a reset or until
-- the clocks lock (the PHY did not start reliably without this)
phy_reset_proc : process(CLK100MHZ)
begin
    if rising_edge(CLK100MHZ) then
        if rst = '1' or clock_locked = '0' then
            phy_reset_counter <= (others => '0');
            phy_rstn_reg      <= '0';
        elsif phy_rstn_reg = '0' then
            if phy_reset_counter = to_unsigned(20_000_000, 25) then
                phy_rstn_reg <= '1';
            else
                phy_reset_counter <= phy_reset_counter + 1;
            end if;
        end if;
    end if;
end process;

-- Clock wizard
clock_gen : clk_wiz_0
    port map (
        reset => rst,
        clk_in1 => CLK100MHZ,
        clk_out1 => clk_mac,
        clk_out2 => clk_phy,
        clk_out3 => clk_HFMA,
        locked => clock_locked
    );

--==============================================================================
-- Ethernet receive path
--==============================================================================
-- Ethernet MAC (receive only; transmit and register ports tied off)
eth_mac_inst : eth_mac
    generic map (AXIS_INTERFACES => 1)
    port map (
        clk_mac => clk_mac, clk_phy => clk_phy,
        rst_n => phy_rstn_reg, mode_straps => "001",
        eth_mdc => ETH_MDC, eth_mdio => ETH_MDIO,
        eth_rstn => ETH_RSTN, eth_crsdv => RMII_CRS_DV,
        eth_rxerr => RMII_RX_ERR, eth_rxd => RMII_RX_DATA,
        eth_txen => RMII_TX_EN, eth_txd => RMII_TX_DATA,
        eth_clkin => ETH_CLKIN, eth_intn => ETH_INTN,
        rx_axis_mac_tdata  => rx_axis_tdata,
        rx_axis_mac_tvalid => rx_axis_tvalid,
        rx_axis_mac_tlast  => rx_axis_tlast,
        rx_axis_mac_tuser  => rx_bad_frame,
        rx_vld => open, rx_dat => open, rx_sof => open,
        rx_eof => open, rx_err => open,
        tx_vld => '0', tx_dat => (others => '0'),
        tx_sof => '0', tx_eof => '0', tx_ack => open,
        tx_axis_mac_tdata => (others => '0'),
        tx_axis_mac_tvalid => '0', tx_axis_mac_tlast => '0',
        tx_axis_mac_tready => open,
        reg_vld => '0', reg_addr => (others => '0'),
        reg_write => '0', reg_wval => (others => '0'),
        reg_rval => open, reg_ack => open,
        speed_100 => open, full_duplex => open,
        link_up => link_up, remote_fault => open,
        auto_neg_done => open
    );

-- Packet parser: one 4-channel frame per AUDIO_FRAME_VALID pulse
parser_inst : ethernet_audio_parser
    generic map (
        CHANNELS          => CHANNELS_C,
        RESOLUTION        => RESOLUTION_C,
        FRAMES_PER_PACKET => 64
    )
    port map (
        CLK    => clk_mac,
        RESET  => rst,
        RX_DATA  => rx_axis_tdata,
        RX_VALID => rx_axis_tvalid,
        RX_LAST  => rx_axis_tlast,
        AUDIO_DATA_OUT     => open,
        AUDIO_BYTE_VALID   => clean_audio_valid,
        AUDIO_CHANNELS_OUT => audio_channels_sig,
        AUDIO_FRAME_VALID  => audio_frame_valid_sig,
        SEQ_NUM_OUT        => seq_num_display,
        FRAG_ID_OUT        => frag_id_display,
        FRAG_TOTAL_OUT     => frag_total_display,
        HEADER_OUT         => header_display,
        PACKET_READY       => packet_ready_sig,
        DEBUG_STATE        => debug_state
    );

--==============================================================================
-- Diagnostics
--==============================================================================
-- Synchronise PACKET_READY into the 100 MHz domain and keep the last header
-- (debug only, not used by the HFMA path)
cdc_sync : process(CLK100MHZ)
begin
    if rising_edge(CLK100MHZ) then
        if rst = '1' then
            pkt_ready_sync1 <= '0';
            pkt_ready_sync2 <= '0';
            header_display_stable <= (others => '0');
        else
            pkt_ready_sync1 <= packet_ready_sig;
            pkt_ready_sync2 <= pkt_ready_sync1;
            if pkt_ready_sync2 = '1' then
                header_display_stable <= header_display;
            end if;
        end if;
    end if;
end process;

-- Count received packets
packet_counter : process(clk_mac)
begin
    if rising_edge(clk_mac) then
        if rst = '1' then
            packet_count <= (others => '0');
        elsif packet_ready_sig = '1' then
            packet_count <= std_logic_vector(unsigned(packet_count) + 1);
        end if;
    end if;
end process;

-- Heartbeat: LED_signal toggles every 0.5 s (1 Hz blink)
heartbeat : process(CLK100MHZ)
begin
    if rising_edge(CLK100MHZ) then
        if rst = '1' then
            diag_counter <= 0;
            LED_signal <= '0';
        else
            if diag_counter >= 49999999 then
                diag_counter <= 0;
                LED_signal <= not LED_signal;
            else
                diag_counter <= diag_counter + 1;
            end if;
        end if;
    end if;
end process;

-- Sticky flags: set once receive data, a full packet or audio has been seen
debug_latches : process(clk_mac)
begin
    if rising_edge(clk_mac) then
        if rst = '1' then
            rx_seen_sticky <= '0';
            packet_seen_sticky <= '0';
            audio_seen_sticky <= '0';
        else
            if rx_axis_tvalid = '1' then rx_seen_sticky <= '1'; end if;
            if packet_ready_sig = '1' then packet_seen_sticky <= '1'; end if;
            if clean_audio_valid = '1' then audio_seen_sticky <= '1'; end if;
        end if;
    end if;
end process;

--==============================================================================
-- HFMA: clock-domain crossing (clk_mac -> clk_HFMA)
--==============================================================================
cdc_wr_en <= audio_frame_valid_sig and not cdc_full and not cdc_wr_busy;

-- Sticky flag: a frame arrived while the FIFO was full
cdc_overflow_proc : process(clk_mac)
begin
    if rising_edge(clk_mac) then
        if rst = '1' then
            cdc_overflow_seen <= '0';
        elsif audio_frame_valid_sig = '1' and cdc_full = '1' then
            cdc_overflow_seen <= '1';
        end if;
    end if;
end process;

hfma_cdc_fifo : xpm_fifo_async
    generic map (
        FIFO_MEMORY_TYPE    => "distributed",   -- LUT RAM: no block RAM used
        FIFO_WRITE_DEPTH    => 512,
        WRITE_DATA_WIDTH    => 64,
        READ_DATA_WIDTH     => 64,
        READ_MODE           => "fwft",
        FIFO_READ_LATENCY   => 0,
        WR_DATA_COUNT_WIDTH => 10,
        RD_DATA_COUNT_WIDTH => 10,
        USE_ADV_FEATURES    => "0400",          -- Enable rd_data_count
        CDC_SYNC_STAGES     => 2,
        RELATED_CLOCKS      => 0,
        ECC_MODE            => "no_ecc",
        DOUT_RESET_VALUE    => "0",
        FULL_RESET_VALUE    => 0,
        PROG_EMPTY_THRESH   => 10,
        PROG_FULL_THRESH    => 10,
        SIM_ASSERT_CHK      => 0,
        WAKEUP_TIME         => 0 )
    port map (
        sleep => '0', rst => rst,
        wr_clk => clk_mac, wr_en => cdc_wr_en, din => audio_channels_sig,
        full => cdc_full, wr_rst_busy => cdc_wr_busy,
        prog_full => open, wr_data_count => open, overflow => open,
        almost_full => open, wr_ack => open,
        rd_clk => clk_HFMA, rd_en => cdc_rd_en, dout => cdc_dout,
        empty => cdc_empty, rd_data_count => cdc_rd_count, rd_rst_busy => cdc_rd_busy,
        prog_empty => open, underflow => open, almost_empty => open, data_valid => open,
        injectsbiterr => '0', injectdbiterr => '0', sbiterr => open, dbiterr => open );

--==============================================================================
-- HFMA: core clock domain (10 MHz)
--==============================================================================
-- Synchronise the reset into clk_HFMA
hfma_reset_sync : process(clk_HFMA)
begin
    if rising_edge(clk_HFMA) then
        hfma_rst_meta <= rst or not clock_locked;
        hfma_rst      <= hfma_rst_meta;
    end if;
end process;

-- 8 kHz tick; once PRIME_FRAMES frames are buffered, pop one frame per tick
hfma_sample_proc : process(clk_HFMA)
begin
    if rising_edge(clk_HFMA) then
        tick      <= '0';
        latch_now <= '0';
        cdc_rd_en <= '0';
        if hfma_rst = '1' then
            samp_cnt <= 0;  primed <= '0';  underflow_seen <= '0';
        else
            if samp_cnt = CLKS_PER_SAMPLE-1 then
                samp_cnt <= 0;
                tick     <= '1';
            else
                samp_cnt <= samp_cnt + 1;
            end if;

            if primed = '0' then
                if unsigned(cdc_rd_count) >= PRIME_FRAMES and cdc_rd_busy = '0' then
                    primed <= '1';
                end if;
            elsif tick = '1' then
                if cdc_empty = '0' then
                    -- FWFT: dout already holds the oldest frame; copy it, then pop
                    mic1_r    <= cdc_dout(15 downto 0);    -- Channel 1 is in the LSBs
                    mic2_r    <= cdc_dout(31 downto 16);
                    mic3_r    <= cdc_dout(47 downto 32);
                    mic4_r    <= cdc_dout(63 downto 48);
                    cdc_rd_en <= '1';
                    latch_now <= '1';
                else
                    primed         <= '0';                 -- Ran dry: wait to re-prime
                    underflow_seen <= '1';
                end if;
            end if;
        end if;
    end if;
end process;

-- After each new sample: exactly OVERSAMPLE enabled clocks with the inputs held
hfma_burst_proc : process(clk_HFMA)
begin
    if rising_edge(clk_HFMA) then
        if latch_now = '1' then
            burst_cnt <= OVERSAMPLE;
        elsif burst_cnt /= 0 then
            burst_cnt <= burst_cnt - 1;
        end if;

        -- Keep the covariance averages cleared while no audio is flowing
        if hfma_rst = '1' or primed = '0' then
            core_reset <= '1';
        elsif burst_cnt = 1 then
            core_reset <= '0';
        end if;
    end if;
end process;

core_ce <= '1' when burst_cnt /= 0 else '0';

hfma_inst : hfma_core_0
    port map (
        clk        => clk_HFMA,
        clk_enable => core_ce,
        Mic1       => mic1_r,
        Mic2       => mic2_r,
        Mic3       => mic3_r,
        Mic4       => mic4_r,
        RESET_1    => core_reset,
        ce_out     => core_ce_out,
        DOA        => core_doa
    );

-- DOA is sfix18_En10: round to whole degrees
hfma_doa_proc : process(clk_HFMA)
    variable r : signed(18 downto 0);
    variable v : integer range -256 to 255;
begin
    if rising_edge(clk_HFMA) then
        if core_ce_out = '1' and core_ce = '1' then
            r := resize(signed(core_doa), 19) + 512;
            v := to_integer(shift_right(r, 10));
            if v > 127 then v := 127; elsif v < -128 then v := -128; end if;
            doa_deg   <= v;
            doa_count <= doa_count + 1;
        end if;
    end if;
end process;

--==============================================================================
-- USB-UART: DOA (same value as the display), 50 readings per second
--==============================================================================
doa_deg_s <= to_signed(doa_deg, 8);

hfma_uart_inst : entity work.angle_uart_reporter
    generic map (
        CLK_HZ    => 10_000_000,     -- clk_HFMA
        BAUD      => 115_200,
        REPORT_HZ => 50,
        ANGLE_W   => 8               -- doa_deg is -128..127
    )
    port map (
        clk   => clk_HFMA,
        rst   => hfma_rst,
        angle => doa_deg_s,
        tx    => UART_TXD
    );

--==============================================================================
-- 7-segment display: left "----" while waiting for audio, right = DOA (degrees)
--==============================================================================
ssd_digits_proc : process(clk_HFMA)
    variable a : integer range 0 to 128;
begin
    if rising_edge(clk_HFMA) then
        a := abs(doa_deg);
        if a > 99 then a := 99; end if;
        for i in 4 to 7 loop
            if primed = '0' then digits(i) <= 10; else digits(i) <= 11; end if;
        end loop;
        digits(3) <= 11;
        if doa_deg < 0 then digits(2) <= 10; else digits(2) <= 11; end if;
        if a >= 10 then digits(1) <= a / 10; else digits(1) <= 11; end if;
        digits(0) <= a mod 10;
    end if;
end process;

-- Multiplex the 8 digits, about 610 Hz refresh
ssd_scan_proc : process(clk_HFMA)
    variable sel : integer range 0 to 7;
begin
    if rising_edge(clk_HFMA) then
        scan_cnt <= scan_cnt + 1;
        sel := to_integer(scan_cnt(13 downto 11));
        SSD_ANODE      <= (others => '1');
        SSD_ANODE(sel) <= '0';
        SSD_CATHODE(7 downto 1) <= not seg7(digits(sel));
        SSD_CATHODE(0)          <= '1';
    end if;
end process;

--==============================================================================
-- LEDs
--==============================================================================
leds : led_driver
    port map (
        reset => rst,
        SIG0  => rx_axis_tvalid,       -- Ethernet byte received
        SIG1  => rx_axis_tlast,        -- End of Ethernet frame
        SIG2  => packet_ready_sig,     -- Audio packet parsed
        SIG3  => audio_frame_valid_sig,-- Audio frame parsed
        SIG4  => link_up,              -- Ethernet link up
        SIG5  => link_up,              -- Ethernet link up
        SIG6  => primed,               -- Audio buffered, core running
        SIG7  => doa_count(12),        -- Blinks about 1 Hz while the core produces estimates
        SIG8  => underflow_seen,       -- HFMA FIFO ran dry (sticky)
        SIG9  => cdc_overflow_seen,    -- HFMA FIFO overflowed (sticky)
        SIG10 => '0',                  -- Unused
        SIG11 => '0',                  -- Unused
        SIG12 => RESET,                -- Reset button (lit when not pressed)
        SIG13 => rst,                  -- Internal reset
        SIG14 => clock_locked,         -- Clocks locked
        SIG15 => LED_signal,           -- Heartbeat
        LEDS  => LED
    );

end Structural;
