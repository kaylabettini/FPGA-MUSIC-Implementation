-- =============================================================================
-- tb_main.vhd  -  Full-system testbench for main.vhd
--
-- Strategy
-- --------
-- Rather than driving the real PHY reset sequence through eth_mac, we inject
-- data directly at the RMII layer.  The RMII interface runs at 50 MHz (one
-- 2-bit dibit per cycle).  We build a complete Ethernet frame that carries:
--
--   Ethernet header  (14 bytes)
--   IP header        (20 bytes)
--   UDP header       ( 8 bytes)   → 42 bytes total L2/L3/L4
--   UDP payload:
--     Sync word      4 bytes  0x0D 0xF0 0x5A 0xA5
--     Seq number     4 bytes  0x00 0x00 0x00 0x01
--     Frag id        1 byte   0x00
--     Frag total     1 byte   0x01
--     Audio data   1024 bytes 0xAB repeated
--   FCS              4 bytes  (dummy, eth_mac accepts any)
--
-- The frame is serialised as 2-bit dibits on RMII_RX_DATA while
-- RMII_CRS_DV is held high, exactly as the LAN8720A would do.
-- A 7-byte preamble (0x55 x7) + 1-byte SFD (0xD5) precede the frame.
--
-- Stub components
-- ---------------
-- clk_wiz_0, SSDDriver_0 and ila_0 are stubbed out below so the bench
-- compiles without Vivado IP.  Replace with your real IP for gate-level sim.
-- =============================================================================

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;
-- =============================================================================
-- STUB: clk_wiz_0  (passes clk_in1 straight through as both outputs)
-- =============================================================================
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
entity clk_wiz_0 is
    port(
        reset    : in  STD_LOGIC;
        clk_in1  : in  STD_LOGIC;
        clk_out1 : out STD_LOGIC;
        clk_out2 : out STD_LOGIC;
        locked   : out STD_LOGIC
    );
end clk_wiz_0;
architecture stub of clk_wiz_0 is
begin
    clk_out1 <= clk_in1;
    clk_out2 <= clk_in1;
    locked   <= not reset;          -- locked as soon as reset deasserts
end stub;

-- =============================================================================
-- STUB: SSDDriver_0
-- =============================================================================
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
entity SSDDriver_0 is
    port(
        CLK100MHZ    : in  STD_LOGIC;
        NumberVector : in  STD_LOGIC_VECTOR(31 downto 0);
        SSD_CATHODE  : out STD_LOGIC_VECTOR(7 downto 0);
        SSD_ANODE    : out STD_LOGIC_VECTOR(7 downto 0)
    );
end SSDDriver_0;
architecture stub of SSDDriver_0 is
begin
    SSD_CATHODE <= (others => '0');
    SSD_ANODE   <= (others => '0');
end stub;

-- =============================================================================
-- STUB: ila_0
-- =============================================================================
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
entity ila_0 is
    port (
        clk    : in std_logic;
        probe0 : in std_logic_vector(0 downto 0);
        probe1 : in std_logic_vector(1 downto 0);
        probe2 : in std_logic_vector(7 downto 0);
        probe3 : in std_logic_vector(0 downto 0);
        probe4 : in std_logic_vector(0 downto 0)
    );
end ila_0;
architecture stub of ila_0 is
begin
end stub;



-- =============================================================================
-- TESTBENCH
-- =============================================================================
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity tb_main is
end tb_main;

architecture sim of tb_main is

    -- -------------------------------------------------------------------------
    -- DUT component
    -- -------------------------------------------------------------------------
    component main is
        Port (
            CLK100MHZ    : in    STD_LOGIC;
            RESET        : in    STD_LOGIC;
            RMII_CLK     : in    STD_LOGIC;
            RMII_RX_DATA : inout STD_LOGIC_VECTOR(1 downto 0);
            RMII_CRS_DV  : inout STD_LOGIC;
            RMII_RX_ERR  : inout STD_LOGIC;
            ETH_RSTN     : out   STD_LOGIC;
            ETH_MDC      : out   STD_LOGIC;
            ETH_MDIO     : inout STD_LOGIC;
            SSD_CATHODE  : out   STD_LOGIC_VECTOR(7 downto 0);
            SSD_ANODE    : out   STD_LOGIC_VECTOR(7 downto 0);
            LED          : out   STD_LOGIC_VECTOR(15 downto 0)
        );
    end component;

    -- -------------------------------------------------------------------------
    -- Clock periods
    -- -------------------------------------------------------------------------
    constant T_SYS  : time := 10 ns;   -- 100 MHz system clock
    constant T_RMII : time := 20 ns;   -- 50  MHz RMII clock

    -- -------------------------------------------------------------------------
    -- Testbench signals
    -- -------------------------------------------------------------------------
    signal CLK100MHZ    : std_logic := '0';
    signal RMII_CLK     : std_logic := '0';
    signal RESET        : std_logic := '0';   -- active-low, start de-asserted

    -- RMII driven by TB
    signal tb_rxd       : std_logic_vector(1 downto 0) := "00";
    signal tb_crsdv     : std_logic := '0';
    signal tb_rxerr     : std_logic := '0';

    -- Resolved inout connections
    signal RMII_RX_DATA : std_logic_vector(1 downto 0);
    signal RMII_CRS_DV  : std_logic;
    signal RMII_RX_ERR  : std_logic;
    signal ETH_MDIO     : std_logic;

    -- DUT outputs
    signal ETH_RSTN     : std_logic;
    signal ETH_MDC      : std_logic;
    signal SSD_CATHODE  : std_logic_vector(7 downto 0);
    signal SSD_ANODE    : std_logic_vector(7 downto 0);
    signal LED          : std_logic_vector(15 downto 0);

    -- -------------------------------------------------------------------------
    -- Packet buffer
    -- We build the full frame (preamble+SFD+payload+FCS) into this array
    -- and then serialise it dibit-by-dibit onto the RMII bus.
    -- -------------------------------------------------------------------------
    constant AUDIO_BYTES   : integer := 1024;
    constant HEADER_BYTES  : integer := 42;   -- ETH+IP+UDP
    constant APP_HDR_BYTES : integer := 10;   -- sync(4)+seq(4)+frag_id(1)+frag_total(1)
    constant FCS_BYTES     : integer := 4;
    constant PRE_SFD_BYTES : integer := 8;    -- 7x 0x55 preamble + 0xD5 SFD

    constant FRAME_BYTES   : integer := PRE_SFD_BYTES + HEADER_BYTES +
                                        APP_HDR_BYTES + AUDIO_BYTES + FCS_BYTES;

    type byte_array_t is array (0 to FRAME_BYTES-1) of std_logic_vector(7 downto 0);

    -- -------------------------------------------------------------------------
    -- Build the frame as a constant
    -- Layout (index 0 = first byte transmitted)
    --
    --  [0..6]   preamble  0x55
    --  [7]      SFD       0xD5
    --  [8..13]  dst MAC   FF:FF:FF:FF:FF:FF  (broadcast)
    --  [14..19] src MAC   02:00:00:00:00:01
    --  [20..21] ethertype 0x0800  (IPv4)
    --  [22..41] IP header (ver=4, IHL=5, proto=17/UDP, src=192.168.1.10,
    --                      dst=192.168.1.255, len covers UDP+payload)
    --  [42..49] UDP header src=5000 dst=5000, len covers payload
    --  [50..53] sync word  0x0D 0xF0 0x5A 0xA5
    --  [54..57] seq num    0x00 0x00 0x00 0x01
    --  [58]     frag_id    0x00
    --  [59]     frag_total 0x01
    --  [60..1083] audio    0xAB x1024
    --  [1084..1087] FCS    0x00 (dummy - eth_mac strips/ignores in this MAC)
    -- -------------------------------------------------------------------------
    function build_frame return byte_array_t is
        variable f : byte_array_t := (others => x"00");
        variable i : integer;
    begin
        -- Preamble
        for k in 0 to 6 loop
            f(k) := x"55";
        end loop;
        f(7) := x"D5";  -- SFD

        -- Ethernet header (bytes 8..21)
        -- Destination MAC: FF:FF:FF:FF:FF:FF
        f(8)  := x"FF"; f(9)  := x"FF"; f(10) := x"FF";
        f(11) := x"FF"; f(12) := x"FF"; f(13) := x"FF";
        -- Source MAC: 02:00:00:00:00:01
        f(14) := x"02"; f(15) := x"00"; f(16) := x"00";
        f(17) := x"00"; f(18) := x"00"; f(19) := x"01";
        -- EtherType: 0x0800 (IPv4)
        f(20) := x"08"; f(21) := x"00";

        -- IPv4 header (bytes 22..41, 20 bytes)
        -- Ver=4 IHL=5 → 0x45, DSCP/ECN=0, TotalLen=1062 (20+8+10+1024)
        -- TotalLen = 1062 = 0x0426
        f(22) := x"45"; f(23) := x"00";
        f(24) := x"04"; f(25) := x"26";  -- total length 1062
        f(26) := x"00"; f(27) := x"01";  -- ID
        f(28) := x"00"; f(29) := x"00";  -- flags/fragment offset
        f(30) := x"40";                  -- TTL = 64
        f(31) := x"11";                  -- Protocol = 17 (UDP)
        f(32) := x"00"; f(33) := x"00";  -- checksum (0 = ignored by MAC)
        -- Src IP: 192.168.1.10
        f(34) := x"C0"; f(35) := x"A8"; f(36) := x"01"; f(37) := x"0A";
        -- Dst IP: 192.168.1.255
        f(38) := x"C0"; f(39) := x"A8"; f(40) := x"01"; f(41) := x"FF";

        -- UDP header (bytes 42..49, 8 bytes)
        -- Src port 5000 = 0x1388, Dst port 5000 = 0x1388
        f(42) := x"13"; f(43) := x"88";  -- src port
        f(44) := x"13"; f(45) := x"88";  -- dst port
        -- UDP length = 8 + 10 + 1024 = 1042 = 0x0412
        f(46) := x"04"; f(47) := x"12";
        f(48) := x"00"; f(49) := x"00";  -- checksum (ignored)

        -- App header (bytes 50..59)
        -- Sync word: 0x0DF05AA5
        f(50) := x"0D"; f(51) := x"F0"; f(52) := x"5A"; f(53) := x"A5";
        -- Sequence number: 1
        f(54) := x"00"; f(55) := x"00"; f(56) := x"00"; f(57) := x"01";
        -- frag_id = 0, frag_total = 1
        f(58) := x"00"; f(59) := x"01";

        -- Audio payload (bytes 60..1083): 0xAB
        for k in 60 to 60 + AUDIO_BYTES - 1 loop
            f(k) := x"AB";
        end loop;

        -- FCS (bytes 1084..1087): dummy zeros
        f(1084) := x"00"; f(1085) := x"00";
        f(1086) := x"00"; f(1087) := x"00";

        return f;
    end function;

    constant FRAME : byte_array_t := build_frame;

    -- -------------------------------------------------------------------------
    -- Simulation control
    -- -------------------------------------------------------------------------
    signal sim_done : boolean := false;

begin

    -- -------------------------------------------------------------------------
    -- Clock generation
    -- -------------------------------------------------------------------------
    CLK100MHZ <= not CLK100MHZ after T_SYS / 2 when not sim_done else '0';
    RMII_CLK  <= not RMII_CLK  after T_RMII / 2 when not sim_done else '0';

    -- -------------------------------------------------------------------------
    -- Inout drive: TB drives when RMII_CRS_DV is asserted by TB
    -- The MAC's IOBUFs drive the same nets during reset (mode strap phase).
    -- Use Z when the TB is not transmitting to avoid conflicts.
    -- -------------------------------------------------------------------------
    RMII_RX_DATA <= tb_rxd   when tb_crsdv = '1' else "ZZ";
    RMII_CRS_DV  <= tb_crsdv when tb_crsdv = '1' else 'Z';
    RMII_RX_ERR  <= tb_rxerr;
    ETH_MDIO     <= 'Z';  -- pulled up externally, not driven by TB

    -- -------------------------------------------------------------------------
    -- DUT instantiation
    -- -------------------------------------------------------------------------
    dut : main
        port map (
            CLK100MHZ    => CLK100MHZ,
            RESET        => RESET,
            RMII_CLK     => RMII_CLK,
            RMII_RX_DATA => RMII_RX_DATA,
            RMII_CRS_DV  => RMII_CRS_DV,
            RMII_RX_ERR  => RMII_RX_ERR,
            ETH_RSTN     => ETH_RSTN,
            ETH_MDC      => ETH_MDC,
            ETH_MDIO     => ETH_MDIO,
            SSD_CATHODE  => SSD_CATHODE,
            SSD_ANODE    => SSD_ANODE,
            LED          => LED
        );

    -- -------------------------------------------------------------------------
    -- Stimulus process
    -- -------------------------------------------------------------------------
    stim : process
        -- Send one byte LSB-first as 4 RMII dibits on the rising edge of RMII_CLK
        procedure send_byte(byte : std_logic_vector(7 downto 0)) is
        begin
            for d in 0 to 3 loop
                tb_rxd <= byte(d*2+1 downto d*2);
                wait until rising_edge(RMII_CLK);
            end loop;
        end procedure;

        -- Send the entire frame array
        procedure send_frame is
        begin
            tb_crsdv <= '1';
            tb_rxerr <= '0';
            for i in 0 to FRAME_BYTES-1 loop
                send_byte(FRAME(i));
            end loop;
            tb_crsdv <= '0';
            tb_rxd   <= "00";
        end procedure;

    begin
        -- ---------------------------------------------------------------
        -- 1. Assert reset (active-low → drive '0')
        -- ---------------------------------------------------------------
        RESET <= '0';
        wait for 100 ns;

        -- ---------------------------------------------------------------
        -- 2. De-assert reset
        -- ---------------------------------------------------------------
        RESET <= '1';
        report "RESET released";

        -- ---------------------------------------------------------------
        -- 3. Wait for PHY reset stretcher (~200 ms simulated as 21 M
        --    cycles at 100 MHz).  For simulation we reduce the stretch
        --    by using a shortened wait - if you synthesis-simulate with
        --    the real counter just increase this wait.
        --    20_000_000 cycles × 10 ns = 200 ms real; in sim we just
        --    wait the equivalent number of RMII cycles.
        -- ---------------------------------------------------------------
        report "Waiting for PHY reset stretcher...";
        -- Wait 20_000_001 system clock cycles (mirrors phy_reset_counter)
        for i in 0 to 20_000_000 loop
            wait until rising_edge(CLK100MHZ);
        end loop;
        report "PHY reset released (phy_rstn_reg should be '1')";

        -- ---------------------------------------------------------------
        -- 4. Idle gap before first frame (inter-frame gap ≥ 12 bytes at
        --    50 MHz RMII = 96 bit times = 48 RMII cycles)
        -- ---------------------------------------------------------------
        for i in 0 to 63 loop
            wait until rising_edge(RMII_CLK);
        end loop;

        -- ---------------------------------------------------------------
        -- 5. Transmit valid UDP frame
        -- ---------------------------------------------------------------
        report "Sending UDP frame with sync word 0x0DF05AA5, seq=1...";
        send_frame;
        report "Frame transmission complete";

        -- ---------------------------------------------------------------
        -- 6. Wait for parser to assert PACKET_READY (LED10)
        --    Give it plenty of time: frame is ~1088 bytes × 4 dibits ×
        --    20 ns = ~87 µs, plus parser latency.
        -- ---------------------------------------------------------------
        wait for 200 us;

        -- ---------------------------------------------------------------
        -- 7. Check outputs
        -- ---------------------------------------------------------------
        report "=== RESULTS ===";

        if LED(10) = '1' then
            report "PASS: LED10 (packet_ready_sig) is HIGH - parser completed a packet"
                severity NOTE;
        else
            report "FAIL: LED10 (packet_ready_sig) never asserted - check parser / MAC"
                severity ERROR;
        end if;

        if LED(9) = '1' then
            report "INFO: LED9 (rx_axis_tvalid) still high at end of sim"
                severity NOTE;
        end if;

        if LED(12) = '1' then
            report "INFO: LED12 (link_up) is HIGH"
                severity NOTE;
        end if;

        -- ---------------------------------------------------------------
        -- 8. Send a second frame to verify back-to-back operation
        -- ---------------------------------------------------------------
        for i in 0 to 63 loop
            wait until rising_edge(RMII_CLK);
        end loop;

        report "Sending second frame (seq will not increment in parser, only one seq per call)...";
        send_frame;

        wait for 200 us;

        if LED(3 downto 0) = "0010" then
            report "PASS: packet_count[3:0] = 2 after two frames" severity NOTE;
        else
            report "INFO: packet_count[3:0] = " &
                   integer'image(to_integer(unsigned(LED(3 downto 0))))
                   severity NOTE;
        end if;

        -- ---------------------------------------------------------------
        -- 9. Done
        -- ---------------------------------------------------------------
        report "Simulation complete";
        sim_done <= true;
        wait;
    end process;

    -- -------------------------------------------------------------------------
    -- Watchdog: fail after 500 ms simulated time if nothing interesting happens
    -- -------------------------------------------------------------------------
    watchdog : process
    begin
        wait for 500 ms;
        if not sim_done then
            report "WATCHDOG: simulation exceeded 500 ms - possible hang" severity FAILURE;
        end if;
        wait;
    end process;

    -- -------------------------------------------------------------------------
    -- Monitor: print when key signals change
    -- -------------------------------------------------------------------------
    monitor : process(LED)
    begin
        if LED(10)'event and LED(10) = '1' then
            report ">>> PACKET_READY asserted at " & time'image(now) severity NOTE;
        end if;
        if LED(12)'event then
            report ">>> link_up changed to " & std_logic'image(LED(12)) &
                   " at " & time'image(now) severity NOTE;
        end if;
    end process;

end sim;