----------------------------------------------------------------------------------
-- Company:        University of Queensland
-- Engineer:       Kayla Bettini
--
-- Module Name:    ethernet_audio_parser
-- Project Name:   Implementation of the MUSIC algorithm on an FPGA
-- Target Devices: Nexys 4 (XC7A100T)
-- Description:    Extracts multichannel audio frames from the UDP stream sent by
--                 the Raspberry Pi.
--
--                 Packet payload (one fragment per packet):
--                 - Bytes 0..3  sync word 0xF00DA55A (big-endian)
--                 - Bytes 4..7  sequence number (big-endian)
--                 - Byte  8     fragment id
--                 - Byte  9     fragment total
--                 - Then FRAMES_PER_PACKET frames x CHANNELS samples,
--                   little-endian, interleaved by channel
--
--                 The sync word is searched for in the byte stream, so the
--                 Ethernet, IP and UDP headers before it are skipped.
----------------------------------------------------------------------------------

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity ethernet_audio_parser is
    Generic (
        CHANNELS          : integer := 4;
        RESOLUTION        : integer := 16;
        FRAMES_PER_PACKET : integer := 64
    );
    Port (
        -- Clock and reset
        CLK                : in  STD_LOGIC;
        RESET              : in  STD_LOGIC;

        -- Ethernet MAC receive stream
        RX_DATA            : in  STD_LOGIC_VECTOR(7 downto 0);
        RX_VALID           : in  STD_LOGIC;
        RX_LAST            : in  STD_LOGIC;

        -- Raw payload bytes (debug)
        AUDIO_DATA_OUT     : out STD_LOGIC_VECTOR(7 downto 0);
        AUDIO_BYTE_VALID   : out STD_LOGIC;

        -- Audio frames
        AUDIO_CHANNELS_OUT : out STD_LOGIC_VECTOR((RESOLUTION*CHANNELS)-1 downto 0); -- Channel 1 in the LSBs
        AUDIO_FRAME_VALID  : out STD_LOGIC;                                            -- 1-clock pulse

        -- Packet header fields and status
        SEQ_NUM_OUT        : out STD_LOGIC_VECTOR(31 downto 0);
        FRAG_ID_OUT        : out STD_LOGIC_VECTOR(7 downto 0);
        FRAG_TOTAL_OUT     : out STD_LOGIC_VECTOR(7 downto 0);
        HEADER_OUT         : out STD_LOGIC_VECTOR(31 downto 0);
        PACKET_READY       : out STD_LOGIC;                                            -- Pulse at end of payload
        DEBUG_STATE        : out STD_LOGIC_VECTOR(2 downto 0)
    );
end ethernet_audio_parser;

architecture Behavioral of ethernet_audio_parser is

    --==========================================================================
    -- Constants and signals
    --==========================================================================
    type state_type is (IDLE, READ_METADATA, EXTRACT_AUDIO, DROP_FRAME);
    constant BYTES_PER_SAMPLE : integer := RESOLUTION / 8;
    constant SYNC_WORD        : std_logic_vector(31 downto 0) := x"F00DA55A";

    -- State and header fields
    signal state          : state_type := IDLE;
    signal meta_count     : integer range 0 to 5 := 0;
    signal sync_buf       : std_logic_vector(23 downto 0) := (others => '0');
    signal seq_num_reg    : std_logic_vector(31 downto 0) := (others => '0');
    signal frag_id_reg    : std_logic_vector(7 downto 0)  := (others => '0');
    signal frag_total_reg : std_logic_vector(7 downto 0)  := (others => '0');
    signal header_reg     : std_logic_vector(31 downto 0) := (others => '0');

    -- Sample and frame assembly
    signal sample         : std_logic_vector(RESOLUTION-1 downto 0) := (others => '0');
    signal frame_reg      : std_logic_vector((RESOLUTION*CHANNELS)-1 downto 0) := (others => '0');
    signal frame_out      : std_logic_vector((RESOLUTION*CHANNELS)-1 downto 0) := (others => '0');
    signal byte_in_sample : integer range 0 to BYTES_PER_SAMPLE-1 := 0;
    signal channel_count  : integer range 0 to CHANNELS-1 := 0;
    signal frame_count    : integer range 0 to FRAMES_PER_PACKET := 0;

    -- Output registers
    signal byte_valid_reg  : std_logic := '0';
    signal byte_out_reg    : std_logic_vector(7 downto 0) := (others => '0');
    signal frame_valid_reg : std_logic := '0';
    signal packet_rdy_reg  : std_logic := '0';

begin

    --==========================================================================
    -- Outputs
    --==========================================================================
    SEQ_NUM_OUT        <= seq_num_reg;
    FRAG_ID_OUT        <= frag_id_reg;
    FRAG_TOTAL_OUT     <= frag_total_reg;
    HEADER_OUT         <= header_reg;
    PACKET_READY       <= packet_rdy_reg;
    AUDIO_DATA_OUT     <= byte_out_reg;
    AUDIO_BYTE_VALID   <= byte_valid_reg;
    AUDIO_CHANNELS_OUT <= frame_out;
    AUDIO_FRAME_VALID  <= frame_valid_reg;

    with state select DEBUG_STATE <=
        "000" when IDLE,
        "010" when READ_METADATA,
        "011" when EXTRACT_AUDIO,
        "100" when DROP_FRAME;

    --==========================================================================
    -- Parser state machine
    --==========================================================================
    process (CLK)
        variable s : std_logic_vector(RESOLUTION-1 downto 0);
        variable f : std_logic_vector((RESOLUTION*CHANNELS)-1 downto 0);
    begin
        if rising_edge(CLK) then
            byte_valid_reg  <= '0';
            frame_valid_reg <= '0';
            packet_rdy_reg  <= '0';

            if RESET = '1' then
                state          <= IDLE;
                sync_buf       <= (others => '0');
                meta_count     <= 0;
                byte_in_sample <= 0;
                channel_count  <= 0;
                frame_count    <= 0;
            elsif RX_VALID = '1' then
                case state is

                    -- Search the byte stream for the sync word
                    when IDLE =>
                        sync_buf <= sync_buf(15 downto 0) & RX_DATA;
                        if (sync_buf & RX_DATA) = SYNC_WORD then
                            header_reg     <= sync_buf & RX_DATA;
                            meta_count     <= 0;
                            byte_in_sample <= 0;       -- Start every packet aligned
                            channel_count  <= 0;
                            frame_count    <= 0;
                            state          <= READ_METADATA;
                        end if;

                    -- Sequence number, fragment id and fragment total
                    when READ_METADATA =>
                        case meta_count is
                            when 0 => seq_num_reg(31 downto 24) <= RX_DATA;
                            when 1 => seq_num_reg(23 downto 16) <= RX_DATA;
                            when 2 => seq_num_reg(15 downto 8)  <= RX_DATA;
                            when 3 => seq_num_reg(7 downto 0)   <= RX_DATA;
                            when 4 => frag_id_reg               <= RX_DATA;
                            when others => frag_total_reg       <= RX_DATA;
                        end case;
                        if meta_count = 5 then
                            state <= EXTRACT_AUDIO;
                        else
                            meta_count <= meta_count + 1;
                        end if;

                    -- Assemble bytes into samples and samples into frames
                    when EXTRACT_AUDIO =>
                        byte_out_reg   <= RX_DATA;
                        byte_valid_reg <= '1';

                        -- Little-endian: the first byte is the LSB
                        s := sample;
                        s(8*byte_in_sample + 7 downto 8*byte_in_sample) := RX_DATA;
                        sample <= s;

                        if byte_in_sample = BYTES_PER_SAMPLE-1 then
                            byte_in_sample <= 0;
                            f := frame_reg;
                            f((channel_count+1)*RESOLUTION-1 downto channel_count*RESOLUTION) := s;
                            frame_reg <= f;
                            if channel_count = CHANNELS-1 then
                                channel_count   <= 0;
                                frame_out       <= f;
                                frame_valid_reg <= '1';
                                if frame_count = FRAMES_PER_PACKET-1 then
                                    packet_rdy_reg <= '1';
                                    if RX_LAST = '1' then state <= IDLE; else state <= DROP_FRAME; end if;
                                else
                                    frame_count <= frame_count + 1;
                                end if;
                            else
                                channel_count <= channel_count + 1;
                            end if;
                        else
                            byte_in_sample <= byte_in_sample + 1;
                        end if;

                        if RX_LAST = '1' then           -- Short packet: give up on it
                            state <= IDLE;
                        end if;

                    -- Ignore trailing bytes (FCS etc.)
                    when DROP_FRAME =>
                        if RX_LAST = '1' then
                            state <= IDLE;
                        end if;

                end case;

            elsif RX_LAST = '1' then
                state <= IDLE;
            end if;

            if RX_VALID = '1' and RX_LAST = '1' then
                sync_buf <= (others => '0');            -- Never match across packets
            end if;
        end if;
    end process;

end Behavioral;