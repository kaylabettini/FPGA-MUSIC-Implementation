----------------------------------------------------------------------------------
-- Company: University of Queensland
-- Engineer: Kayla Bettini
--
-- Module Name: sdspi_audio_writer.vhd
-- Description:
--   Writes raw audio bytes from an external FIFO to SD card via ZipCPU sdspi.
--
-- Data path:
--   ethernet_audio_parser -> byte FIFO -> sdspi_audio_writer -> SD card
--
-- This writer waits until at least 512 bytes are available in the FIFO,
-- reads exactly 512 bytes, packs them into 128 x 32-bit words, loads the
-- ZipCPU sdspi FIFO, then issues CMD24 to write one SD sector.
--
-- Important:
--   This version assumes the Xilinx FIFO is STANDARD READ MODE:
--     rd_en pulse now -> dout valid next clk
--
-- If your FIFO is First-Word-Fall-Through, tell me before using this version.
----------------------------------------------------------------------------------

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity sdspi_audio_writer is
    generic (
        CHANNELS    : integer := 8;
        RESOLUTION  : integer := 16;
        START_BLOCK : std_logic_vector(31 downto 0) := x"00000800"
    );
    port (
        clk   : in  std_logic;
        reset : in  std_logic;

        -- Byte stream from external FIFO
        audio_byte       : in  std_logic_vector(7 downto 0);
        audio_byte_valid : in  std_logic;
        audio_byte_ready : out std_logic;
        fifo_count       : in  std_logic_vector(12 downto 0);

        -- SD card SPI pins
        sd_cs_n : out std_logic;
        sd_sck  : out std_logic;
        sd_mosi : out std_logic;
        sd_miso : in  std_logic;

        -- Debug/status
        writer_busy      : out std_logic;
        done             : out std_logic;
        error            : out std_logic;
        debug_out        : out std_logic_vector(31 downto 0);
        state_debug      : out std_logic_vector(7 downto 0);
        last_error_state : out std_logic_vector(7 downto 0);
        status_debug     : out std_logic_vector(31 downto 0);
        cmd_debug        : out std_logic_vector(7 downto 0)
    );
end sdspi_audio_writer;

architecture Behavioral of sdspi_audio_writer is

    component sdspi
        generic (
            OPT_CARD_DETECT     : integer := 0;
            OPT_LITTLE_ENDIAN   : integer := 0;
            LGFIFOLN            : integer := 7;
            POWERUP_IDLE        : integer := 50000;
            STARTUP_CLOCKS      : integer := 80;
            CKDIV_BITS          : integer := 8;
            INITIAL_CLKDIV      : std_logic_vector(7 downto 0) := x"7C";
            OPT_SPI_ARBITRATION : integer := 0;
            OPT_EXTRA_WB_CLOCK  : integer := 0
        );
        port (
            i_clk      : in  std_logic;
            i_sd_reset : in  std_logic;
            i_wb_cyc   : in  std_logic;
            i_wb_stb   : in  std_logic;
            i_wb_we    : in  std_logic;
            i_wb_addr  : in  std_logic_vector(1 downto 0);
            i_wb_data  : in  std_logic_vector(31 downto 0);
            i_wb_sel   : in  std_logic_vector(3 downto 0);
            o_wb_stall : out std_logic;
            o_wb_ack   : out std_logic;
            o_wb_data  : out std_logic_vector(31 downto 0);

            o_cs_n        : out std_logic;
            o_sck         : out std_logic;
            o_mosi        : out std_logic;
            i_miso        : in  std_logic;
            i_card_detect : in  std_logic;

            o_int       : out std_logic;
            i_bus_grant : in  std_logic;
            o_debug     : out std_logic_vector(31 downto 0)
        );
    end component;

    constant REG_CMD    : std_logic_vector(1 downto 0) := "00";
    constant REG_DATA   : std_logic_vector(1 downto 0) := "01";
    constant REG_FIFO_A : std_logic_vector(1 downto 0) := "10";

    constant CMD0  : natural := 0;
    constant CMD8  : natural := 8;
    constant CMD16 : natural := 16;
    constant CMD24 : natural := 24;
    constant CMD41 : natural := 41;
    constant CMD55 : natural := 55;
    constant CMD58 : natural := 58;

    constant BYTES_PER_BLOCK : integer := 512;
    constant WORDS_PER_BLOCK : integer := 128;

    type sector_buf_type is array (0 to WORDS_PER_BLOCK - 1) of std_logic_vector(31 downto 0);
    signal sector_buf : sector_buf_type := (others => (others => '0'));

    signal byte_index      : integer range 0 to BYTES_PER_BLOCK - 1 := 0;
    signal word_index      : integer range 0 to WORDS_PER_BLOCK - 1 := 0;
    signal byte_lane       : integer range 0 to 3 := 0;
    signal fifo_load_idx   : integer range 0 to WORDS_PER_BLOCK - 1 := 0;

    signal audio_byte_ready_reg : std_logic := '0';

    signal wb_cyc   : std_logic := '0';
    signal wb_stb   : std_logic := '0';
    signal wb_we    : std_logic := '0';
    signal wb_addr  : std_logic_vector(1 downto 0) := (others => '0');
    signal wb_wdata : std_logic_vector(31 downto 0) := (others => '0');
    signal wb_sel   : std_logic_vector(3 downto 0) := x"F";

    signal wb_stall        : std_logic;
    signal wb_ack          : std_logic;
    signal wb_rdata        : std_logic_vector(31 downto 0);
    signal wb_read_latched : std_logic_vector(31 downto 0) := (others => '0');

    signal sd_int       : std_logic;
    signal sd_debug_sig : std_logic_vector(31 downto 0);

    signal cmd_index      : natural range 0 to 63 := 0;
    signal cmd_arg        : std_logic_vector(31 downto 0) := (others => '0');
    signal cmd_fifo_write : std_logic := '0';
    signal last_status    : std_logic_vector(31 downto 0) := (others => '0');
    signal use_cmd1       : std_logic := '0';

    signal block_addr : std_logic_vector(31 downto 0) := START_BLOCK;

    signal done_reg  : std_logic := '0';
    signal error_reg : std_logic := '0';

    signal state_debug_int         : std_logic_vector(7 downto 0);
    signal last_state_before_error : std_logic_vector(7 downto 0) := (others => '0');

    type state_type is (
        ST_RESET_WAIT,

        ST_START_CMD0,
        ST_START_CMD8,
        ST_START_CMD55,
        ST_START_ACMD41,
        ST_CHECK_ACMD41,
        ST_START_CMD58,
        ST_START_CMD16,
        ST_START_CMD1,

        ST_CMD_ARG_WRITE,
        ST_CMD_SEND_WRITE,
        ST_CMD_POLL_WAIT,
        ST_CMD_POLL_READ,
        ST_CMD_POLL_CHECK,

        ST_WAIT_FIFO,
        ST_FILL_REQ,
        ST_FILL_WAIT,
        ST_FILL_CAPTURE,

        ST_CLEAR_ERR,
        ST_SD_FIFO_LOAD,
        ST_SD_FIFO_LOAD_WAIT,
        ST_START_WRITE_BLOCK,
        ST_WRITE_DONE,

        ST_WB_WAIT,
        ST_ERROR
    );

    signal state      : state_type := ST_RESET_WAIT;
    signal wb_return  : state_type := ST_RESET_WAIT;
    signal cmd_return : state_type := ST_RESET_WAIT;

    signal wait_counter : unsigned(23 downto 0) := (others => '0');
    signal init_tries   : unsigned(25 downto 0) := (others => '0');

    function make_cmd(cmd : natural) return std_logic_vector is
        variable r : std_logic_vector(31 downto 0);
    begin
        r := (others => '0');
        r(7 downto 6) := "01";
        r(5 downto 0) := std_logic_vector(to_unsigned(cmd, 6));
        return r;
    end function;

    function make_fifo_write_cmd(cmd : natural) return std_logic_vector is
        variable r : std_logic_vector(31 downto 0);
    begin
        r := (others => '0');

        -- Start command and tell sdspi to use its internal data FIFO.
        r(11) := '1';
        r(10) := '1';
        r(7 downto 6) := "01";
        r(5 downto 0) := std_logic_vector(to_unsigned(cmd, 6));

        return r;
    end function;

begin

    audio_byte_ready <= audio_byte_ready_reg;

    done             <= done_reg;
    error            <= error_reg;
    debug_out        <= sd_debug_sig;
    writer_busy      <= '0' when state = ST_WAIT_FIFO else '1';
    state_debug      <= state_debug_int;
    last_error_state <= last_state_before_error;
    status_debug     <= wb_read_latched;
    cmd_debug        <= std_logic_vector(to_unsigned(cmd_index, 8));

    sdspi_inst : sdspi
        generic map (
            OPT_CARD_DETECT     => 0,
            OPT_LITTLE_ENDIAN   => 0,
            LGFIFOLN            => 7,
            POWERUP_IDLE        => 50000,
            STARTUP_CLOCKS      => 80,
            CKDIV_BITS          => 8,
            INITIAL_CLKDIV      => x"00",
            OPT_SPI_ARBITRATION => 0,
            OPT_EXTRA_WB_CLOCK  => 0
        )
        port map (
            i_clk      => clk,
            i_sd_reset => reset,

            i_wb_cyc   => wb_cyc,
            i_wb_stb   => wb_stb,
            i_wb_we    => wb_we,
            i_wb_addr  => wb_addr,
            i_wb_data  => wb_wdata,
            i_wb_sel   => wb_sel,

            o_wb_stall => wb_stall,
            o_wb_ack   => wb_ack,
            o_wb_data  => wb_rdata,

            o_cs_n        => sd_cs_n,
            o_sck         => sd_sck,
            o_mosi        => sd_mosi,
            i_miso        => sd_miso,
            i_card_detect => '1',

            o_int       => sd_int,
            i_bus_grant => '1',
            o_debug     => sd_debug_sig
        );

    main_fsm : process(clk)
    begin
        if rising_edge(clk) then
            if reset = '1' then
                state           <= ST_RESET_WAIT;
                wb_return       <= ST_RESET_WAIT;
                cmd_return      <= ST_RESET_WAIT;

                wb_cyc          <= '0';
                wb_stb          <= '0';
                wb_we           <= '0';
                wb_addr         <= (others => '0');
                wb_wdata        <= (others => '0');
                wb_sel          <= x"F";
                wb_read_latched <= (others => '0');

                wait_counter    <= (others => '0');
                init_tries      <= (others => '0');

                cmd_index       <= 0;
                cmd_arg         <= (others => '0');
                cmd_fifo_write  <= '0';
                last_status     <= (others => '0');
                use_cmd1        <= '0';

                block_addr      <= START_BLOCK;

                byte_index      <= 0;
                word_index      <= 0;
                byte_lane       <= 0;
                fifo_load_idx   <= 0;

                audio_byte_ready_reg <= '0';

                done_reg        <= '0';
                error_reg       <= '0';

            else
                -- Defaults every cycle
                audio_byte_ready_reg <= '0';

                case state is

                    ----------------------------------------------------------------
                    -- SD initialisation
                    ----------------------------------------------------------------

                    when ST_RESET_WAIT =>
                        wb_cyc <= '0';
                        wb_stb <= '0';
                        wb_we  <= '0';

                        if wait_counter < to_unsigned(5_000_000, wait_counter'length) then
                            wait_counter <= wait_counter + 1;
                        else
                            wait_counter <= (others => '0');
                            state <= ST_START_CMD0;
                        end if;

                    when ST_START_CMD0 =>
                        cmd_arg        <= x"00000000";
                        cmd_index      <= CMD0;
                        cmd_fifo_write <= '0';
                        cmd_return     <= ST_START_CMD8;
                        state          <= ST_CMD_ARG_WRITE;

                    when ST_START_CMD8 =>
                        cmd_arg        <= x"000001AA";
                        cmd_index      <= CMD8;
                        cmd_fifo_write <= '0';
                        cmd_return     <= ST_START_CMD55;
                        state          <= ST_CMD_ARG_WRITE;

                    when ST_START_CMD55 =>
                        cmd_arg        <= x"00000000";
                        cmd_index      <= CMD55;
                        cmd_fifo_write <= '0';
                        cmd_return     <= ST_START_ACMD41;
                        state          <= ST_CMD_ARG_WRITE;

                    when ST_START_ACMD41 =>
                        cmd_arg        <= x"40000000";
                        cmd_index      <= CMD41;
                        cmd_fifo_write <= '0';
                        cmd_return     <= ST_CHECK_ACMD41;
                        state          <= ST_CMD_ARG_WRITE;

                    when ST_START_CMD1 =>
                        cmd_arg        <= x"40000000";
                        cmd_index      <= 1;
                        cmd_fifo_write <= '0';
                        cmd_return     <= ST_CHECK_ACMD41;
                        state          <= ST_CMD_ARG_WRITE;

                    when ST_CHECK_ACMD41 =>
                        if wb_read_latched(7 downto 0) = x"00" then
                            init_tries <= (others => '0');
                            state      <= ST_START_CMD58;

                        elsif wb_read_latched(7 downto 0) = x"01" then
                            if init_tries < to_unsigned(50_000_000, init_tries'length) then
                                init_tries <= init_tries + 1;

                                if use_cmd1 = '1' then
                                    state <= ST_START_CMD1;
                                else
                                    state <= ST_START_CMD55;
                                end if;
                            else
                                error_reg <= '1';
                                state <= ST_ERROR;
                            end if;

                        else
                            error_reg <= '1';
                            state <= ST_ERROR;
                        end if;

                    when ST_START_CMD58 =>
                        cmd_arg        <= x"00000000";
                        cmd_index      <= CMD58;
                        cmd_fifo_write <= '0';
                        cmd_return     <= ST_START_CMD16;
                        state          <= ST_CMD_ARG_WRITE;

                    when ST_START_CMD16 =>
                        cmd_arg        <= x"00000200";
                        cmd_index      <= CMD16;
                        cmd_fifo_write <= '0';
                        cmd_return     <= ST_WAIT_FIFO;
                        state          <= ST_CMD_ARG_WRITE;

                    ----------------------------------------------------------------
                    -- Wait for 512 bytes, then pull one full sector from byte FIFO
                    ----------------------------------------------------------------

                    when ST_WAIT_FIFO =>
                        done_reg <= '0';
                    
                        byte_index    <= 0;
                        word_index    <= 0;
                        byte_lane     <= 0;
                        fifo_load_idx <= 0;
                    
                        if unsigned(fifo_count) >= to_unsigned(BYTES_PER_BLOCK, fifo_count'length) then
                            state <= ST_FILL_REQ;
                        end if;

                    -- Standard FIFO read:
                    -- assert ready for one clock. Top-level uses this as fifo_rd_en.
                    when ST_FILL_REQ =>
                        if audio_byte_valid = '1' then
                            audio_byte_ready_reg <= '1';
                            state <= ST_FILL_WAIT;
                        else
                            state <= ST_WAIT_FIFO;
                        end if;
                    
                    when ST_FILL_WAIT =>
                        state <= ST_FILL_CAPTURE;
                        
                    -- Capture FIFO dout one clock after rd_en.
                    when ST_FILL_CAPTURE =>
                        case byte_lane is
                            when 0 =>
                                sector_buf(word_index)(31 downto 24) <= audio_byte;
                            when 1 =>
                                sector_buf(word_index)(23 downto 16) <= audio_byte;
                            when 2 =>
                                sector_buf(word_index)(15 downto 8) <= audio_byte;
                            when others =>
                                sector_buf(word_index)(7 downto 0) <= audio_byte;
                        end case;
                    
                        if byte_index = BYTES_PER_BLOCK - 1 then
                            byte_index    <= 0;
                            word_index    <= 0;
                            byte_lane     <= 0;
                            fifo_load_idx <= 0;
                            state         <= ST_CLEAR_ERR;
                    
                        else
                            byte_index <= byte_index + 1;
                    
                            if byte_lane = 3 then
                                byte_lane  <= 0;
                                word_index <= word_index + 1;
                            else
                                byte_lane <= byte_lane + 1;
                            end if;
                    
                            state <= ST_FILL_REQ;
                        end if;
    
                    ----------------------------------------------------------------
                    -- Load sdspi FIFO and write block
                    ----------------------------------------------------------------

                    when ST_CLEAR_ERR =>
                        wb_cyc    <= '1';
                        wb_stb    <= '1';
                        wb_we     <= '1';
                        wb_addr   <= REG_CMD;
                        wb_wdata  <= x"00008000";
                        wb_sel    <= x"F";
                        wb_return <= ST_SD_FIFO_LOAD;
                        state     <= ST_WB_WAIT;

                    when ST_SD_FIFO_LOAD =>
                        wb_cyc    <= '1';
                        wb_stb    <= '1';
                        wb_we     <= '1';
                        wb_addr   <= REG_FIFO_A;
                        wb_wdata  <= sector_buf(fifo_load_idx);
                        wb_sel    <= x"F";
                        wb_return <= ST_SD_FIFO_LOAD_WAIT;
                        state     <= ST_WB_WAIT;

                    when ST_SD_FIFO_LOAD_WAIT =>
                        if fifo_load_idx = WORDS_PER_BLOCK - 1 then
                            fifo_load_idx <= 0;
                            state <= ST_START_WRITE_BLOCK;
                        else
                            fifo_load_idx <= fifo_load_idx + 1;
                            state <= ST_SD_FIFO_LOAD;
                        end if;

                    when ST_START_WRITE_BLOCK =>
                        cmd_arg        <= block_addr;
                        cmd_index      <= CMD24;
                        cmd_fifo_write <= '1';
                        cmd_return     <= ST_WRITE_DONE;
                        state          <= ST_CMD_ARG_WRITE;

                    when ST_WRITE_DONE =>
                        done_reg   <= '1';
                        block_addr <= std_logic_vector(unsigned(block_addr) + 1);
                        state      <= ST_WAIT_FIFO;

                    ----------------------------------------------------------------
                    -- Generic command/write/read helpers
                    ----------------------------------------------------------------

                    when ST_CMD_ARG_WRITE =>
                        wb_cyc    <= '1';
                        wb_stb    <= '1';
                        wb_we     <= '1';
                        wb_addr   <= REG_DATA;
                        wb_wdata  <= cmd_arg;
                        wb_sel    <= x"F";
                        wb_return <= ST_CMD_SEND_WRITE;
                        state     <= ST_WB_WAIT;

                    when ST_CMD_SEND_WRITE =>
                        wb_cyc <= '1';
                        wb_stb <= '1';
                        wb_we  <= '1';
                        wb_addr <= REG_CMD;
                        wb_sel <= x"F";

                        if cmd_fifo_write = '1' then
                            wb_wdata <= make_fifo_write_cmd(cmd_index);
                        else
                            wb_wdata <= make_cmd(cmd_index);
                        end if;

                        wait_counter <= (others => '0');
                        wb_return    <= ST_CMD_POLL_WAIT;
                        state        <= ST_WB_WAIT;

                    when ST_CMD_POLL_WAIT =>
                        if wait_counter < to_unsigned(8, wait_counter'length) then
                            wait_counter <= wait_counter + 1;
                        else
                            wait_counter <= (others => '0');
                            state <= ST_CMD_POLL_READ;
                        end if;

                    when ST_CMD_POLL_READ =>
                        wb_cyc    <= '1';
                        wb_stb    <= '1';
                        wb_we     <= '0';
                        wb_addr   <= REG_CMD;
                        wb_sel    <= x"F";
                        wb_return <= ST_CMD_POLL_CHECK;
                        state     <= ST_WB_WAIT;

                    when ST_CMD_POLL_CHECK =>
                        last_status <= wb_read_latched;

                        if wb_read_latched(14) = '1' then
                            state <= ST_CMD_POLL_READ;
                        else
                            case cmd_index is

                                when CMD0 =>
                                    if wb_read_latched(7 downto 0) = x"01" then
                                        state <= cmd_return;
                                    else
                                        error_reg <= '1';
                                        state <= ST_ERROR;
                                    end if;

                                when CMD8 =>
                                    if wb_read_latched(7 downto 0) = x"01" or
                                       wb_read_latched(7 downto 0) = x"05" then
                                        state <= cmd_return;
                                    else
                                        error_reg <= '1';
                                        state <= ST_ERROR;
                                    end if;

                                when CMD55 =>
                                    if wb_read_latched(7 downto 0) = x"01" or
                                       wb_read_latched(7 downto 0) = x"00" then
                                        state <= cmd_return;
                                    elsif wb_read_latched(7 downto 0) = x"05" or
                                          wb_read_latched(7 downto 0) = x"07" then
                                        use_cmd1 <= '1';
                                        state <= ST_START_CMD1;
                                    else
                                        error_reg <= '1';
                                        state <= ST_ERROR;
                                    end if;

                                when CMD41 =>
                                    if wb_read_latched(7 downto 0) = x"01" or
                                       wb_read_latched(7 downto 0) = x"00" then
                                        state <= cmd_return;
                                    else
                                        error_reg <= '1';
                                        state <= ST_ERROR;
                                    end if;

                                when 1 =>
                                    if wb_read_latched(7 downto 0) = x"01" or
                                       wb_read_latched(7 downto 0) = x"00" then
                                        state <= cmd_return;
                                    else
                                        error_reg <= '1';
                                        state <= ST_ERROR;
                                    end if;

                                when CMD58 =>
                                    state <= cmd_return;

                                when CMD16 =>
                                    state <= cmd_return;

                                when CMD24 =>
                                    if wb_read_latched(15) = '1' and
                                       wb_read_latched(7 downto 0) /= x"00" then
                                        error_reg <= '1';
                                        state <= ST_ERROR;
                                    else
                                        state <= cmd_return;
                                    end if;

                                when others =>
                                    error_reg <= '1';
                                    state <= ST_ERROR;

                            end case;
                        end if;

                    when ST_WB_WAIT =>
                        wb_stb <= '0';

                        if wb_ack = '1' then
                            wb_read_latched <= wb_rdata;
                            wb_cyc <= '0';
                            wb_we  <= '0';
                            state  <= wb_return;
                        end if;

                    when ST_ERROR =>
                        error_reg <= '1';
                        wb_cyc <= '0';
                        wb_stb <= '0';
                        wb_we  <= '0';
                        audio_byte_ready_reg <= '0';

                    when others =>
                        state <= ST_ERROR;

                end case;
            end if;
        end if;
    end process;

    with state select state_debug_int <=
        x"00" when ST_RESET_WAIT,

        x"01" when ST_START_CMD0,
        x"02" when ST_START_CMD8,
        x"03" when ST_START_CMD55,
        x"04" when ST_START_ACMD41,
        x"05" when ST_CHECK_ACMD41,
        x"06" when ST_START_CMD58,
        x"07" when ST_START_CMD16,
        x"12" when ST_START_CMD1,

        x"08" when ST_CMD_ARG_WRITE,
        x"09" when ST_CMD_SEND_WRITE,
        x"EF" when ST_CMD_POLL_WAIT,
        x"0A" when ST_CMD_POLL_READ,
        x"0B" when ST_CMD_POLL_CHECK,

        x"0C" when ST_WAIT_FIFO,
        x"20" when ST_FILL_REQ,
        x"21" when ST_FILL_CAPTURE,
        x"22" when ST_FILL_WAIT,

        x"13" when ST_CLEAR_ERR,
        x"0D" when ST_SD_FIFO_LOAD,
        x"0E" when ST_SD_FIFO_LOAD_WAIT,
        x"10" when ST_START_WRITE_BLOCK,
        x"11" when ST_WRITE_DONE,

        x"EE" when ST_ERROR,
        x"FF" when others;

    process(clk)
    begin
        if rising_edge(clk) then
            if state /= ST_ERROR then
                last_state_before_error <= state_debug_int;
            end if;
        end if;
    end process;

end Behavioral;