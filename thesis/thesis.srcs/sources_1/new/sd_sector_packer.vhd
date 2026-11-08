----------------------------------------------------------------------------------
-- Company: University of Queensland
-- Engineer: Kayla Bettini
-- 
-- Create Date: 03.06.2026 13:05:20
-- Module Name: sd_sector_packer.vhd 
-- Project Name: Implementation of the MUSIC algorithm on an FPGA
-- Target Devices: Nexys 4
-- Description: File that handles receiving bytes from the FIFO buffer and then
-- prepares the bytes into blocks of 512 bytes for SD card saving
-- 
-- Revision: 1.0
-- Revision 1.0 - File Created
----------------------------------------------------------------------------------

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity sd_sector_packer is
    Port (
        clk   : in  std_logic;
        reset : in  std_logic;

        -- FIFO read side
        fifo_dout       : in  std_logic_vector(7 downto 0);
        fifo_empty      : in  std_logic;
        fifo_rd_en      : out std_logic;
        fifo_has_sector : in std_logic;

        -- Sector output stream
        sector_data  : out std_logic_vector(7 downto 0);
        sector_valid : out std_logic;
        sector_last  : out std_logic;

        -- SD writer handshake
        sector_ready : out std_logic;
        sd_busy      : in  std_logic;
        sd_accept    : in  std_logic
    );
end sd_sector_packer;

architecture Behavioral of sd_sector_packer is

    type state_type is (IDLE, READ_FIFO, OUTPUT_BYTE, WAIT_FOR_DATA, WAIT_WRITER);
    signal state : state_type := IDLE;

    signal byte_counter : unsigned(8 downto 0) := (others => '0'); -- 0 to 511
    signal data_reg     : std_logic_vector(7 downto 0) := (others => '0');

begin

process(clk)
begin
    if rising_edge(clk) then
        if reset = '1' then
            state         <= IDLE;
            byte_counter  <= (others => '0');
            data_reg      <= (others => '0');

            fifo_rd_en    <= '0';
            sector_data   <= (others => '0');
            sector_valid  <= '0';
            sector_last   <= '0';
            sector_ready  <= '0';

        else
            fifo_rd_en   <= '0';
            sector_valid <= '0';
            sector_last  <= '0';
            sector_ready <= '0';

            case state is
                when IDLE =>
                    byte_counter <= (others => '0');

                    -- Only START when FIFO has at least 512 bytes.
                    if sd_busy = '0' and fifo_has_sector = '1' then
                        fifo_rd_en <= '1';
                        state      <= READ_FIFO;
                    end if;
                -- Standard FIFO mode: dout valid one clock after rd_en.
                when READ_FIFO =>
                    data_reg <= fifo_dout;
                    state    <= OUTPUT_BYTE;
                when OUTPUT_BYTE =>
                    if sd_accept = '1' then
                        sector_data  <= data_reg;
                        sector_valid <= '1';

                        if byte_counter = 511 then
                            sector_last  <= '1';
                            sector_ready <= '1';
                            state        <= WAIT_WRITER;
                        else
                            byte_counter <= byte_counter + 1;

                            -- Once a sector starts, continue using REAL fifo_empty.
                            if fifo_empty = '0' then
                                fifo_rd_en <= '1';
                                state      <= READ_FIFO;
                            else
                                state <= WAIT_FOR_DATA;
                            end if;
                        end if;
                    end if;
                when WAIT_FOR_DATA =>
                    -- Wait for FIFO to have another byte, then continue.
                    if fifo_empty = '0' then
                        fifo_rd_en <= '1';
                        state      <= READ_FIFO;
                    end if;
                when WAIT_WRITER =>
                    if sd_busy = '0' then
                        state <= IDLE;
                    end if;
                when others =>
                    state <= IDLE;
            end case;
        end if;
    end if;
end process;

end Behavioral;