library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity sampleGatherer is
    Port(
        SCLK : IN STD_LOGIC;
        CLK : IN STD_LOGIC;
        MOSI : IN STD_LOGIC;
        CS : IN STD_LOGIC;
        RESET : IN STD_LOGIC;
        CH0,CH1,CH2,CH3,CH4,CH5,CH6,CH7 : OUT STD_LOGIC_VECTOR(31 downto 0);
        CH0_ENABLE : OUT STD_LOGIC;
        DEBUG_LED : OUT STD_LOGIC
    );
end sampleGatherer;

architecture Behavioral of sampleGatherer is

component spiReceiver
    Generic (
        length : integer
    );
    Port ( 
        SCLK : IN STD_LOGIC;
        MOSI : IN STD_LOGIC;
        CS : IN STD_LOGIC;
        RESET : IN STD_LOGIC;
        D_OUT : OUT STD_LOGIC_VECTOR(7 downto 0);
        D_VALID : OUT STD_LOGIC
    ); 
end component;

signal D_OUT : STD_LOGIC_VECTOR(7 downto 0);
signal D_VALID : STD_LOGIC;
signal LAST4 : STD_LOGIC_VECTOR(31 downto 0);
signal HEADER_FOUND : STD_LOGIC;
signal IGNORE : integer range 0 to 4 := 0;
signal HEADER_SEEN : STD_LOGIC := '0';
signal FRAME_COUNTER : integer range 0 to 512 := 0;
signal BYTE_COUNTER : integer range 0 to 31 := 0;
signal HEADER_SEEN_LATCH : STD_LOGIC := '0';

begin

spi: spiReceiver
    Generic map (
        length => 8
    )
    Port map ( 
        SCLK => SCLK,
        MOSI => MOSI,
        CS => CS,
        RESET => RESET,
        D_OUT => D_OUT,
        D_VALID => D_VALID
    ); 
    
process(SCLK, RESET)
begin
    if RESET = '0' then
        LAST4 <= (others => '0');
        IGNORE <= 0;
        HEADER_SEEN <= '0';
        BYTE_COUNTER <= 0;
        FRAME_COUNTER <= 0;
        CH0_ENABLE <= '0';

        CH0 <= (others => '0');
        CH1 <= (others => '0');
        CH2 <= (others => '0');
        CH3 <= (others => '0');
        CH4 <= (others => '0');
        CH5 <= (others => '0');
        CH6 <= (others => '0');
        CH7 <= (others => '0');

    elsif falling_edge(SCLK) then
        CH0_ENABLE <= '0';

        if D_VALID = '1' then

            -- 4 byte window
            LAST4 <= D_OUT & LAST4(31 downto 8);

            -- header found
            if (D_OUT & LAST4(31 downto 8)) = x"A55AF00D" then
                IGNORE <= 4;
                HEADER_SEEN <= '1';
                HEADER_SEEN_LATCH <= '1';  -- latches forever once header found
                FRAME_COUNTER <= 1;
                BYTE_COUNTER <= 0;
                
            elsif (IGNORE > 0) then
                IGNORE <= IGNORE - 1;

            elsif (FRAME_COUNTER > 0) then
                if (BYTE_COUNTER = 3) then
                    CH0 <= D_OUT & LAST4(31 downto 8);
                    CH0_ENABLE <= '1';
                elsif (BYTE_COUNTER = 7) then
                    CH1 <= D_OUT & LAST4(31 downto 8);
                elsif (BYTE_COUNTER = 11) then
                    CH2 <= D_OUT & LAST4(31 downto 8);
                elsif (BYTE_COUNTER = 15) then
                    CH3 <= D_OUT & LAST4(31 downto 8);
                elsif (BYTE_COUNTER = 19) then
                    CH4 <= D_OUT & LAST4(31 downto 8);
                elsif (BYTE_COUNTER = 23) then
                    CH5 <= D_OUT & LAST4(31 downto 8);
                elsif (BYTE_COUNTER = 27) then
                    CH6 <= D_OUT & LAST4(31 downto 8);
                elsif (BYTE_COUNTER = 31) then
                    CH7 <= D_OUT & LAST4(31 downto 8);

                    if FRAME_COUNTER = 512 then
                        FRAME_COUNTER <= 0;
                        HEADER_SEEN   <= '0';
                    else
                        FRAME_COUNTER <= FRAME_COUNTER + 1;
                    end if;
                end if;
               
                if (BYTE_COUNTER = 31) then
                    BYTE_COUNTER <= 0;
                else 
                    BYTE_COUNTER <= BYTE_COUNTER + 1;
                end if;
            end if;
        end if;
    end if;
end process;

DEBUG_LED <= HEADER_SEEN_LATCH;
HEADER_FOUND <= '1' when LAST4 = x"A55AF00D" else '0';

end Behavioral;