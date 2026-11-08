----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date: 12.05.2026 20:35:40
-- Design Name: Thesis Project
-- Module Name: PCMtoDUTY
-- Project Name: 3D MUSIC Algorithm Implementation
-- Target Devices: Nexys 4
-- Tool Versions: Vivado 2025.2
-- Description: Converts the raw PCM signal (signed 32 bit number) to a 
-- X bit number equivalent duty cycle to be processed by the audioPWM file
-- 
-- Dependencies: N/A
-- 
-- Revision:
-- Revision 0.01 - File Created
-- Additional Comments:
-- 
----------------------------------------------------------------------------------

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity PCMtoDUTY is
    Generic (
        RESOLUTION : integer := 8 -- audio bit resolution
    );
    Port (
        CLK : in STD_LOGIC;
        PCM_IN : in STD_LOGIC_VECTOR(31 downto 0);
        DUTY_OUT : out STD_LOGIC_VECTOR(RESOLUTION-1 downto 0)
    );
end PCMtoDUTY;
architecture Behavioral of PCMtoDUTY is

signal SCALED : signed(31 downto 0);
signal DUTY_INT : integer;
signal CLAMPED : integer;

begin
-- The PCM signal is a SIGNED number; hence will need to convert the number

--Converting the 2s comp to a positive integer
SCALED <= shift_right(signed(PCM_IN), 32-RESOLUTION);
DUTY_INT <= to_integer(SCALED) + (2 ** (RESOLUTION-1));

--clamping the duty int to the resolution
process(DUTY_INT)
begin   
    CLAMPED <= DUTY_INT;
    if (DUTY_INT > (2**RESOLUTION) - 1) then
        CLAMPED <= (2**RESOLUTION) - 1;
    elsif (DUTY_INT < 0) then
        CLAMPED <= 0;
    end if;
end process;

DUTY_OUT <= std_logic_vector(to_unsigned(CLAMPED, RESOLUTION));

end Behavioral;
