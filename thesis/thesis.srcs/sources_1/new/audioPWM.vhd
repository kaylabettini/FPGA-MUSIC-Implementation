----------------------------------------------------------------------------------
-- Company: University of Queensland
-- Engineer: Kayla Bettini
-- 
-- Create Date: 12.05.2026 19:26:00
-- Design Name: Thesis Project
-- Module Name: audioPWM - Behavioural
-- Project Name: 3D MUSIC Algorithm Implementation
-- Target Devices: Nexys 4
-- Tool Versions: Vivado 2025.2
-- Description: Depending on the currently received PCM value, it will convert 
-- this pcm value to an integer, and output the signal to be connected to the
-- MONO AUDIO OUT jack of the Nexys 4
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

entity audioPWM is
    Generic (
        RESOLUTION : integer := 8 --audio bit resolution
    );
    Port (
        D_IN : IN STD_LOGIC_VECTOR(RESOLUTION-1 downto 0);
        CLK : IN STD_LOGIC;
        A_OUT : OUT STD_LOGIC
    );
end audioPWM;

architecture Behavioral of audioPWM is
signal COUNTER : unsigned(RESOLUTION-1 downto 0) := (others => '0');
signal DUTY_CYCLE : unsigned(RESOLUTION-1 downto 0) := (others => '0');
begin

process(CLK)
begin
    if rising_edge(CLK) then
        COUNTER <= COUNTER + 1;
    end if;
end process;

A_OUT <= '1' when COUNTER < DUTY_CYCLE else '0';      
DUTY_CYCLE <= unsigned(D_IN);

end Behavioral;
