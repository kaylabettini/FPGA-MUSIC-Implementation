----------------------------------------------------------------------------------
-- Company: University of Queensland
-- Engineer: Kayla Bettini
-- 
-- Create Date: 11.05.2026 00:50:00
-- Design Name: Thesis Project
-- Module Name: spiReceiver
-- Project Name: 3D MUSIC Algorithm Implementation
-- Target Devices: Nexys 4
-- Tool Versions: Vivado 2025.2
-- Description: recieves an incoming byte from SPI
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

entity spiReceiver is
    Generic (
        length: integer := 8 --data block length
    );
    Port ( 
        SCLK : IN STD_LOGIC;
        MOSI : IN STD_LOGIC;
        CS : IN STD_LOGIC;
        RESET : IN STD_LOGIC;
        D_OUT : OUT STD_LOGIC_VECTOR(length-1 downto 0);
        D_VALID : OUT STD_LOGIC
    );
end spiReceiver;

architecture Behavioral of spiReceiver is

signal current : STD_LOGIC_VECTOR(length-1 downto 0) := (others => '0');
signal count : integer range 0 to length-1 := 0;

begin

process(SCLK, RESET, CS)
begin
    if (RESET = '0') then
        current <= (others => '0');
        count <= 0;
        D_VALID <= '0';
        D_OUT <= (others => '0');
    elsif (CS = '1') then --slave not selected
        count <= 0;
        D_VALID <= '0';
    elsif (rising_edge(SCLK)) then
        current <= current(length-2 downto 0) & MOSI;
        
        if (count = length-1) then --set the output to the current bytea
            D_OUT <= current(length-2 downto 0) & MOSI;
            D_VALID <= '1'; --recieved full byte
            count <= 0; --reset counter to 0
        else
            D_VALID <= '0';
            count <= count + 1; --increment
        end if;
    end if;
end process;
    
end Behavioral;
