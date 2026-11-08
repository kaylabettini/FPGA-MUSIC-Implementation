----------------------------------------------------------------------------------
-- Company: University of Queensland
-- Engineer: Kayla Bettini
-- 
-- Create Date: 11.05.2026 00:50:00
-- Design Name: Thesis Project
-- Module Name: Shift Register
-- Project Name: 3D MUSIC Algorithm Implementation
-- Target Devices: Nexys 4
-- Tool Versions: Vivado 2025.2
-- Description: Shift Register code for use in the SPI block.
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

entity shiftReg is
    port(
        d_in : in std_logic_vector(7 downto 0);
        clk_in : in std_logic;
        reset : in std_logic;
        enable : in std_logic;
        q_out : out std_logic_vector(31 downto 0)
    );
end shiftReg;

architecture Behavioral of shiftReg is
signal reg : std_logic_vector(31 downto 0);
begin

process (clk_in, reset) 
begin
    if (reset = '1') then
        reg <= (others => '0'); 
    elsif rising_edge(clk_in) then
        if enable = '1' then
            reg <= reg(23 downto 0) & d_in;
        end if;
    end if;
end process;
 
q_out <= reg;
 
end Behavioral;