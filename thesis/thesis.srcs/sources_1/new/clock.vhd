----------------------------------------------------------------------------------
-- Company: University of Queensland
-- Engineer: Kayla Bettini
-- 
-- Create Date: 11.05.2026 00:50:00
-- Design Name: Thesis Project
-- Module Name: clock
-- Project Name: 3D MUSIC Algorithm Implementation
-- Target Devices: Nexys 4
-- Tool Versions: Vivado 2025.2
-- Description:  code that divides the clock based on an integer and outputs a clock
-- based on the divisor 
-- Dependencies: N/A
-- 
-- Revision:
-- Revision 0.01 - File Created
-- Additional Comments:
-- 
----------------------------------------------------------------------------------

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

entity clockDiv is
generic (
    n: integer := 100000
    ); -- Division value, eg 100Mhz in, div=2 => 50Mhz out
port (
    clk_in: in std_logic;
    clk_out: out std_logic
    );
end clockDiv;

architecture Behavioral of clockDiv is

signal clk_divider_counter : natural range 0 to n;
signal clock_state : std_logic := '0';
 
begin process (clk_in)
begin
  if rising_edge(clk_in) then 
     if (clk_divider_counter = n-1) then
      clk_divider_counter <= 0;
      clock_state <= not clock_state;
     else
      clk_divider_counter <= clk_divider_counter + 1;
      end if;
  end if;
end process;
       
clk_out <= clock_state;

end Behavioral;