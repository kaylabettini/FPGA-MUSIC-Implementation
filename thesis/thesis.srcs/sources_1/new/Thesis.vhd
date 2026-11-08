----------------------------------------------------------------------------------
-- Company: University of Queensland
-- Engineer: Kayla Bettini
-- 
-- Create Date: 11.05.2026 00:50:00
-- Design Name: Thesis Project
-- Module Name: thesis
-- Project Name: 3D MUSIC Algorithm Implementation
-- Target Devices: Nexys 4
-- Tool Versions: Vivado 2025.2
-- Description: Main structural file for my thesis project,
-- involving the implementation of the MUSIC algorithm on an FPGA.
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

entity thesis is
    Port (
        CLK100MHZ : in STD_LOGIC;
        RESET : in STD_LOGIC;
        JA : out STD_LOGIC_VECTOR(10 downto 1);
        AUD_SD : out STD_LOGIC;
        AUD_PWM : out STD_LOGIC;
        SSD_CATHODE, SSD_ANODE : out STD_LOGIC_VECTOR(7 downto 0)--;
     );
end thesis;

architecture Structural of thesis is

component SSDDriver_1
    port(
        CLK100MHZ : in STD_LOGIC;
        NumberVector : in STD_LOGIC_VECTOR(31 downto 0);
        SSD_CATHODE : out STD_LOGIC_VECTOR(7 downto 0); 
        SSD_ANODE : out STD_LOGIC_VECTOR(7 downto 0)
    );
end component;

component clock_divider
    generic (
        n: integer
    );
    port (
        clk_in : in STD_LOGIC;
        clk_out : out STD_LOGIC
    );
end component;

signal display : STD_LOGIC_VECTOR(31 downto 0) := (others => '0');

begin

display <= x"01020304";

SSD : SSDDriver_1
    port map (
        CLK100MHZ => CLK100MHZ,
        NumberVector => display,
        SSD_CATHODE => SSD_CATHODE,
        SSD_ANODE => SSD_ANODE
    );

end Structural;
