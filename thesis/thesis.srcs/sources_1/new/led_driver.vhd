----------------------------------------------------------------------------------
-- Company:        University of Queensland
-- Engineer:       Kayla Bettini
--
-- Module Name:    led_driver
-- Project Name:   Implementation of the MUSIC algorithm on an FPGA
-- Target Devices: Nexys 4 (XC7A100T)
-- Description:    Drives the 16 board LEDs from 16 debug signals (SIGn -> LED n).
--                 All LEDs are off while reset is high.
----------------------------------------------------------------------------------

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

entity led_driver is
    Port ( 
        -- Reset (active high): all LEDs off
        reset : in std_logic;

        -- Debug signals, SIGn drives LED n
        SIG0  : in std_logic;
        SIG1  : in std_logic;
        SIG2  : in std_logic;
        SIG3  : in std_logic;
        SIG4  : in std_logic;
        SIG5  : in std_logic;
        SIG6  : in std_logic;
        SIG7  : in std_logic;
        SIG8  : in std_logic;
        SIG9  : in std_logic;
        SIG10 : in std_logic;
        SIG11 : in std_logic;
        SIG12 : in std_logic;
        SIG13 : in std_logic;
        SIG14 : in std_logic;
        SIG15 : in std_logic;
    
        -- Board LEDs
        LEDS : out std_logic_vector(15 downto 0)
    );
end led_driver;

architecture rtl of led_driver is

signal led_values : std_logic_vector(15 downto 0);

begin

-- Pack the signals into one vector, SIG0 in bit 0
led_values <= SIG15 & SIG14 & SIG13 & SIG12 &
              SIG11 & SIG10 & SIG9  & SIG8  &
              SIG7  & SIG6  & SIG5  & SIG4  &
              SIG3  & SIG2  & SIG1  & SIG0;

LEDS <= (others => '0') when reset = '1' else led_values;

end rtl;