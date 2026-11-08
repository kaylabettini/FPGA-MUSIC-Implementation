library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity FIFObuffer is
    generic (
        data_width : integer := 8;
        fifo_depth : integer := 8192
    );
    port (
        clk         : in  std_logic;
        reset       : in  std_logic;
        -- Write interface
        wr_en       : in  std_logic;
        wr_data     : in  std_logic_vector(data_width-1 downto 0);
        -- Read interface
        rd_en       : in  std_logic;
        rd_data     : out std_logic_vector(data_width-1 downto 0) := (others => '0');
        -- Status signals
        empty       : out std_logic;
        full        : out std_logic;
        almost_empty: out std_logic;
        almost_full : out std_logic;
        -- Debug signals
        occupancy   : out integer range 0 to fifo_depth
    );
end FIFObuffer;

architecture RTL of FIFObuffer is
    type fifo_memory is array (0 to fifo_depth-1) of std_logic_vector(data_width-1 downto 0);
    signal memory : fifo_memory := (others => (others => '0'));
    
    signal read_ptr  : integer range 0 to fifo_depth-1 := 0;
    signal write_ptr : integer range 0 to fifo_depth-1 := 0;
    signal count     : integer range 0 to fifo_depth := 0;
    
    constant ALMOST_EMPTY_THRESHOLD : integer := 2;
    constant ALMOST_FULL_THRESHOLD  : integer := fifo_depth - 2;
    
begin
    -- Write process
    write_proc: process(clk)
    begin
        if rising_edge(clk) thenf
            if reset = '1' then
                write_ptr <= 0;
            elsif wr_en = '1' and count < fifo_depth then					-- writes only when write enable is high and count hasn't reached max value.
                memory(write_ptr) <= wr_data;
                if write_ptr = fifo_depth-1 then
                    write_ptr <= 0;
                else
                    write_ptr <= write_ptr + 1;
                end if;
            end if;
        end if;
    end process;

    -- Read process
    read_proc: process(clk)
    begin
    
         
         if rising_edge(clk) then
            if reset = '1' then
                read_ptr <= 0;
                rd_data <= (others => '0');
                
            elsif rd_en = '1' and count > 0 then							-- reads only when read enable is high and count is more than zero.
                rd_data <= memory(read_ptr);
                
                if read_ptr = fifo_depth-1 then
                    read_ptr <= 0;
                else
                    read_ptr <= read_ptr + 1;
                end if;
            else 
            rd_data <= (others => '0');
            end if;
        end if;
    end process;
   

    -- Count process
    count_proc: process(clk)														
    begin
        if rising_edge(clk) then
            if reset = '1' then
                count <= 0;
            else
                if (wr_en = '1' and rd_en = '0' and count < FIFO_DEPTH) then						-- count up when writing
                    count <= count + 1;
                elsif (wr_en = '0' and rd_en = '1' and count > 0) then								-- count down when reading
                    count <= count - 1;
                elsif (wr_en = '1' and rd_en = '1' and count > 0 and count < FIFO_DEPTH) then		-- count stays same when reading/ writing at the same time.
                    count <= count;  																
                end if;
            end if;
        end if;
    end process;

    -- Status signals
    empty 			<= '1' when count = 0 else '0';
    full 			<= '1' when count = fifo_depth else '0';
    almost_empty 	<= '1' when count <= ALMOST_EMPTY_THRESHOLD else '0';
    almost_full 	<= '1' when count >= ALMOST_FULL_THRESHOLD else '0';
    occupancy 		<= count;

end RTL;