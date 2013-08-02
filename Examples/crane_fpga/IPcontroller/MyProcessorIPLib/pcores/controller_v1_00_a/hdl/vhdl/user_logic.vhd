------------------------------------------------------------------------------
-- HDL controller implementation 
------------------------------------------------------------------------------
--The HDL controller interface with the uBlaze is based on 3 registers:
--slv_reg0	->	(32 bits) the uBlaze write the data to pass to the HDL controller, one every clock cycles
--slv_reg2 	->	status register. uBlaze sets slv_reg2 to FFFFFFFFh when it has compleated the data transfer over slv_reg0. The HDL controller set slv_reg2 to 00000000h to inform uBlaze it has compute the control action and can be readed through slv_reg1
--
------------------------------------------------------------------------------

-- DO NOT EDIT BELOW THIS LINE --------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_arith.all;
use ieee.std_logic_unsigned.all;

-- library proc_common_v3_00_a;
-- use proc_common_v3_00_a.proc_common_pkg.all;

-- DO NOT EDIT ABOVE THIS LINE --------------------

--USER libraries added here

------------------------------------------------------------------------------
-- Entity section
------------------------------------------------------------------------------
-- Definition of Generics:
--   C_NUM_REG                    -- Number of software accessible registers
--   C_SLV_DWIDTH                 -- Slave interface data bus width
--
-- Definition of Ports:
--   Bus2IP_Clk                   -- Bus to IP clock
--   Bus2IP_Resetn                -- Bus to IP reset
--   Bus2IP_Data                  -- Bus to IP data bus
--   Bus2IP_BE                    -- Bus to IP byte enables
--   Bus2IP_RdCE                  -- Bus to IP read chip enable
--   Bus2IP_WrCE                  -- Bus to IP write chip enable
--   IP2Bus_Data                  -- IP to Bus data bus
--   IP2Bus_RdAck                 -- IP to Bus read transfer acknowledgement
--   IP2Bus_WrAck                 -- IP to Bus write transfer acknowledgement
--   IP2Bus_Error                 -- IP to Bus error response
------------------------------------------------------------------------------

entity user_logic is
  generic
  (
    -- ADD USER GENERICS BELOW THIS LINE ---------------
    --USER generics added here
    -- ADD USER GENERICS ABOVE THIS LINE ---------------

    -- DO NOT EDIT BELOW THIS LINE ---------------------
    -- Bus protocol parameters, do not add to or delete
    C_NUM_REG                      : integer              := 3;
    C_SLV_DWIDTH                   : integer              := 32
    -- DO NOT EDIT ABOVE THIS LINE ---------------------
  );
  port
  (
    -- ADD USER PORTS BELOW THIS LINE ------------------
    --USER ports added here
    -- ADD USER PORTS ABOVE THIS LINE ------------------

    -- DO NOT EDIT BELOW THIS LINE ---------------------
    -- Bus protocol ports, do not add to or delete
    Bus2IP_Clk                     : in  std_logic;
    Bus2IP_Resetn                  : in  std_logic;
    Bus2IP_Data                    : in  std_logic_vector(C_SLV_DWIDTH-1 downto 0);
    Bus2IP_BE                      : in  std_logic_vector(C_SLV_DWIDTH/8-1 downto 0);
    Bus2IP_RdCE                    : in  std_logic_vector(C_NUM_REG-1 downto 0);
    Bus2IP_WrCE                    : in  std_logic_vector(C_NUM_REG-1 downto 0);
    IP2Bus_Data                    : out std_logic_vector(C_SLV_DWIDTH-1 downto 0);
    IP2Bus_RdAck                   : out std_logic;
    IP2Bus_WrAck                   : out std_logic;
    IP2Bus_Error                   : out std_logic
    -- DO NOT EDIT ABOVE THIS LINE ---------------------
  );

  attribute MAX_FANOUT : string;
  attribute SIGIS : string;


  attribute SIGIS of Bus2IP_Clk    : signal is "CLK";
  attribute SIGIS of Bus2IP_Resetn : signal is "RST";
  
  
  
  

end entity user_logic;

------------------------------------------------------------------------------
-- Architecture section
------------------------------------------------------------------------------

architecture IMP of user_logic is

  --USER signal declarations added here, as needed for user logic

	
	signal slv_reg0_wr_en: std_logic:='0';
	
	type type_state_input is (idle,wait_Ack);

	signal IP2Bus_state: type_state_input;
	

  
	signal rst: std_logic;
	

  ------------------------------------------
  -- Signals for user logic slave model s/w accessible register example
  ------------------------------------------
  signal slv_reg0                       : std_logic_vector(C_SLV_DWIDTH-1 downto 0):=(others=>'0');
  signal slv_reg1                       : std_logic_vector(C_SLV_DWIDTH-1 downto 0):=(others=>'0');
  signal slv_reg2                       : std_logic_vector(C_SLV_DWIDTH-1 downto 0):=(others=>'0');
  signal slv_reg_write_sel              : std_logic_vector(2 downto 0);
  signal slv_reg_read_sel               : std_logic_vector(2 downto 0);
  signal slv_read_ack                   : std_logic:='0';
  signal slv_write_ack                  : std_logic:='0';
  
    attribute keep : string;
    attribute keep of slv_reg0: signal is "true";
    attribute keep of slv_reg1: signal is "true";
    attribute keep of slv_reg2: signal is "true";
    attribute keep of slv_reg_write_sel: signal is "true";
    attribute keep of slv_reg_read_sel: signal is "true";
    attribute keep of slv_read_ack: signal is "true";
    attribute keep of slv_write_ack: signal is "true";
    

begin

  --USER logic implementation added here
  
  rst<=not Bus2IP_Resetn;
  

	process (Bus2IP_Clk)
	begin
		if rising_edge(Bus2IP_Clk) then
		
			if rst='1' then
				IP2Bus_RdAck<='0';
				IP2Bus_Data<=(others=>'0');
				IP2Bus_state<=idle;
			else
				
				case IP2Bus_state is
					when idle =>
					
						if slv_reg_read_sel="001"  then
							IP2Bus_Data <= slv_reg2;
							IP2Bus_RdAck<='1';
							IP2Bus_state<=wait_Ack;
						else
							IP2Bus_state<=idle;
							IP2Bus_RdAck<='0';
						end if;

						
					when wait_Ack =>
						IP2Bus_state<=idle;
						IP2Bus_RdAck<='0';
					
					when others =>
						null;
				end case;

			end if;
		end if;
	end process;  
	

  ------------------------------------------
  -- Example code to read/write user logic slave model s/w accessible registers
  -- 
  -- Note:
  -- The example code presented here is to show you one way of reading/writing
  -- software accessible registers implemented in the user logic slave model.
  -- Each bit of the Bus2IP_WrCE/Bus2IP_RdCE signals is configured to correspond
  -- to one software accessible register by the top level template. For example,
  -- if you have four 32 bit software accessible registers in the user logic,
  -- you are basically operating on the following memory mapped registers:
  -- 
  --    Bus2IP_WrCE/Bus2IP_RdCE   Memory Mapped Register
  --                     "1000"   C_BASEADDR + 0x0
  --                     "0100"   C_BASEADDR + 0x4
  --                     "0010"   C_BASEADDR + 0x8
  --                     "0001"   C_BASEADDR + 0xC
  -- 
  ------------------------------------------
  slv_reg_write_sel <= Bus2IP_WrCE(2 downto 0);
  slv_reg_read_sel  <= Bus2IP_RdCE(2 downto 0);
  slv_write_ack     <= Bus2IP_WrCE(0) or Bus2IP_WrCE(1) or Bus2IP_WrCE(2);


  -- implement slave model software accessible register(s)
  SLAVE_REG_WRITE_PROC : process( Bus2IP_Clk ) is
  begin

    if Bus2IP_Clk'event and Bus2IP_Clk = '1' then
      if Bus2IP_Resetn = '0' then
        slv_reg0 <= (others => '0');
        slv_reg1 <= (others => '0');
        slv_reg2 <= (others => '0');
	slv_reg0_wr_en<='0';
      else
	
	
	slv_reg0_wr_en<='0';
	
        case slv_reg_write_sel is
          when "100" =>
            for byte_index in 0 to (C_SLV_DWIDTH/8)-1 loop
              if ( Bus2IP_BE(byte_index) = '1' ) then
                slv_reg0(byte_index*8+7 downto byte_index*8) <= Bus2IP_Data(byte_index*8+7 downto byte_index*8);
              end if;
            end loop;
	    slv_reg0_wr_en<='1';
          -- when "010" =>
            -- for byte_index in 0 to (C_SLV_DWIDTH/8)-1 loop
              -- if ( Bus2IP_BE(byte_index) = '1' ) then
                -- slv_reg1(byte_index*8+7 downto byte_index*8) <= Bus2IP_Data(byte_index*8+7 downto byte_index*8);
              -- end if;
            -- end loop;
	    -- slv_reg0_wr_en<='0';
          when "001" =>
            for byte_index in 0 to (C_SLV_DWIDTH/8)-1 loop
              if ( Bus2IP_BE(byte_index) = '1' ) then
                slv_reg2(byte_index*8+7 downto byte_index*8) <= Bus2IP_Data(byte_index*8+7 downto byte_index*8);
              end if;
            end loop;
	    slv_reg0_wr_en<='0';
          when others => null;
        end case;
	
 
 

	
      end if;
    end if;

  end process SLAVE_REG_WRITE_PROC;

  IP2Bus_Error <= '0';
  IP2Bus_WrAck<=slv_write_ack;

end IMP;
