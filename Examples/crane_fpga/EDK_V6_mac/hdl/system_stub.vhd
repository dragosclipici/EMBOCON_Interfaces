-------------------------------------------------------------------------------
-- system_stub.vhd
-------------------------------------------------------------------------------
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

library UNISIM;
use UNISIM.VCOMPONENTS.ALL;

entity system_stub is
  port (
    ddr_memory_we_n : out std_logic;
    ddr_memory_ras_n : out std_logic;
    ddr_memory_odt : out std_logic;
    ddr_memory_dqs_n : inout std_logic_vector(0 to 0);
    ddr_memory_dqs : inout std_logic_vector(0 to 0);
    ddr_memory_dq : inout std_logic_vector(7 downto 0);
    ddr_memory_dm : out std_logic_vector(0 to 0);
    ddr_memory_ddr3_rst : out std_logic;
    ddr_memory_cs_n : out std_logic;
    ddr_memory_clk_n : out std_logic;
    ddr_memory_clk : out std_logic;
    ddr_memory_cke : out std_logic;
    ddr_memory_cas_n : out std_logic;
    ddr_memory_ba : out std_logic_vector(2 downto 0);
    ddr_memory_addr : out std_logic_vector(12 downto 0);
    SysACE_WEN : out std_logic;
    SysACE_OEN : out std_logic;
    SysACE_MPIRQ : in std_logic;
    SysACE_MPD : inout std_logic_vector(7 downto 0);
    SysACE_MPA : out std_logic_vector(6 downto 0);
    SysACE_CLK : in std_logic;
    SysACE_CEN : out std_logic;
    RS232_Uart_1_sout : out std_logic;
    RS232_Uart_1_sin : in std_logic;
    RESET : in std_logic;
    ETHERNET_TX_ER : out std_logic;
    ETHERNET_TX_EN : out std_logic;
    ETHERNET_TX_CLK : out std_logic;
    ETHERNET_TXD : out std_logic_vector(7 downto 0);
    ETHERNET_RX_ER : in std_logic;
    ETHERNET_RX_DV : in std_logic;
    ETHERNET_RX_CLK : in std_logic;
    ETHERNET_RXD : in std_logic_vector(7 downto 0);
    ETHERNET_PHY_RST_N : out std_logic;
    ETHERNET_MII_TX_CLK : in std_logic;
    ETHERNET_MDIO : inout std_logic;
    ETHERNET_MDC : out std_logic;
    CLK_P : in std_logic;
    CLK_N : in std_logic
  );
end system_stub;

architecture STRUCTURE of system_stub is

  component system is
    port (
      ddr_memory_we_n : out std_logic;
      ddr_memory_ras_n : out std_logic;
      ddr_memory_odt : out std_logic;
      ddr_memory_dqs_n : inout std_logic_vector(0 to 0);
      ddr_memory_dqs : inout std_logic_vector(0 to 0);
      ddr_memory_dq : inout std_logic_vector(7 downto 0);
      ddr_memory_dm : out std_logic_vector(0 to 0);
      ddr_memory_ddr3_rst : out std_logic;
      ddr_memory_cs_n : out std_logic;
      ddr_memory_clk_n : out std_logic;
      ddr_memory_clk : out std_logic;
      ddr_memory_cke : out std_logic;
      ddr_memory_cas_n : out std_logic;
      ddr_memory_ba : out std_logic_vector(2 downto 0);
      ddr_memory_addr : out std_logic_vector(12 downto 0);
      SysACE_WEN : out std_logic;
      SysACE_OEN : out std_logic;
      SysACE_MPIRQ : in std_logic;
      SysACE_MPD : inout std_logic_vector(7 downto 0);
      SysACE_MPA : out std_logic_vector(6 downto 0);
      SysACE_CLK : in std_logic;
      SysACE_CEN : out std_logic;
      RS232_Uart_1_sout : out std_logic;
      RS232_Uart_1_sin : in std_logic;
      RESET : in std_logic;
      ETHERNET_TX_ER : out std_logic;
      ETHERNET_TX_EN : out std_logic;
      ETHERNET_TX_CLK : out std_logic;
      ETHERNET_TXD : out std_logic_vector(7 downto 0);
      ETHERNET_RX_ER : in std_logic;
      ETHERNET_RX_DV : in std_logic;
      ETHERNET_RX_CLK : in std_logic;
      ETHERNET_RXD : in std_logic_vector(7 downto 0);
      ETHERNET_PHY_RST_N : out std_logic;
      ETHERNET_MII_TX_CLK : in std_logic;
      ETHERNET_MDIO : inout std_logic;
      ETHERNET_MDC : out std_logic;
      CLK_P : in std_logic;
      CLK_N : in std_logic
    );
  end component;

  attribute BUFFER_TYPE : STRING;
  attribute BOX_TYPE : STRING;
  attribute BUFFER_TYPE of SysACE_CLK : signal is "BUFGP";
  attribute BOX_TYPE of system : component is "user_black_box";

begin

  system_i : system
    port map (
      ddr_memory_we_n => ddr_memory_we_n,
      ddr_memory_ras_n => ddr_memory_ras_n,
      ddr_memory_odt => ddr_memory_odt,
      ddr_memory_dqs_n => ddr_memory_dqs_n(0 to 0),
      ddr_memory_dqs => ddr_memory_dqs(0 to 0),
      ddr_memory_dq => ddr_memory_dq,
      ddr_memory_dm => ddr_memory_dm(0 to 0),
      ddr_memory_ddr3_rst => ddr_memory_ddr3_rst,
      ddr_memory_cs_n => ddr_memory_cs_n,
      ddr_memory_clk_n => ddr_memory_clk_n,
      ddr_memory_clk => ddr_memory_clk,
      ddr_memory_cke => ddr_memory_cke,
      ddr_memory_cas_n => ddr_memory_cas_n,
      ddr_memory_ba => ddr_memory_ba,
      ddr_memory_addr => ddr_memory_addr,
      SysACE_WEN => SysACE_WEN,
      SysACE_OEN => SysACE_OEN,
      SysACE_MPIRQ => SysACE_MPIRQ,
      SysACE_MPD => SysACE_MPD,
      SysACE_MPA => SysACE_MPA,
      SysACE_CLK => SysACE_CLK,
      SysACE_CEN => SysACE_CEN,
      RS232_Uart_1_sout => RS232_Uart_1_sout,
      RS232_Uart_1_sin => RS232_Uart_1_sin,
      RESET => RESET,
      ETHERNET_TX_ER => ETHERNET_TX_ER,
      ETHERNET_TX_EN => ETHERNET_TX_EN,
      ETHERNET_TX_CLK => ETHERNET_TX_CLK,
      ETHERNET_TXD => ETHERNET_TXD,
      ETHERNET_RX_ER => ETHERNET_RX_ER,
      ETHERNET_RX_DV => ETHERNET_RX_DV,
      ETHERNET_RX_CLK => ETHERNET_RX_CLK,
      ETHERNET_RXD => ETHERNET_RXD,
      ETHERNET_PHY_RST_N => ETHERNET_PHY_RST_N,
      ETHERNET_MII_TX_CLK => ETHERNET_MII_TX_CLK,
      ETHERNET_MDIO => ETHERNET_MDIO,
      ETHERNET_MDC => ETHERNET_MDC,
      CLK_P => CLK_P,
      CLK_N => CLK_N
    );

end architecture STRUCTURE;

