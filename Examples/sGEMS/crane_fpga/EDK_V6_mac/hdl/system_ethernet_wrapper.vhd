-------------------------------------------------------------------------------
-- system_ethernet_wrapper.vhd
-------------------------------------------------------------------------------
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

library UNISIM;
use UNISIM.VCOMPONENTS.ALL;

library axi_ethernet_v3_01_a;
use axi_ethernet_v3_01_a.all;

entity system_ethernet_wrapper is
  port (
    S_AXI_ACLK : in std_logic;
    S_AXI_ARESETN : in std_logic;
    INTERRUPT : out std_logic;
    S_AXI_AWADDR : in std_logic_vector(31 downto 0);
    S_AXI_AWVALID : in std_logic;
    S_AXI_AWREADY : out std_logic;
    S_AXI_WDATA : in std_logic_vector(31 downto 0);
    S_AXI_WSTRB : in std_logic_vector(3 downto 0);
    S_AXI_WVALID : in std_logic;
    S_AXI_WREADY : out std_logic;
    S_AXI_BRESP : out std_logic_vector(1 downto 0);
    S_AXI_BVALID : out std_logic;
    S_AXI_BREADY : in std_logic;
    S_AXI_ARADDR : in std_logic_vector(31 downto 0);
    S_AXI_ARVALID : in std_logic;
    S_AXI_ARREADY : out std_logic;
    S_AXI_RDATA : out std_logic_vector(31 downto 0);
    S_AXI_RRESP : out std_logic_vector(1 downto 0);
    S_AXI_RVALID : out std_logic;
    S_AXI_RREADY : in std_logic;
    AXI_STR_TXD_ACLK : in std_logic;
    AXI_STR_TXD_ARESETN : in std_logic;
    AXI_STR_TXD_TVALID : in std_logic;
    AXI_STR_TXD_TREADY : out std_logic;
    AXI_STR_TXD_TLAST : in std_logic;
    AXI_STR_TXD_TKEEP : in std_logic_vector(3 downto 0);
    AXI_STR_TXD_TDATA : in std_logic_vector(31 downto 0);
    AXI_STR_TXC_ACLK : in std_logic;
    AXI_STR_TXC_ARESETN : in std_logic;
    AXI_STR_TXC_TVALID : in std_logic;
    AXI_STR_TXC_TREADY : out std_logic;
    AXI_STR_TXC_TLAST : in std_logic;
    AXI_STR_TXC_TKEEP : in std_logic_vector(3 downto 0);
    AXI_STR_TXC_TDATA : in std_logic_vector(31 downto 0);
    AXI_STR_RXD_ACLK : in std_logic;
    AXI_STR_RXD_ARESETN : in std_logic;
    AXI_STR_RXD_TVALID : out std_logic;
    AXI_STR_RXD_TREADY : in std_logic;
    AXI_STR_RXD_TLAST : out std_logic;
    AXI_STR_RXD_TKEEP : out std_logic_vector(3 downto 0);
    AXI_STR_RXD_TDATA : out std_logic_vector(31 downto 0);
    AXI_STR_RXS_ACLK : in std_logic;
    AXI_STR_RXS_ARESETN : in std_logic;
    AXI_STR_RXS_TVALID : out std_logic;
    AXI_STR_RXS_TREADY : in std_logic;
    AXI_STR_RXS_TLAST : out std_logic;
    AXI_STR_RXS_TKEEP : out std_logic_vector(3 downto 0);
    AXI_STR_RXS_TDATA : out std_logic_vector(31 downto 0);
    PHY_RST_N : out std_logic;
    GTX_CLK : in std_logic;
    MGT_CLK_P : in std_logic;
    MGT_CLK_N : in std_logic;
    REF_CLK : in std_logic;
    MII_TXD : out std_logic_vector(3 downto 0);
    MII_TX_EN : out std_logic;
    MII_TX_ER : out std_logic;
    MII_RXD : in std_logic_vector(3 downto 0);
    MII_RX_DV : in std_logic;
    MII_RX_ER : in std_logic;
    MII_RX_CLK : in std_logic;
    MII_TX_CLK : in std_logic;
    MII_COL : in std_logic;
    MII_CRS : in std_logic;
    GMII_TXD : out std_logic_vector(7 downto 0);
    GMII_TX_EN : out std_logic;
    GMII_TX_ER : out std_logic;
    GMII_TX_CLK : out std_logic;
    GMII_RXD : in std_logic_vector(7 downto 0);
    GMII_RX_DV : in std_logic;
    GMII_RX_ER : in std_logic;
    GMII_RX_CLK : in std_logic;
    GMII_COL : in std_logic;
    GMII_CRS : in std_logic;
    TXP : out std_logic;
    TXN : out std_logic;
    RXP : in std_logic;
    RXN : in std_logic;
    RGMII_TXD : out std_logic_vector(3 downto 0);
    RGMII_TX_CTL : out std_logic;
    RGMII_TXC : out std_logic;
    RGMII_RXD : in std_logic_vector(3 downto 0);
    RGMII_RX_CTL : in std_logic;
    RGMII_RXC : in std_logic;
    MDC : out std_logic;
    MDIO_I : in std_logic;
    MDIO_O : out std_logic;
    MDIO_T : out std_logic;
    AXI_STR_AVBTX_ACLK : out std_logic;
    AXI_STR_AVBTX_ARESETN : in std_logic;
    AXI_STR_AVBTX_TVALID : in std_logic;
    AXI_STR_AVBTX_TREADY : out std_logic;
    AXI_STR_AVBTX_TLAST : in std_logic;
    AXI_STR_AVBTX_TDATA : in std_logic_vector(7 downto 0);
    AXI_STR_AVBTX_TUSER : in std_logic_vector(0 downto 0);
    AXI_STR_AVBRX_ACLK : out std_logic;
    AXI_STR_AVBRX_ARESETN : in std_logic;
    AXI_STR_AVBRX_TVALID : out std_logic;
    AXI_STR_AVBRX_TLAST : out std_logic;
    AXI_STR_AVBRX_TDATA : out std_logic_vector(7 downto 0);
    AXI_STR_AVBRX_TUSER : out std_logic_vector(0 downto 0);
    RTC_CLK : in std_logic;
    AV_INTERRUPT_10MS : out std_logic;
    AV_INTERRUPT_PTP_TX : out std_logic;
    AV_INTERRUPT_PTP_RX : out std_logic;
    AV_RTC_NANOSECFIELD : out std_logic_vector(31 downto 0);
    AV_RTC_SECFIELD : out std_logic_vector(47 downto 0);
    AV_CLK_8K : out std_logic;
    AV_RTC_NANOSECFIELD_1722 : out std_logic_vector(31 downto 0)
  );

  attribute x_core_info : STRING;
  attribute x_core_info of system_ethernet_wrapper : entity is "axi_ethernet_v3_01_a";

end system_ethernet_wrapper;

architecture STRUCTURE of system_ethernet_wrapper is

  component axi_ethernet is
    generic (
      C_FAMILY : STRING;
      C_DEVICE : STRING;
      C_INSTANCE : string;
      C_S_AXI_ACLK_FREQ_HZ : INTEGER;
      C_S_AXI_ADDR_WIDTH : INTEGER;
      C_S_AXI_DATA_WIDTH : INTEGER;
      C_S_AXI_ID_WIDTH : INTEGER;
      C_TRANS : STRING;
      C_PHYADDR : std_logic_vector;
      C_INCLUDE_IO : INTEGER;
      C_TYPE : INTEGER;
      C_PHY_TYPE : INTEGER;
      C_USE_GTH : INTEGER;
      C_HALFDUP : INTEGER;
      C_TXMEM : INTEGER;
      C_RXMEM : INTEGER;
      C_TXCSUM : INTEGER;
      C_RXCSUM : INTEGER;
      C_TXVLAN_TRAN : INTEGER;
      C_RXVLAN_TRAN : INTEGER;
      C_TXVLAN_TAG : INTEGER;
      C_RXVLAN_TAG : INTEGER;
      C_TXVLAN_STRP : INTEGER;
      C_RXVLAN_STRP : INTEGER;
      C_MCAST_EXTEND : INTEGER;
      C_STATS : INTEGER;
      C_AVB : INTEGER;
      C_SIMULATION : INTEGER;
      C_STATS_WIDTH : INTEGER
    );
    port (
      S_AXI_ACLK : in std_logic;
      S_AXI_ARESETN : in std_logic;
      INTERRUPT : out std_logic;
      S_AXI_AWADDR : in std_logic_vector((C_S_AXI_ADDR_WIDTH-1) downto 0);
      S_AXI_AWVALID : in std_logic;
      S_AXI_AWREADY : out std_logic;
      S_AXI_WDATA : in std_logic_vector((C_S_AXI_DATA_WIDTH-1) downto 0);
      S_AXI_WSTRB : in std_logic_vector(((C_S_AXI_DATA_WIDTH/8)-1) downto 0);
      S_AXI_WVALID : in std_logic;
      S_AXI_WREADY : out std_logic;
      S_AXI_BRESP : out std_logic_vector(1 downto 0);
      S_AXI_BVALID : out std_logic;
      S_AXI_BREADY : in std_logic;
      S_AXI_ARADDR : in std_logic_vector((C_S_AXI_ADDR_WIDTH-1) downto 0);
      S_AXI_ARVALID : in std_logic;
      S_AXI_ARREADY : out std_logic;
      S_AXI_RDATA : out std_logic_vector((C_S_AXI_DATA_WIDTH-1) downto 0);
      S_AXI_RRESP : out std_logic_vector(1 downto 0);
      S_AXI_RVALID : out std_logic;
      S_AXI_RREADY : in std_logic;
      AXI_STR_TXD_ACLK : in std_logic;
      AXI_STR_TXD_ARESETN : in std_logic;
      AXI_STR_TXD_TVALID : in std_logic;
      AXI_STR_TXD_TREADY : out std_logic;
      AXI_STR_TXD_TLAST : in std_logic;
      AXI_STR_TXD_TKEEP : in std_logic_vector(3 downto 0);
      AXI_STR_TXD_TDATA : in std_logic_vector(31 downto 0);
      AXI_STR_TXC_ACLK : in std_logic;
      AXI_STR_TXC_ARESETN : in std_logic;
      AXI_STR_TXC_TVALID : in std_logic;
      AXI_STR_TXC_TREADY : out std_logic;
      AXI_STR_TXC_TLAST : in std_logic;
      AXI_STR_TXC_TKEEP : in std_logic_vector(3 downto 0);
      AXI_STR_TXC_TDATA : in std_logic_vector(31 downto 0);
      AXI_STR_RXD_ACLK : in std_logic;
      AXI_STR_RXD_ARESETN : in std_logic;
      AXI_STR_RXD_TVALID : out std_logic;
      AXI_STR_RXD_TREADY : in std_logic;
      AXI_STR_RXD_TLAST : out std_logic;
      AXI_STR_RXD_TKEEP : out std_logic_vector(3 downto 0);
      AXI_STR_RXD_TDATA : out std_logic_vector(31 downto 0);
      AXI_STR_RXS_ACLK : in std_logic;
      AXI_STR_RXS_ARESETN : in std_logic;
      AXI_STR_RXS_TVALID : out std_logic;
      AXI_STR_RXS_TREADY : in std_logic;
      AXI_STR_RXS_TLAST : out std_logic;
      AXI_STR_RXS_TKEEP : out std_logic_vector(3 downto 0);
      AXI_STR_RXS_TDATA : out std_logic_vector(31 downto 0);
      PHY_RST_N : out std_logic;
      GTX_CLK : in std_logic;
      MGT_CLK_P : in std_logic;
      MGT_CLK_N : in std_logic;
      REF_CLK : in std_logic;
      MII_TXD : out std_logic_vector(3 downto 0);
      MII_TX_EN : out std_logic;
      MII_TX_ER : out std_logic;
      MII_RXD : in std_logic_vector(3 downto 0);
      MII_RX_DV : in std_logic;
      MII_RX_ER : in std_logic;
      MII_RX_CLK : in std_logic;
      MII_TX_CLK : in std_logic;
      MII_COL : in std_logic;
      MII_CRS : in std_logic;
      GMII_TXD : out std_logic_vector(7 downto 0);
      GMII_TX_EN : out std_logic;
      GMII_TX_ER : out std_logic;
      GMII_TX_CLK : out std_logic;
      GMII_RXD : in std_logic_vector(7 downto 0);
      GMII_RX_DV : in std_logic;
      GMII_RX_ER : in std_logic;
      GMII_RX_CLK : in std_logic;
      GMII_COL : in std_logic;
      GMII_CRS : in std_logic;
      TXP : out std_logic;
      TXN : out std_logic;
      RXP : in std_logic;
      RXN : in std_logic;
      RGMII_TXD : out std_logic_vector(3 downto 0);
      RGMII_TX_CTL : out std_logic;
      RGMII_TXC : out std_logic;
      RGMII_RXD : in std_logic_vector(3 downto 0);
      RGMII_RX_CTL : in std_logic;
      RGMII_RXC : in std_logic;
      MDC : out std_logic;
      MDIO_I : in std_logic;
      MDIO_O : out std_logic;
      MDIO_T : out std_logic;
      AXI_STR_AVBTX_ACLK : out std_logic;
      AXI_STR_AVBTX_ARESETN : in std_logic;
      AXI_STR_AVBTX_TVALID : in std_logic;
      AXI_STR_AVBTX_TREADY : out std_logic;
      AXI_STR_AVBTX_TLAST : in std_logic;
      AXI_STR_AVBTX_TDATA : in std_logic_vector(7 downto 0);
      AXI_STR_AVBTX_TUSER : in std_logic_vector(0 downto 0);
      AXI_STR_AVBRX_ACLK : out std_logic;
      AXI_STR_AVBRX_ARESETN : in std_logic;
      AXI_STR_AVBRX_TVALID : out std_logic;
      AXI_STR_AVBRX_TLAST : out std_logic;
      AXI_STR_AVBRX_TDATA : out std_logic_vector(7 downto 0);
      AXI_STR_AVBRX_TUSER : out std_logic_vector(0 downto 0);
      RTC_CLK : in std_logic;
      AV_INTERRUPT_10MS : out std_logic;
      AV_INTERRUPT_PTP_TX : out std_logic;
      AV_INTERRUPT_PTP_RX : out std_logic;
      AV_RTC_NANOSECFIELD : out std_logic_vector(31 downto 0);
      AV_RTC_SECFIELD : out std_logic_vector(47 downto 0);
      AV_CLK_8K : out std_logic;
      AV_RTC_NANOSECFIELD_1722 : out std_logic_vector(31 downto 0)
    );
  end component;

begin

  ETHERNET : axi_ethernet
    generic map (
      C_FAMILY => "virtex6",
      C_DEVICE => "xc6vlx240t",
      C_INSTANCE => "ETHERNET",
      C_S_AXI_ACLK_FREQ_HZ => 100000000,
      C_S_AXI_ADDR_WIDTH => 32,
      C_S_AXI_DATA_WIDTH => 32,
      C_S_AXI_ID_WIDTH => 1,
      C_TRANS => "A",
      C_PHYADDR => B"00001",
      C_INCLUDE_IO => 1,
      C_TYPE => 2,
      C_PHY_TYPE => 1,
      C_USE_GTH => 0,
      C_HALFDUP => 0,
      C_TXMEM => 4096,
      C_RXMEM => 4096,
      C_TXCSUM => 0,
      C_RXCSUM => 0,
      C_TXVLAN_TRAN => 0,
      C_RXVLAN_TRAN => 0,
      C_TXVLAN_TAG => 0,
      C_RXVLAN_TAG => 0,
      C_TXVLAN_STRP => 0,
      C_RXVLAN_STRP => 0,
      C_MCAST_EXTEND => 0,
      C_STATS => 0,
      C_AVB => 0,
      C_SIMULATION => 0,
      C_STATS_WIDTH => 64
    )
    port map (
      S_AXI_ACLK => S_AXI_ACLK,
      S_AXI_ARESETN => S_AXI_ARESETN,
      INTERRUPT => INTERRUPT,
      S_AXI_AWADDR => S_AXI_AWADDR,
      S_AXI_AWVALID => S_AXI_AWVALID,
      S_AXI_AWREADY => S_AXI_AWREADY,
      S_AXI_WDATA => S_AXI_WDATA,
      S_AXI_WSTRB => S_AXI_WSTRB,
      S_AXI_WVALID => S_AXI_WVALID,
      S_AXI_WREADY => S_AXI_WREADY,
      S_AXI_BRESP => S_AXI_BRESP,
      S_AXI_BVALID => S_AXI_BVALID,
      S_AXI_BREADY => S_AXI_BREADY,
      S_AXI_ARADDR => S_AXI_ARADDR,
      S_AXI_ARVALID => S_AXI_ARVALID,
      S_AXI_ARREADY => S_AXI_ARREADY,
      S_AXI_RDATA => S_AXI_RDATA,
      S_AXI_RRESP => S_AXI_RRESP,
      S_AXI_RVALID => S_AXI_RVALID,
      S_AXI_RREADY => S_AXI_RREADY,
      AXI_STR_TXD_ACLK => AXI_STR_TXD_ACLK,
      AXI_STR_TXD_ARESETN => AXI_STR_TXD_ARESETN,
      AXI_STR_TXD_TVALID => AXI_STR_TXD_TVALID,
      AXI_STR_TXD_TREADY => AXI_STR_TXD_TREADY,
      AXI_STR_TXD_TLAST => AXI_STR_TXD_TLAST,
      AXI_STR_TXD_TKEEP => AXI_STR_TXD_TKEEP,
      AXI_STR_TXD_TDATA => AXI_STR_TXD_TDATA,
      AXI_STR_TXC_ACLK => AXI_STR_TXC_ACLK,
      AXI_STR_TXC_ARESETN => AXI_STR_TXC_ARESETN,
      AXI_STR_TXC_TVALID => AXI_STR_TXC_TVALID,
      AXI_STR_TXC_TREADY => AXI_STR_TXC_TREADY,
      AXI_STR_TXC_TLAST => AXI_STR_TXC_TLAST,
      AXI_STR_TXC_TKEEP => AXI_STR_TXC_TKEEP,
      AXI_STR_TXC_TDATA => AXI_STR_TXC_TDATA,
      AXI_STR_RXD_ACLK => AXI_STR_RXD_ACLK,
      AXI_STR_RXD_ARESETN => AXI_STR_RXD_ARESETN,
      AXI_STR_RXD_TVALID => AXI_STR_RXD_TVALID,
      AXI_STR_RXD_TREADY => AXI_STR_RXD_TREADY,
      AXI_STR_RXD_TLAST => AXI_STR_RXD_TLAST,
      AXI_STR_RXD_TKEEP => AXI_STR_RXD_TKEEP,
      AXI_STR_RXD_TDATA => AXI_STR_RXD_TDATA,
      AXI_STR_RXS_ACLK => AXI_STR_RXS_ACLK,
      AXI_STR_RXS_ARESETN => AXI_STR_RXS_ARESETN,
      AXI_STR_RXS_TVALID => AXI_STR_RXS_TVALID,
      AXI_STR_RXS_TREADY => AXI_STR_RXS_TREADY,
      AXI_STR_RXS_TLAST => AXI_STR_RXS_TLAST,
      AXI_STR_RXS_TKEEP => AXI_STR_RXS_TKEEP,
      AXI_STR_RXS_TDATA => AXI_STR_RXS_TDATA,
      PHY_RST_N => PHY_RST_N,
      GTX_CLK => GTX_CLK,
      MGT_CLK_P => MGT_CLK_P,
      MGT_CLK_N => MGT_CLK_N,
      REF_CLK => REF_CLK,
      MII_TXD => MII_TXD,
      MII_TX_EN => MII_TX_EN,
      MII_TX_ER => MII_TX_ER,
      MII_RXD => MII_RXD,
      MII_RX_DV => MII_RX_DV,
      MII_RX_ER => MII_RX_ER,
      MII_RX_CLK => MII_RX_CLK,
      MII_TX_CLK => MII_TX_CLK,
      MII_COL => MII_COL,
      MII_CRS => MII_CRS,
      GMII_TXD => GMII_TXD,
      GMII_TX_EN => GMII_TX_EN,
      GMII_TX_ER => GMII_TX_ER,
      GMII_TX_CLK => GMII_TX_CLK,
      GMII_RXD => GMII_RXD,
      GMII_RX_DV => GMII_RX_DV,
      GMII_RX_ER => GMII_RX_ER,
      GMII_RX_CLK => GMII_RX_CLK,
      GMII_COL => GMII_COL,
      GMII_CRS => GMII_CRS,
      TXP => TXP,
      TXN => TXN,
      RXP => RXP,
      RXN => RXN,
      RGMII_TXD => RGMII_TXD,
      RGMII_TX_CTL => RGMII_TX_CTL,
      RGMII_TXC => RGMII_TXC,
      RGMII_RXD => RGMII_RXD,
      RGMII_RX_CTL => RGMII_RX_CTL,
      RGMII_RXC => RGMII_RXC,
      MDC => MDC,
      MDIO_I => MDIO_I,
      MDIO_O => MDIO_O,
      MDIO_T => MDIO_T,
      AXI_STR_AVBTX_ACLK => AXI_STR_AVBTX_ACLK,
      AXI_STR_AVBTX_ARESETN => AXI_STR_AVBTX_ARESETN,
      AXI_STR_AVBTX_TVALID => AXI_STR_AVBTX_TVALID,
      AXI_STR_AVBTX_TREADY => AXI_STR_AVBTX_TREADY,
      AXI_STR_AVBTX_TLAST => AXI_STR_AVBTX_TLAST,
      AXI_STR_AVBTX_TDATA => AXI_STR_AVBTX_TDATA,
      AXI_STR_AVBTX_TUSER => AXI_STR_AVBTX_TUSER,
      AXI_STR_AVBRX_ACLK => AXI_STR_AVBRX_ACLK,
      AXI_STR_AVBRX_ARESETN => AXI_STR_AVBRX_ARESETN,
      AXI_STR_AVBRX_TVALID => AXI_STR_AVBRX_TVALID,
      AXI_STR_AVBRX_TLAST => AXI_STR_AVBRX_TLAST,
      AXI_STR_AVBRX_TDATA => AXI_STR_AVBRX_TDATA,
      AXI_STR_AVBRX_TUSER => AXI_STR_AVBRX_TUSER,
      RTC_CLK => RTC_CLK,
      AV_INTERRUPT_10MS => AV_INTERRUPT_10MS,
      AV_INTERRUPT_PTP_TX => AV_INTERRUPT_PTP_TX,
      AV_INTERRUPT_PTP_RX => AV_INTERRUPT_PTP_RX,
      AV_RTC_NANOSECFIELD => AV_RTC_NANOSECFIELD,
      AV_RTC_SECFIELD => AV_RTC_SECFIELD,
      AV_CLK_8K => AV_CLK_8K,
      AV_RTC_NANOSECFIELD_1722 => AV_RTC_NANOSECFIELD_1722
    );

end architecture STRUCTURE;

