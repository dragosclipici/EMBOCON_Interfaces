-------------------------------------------------------------------------------
-- system.vhd
-------------------------------------------------------------------------------
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

library UNISIM;
use UNISIM.VCOMPONENTS.ALL;

entity system is
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
end system;

architecture STRUCTURE of system is

  component system_proc_sys_reset_0_wrapper is
    port (
      Slowest_sync_clk : in std_logic;
      Ext_Reset_In : in std_logic;
      Aux_Reset_In : in std_logic;
      MB_Debug_Sys_Rst : in std_logic;
      Core_Reset_Req_0 : in std_logic;
      Chip_Reset_Req_0 : in std_logic;
      System_Reset_Req_0 : in std_logic;
      Core_Reset_Req_1 : in std_logic;
      Chip_Reset_Req_1 : in std_logic;
      System_Reset_Req_1 : in std_logic;
      Dcm_locked : in std_logic;
      RstcPPCresetcore_0 : out std_logic;
      RstcPPCresetchip_0 : out std_logic;
      RstcPPCresetsys_0 : out std_logic;
      RstcPPCresetcore_1 : out std_logic;
      RstcPPCresetchip_1 : out std_logic;
      RstcPPCresetsys_1 : out std_logic;
      MB_Reset : out std_logic;
      Bus_Struct_Reset : out std_logic_vector(0 to 0);
      Peripheral_Reset : out std_logic_vector(0 to 0);
      Interconnect_aresetn : out std_logic_vector(0 to 0);
      Peripheral_aresetn : out std_logic_vector(0 to 0)
    );
  end component;

  component system_microblaze_0_intc_wrapper is
    port (
      S_AXI_ACLK : in std_logic;
      S_AXI_ARESETN : in std_logic;
      S_AXI_AWADDR : in std_logic_vector(8 downto 0);
      S_AXI_AWVALID : in std_logic;
      S_AXI_AWREADY : out std_logic;
      S_AXI_WDATA : in std_logic_vector(31 downto 0);
      S_AXI_WSTRB : in std_logic_vector(3 downto 0);
      S_AXI_WVALID : in std_logic;
      S_AXI_WREADY : out std_logic;
      S_AXI_BRESP : out std_logic_vector(1 downto 0);
      S_AXI_BVALID : out std_logic;
      S_AXI_BREADY : in std_logic;
      S_AXI_ARADDR : in std_logic_vector(8 downto 0);
      S_AXI_ARVALID : in std_logic;
      S_AXI_ARREADY : out std_logic;
      S_AXI_RDATA : out std_logic_vector(31 downto 0);
      S_AXI_RRESP : out std_logic_vector(1 downto 0);
      S_AXI_RVALID : out std_logic;
      S_AXI_RREADY : in std_logic;
      Intr : in std_logic_vector(3 downto 0);
      Processor_clk : in std_logic;
      Processor_rst : in std_logic;
      Irq : out std_logic;
      Interrupt_address : out std_logic_vector(31 downto 0);
      Processor_ack : in std_logic_vector(1 downto 0)
    );
  end component;

  component system_microblaze_0_ilmb_wrapper is
    port (
      LMB_Clk : in std_logic;
      SYS_Rst : in std_logic;
      LMB_Rst : out std_logic;
      M_ABus : in std_logic_vector(0 to 31);
      M_ReadStrobe : in std_logic;
      M_WriteStrobe : in std_logic;
      M_AddrStrobe : in std_logic;
      M_DBus : in std_logic_vector(0 to 31);
      M_BE : in std_logic_vector(0 to 3);
      Sl_DBus : in std_logic_vector(0 to 31);
      Sl_Ready : in std_logic_vector(0 to 0);
      Sl_Wait : in std_logic_vector(0 to 0);
      Sl_UE : in std_logic_vector(0 to 0);
      Sl_CE : in std_logic_vector(0 to 0);
      LMB_ABus : out std_logic_vector(0 to 31);
      LMB_ReadStrobe : out std_logic;
      LMB_WriteStrobe : out std_logic;
      LMB_AddrStrobe : out std_logic;
      LMB_ReadDBus : out std_logic_vector(0 to 31);
      LMB_WriteDBus : out std_logic_vector(0 to 31);
      LMB_Ready : out std_logic;
      LMB_Wait : out std_logic;
      LMB_UE : out std_logic;
      LMB_CE : out std_logic;
      LMB_BE : out std_logic_vector(0 to 3)
    );
  end component;

  component system_microblaze_0_i_bram_ctrl_wrapper is
    port (
      LMB_Clk : in std_logic;
      LMB_Rst : in std_logic;
      LMB_ABus : in std_logic_vector(0 to 31);
      LMB_WriteDBus : in std_logic_vector(0 to 31);
      LMB_AddrStrobe : in std_logic;
      LMB_ReadStrobe : in std_logic;
      LMB_WriteStrobe : in std_logic;
      LMB_BE : in std_logic_vector(0 to 3);
      Sl_DBus : out std_logic_vector(0 to 31);
      Sl_Ready : out std_logic;
      Sl_Wait : out std_logic;
      Sl_UE : out std_logic;
      Sl_CE : out std_logic;
      LMB1_ABus : in std_logic_vector(0 to 31);
      LMB1_WriteDBus : in std_logic_vector(0 to 31);
      LMB1_AddrStrobe : in std_logic;
      LMB1_ReadStrobe : in std_logic;
      LMB1_WriteStrobe : in std_logic;
      LMB1_BE : in std_logic_vector(0 to 3);
      Sl1_DBus : out std_logic_vector(0 to 31);
      Sl1_Ready : out std_logic;
      Sl1_Wait : out std_logic;
      Sl1_UE : out std_logic;
      Sl1_CE : out std_logic;
      LMB2_ABus : in std_logic_vector(0 to 31);
      LMB2_WriteDBus : in std_logic_vector(0 to 31);
      LMB2_AddrStrobe : in std_logic;
      LMB2_ReadStrobe : in std_logic;
      LMB2_WriteStrobe : in std_logic;
      LMB2_BE : in std_logic_vector(0 to 3);
      Sl2_DBus : out std_logic_vector(0 to 31);
      Sl2_Ready : out std_logic;
      Sl2_Wait : out std_logic;
      Sl2_UE : out std_logic;
      Sl2_CE : out std_logic;
      LMB3_ABus : in std_logic_vector(0 to 31);
      LMB3_WriteDBus : in std_logic_vector(0 to 31);
      LMB3_AddrStrobe : in std_logic;
      LMB3_ReadStrobe : in std_logic;
      LMB3_WriteStrobe : in std_logic;
      LMB3_BE : in std_logic_vector(0 to 3);
      Sl3_DBus : out std_logic_vector(0 to 31);
      Sl3_Ready : out std_logic;
      Sl3_Wait : out std_logic;
      Sl3_UE : out std_logic;
      Sl3_CE : out std_logic;
      BRAM_Rst_A : out std_logic;
      BRAM_Clk_A : out std_logic;
      BRAM_EN_A : out std_logic;
      BRAM_WEN_A : out std_logic_vector(0 to 3);
      BRAM_Addr_A : out std_logic_vector(0 to 31);
      BRAM_Din_A : in std_logic_vector(0 to 31);
      BRAM_Dout_A : out std_logic_vector(0 to 31);
      Interrupt : out std_logic;
      UE : out std_logic;
      CE : out std_logic;
      SPLB_CTRL_PLB_ABus : in std_logic_vector(0 to 31);
      SPLB_CTRL_PLB_PAValid : in std_logic;
      SPLB_CTRL_PLB_masterID : in std_logic_vector(0 to 0);
      SPLB_CTRL_PLB_RNW : in std_logic;
      SPLB_CTRL_PLB_BE : in std_logic_vector(0 to 3);
      SPLB_CTRL_PLB_size : in std_logic_vector(0 to 3);
      SPLB_CTRL_PLB_type : in std_logic_vector(0 to 2);
      SPLB_CTRL_PLB_wrDBus : in std_logic_vector(0 to 31);
      SPLB_CTRL_Sl_addrAck : out std_logic;
      SPLB_CTRL_Sl_SSize : out std_logic_vector(0 to 1);
      SPLB_CTRL_Sl_wait : out std_logic;
      SPLB_CTRL_Sl_rearbitrate : out std_logic;
      SPLB_CTRL_Sl_wrDAck : out std_logic;
      SPLB_CTRL_Sl_wrComp : out std_logic;
      SPLB_CTRL_Sl_rdDBus : out std_logic_vector(0 to 31);
      SPLB_CTRL_Sl_rdDAck : out std_logic;
      SPLB_CTRL_Sl_rdComp : out std_logic;
      SPLB_CTRL_Sl_MBusy : out std_logic_vector(0 to 0);
      SPLB_CTRL_Sl_MWrErr : out std_logic_vector(0 to 0);
      SPLB_CTRL_Sl_MRdErr : out std_logic_vector(0 to 0);
      SPLB_CTRL_PLB_UABus : in std_logic_vector(0 to 31);
      SPLB_CTRL_PLB_SAValid : in std_logic;
      SPLB_CTRL_PLB_rdPrim : in std_logic;
      SPLB_CTRL_PLB_wrPrim : in std_logic;
      SPLB_CTRL_PLB_abort : in std_logic;
      SPLB_CTRL_PLB_busLock : in std_logic;
      SPLB_CTRL_PLB_MSize : in std_logic_vector(0 to 1);
      SPLB_CTRL_PLB_lockErr : in std_logic;
      SPLB_CTRL_PLB_wrBurst : in std_logic;
      SPLB_CTRL_PLB_rdBurst : in std_logic;
      SPLB_CTRL_PLB_wrPendReq : in std_logic;
      SPLB_CTRL_PLB_rdPendReq : in std_logic;
      SPLB_CTRL_PLB_wrPendPri : in std_logic_vector(0 to 1);
      SPLB_CTRL_PLB_rdPendPri : in std_logic_vector(0 to 1);
      SPLB_CTRL_PLB_reqPri : in std_logic_vector(0 to 1);
      SPLB_CTRL_PLB_TAttribute : in std_logic_vector(0 to 15);
      SPLB_CTRL_Sl_wrBTerm : out std_logic;
      SPLB_CTRL_Sl_rdWdAddr : out std_logic_vector(0 to 3);
      SPLB_CTRL_Sl_rdBTerm : out std_logic;
      SPLB_CTRL_Sl_MIRQ : out std_logic_vector(0 to 0);
      S_AXI_CTRL_ACLK : in std_logic;
      S_AXI_CTRL_ARESETN : in std_logic;
      S_AXI_CTRL_AWADDR : in std_logic_vector(31 downto 0);
      S_AXI_CTRL_AWVALID : in std_logic;
      S_AXI_CTRL_AWREADY : out std_logic;
      S_AXI_CTRL_WDATA : in std_logic_vector(31 downto 0);
      S_AXI_CTRL_WSTRB : in std_logic_vector(3 downto 0);
      S_AXI_CTRL_WVALID : in std_logic;
      S_AXI_CTRL_WREADY : out std_logic;
      S_AXI_CTRL_BRESP : out std_logic_vector(1 downto 0);
      S_AXI_CTRL_BVALID : out std_logic;
      S_AXI_CTRL_BREADY : in std_logic;
      S_AXI_CTRL_ARADDR : in std_logic_vector(31 downto 0);
      S_AXI_CTRL_ARVALID : in std_logic;
      S_AXI_CTRL_ARREADY : out std_logic;
      S_AXI_CTRL_RDATA : out std_logic_vector(31 downto 0);
      S_AXI_CTRL_RRESP : out std_logic_vector(1 downto 0);
      S_AXI_CTRL_RVALID : out std_logic;
      S_AXI_CTRL_RREADY : in std_logic
    );
  end component;

  component system_microblaze_0_dlmb_wrapper is
    port (
      LMB_Clk : in std_logic;
      SYS_Rst : in std_logic;
      LMB_Rst : out std_logic;
      M_ABus : in std_logic_vector(0 to 31);
      M_ReadStrobe : in std_logic;
      M_WriteStrobe : in std_logic;
      M_AddrStrobe : in std_logic;
      M_DBus : in std_logic_vector(0 to 31);
      M_BE : in std_logic_vector(0 to 3);
      Sl_DBus : in std_logic_vector(0 to 31);
      Sl_Ready : in std_logic_vector(0 to 0);
      Sl_Wait : in std_logic_vector(0 to 0);
      Sl_UE : in std_logic_vector(0 to 0);
      Sl_CE : in std_logic_vector(0 to 0);
      LMB_ABus : out std_logic_vector(0 to 31);
      LMB_ReadStrobe : out std_logic;
      LMB_WriteStrobe : out std_logic;
      LMB_AddrStrobe : out std_logic;
      LMB_ReadDBus : out std_logic_vector(0 to 31);
      LMB_WriteDBus : out std_logic_vector(0 to 31);
      LMB_Ready : out std_logic;
      LMB_Wait : out std_logic;
      LMB_UE : out std_logic;
      LMB_CE : out std_logic;
      LMB_BE : out std_logic_vector(0 to 3)
    );
  end component;

  component system_microblaze_0_d_bram_ctrl_wrapper is
    port (
      LMB_Clk : in std_logic;
      LMB_Rst : in std_logic;
      LMB_ABus : in std_logic_vector(0 to 31);
      LMB_WriteDBus : in std_logic_vector(0 to 31);
      LMB_AddrStrobe : in std_logic;
      LMB_ReadStrobe : in std_logic;
      LMB_WriteStrobe : in std_logic;
      LMB_BE : in std_logic_vector(0 to 3);
      Sl_DBus : out std_logic_vector(0 to 31);
      Sl_Ready : out std_logic;
      Sl_Wait : out std_logic;
      Sl_UE : out std_logic;
      Sl_CE : out std_logic;
      LMB1_ABus : in std_logic_vector(0 to 31);
      LMB1_WriteDBus : in std_logic_vector(0 to 31);
      LMB1_AddrStrobe : in std_logic;
      LMB1_ReadStrobe : in std_logic;
      LMB1_WriteStrobe : in std_logic;
      LMB1_BE : in std_logic_vector(0 to 3);
      Sl1_DBus : out std_logic_vector(0 to 31);
      Sl1_Ready : out std_logic;
      Sl1_Wait : out std_logic;
      Sl1_UE : out std_logic;
      Sl1_CE : out std_logic;
      LMB2_ABus : in std_logic_vector(0 to 31);
      LMB2_WriteDBus : in std_logic_vector(0 to 31);
      LMB2_AddrStrobe : in std_logic;
      LMB2_ReadStrobe : in std_logic;
      LMB2_WriteStrobe : in std_logic;
      LMB2_BE : in std_logic_vector(0 to 3);
      Sl2_DBus : out std_logic_vector(0 to 31);
      Sl2_Ready : out std_logic;
      Sl2_Wait : out std_logic;
      Sl2_UE : out std_logic;
      Sl2_CE : out std_logic;
      LMB3_ABus : in std_logic_vector(0 to 31);
      LMB3_WriteDBus : in std_logic_vector(0 to 31);
      LMB3_AddrStrobe : in std_logic;
      LMB3_ReadStrobe : in std_logic;
      LMB3_WriteStrobe : in std_logic;
      LMB3_BE : in std_logic_vector(0 to 3);
      Sl3_DBus : out std_logic_vector(0 to 31);
      Sl3_Ready : out std_logic;
      Sl3_Wait : out std_logic;
      Sl3_UE : out std_logic;
      Sl3_CE : out std_logic;
      BRAM_Rst_A : out std_logic;
      BRAM_Clk_A : out std_logic;
      BRAM_EN_A : out std_logic;
      BRAM_WEN_A : out std_logic_vector(0 to 3);
      BRAM_Addr_A : out std_logic_vector(0 to 31);
      BRAM_Din_A : in std_logic_vector(0 to 31);
      BRAM_Dout_A : out std_logic_vector(0 to 31);
      Interrupt : out std_logic;
      UE : out std_logic;
      CE : out std_logic;
      SPLB_CTRL_PLB_ABus : in std_logic_vector(0 to 31);
      SPLB_CTRL_PLB_PAValid : in std_logic;
      SPLB_CTRL_PLB_masterID : in std_logic_vector(0 to 0);
      SPLB_CTRL_PLB_RNW : in std_logic;
      SPLB_CTRL_PLB_BE : in std_logic_vector(0 to 3);
      SPLB_CTRL_PLB_size : in std_logic_vector(0 to 3);
      SPLB_CTRL_PLB_type : in std_logic_vector(0 to 2);
      SPLB_CTRL_PLB_wrDBus : in std_logic_vector(0 to 31);
      SPLB_CTRL_Sl_addrAck : out std_logic;
      SPLB_CTRL_Sl_SSize : out std_logic_vector(0 to 1);
      SPLB_CTRL_Sl_wait : out std_logic;
      SPLB_CTRL_Sl_rearbitrate : out std_logic;
      SPLB_CTRL_Sl_wrDAck : out std_logic;
      SPLB_CTRL_Sl_wrComp : out std_logic;
      SPLB_CTRL_Sl_rdDBus : out std_logic_vector(0 to 31);
      SPLB_CTRL_Sl_rdDAck : out std_logic;
      SPLB_CTRL_Sl_rdComp : out std_logic;
      SPLB_CTRL_Sl_MBusy : out std_logic_vector(0 to 0);
      SPLB_CTRL_Sl_MWrErr : out std_logic_vector(0 to 0);
      SPLB_CTRL_Sl_MRdErr : out std_logic_vector(0 to 0);
      SPLB_CTRL_PLB_UABus : in std_logic_vector(0 to 31);
      SPLB_CTRL_PLB_SAValid : in std_logic;
      SPLB_CTRL_PLB_rdPrim : in std_logic;
      SPLB_CTRL_PLB_wrPrim : in std_logic;
      SPLB_CTRL_PLB_abort : in std_logic;
      SPLB_CTRL_PLB_busLock : in std_logic;
      SPLB_CTRL_PLB_MSize : in std_logic_vector(0 to 1);
      SPLB_CTRL_PLB_lockErr : in std_logic;
      SPLB_CTRL_PLB_wrBurst : in std_logic;
      SPLB_CTRL_PLB_rdBurst : in std_logic;
      SPLB_CTRL_PLB_wrPendReq : in std_logic;
      SPLB_CTRL_PLB_rdPendReq : in std_logic;
      SPLB_CTRL_PLB_wrPendPri : in std_logic_vector(0 to 1);
      SPLB_CTRL_PLB_rdPendPri : in std_logic_vector(0 to 1);
      SPLB_CTRL_PLB_reqPri : in std_logic_vector(0 to 1);
      SPLB_CTRL_PLB_TAttribute : in std_logic_vector(0 to 15);
      SPLB_CTRL_Sl_wrBTerm : out std_logic;
      SPLB_CTRL_Sl_rdWdAddr : out std_logic_vector(0 to 3);
      SPLB_CTRL_Sl_rdBTerm : out std_logic;
      SPLB_CTRL_Sl_MIRQ : out std_logic_vector(0 to 0);
      S_AXI_CTRL_ACLK : in std_logic;
      S_AXI_CTRL_ARESETN : in std_logic;
      S_AXI_CTRL_AWADDR : in std_logic_vector(31 downto 0);
      S_AXI_CTRL_AWVALID : in std_logic;
      S_AXI_CTRL_AWREADY : out std_logic;
      S_AXI_CTRL_WDATA : in std_logic_vector(31 downto 0);
      S_AXI_CTRL_WSTRB : in std_logic_vector(3 downto 0);
      S_AXI_CTRL_WVALID : in std_logic;
      S_AXI_CTRL_WREADY : out std_logic;
      S_AXI_CTRL_BRESP : out std_logic_vector(1 downto 0);
      S_AXI_CTRL_BVALID : out std_logic;
      S_AXI_CTRL_BREADY : in std_logic;
      S_AXI_CTRL_ARADDR : in std_logic_vector(31 downto 0);
      S_AXI_CTRL_ARVALID : in std_logic;
      S_AXI_CTRL_ARREADY : out std_logic;
      S_AXI_CTRL_RDATA : out std_logic_vector(31 downto 0);
      S_AXI_CTRL_RRESP : out std_logic_vector(1 downto 0);
      S_AXI_CTRL_RVALID : out std_logic;
      S_AXI_CTRL_RREADY : in std_logic
    );
  end component;

  component system_microblaze_0_bram_block_wrapper is
    port (
      BRAM_Rst_A : in std_logic;
      BRAM_Clk_A : in std_logic;
      BRAM_EN_A : in std_logic;
      BRAM_WEN_A : in std_logic_vector(0 to 3);
      BRAM_Addr_A : in std_logic_vector(0 to 31);
      BRAM_Din_A : out std_logic_vector(0 to 31);
      BRAM_Dout_A : in std_logic_vector(0 to 31);
      BRAM_Rst_B : in std_logic;
      BRAM_Clk_B : in std_logic;
      BRAM_EN_B : in std_logic;
      BRAM_WEN_B : in std_logic_vector(0 to 3);
      BRAM_Addr_B : in std_logic_vector(0 to 31);
      BRAM_Din_B : out std_logic_vector(0 to 31);
      BRAM_Dout_B : in std_logic_vector(0 to 31)
    );
  end component;

  component system_microblaze_0_wrapper is
    port (
      CLK : in std_logic;
      RESET : in std_logic;
      MB_RESET : in std_logic;
      INTERRUPT : in std_logic;
      INTERRUPT_ADDRESS : in std_logic_vector(0 to 31);
      INTERRUPT_ACK : out std_logic_vector(0 to 1);
      EXT_BRK : in std_logic;
      EXT_NM_BRK : in std_logic;
      DBG_STOP : in std_logic;
      MB_Halted : out std_logic;
      MB_Error : out std_logic;
      WAKEUP : in std_logic_vector(0 to 1);
      SLEEP : out std_logic;
      DBG_WAKEUP : out std_logic;
      LOCKSTEP_MASTER_OUT : out std_logic_vector(0 to 4095);
      LOCKSTEP_SLAVE_IN : in std_logic_vector(0 to 4095);
      LOCKSTEP_OUT : out std_logic_vector(0 to 4095);
      INSTR : in std_logic_vector(0 to 31);
      IREADY : in std_logic;
      IWAIT : in std_logic;
      ICE : in std_logic;
      IUE : in std_logic;
      INSTR_ADDR : out std_logic_vector(0 to 31);
      IFETCH : out std_logic;
      I_AS : out std_logic;
      IPLB_M_ABort : out std_logic;
      IPLB_M_ABus : out std_logic_vector(0 to 31);
      IPLB_M_UABus : out std_logic_vector(0 to 31);
      IPLB_M_BE : out std_logic_vector(0 to 3);
      IPLB_M_busLock : out std_logic;
      IPLB_M_lockErr : out std_logic;
      IPLB_M_MSize : out std_logic_vector(0 to 1);
      IPLB_M_priority : out std_logic_vector(0 to 1);
      IPLB_M_rdBurst : out std_logic;
      IPLB_M_request : out std_logic;
      IPLB_M_RNW : out std_logic;
      IPLB_M_size : out std_logic_vector(0 to 3);
      IPLB_M_TAttribute : out std_logic_vector(0 to 15);
      IPLB_M_type : out std_logic_vector(0 to 2);
      IPLB_M_wrBurst : out std_logic;
      IPLB_M_wrDBus : out std_logic_vector(0 to 31);
      IPLB_MBusy : in std_logic;
      IPLB_MRdErr : in std_logic;
      IPLB_MWrErr : in std_logic;
      IPLB_MIRQ : in std_logic;
      IPLB_MWrBTerm : in std_logic;
      IPLB_MWrDAck : in std_logic;
      IPLB_MAddrAck : in std_logic;
      IPLB_MRdBTerm : in std_logic;
      IPLB_MRdDAck : in std_logic;
      IPLB_MRdDBus : in std_logic_vector(0 to 31);
      IPLB_MRdWdAddr : in std_logic_vector(0 to 3);
      IPLB_MRearbitrate : in std_logic;
      IPLB_MSSize : in std_logic_vector(0 to 1);
      IPLB_MTimeout : in std_logic;
      DATA_READ : in std_logic_vector(0 to 31);
      DREADY : in std_logic;
      DWAIT : in std_logic;
      DCE : in std_logic;
      DUE : in std_logic;
      DATA_WRITE : out std_logic_vector(0 to 31);
      DATA_ADDR : out std_logic_vector(0 to 31);
      D_AS : out std_logic;
      READ_STROBE : out std_logic;
      WRITE_STROBE : out std_logic;
      BYTE_ENABLE : out std_logic_vector(0 to 3);
      DPLB_M_ABort : out std_logic;
      DPLB_M_ABus : out std_logic_vector(0 to 31);
      DPLB_M_UABus : out std_logic_vector(0 to 31);
      DPLB_M_BE : out std_logic_vector(0 to 3);
      DPLB_M_busLock : out std_logic;
      DPLB_M_lockErr : out std_logic;
      DPLB_M_MSize : out std_logic_vector(0 to 1);
      DPLB_M_priority : out std_logic_vector(0 to 1);
      DPLB_M_rdBurst : out std_logic;
      DPLB_M_request : out std_logic;
      DPLB_M_RNW : out std_logic;
      DPLB_M_size : out std_logic_vector(0 to 3);
      DPLB_M_TAttribute : out std_logic_vector(0 to 15);
      DPLB_M_type : out std_logic_vector(0 to 2);
      DPLB_M_wrBurst : out std_logic;
      DPLB_M_wrDBus : out std_logic_vector(0 to 31);
      DPLB_MBusy : in std_logic;
      DPLB_MRdErr : in std_logic;
      DPLB_MWrErr : in std_logic;
      DPLB_MIRQ : in std_logic;
      DPLB_MWrBTerm : in std_logic;
      DPLB_MWrDAck : in std_logic;
      DPLB_MAddrAck : in std_logic;
      DPLB_MRdBTerm : in std_logic;
      DPLB_MRdDAck : in std_logic;
      DPLB_MRdDBus : in std_logic_vector(0 to 31);
      DPLB_MRdWdAddr : in std_logic_vector(0 to 3);
      DPLB_MRearbitrate : in std_logic;
      DPLB_MSSize : in std_logic_vector(0 to 1);
      DPLB_MTimeout : in std_logic;
      M_AXI_IP_AWID : out std_logic_vector(0 downto 0);
      M_AXI_IP_AWADDR : out std_logic_vector(31 downto 0);
      M_AXI_IP_AWLEN : out std_logic_vector(7 downto 0);
      M_AXI_IP_AWSIZE : out std_logic_vector(2 downto 0);
      M_AXI_IP_AWBURST : out std_logic_vector(1 downto 0);
      M_AXI_IP_AWLOCK : out std_logic;
      M_AXI_IP_AWCACHE : out std_logic_vector(3 downto 0);
      M_AXI_IP_AWPROT : out std_logic_vector(2 downto 0);
      M_AXI_IP_AWQOS : out std_logic_vector(3 downto 0);
      M_AXI_IP_AWVALID : out std_logic;
      M_AXI_IP_AWREADY : in std_logic;
      M_AXI_IP_WDATA : out std_logic_vector(31 downto 0);
      M_AXI_IP_WSTRB : out std_logic_vector(3 downto 0);
      M_AXI_IP_WLAST : out std_logic;
      M_AXI_IP_WVALID : out std_logic;
      M_AXI_IP_WREADY : in std_logic;
      M_AXI_IP_BID : in std_logic_vector(0 downto 0);
      M_AXI_IP_BRESP : in std_logic_vector(1 downto 0);
      M_AXI_IP_BVALID : in std_logic;
      M_AXI_IP_BREADY : out std_logic;
      M_AXI_IP_ARID : out std_logic_vector(0 downto 0);
      M_AXI_IP_ARADDR : out std_logic_vector(31 downto 0);
      M_AXI_IP_ARLEN : out std_logic_vector(7 downto 0);
      M_AXI_IP_ARSIZE : out std_logic_vector(2 downto 0);
      M_AXI_IP_ARBURST : out std_logic_vector(1 downto 0);
      M_AXI_IP_ARLOCK : out std_logic;
      M_AXI_IP_ARCACHE : out std_logic_vector(3 downto 0);
      M_AXI_IP_ARPROT : out std_logic_vector(2 downto 0);
      M_AXI_IP_ARQOS : out std_logic_vector(3 downto 0);
      M_AXI_IP_ARVALID : out std_logic;
      M_AXI_IP_ARREADY : in std_logic;
      M_AXI_IP_RID : in std_logic_vector(0 downto 0);
      M_AXI_IP_RDATA : in std_logic_vector(31 downto 0);
      M_AXI_IP_RRESP : in std_logic_vector(1 downto 0);
      M_AXI_IP_RLAST : in std_logic;
      M_AXI_IP_RVALID : in std_logic;
      M_AXI_IP_RREADY : out std_logic;
      M_AXI_DP_AWID : out std_logic_vector(0 downto 0);
      M_AXI_DP_AWADDR : out std_logic_vector(31 downto 0);
      M_AXI_DP_AWLEN : out std_logic_vector(7 downto 0);
      M_AXI_DP_AWSIZE : out std_logic_vector(2 downto 0);
      M_AXI_DP_AWBURST : out std_logic_vector(1 downto 0);
      M_AXI_DP_AWLOCK : out std_logic;
      M_AXI_DP_AWCACHE : out std_logic_vector(3 downto 0);
      M_AXI_DP_AWPROT : out std_logic_vector(2 downto 0);
      M_AXI_DP_AWQOS : out std_logic_vector(3 downto 0);
      M_AXI_DP_AWVALID : out std_logic;
      M_AXI_DP_AWREADY : in std_logic;
      M_AXI_DP_WDATA : out std_logic_vector(31 downto 0);
      M_AXI_DP_WSTRB : out std_logic_vector(3 downto 0);
      M_AXI_DP_WLAST : out std_logic;
      M_AXI_DP_WVALID : out std_logic;
      M_AXI_DP_WREADY : in std_logic;
      M_AXI_DP_BID : in std_logic_vector(0 downto 0);
      M_AXI_DP_BRESP : in std_logic_vector(1 downto 0);
      M_AXI_DP_BVALID : in std_logic;
      M_AXI_DP_BREADY : out std_logic;
      M_AXI_DP_ARID : out std_logic_vector(0 downto 0);
      M_AXI_DP_ARADDR : out std_logic_vector(31 downto 0);
      M_AXI_DP_ARLEN : out std_logic_vector(7 downto 0);
      M_AXI_DP_ARSIZE : out std_logic_vector(2 downto 0);
      M_AXI_DP_ARBURST : out std_logic_vector(1 downto 0);
      M_AXI_DP_ARLOCK : out std_logic;
      M_AXI_DP_ARCACHE : out std_logic_vector(3 downto 0);
      M_AXI_DP_ARPROT : out std_logic_vector(2 downto 0);
      M_AXI_DP_ARQOS : out std_logic_vector(3 downto 0);
      M_AXI_DP_ARVALID : out std_logic;
      M_AXI_DP_ARREADY : in std_logic;
      M_AXI_DP_RID : in std_logic_vector(0 downto 0);
      M_AXI_DP_RDATA : in std_logic_vector(31 downto 0);
      M_AXI_DP_RRESP : in std_logic_vector(1 downto 0);
      M_AXI_DP_RLAST : in std_logic;
      M_AXI_DP_RVALID : in std_logic;
      M_AXI_DP_RREADY : out std_logic;
      M_AXI_IC_AWID : out std_logic_vector(0 downto 0);
      M_AXI_IC_AWADDR : out std_logic_vector(31 downto 0);
      M_AXI_IC_AWLEN : out std_logic_vector(7 downto 0);
      M_AXI_IC_AWSIZE : out std_logic_vector(2 downto 0);
      M_AXI_IC_AWBURST : out std_logic_vector(1 downto 0);
      M_AXI_IC_AWLOCK : out std_logic;
      M_AXI_IC_AWCACHE : out std_logic_vector(3 downto 0);
      M_AXI_IC_AWPROT : out std_logic_vector(2 downto 0);
      M_AXI_IC_AWQOS : out std_logic_vector(3 downto 0);
      M_AXI_IC_AWVALID : out std_logic;
      M_AXI_IC_AWREADY : in std_logic;
      M_AXI_IC_AWUSER : out std_logic_vector(4 downto 0);
      M_AXI_IC_WDATA : out std_logic_vector(31 downto 0);
      M_AXI_IC_WSTRB : out std_logic_vector(3 downto 0);
      M_AXI_IC_WLAST : out std_logic;
      M_AXI_IC_WVALID : out std_logic;
      M_AXI_IC_WREADY : in std_logic;
      M_AXI_IC_WUSER : out std_logic_vector(0 downto 0);
      M_AXI_IC_BID : in std_logic_vector(0 downto 0);
      M_AXI_IC_BRESP : in std_logic_vector(1 downto 0);
      M_AXI_IC_BVALID : in std_logic;
      M_AXI_IC_BREADY : out std_logic;
      M_AXI_IC_BUSER : in std_logic_vector(0 downto 0);
      M_AXI_IC_ARID : out std_logic_vector(0 downto 0);
      M_AXI_IC_ARADDR : out std_logic_vector(31 downto 0);
      M_AXI_IC_ARLEN : out std_logic_vector(7 downto 0);
      M_AXI_IC_ARSIZE : out std_logic_vector(2 downto 0);
      M_AXI_IC_ARBURST : out std_logic_vector(1 downto 0);
      M_AXI_IC_ARLOCK : out std_logic;
      M_AXI_IC_ARCACHE : out std_logic_vector(3 downto 0);
      M_AXI_IC_ARPROT : out std_logic_vector(2 downto 0);
      M_AXI_IC_ARQOS : out std_logic_vector(3 downto 0);
      M_AXI_IC_ARVALID : out std_logic;
      M_AXI_IC_ARREADY : in std_logic;
      M_AXI_IC_ARUSER : out std_logic_vector(4 downto 0);
      M_AXI_IC_RID : in std_logic_vector(0 downto 0);
      M_AXI_IC_RDATA : in std_logic_vector(31 downto 0);
      M_AXI_IC_RRESP : in std_logic_vector(1 downto 0);
      M_AXI_IC_RLAST : in std_logic;
      M_AXI_IC_RVALID : in std_logic;
      M_AXI_IC_RREADY : out std_logic;
      M_AXI_IC_RUSER : in std_logic_vector(0 downto 0);
      M_AXI_DC_AWID : out std_logic_vector(0 downto 0);
      M_AXI_DC_AWADDR : out std_logic_vector(31 downto 0);
      M_AXI_DC_AWLEN : out std_logic_vector(7 downto 0);
      M_AXI_DC_AWSIZE : out std_logic_vector(2 downto 0);
      M_AXI_DC_AWBURST : out std_logic_vector(1 downto 0);
      M_AXI_DC_AWLOCK : out std_logic;
      M_AXI_DC_AWCACHE : out std_logic_vector(3 downto 0);
      M_AXI_DC_AWPROT : out std_logic_vector(2 downto 0);
      M_AXI_DC_AWQOS : out std_logic_vector(3 downto 0);
      M_AXI_DC_AWVALID : out std_logic;
      M_AXI_DC_AWREADY : in std_logic;
      M_AXI_DC_AWUSER : out std_logic_vector(4 downto 0);
      M_AXI_DC_WDATA : out std_logic_vector(31 downto 0);
      M_AXI_DC_WSTRB : out std_logic_vector(3 downto 0);
      M_AXI_DC_WLAST : out std_logic;
      M_AXI_DC_WVALID : out std_logic;
      M_AXI_DC_WREADY : in std_logic;
      M_AXI_DC_WUSER : out std_logic_vector(0 downto 0);
      M_AXI_DC_BID : in std_logic_vector(0 downto 0);
      M_AXI_DC_BRESP : in std_logic_vector(1 downto 0);
      M_AXI_DC_BVALID : in std_logic;
      M_AXI_DC_BREADY : out std_logic;
      M_AXI_DC_BUSER : in std_logic_vector(0 downto 0);
      M_AXI_DC_ARID : out std_logic_vector(0 downto 0);
      M_AXI_DC_ARADDR : out std_logic_vector(31 downto 0);
      M_AXI_DC_ARLEN : out std_logic_vector(7 downto 0);
      M_AXI_DC_ARSIZE : out std_logic_vector(2 downto 0);
      M_AXI_DC_ARBURST : out std_logic_vector(1 downto 0);
      M_AXI_DC_ARLOCK : out std_logic;
      M_AXI_DC_ARCACHE : out std_logic_vector(3 downto 0);
      M_AXI_DC_ARPROT : out std_logic_vector(2 downto 0);
      M_AXI_DC_ARQOS : out std_logic_vector(3 downto 0);
      M_AXI_DC_ARVALID : out std_logic;
      M_AXI_DC_ARREADY : in std_logic;
      M_AXI_DC_ARUSER : out std_logic_vector(4 downto 0);
      M_AXI_DC_RID : in std_logic_vector(0 downto 0);
      M_AXI_DC_RDATA : in std_logic_vector(31 downto 0);
      M_AXI_DC_RRESP : in std_logic_vector(1 downto 0);
      M_AXI_DC_RLAST : in std_logic;
      M_AXI_DC_RVALID : in std_logic;
      M_AXI_DC_RREADY : out std_logic;
      M_AXI_DC_RUSER : in std_logic_vector(0 downto 0);
      DBG_CLK : in std_logic;
      DBG_TDI : in std_logic;
      DBG_TDO : out std_logic;
      DBG_REG_EN : in std_logic_vector(0 to 7);
      DBG_SHIFT : in std_logic;
      DBG_CAPTURE : in std_logic;
      DBG_UPDATE : in std_logic;
      DEBUG_RST : in std_logic;
      Trace_Instruction : out std_logic_vector(0 to 31);
      Trace_Valid_Instr : out std_logic;
      Trace_PC : out std_logic_vector(0 to 31);
      Trace_Reg_Write : out std_logic;
      Trace_Reg_Addr : out std_logic_vector(0 to 4);
      Trace_MSR_Reg : out std_logic_vector(0 to 14);
      Trace_PID_Reg : out std_logic_vector(0 to 7);
      Trace_New_Reg_Value : out std_logic_vector(0 to 31);
      Trace_Exception_Taken : out std_logic;
      Trace_Exception_Kind : out std_logic_vector(0 to 4);
      Trace_Jump_Taken : out std_logic;
      Trace_Delay_Slot : out std_logic;
      Trace_Data_Address : out std_logic_vector(0 to 31);
      Trace_Data_Access : out std_logic;
      Trace_Data_Read : out std_logic;
      Trace_Data_Write : out std_logic;
      Trace_Data_Write_Value : out std_logic_vector(0 to 31);
      Trace_Data_Byte_Enable : out std_logic_vector(0 to 3);
      Trace_DCache_Req : out std_logic;
      Trace_DCache_Hit : out std_logic;
      Trace_DCache_Rdy : out std_logic;
      Trace_DCache_Read : out std_logic;
      Trace_ICache_Req : out std_logic;
      Trace_ICache_Hit : out std_logic;
      Trace_ICache_Rdy : out std_logic;
      Trace_OF_PipeRun : out std_logic;
      Trace_EX_PipeRun : out std_logic;
      Trace_MEM_PipeRun : out std_logic;
      Trace_MB_Halted : out std_logic;
      Trace_Jump_Hit : out std_logic;
      FSL0_S_CLK : out std_logic;
      FSL0_S_READ : out std_logic;
      FSL0_S_DATA : in std_logic_vector(0 to 31);
      FSL0_S_CONTROL : in std_logic;
      FSL0_S_EXISTS : in std_logic;
      FSL0_M_CLK : out std_logic;
      FSL0_M_WRITE : out std_logic;
      FSL0_M_DATA : out std_logic_vector(0 to 31);
      FSL0_M_CONTROL : out std_logic;
      FSL0_M_FULL : in std_logic;
      FSL1_S_CLK : out std_logic;
      FSL1_S_READ : out std_logic;
      FSL1_S_DATA : in std_logic_vector(0 to 31);
      FSL1_S_CONTROL : in std_logic;
      FSL1_S_EXISTS : in std_logic;
      FSL1_M_CLK : out std_logic;
      FSL1_M_WRITE : out std_logic;
      FSL1_M_DATA : out std_logic_vector(0 to 31);
      FSL1_M_CONTROL : out std_logic;
      FSL1_M_FULL : in std_logic;
      FSL2_S_CLK : out std_logic;
      FSL2_S_READ : out std_logic;
      FSL2_S_DATA : in std_logic_vector(0 to 31);
      FSL2_S_CONTROL : in std_logic;
      FSL2_S_EXISTS : in std_logic;
      FSL2_M_CLK : out std_logic;
      FSL2_M_WRITE : out std_logic;
      FSL2_M_DATA : out std_logic_vector(0 to 31);
      FSL2_M_CONTROL : out std_logic;
      FSL2_M_FULL : in std_logic;
      FSL3_S_CLK : out std_logic;
      FSL3_S_READ : out std_logic;
      FSL3_S_DATA : in std_logic_vector(0 to 31);
      FSL3_S_CONTROL : in std_logic;
      FSL3_S_EXISTS : in std_logic;
      FSL3_M_CLK : out std_logic;
      FSL3_M_WRITE : out std_logic;
      FSL3_M_DATA : out std_logic_vector(0 to 31);
      FSL3_M_CONTROL : out std_logic;
      FSL3_M_FULL : in std_logic;
      FSL4_S_CLK : out std_logic;
      FSL4_S_READ : out std_logic;
      FSL4_S_DATA : in std_logic_vector(0 to 31);
      FSL4_S_CONTROL : in std_logic;
      FSL4_S_EXISTS : in std_logic;
      FSL4_M_CLK : out std_logic;
      FSL4_M_WRITE : out std_logic;
      FSL4_M_DATA : out std_logic_vector(0 to 31);
      FSL4_M_CONTROL : out std_logic;
      FSL4_M_FULL : in std_logic;
      FSL5_S_CLK : out std_logic;
      FSL5_S_READ : out std_logic;
      FSL5_S_DATA : in std_logic_vector(0 to 31);
      FSL5_S_CONTROL : in std_logic;
      FSL5_S_EXISTS : in std_logic;
      FSL5_M_CLK : out std_logic;
      FSL5_M_WRITE : out std_logic;
      FSL5_M_DATA : out std_logic_vector(0 to 31);
      FSL5_M_CONTROL : out std_logic;
      FSL5_M_FULL : in std_logic;
      FSL6_S_CLK : out std_logic;
      FSL6_S_READ : out std_logic;
      FSL6_S_DATA : in std_logic_vector(0 to 31);
      FSL6_S_CONTROL : in std_logic;
      FSL6_S_EXISTS : in std_logic;
      FSL6_M_CLK : out std_logic;
      FSL6_M_WRITE : out std_logic;
      FSL6_M_DATA : out std_logic_vector(0 to 31);
      FSL6_M_CONTROL : out std_logic;
      FSL6_M_FULL : in std_logic;
      FSL7_S_CLK : out std_logic;
      FSL7_S_READ : out std_logic;
      FSL7_S_DATA : in std_logic_vector(0 to 31);
      FSL7_S_CONTROL : in std_logic;
      FSL7_S_EXISTS : in std_logic;
      FSL7_M_CLK : out std_logic;
      FSL7_M_WRITE : out std_logic;
      FSL7_M_DATA : out std_logic_vector(0 to 31);
      FSL7_M_CONTROL : out std_logic;
      FSL7_M_FULL : in std_logic;
      FSL8_S_CLK : out std_logic;
      FSL8_S_READ : out std_logic;
      FSL8_S_DATA : in std_logic_vector(0 to 31);
      FSL8_S_CONTROL : in std_logic;
      FSL8_S_EXISTS : in std_logic;
      FSL8_M_CLK : out std_logic;
      FSL8_M_WRITE : out std_logic;
      FSL8_M_DATA : out std_logic_vector(0 to 31);
      FSL8_M_CONTROL : out std_logic;
      FSL8_M_FULL : in std_logic;
      FSL9_S_CLK : out std_logic;
      FSL9_S_READ : out std_logic;
      FSL9_S_DATA : in std_logic_vector(0 to 31);
      FSL9_S_CONTROL : in std_logic;
      FSL9_S_EXISTS : in std_logic;
      FSL9_M_CLK : out std_logic;
      FSL9_M_WRITE : out std_logic;
      FSL9_M_DATA : out std_logic_vector(0 to 31);
      FSL9_M_CONTROL : out std_logic;
      FSL9_M_FULL : in std_logic;
      FSL10_S_CLK : out std_logic;
      FSL10_S_READ : out std_logic;
      FSL10_S_DATA : in std_logic_vector(0 to 31);
      FSL10_S_CONTROL : in std_logic;
      FSL10_S_EXISTS : in std_logic;
      FSL10_M_CLK : out std_logic;
      FSL10_M_WRITE : out std_logic;
      FSL10_M_DATA : out std_logic_vector(0 to 31);
      FSL10_M_CONTROL : out std_logic;
      FSL10_M_FULL : in std_logic;
      FSL11_S_CLK : out std_logic;
      FSL11_S_READ : out std_logic;
      FSL11_S_DATA : in std_logic_vector(0 to 31);
      FSL11_S_CONTROL : in std_logic;
      FSL11_S_EXISTS : in std_logic;
      FSL11_M_CLK : out std_logic;
      FSL11_M_WRITE : out std_logic;
      FSL11_M_DATA : out std_logic_vector(0 to 31);
      FSL11_M_CONTROL : out std_logic;
      FSL11_M_FULL : in std_logic;
      FSL12_S_CLK : out std_logic;
      FSL12_S_READ : out std_logic;
      FSL12_S_DATA : in std_logic_vector(0 to 31);
      FSL12_S_CONTROL : in std_logic;
      FSL12_S_EXISTS : in std_logic;
      FSL12_M_CLK : out std_logic;
      FSL12_M_WRITE : out std_logic;
      FSL12_M_DATA : out std_logic_vector(0 to 31);
      FSL12_M_CONTROL : out std_logic;
      FSL12_M_FULL : in std_logic;
      FSL13_S_CLK : out std_logic;
      FSL13_S_READ : out std_logic;
      FSL13_S_DATA : in std_logic_vector(0 to 31);
      FSL13_S_CONTROL : in std_logic;
      FSL13_S_EXISTS : in std_logic;
      FSL13_M_CLK : out std_logic;
      FSL13_M_WRITE : out std_logic;
      FSL13_M_DATA : out std_logic_vector(0 to 31);
      FSL13_M_CONTROL : out std_logic;
      FSL13_M_FULL : in std_logic;
      FSL14_S_CLK : out std_logic;
      FSL14_S_READ : out std_logic;
      FSL14_S_DATA : in std_logic_vector(0 to 31);
      FSL14_S_CONTROL : in std_logic;
      FSL14_S_EXISTS : in std_logic;
      FSL14_M_CLK : out std_logic;
      FSL14_M_WRITE : out std_logic;
      FSL14_M_DATA : out std_logic_vector(0 to 31);
      FSL14_M_CONTROL : out std_logic;
      FSL14_M_FULL : in std_logic;
      FSL15_S_CLK : out std_logic;
      FSL15_S_READ : out std_logic;
      FSL15_S_DATA : in std_logic_vector(0 to 31);
      FSL15_S_CONTROL : in std_logic;
      FSL15_S_EXISTS : in std_logic;
      FSL15_M_CLK : out std_logic;
      FSL15_M_WRITE : out std_logic;
      FSL15_M_DATA : out std_logic_vector(0 to 31);
      FSL15_M_CONTROL : out std_logic;
      FSL15_M_FULL : in std_logic;
      M0_AXIS_TLAST : out std_logic;
      M0_AXIS_TDATA : out std_logic_vector(31 downto 0);
      M0_AXIS_TVALID : out std_logic;
      M0_AXIS_TREADY : in std_logic;
      S0_AXIS_TLAST : in std_logic;
      S0_AXIS_TDATA : in std_logic_vector(31 downto 0);
      S0_AXIS_TVALID : in std_logic;
      S0_AXIS_TREADY : out std_logic;
      M1_AXIS_TLAST : out std_logic;
      M1_AXIS_TDATA : out std_logic_vector(31 downto 0);
      M1_AXIS_TVALID : out std_logic;
      M1_AXIS_TREADY : in std_logic;
      S1_AXIS_TLAST : in std_logic;
      S1_AXIS_TDATA : in std_logic_vector(31 downto 0);
      S1_AXIS_TVALID : in std_logic;
      S1_AXIS_TREADY : out std_logic;
      M2_AXIS_TLAST : out std_logic;
      M2_AXIS_TDATA : out std_logic_vector(31 downto 0);
      M2_AXIS_TVALID : out std_logic;
      M2_AXIS_TREADY : in std_logic;
      S2_AXIS_TLAST : in std_logic;
      S2_AXIS_TDATA : in std_logic_vector(31 downto 0);
      S2_AXIS_TVALID : in std_logic;
      S2_AXIS_TREADY : out std_logic;
      M3_AXIS_TLAST : out std_logic;
      M3_AXIS_TDATA : out std_logic_vector(31 downto 0);
      M3_AXIS_TVALID : out std_logic;
      M3_AXIS_TREADY : in std_logic;
      S3_AXIS_TLAST : in std_logic;
      S3_AXIS_TDATA : in std_logic_vector(31 downto 0);
      S3_AXIS_TVALID : in std_logic;
      S3_AXIS_TREADY : out std_logic;
      M4_AXIS_TLAST : out std_logic;
      M4_AXIS_TDATA : out std_logic_vector(31 downto 0);
      M4_AXIS_TVALID : out std_logic;
      M4_AXIS_TREADY : in std_logic;
      S4_AXIS_TLAST : in std_logic;
      S4_AXIS_TDATA : in std_logic_vector(31 downto 0);
      S4_AXIS_TVALID : in std_logic;
      S4_AXIS_TREADY : out std_logic;
      M5_AXIS_TLAST : out std_logic;
      M5_AXIS_TDATA : out std_logic_vector(31 downto 0);
      M5_AXIS_TVALID : out std_logic;
      M5_AXIS_TREADY : in std_logic;
      S5_AXIS_TLAST : in std_logic;
      S5_AXIS_TDATA : in std_logic_vector(31 downto 0);
      S5_AXIS_TVALID : in std_logic;
      S5_AXIS_TREADY : out std_logic;
      M6_AXIS_TLAST : out std_logic;
      M6_AXIS_TDATA : out std_logic_vector(31 downto 0);
      M6_AXIS_TVALID : out std_logic;
      M6_AXIS_TREADY : in std_logic;
      S6_AXIS_TLAST : in std_logic;
      S6_AXIS_TDATA : in std_logic_vector(31 downto 0);
      S6_AXIS_TVALID : in std_logic;
      S6_AXIS_TREADY : out std_logic;
      M7_AXIS_TLAST : out std_logic;
      M7_AXIS_TDATA : out std_logic_vector(31 downto 0);
      M7_AXIS_TVALID : out std_logic;
      M7_AXIS_TREADY : in std_logic;
      S7_AXIS_TLAST : in std_logic;
      S7_AXIS_TDATA : in std_logic_vector(31 downto 0);
      S7_AXIS_TVALID : in std_logic;
      S7_AXIS_TREADY : out std_logic;
      M8_AXIS_TLAST : out std_logic;
      M8_AXIS_TDATA : out std_logic_vector(31 downto 0);
      M8_AXIS_TVALID : out std_logic;
      M8_AXIS_TREADY : in std_logic;
      S8_AXIS_TLAST : in std_logic;
      S8_AXIS_TDATA : in std_logic_vector(31 downto 0);
      S8_AXIS_TVALID : in std_logic;
      S8_AXIS_TREADY : out std_logic;
      M9_AXIS_TLAST : out std_logic;
      M9_AXIS_TDATA : out std_logic_vector(31 downto 0);
      M9_AXIS_TVALID : out std_logic;
      M9_AXIS_TREADY : in std_logic;
      S9_AXIS_TLAST : in std_logic;
      S9_AXIS_TDATA : in std_logic_vector(31 downto 0);
      S9_AXIS_TVALID : in std_logic;
      S9_AXIS_TREADY : out std_logic;
      M10_AXIS_TLAST : out std_logic;
      M10_AXIS_TDATA : out std_logic_vector(31 downto 0);
      M10_AXIS_TVALID : out std_logic;
      M10_AXIS_TREADY : in std_logic;
      S10_AXIS_TLAST : in std_logic;
      S10_AXIS_TDATA : in std_logic_vector(31 downto 0);
      S10_AXIS_TVALID : in std_logic;
      S10_AXIS_TREADY : out std_logic;
      M11_AXIS_TLAST : out std_logic;
      M11_AXIS_TDATA : out std_logic_vector(31 downto 0);
      M11_AXIS_TVALID : out std_logic;
      M11_AXIS_TREADY : in std_logic;
      S11_AXIS_TLAST : in std_logic;
      S11_AXIS_TDATA : in std_logic_vector(31 downto 0);
      S11_AXIS_TVALID : in std_logic;
      S11_AXIS_TREADY : out std_logic;
      M12_AXIS_TLAST : out std_logic;
      M12_AXIS_TDATA : out std_logic_vector(31 downto 0);
      M12_AXIS_TVALID : out std_logic;
      M12_AXIS_TREADY : in std_logic;
      S12_AXIS_TLAST : in std_logic;
      S12_AXIS_TDATA : in std_logic_vector(31 downto 0);
      S12_AXIS_TVALID : in std_logic;
      S12_AXIS_TREADY : out std_logic;
      M13_AXIS_TLAST : out std_logic;
      M13_AXIS_TDATA : out std_logic_vector(31 downto 0);
      M13_AXIS_TVALID : out std_logic;
      M13_AXIS_TREADY : in std_logic;
      S13_AXIS_TLAST : in std_logic;
      S13_AXIS_TDATA : in std_logic_vector(31 downto 0);
      S13_AXIS_TVALID : in std_logic;
      S13_AXIS_TREADY : out std_logic;
      M14_AXIS_TLAST : out std_logic;
      M14_AXIS_TDATA : out std_logic_vector(31 downto 0);
      M14_AXIS_TVALID : out std_logic;
      M14_AXIS_TREADY : in std_logic;
      S14_AXIS_TLAST : in std_logic;
      S14_AXIS_TDATA : in std_logic_vector(31 downto 0);
      S14_AXIS_TVALID : in std_logic;
      S14_AXIS_TREADY : out std_logic;
      M15_AXIS_TLAST : out std_logic;
      M15_AXIS_TDATA : out std_logic_vector(31 downto 0);
      M15_AXIS_TVALID : out std_logic;
      M15_AXIS_TREADY : in std_logic;
      S15_AXIS_TLAST : in std_logic;
      S15_AXIS_TDATA : in std_logic_vector(31 downto 0);
      S15_AXIS_TVALID : in std_logic;
      S15_AXIS_TREADY : out std_logic;
      ICACHE_FSL_IN_CLK : out std_logic;
      ICACHE_FSL_IN_READ : out std_logic;
      ICACHE_FSL_IN_DATA : in std_logic_vector(0 to 31);
      ICACHE_FSL_IN_CONTROL : in std_logic;
      ICACHE_FSL_IN_EXISTS : in std_logic;
      ICACHE_FSL_OUT_CLK : out std_logic;
      ICACHE_FSL_OUT_WRITE : out std_logic;
      ICACHE_FSL_OUT_DATA : out std_logic_vector(0 to 31);
      ICACHE_FSL_OUT_CONTROL : out std_logic;
      ICACHE_FSL_OUT_FULL : in std_logic;
      DCACHE_FSL_IN_CLK : out std_logic;
      DCACHE_FSL_IN_READ : out std_logic;
      DCACHE_FSL_IN_DATA : in std_logic_vector(0 to 31);
      DCACHE_FSL_IN_CONTROL : in std_logic;
      DCACHE_FSL_IN_EXISTS : in std_logic;
      DCACHE_FSL_OUT_CLK : out std_logic;
      DCACHE_FSL_OUT_WRITE : out std_logic;
      DCACHE_FSL_OUT_DATA : out std_logic_vector(0 to 31);
      DCACHE_FSL_OUT_CONTROL : out std_logic;
      DCACHE_FSL_OUT_FULL : in std_logic
    );
  end component;

  component system_debug_module_wrapper is
    port (
      Interrupt : out std_logic;
      Debug_SYS_Rst : out std_logic;
      Ext_BRK : out std_logic;
      Ext_NM_BRK : out std_logic;
      S_AXI_ACLK : in std_logic;
      S_AXI_ARESETN : in std_logic;
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
      SPLB_Clk : in std_logic;
      SPLB_Rst : in std_logic;
      PLB_ABus : in std_logic_vector(0 to 31);
      PLB_UABus : in std_logic_vector(0 to 31);
      PLB_PAValid : in std_logic;
      PLB_SAValid : in std_logic;
      PLB_rdPrim : in std_logic;
      PLB_wrPrim : in std_logic;
      PLB_masterID : in std_logic_vector(0 to 2);
      PLB_abort : in std_logic;
      PLB_busLock : in std_logic;
      PLB_RNW : in std_logic;
      PLB_BE : in std_logic_vector(0 to 3);
      PLB_MSize : in std_logic_vector(0 to 1);
      PLB_size : in std_logic_vector(0 to 3);
      PLB_type : in std_logic_vector(0 to 2);
      PLB_lockErr : in std_logic;
      PLB_wrDBus : in std_logic_vector(0 to 31);
      PLB_wrBurst : in std_logic;
      PLB_rdBurst : in std_logic;
      PLB_wrPendReq : in std_logic;
      PLB_rdPendReq : in std_logic;
      PLB_wrPendPri : in std_logic_vector(0 to 1);
      PLB_rdPendPri : in std_logic_vector(0 to 1);
      PLB_reqPri : in std_logic_vector(0 to 1);
      PLB_TAttribute : in std_logic_vector(0 to 15);
      Sl_addrAck : out std_logic;
      Sl_SSize : out std_logic_vector(0 to 1);
      Sl_wait : out std_logic;
      Sl_rearbitrate : out std_logic;
      Sl_wrDAck : out std_logic;
      Sl_wrComp : out std_logic;
      Sl_wrBTerm : out std_logic;
      Sl_rdDBus : out std_logic_vector(0 to 31);
      Sl_rdWdAddr : out std_logic_vector(0 to 3);
      Sl_rdDAck : out std_logic;
      Sl_rdComp : out std_logic;
      Sl_rdBTerm : out std_logic;
      Sl_MBusy : out std_logic_vector(0 to 7);
      Sl_MWrErr : out std_logic_vector(0 to 7);
      Sl_MRdErr : out std_logic_vector(0 to 7);
      Sl_MIRQ : out std_logic_vector(0 to 7);
      Dbg_Clk_0 : out std_logic;
      Dbg_TDI_0 : out std_logic;
      Dbg_TDO_0 : in std_logic;
      Dbg_Reg_En_0 : out std_logic_vector(0 to 7);
      Dbg_Capture_0 : out std_logic;
      Dbg_Shift_0 : out std_logic;
      Dbg_Update_0 : out std_logic;
      Dbg_Rst_0 : out std_logic;
      Dbg_Clk_1 : out std_logic;
      Dbg_TDI_1 : out std_logic;
      Dbg_TDO_1 : in std_logic;
      Dbg_Reg_En_1 : out std_logic_vector(0 to 7);
      Dbg_Capture_1 : out std_logic;
      Dbg_Shift_1 : out std_logic;
      Dbg_Update_1 : out std_logic;
      Dbg_Rst_1 : out std_logic;
      Dbg_Clk_2 : out std_logic;
      Dbg_TDI_2 : out std_logic;
      Dbg_TDO_2 : in std_logic;
      Dbg_Reg_En_2 : out std_logic_vector(0 to 7);
      Dbg_Capture_2 : out std_logic;
      Dbg_Shift_2 : out std_logic;
      Dbg_Update_2 : out std_logic;
      Dbg_Rst_2 : out std_logic;
      Dbg_Clk_3 : out std_logic;
      Dbg_TDI_3 : out std_logic;
      Dbg_TDO_3 : in std_logic;
      Dbg_Reg_En_3 : out std_logic_vector(0 to 7);
      Dbg_Capture_3 : out std_logic;
      Dbg_Shift_3 : out std_logic;
      Dbg_Update_3 : out std_logic;
      Dbg_Rst_3 : out std_logic;
      Dbg_Clk_4 : out std_logic;
      Dbg_TDI_4 : out std_logic;
      Dbg_TDO_4 : in std_logic;
      Dbg_Reg_En_4 : out std_logic_vector(0 to 7);
      Dbg_Capture_4 : out std_logic;
      Dbg_Shift_4 : out std_logic;
      Dbg_Update_4 : out std_logic;
      Dbg_Rst_4 : out std_logic;
      Dbg_Clk_5 : out std_logic;
      Dbg_TDI_5 : out std_logic;
      Dbg_TDO_5 : in std_logic;
      Dbg_Reg_En_5 : out std_logic_vector(0 to 7);
      Dbg_Capture_5 : out std_logic;
      Dbg_Shift_5 : out std_logic;
      Dbg_Update_5 : out std_logic;
      Dbg_Rst_5 : out std_logic;
      Dbg_Clk_6 : out std_logic;
      Dbg_TDI_6 : out std_logic;
      Dbg_TDO_6 : in std_logic;
      Dbg_Reg_En_6 : out std_logic_vector(0 to 7);
      Dbg_Capture_6 : out std_logic;
      Dbg_Shift_6 : out std_logic;
      Dbg_Update_6 : out std_logic;
      Dbg_Rst_6 : out std_logic;
      Dbg_Clk_7 : out std_logic;
      Dbg_TDI_7 : out std_logic;
      Dbg_TDO_7 : in std_logic;
      Dbg_Reg_En_7 : out std_logic_vector(0 to 7);
      Dbg_Capture_7 : out std_logic;
      Dbg_Shift_7 : out std_logic;
      Dbg_Update_7 : out std_logic;
      Dbg_Rst_7 : out std_logic;
      Dbg_Clk_8 : out std_logic;
      Dbg_TDI_8 : out std_logic;
      Dbg_TDO_8 : in std_logic;
      Dbg_Reg_En_8 : out std_logic_vector(0 to 7);
      Dbg_Capture_8 : out std_logic;
      Dbg_Shift_8 : out std_logic;
      Dbg_Update_8 : out std_logic;
      Dbg_Rst_8 : out std_logic;
      Dbg_Clk_9 : out std_logic;
      Dbg_TDI_9 : out std_logic;
      Dbg_TDO_9 : in std_logic;
      Dbg_Reg_En_9 : out std_logic_vector(0 to 7);
      Dbg_Capture_9 : out std_logic;
      Dbg_Shift_9 : out std_logic;
      Dbg_Update_9 : out std_logic;
      Dbg_Rst_9 : out std_logic;
      Dbg_Clk_10 : out std_logic;
      Dbg_TDI_10 : out std_logic;
      Dbg_TDO_10 : in std_logic;
      Dbg_Reg_En_10 : out std_logic_vector(0 to 7);
      Dbg_Capture_10 : out std_logic;
      Dbg_Shift_10 : out std_logic;
      Dbg_Update_10 : out std_logic;
      Dbg_Rst_10 : out std_logic;
      Dbg_Clk_11 : out std_logic;
      Dbg_TDI_11 : out std_logic;
      Dbg_TDO_11 : in std_logic;
      Dbg_Reg_En_11 : out std_logic_vector(0 to 7);
      Dbg_Capture_11 : out std_logic;
      Dbg_Shift_11 : out std_logic;
      Dbg_Update_11 : out std_logic;
      Dbg_Rst_11 : out std_logic;
      Dbg_Clk_12 : out std_logic;
      Dbg_TDI_12 : out std_logic;
      Dbg_TDO_12 : in std_logic;
      Dbg_Reg_En_12 : out std_logic_vector(0 to 7);
      Dbg_Capture_12 : out std_logic;
      Dbg_Shift_12 : out std_logic;
      Dbg_Update_12 : out std_logic;
      Dbg_Rst_12 : out std_logic;
      Dbg_Clk_13 : out std_logic;
      Dbg_TDI_13 : out std_logic;
      Dbg_TDO_13 : in std_logic;
      Dbg_Reg_En_13 : out std_logic_vector(0 to 7);
      Dbg_Capture_13 : out std_logic;
      Dbg_Shift_13 : out std_logic;
      Dbg_Update_13 : out std_logic;
      Dbg_Rst_13 : out std_logic;
      Dbg_Clk_14 : out std_logic;
      Dbg_TDI_14 : out std_logic;
      Dbg_TDO_14 : in std_logic;
      Dbg_Reg_En_14 : out std_logic_vector(0 to 7);
      Dbg_Capture_14 : out std_logic;
      Dbg_Shift_14 : out std_logic;
      Dbg_Update_14 : out std_logic;
      Dbg_Rst_14 : out std_logic;
      Dbg_Clk_15 : out std_logic;
      Dbg_TDI_15 : out std_logic;
      Dbg_TDO_15 : in std_logic;
      Dbg_Reg_En_15 : out std_logic_vector(0 to 7);
      Dbg_Capture_15 : out std_logic;
      Dbg_Shift_15 : out std_logic;
      Dbg_Update_15 : out std_logic;
      Dbg_Rst_15 : out std_logic;
      Dbg_Clk_16 : out std_logic;
      Dbg_TDI_16 : out std_logic;
      Dbg_TDO_16 : in std_logic;
      Dbg_Reg_En_16 : out std_logic_vector(0 to 7);
      Dbg_Capture_16 : out std_logic;
      Dbg_Shift_16 : out std_logic;
      Dbg_Update_16 : out std_logic;
      Dbg_Rst_16 : out std_logic;
      Dbg_Clk_17 : out std_logic;
      Dbg_TDI_17 : out std_logic;
      Dbg_TDO_17 : in std_logic;
      Dbg_Reg_En_17 : out std_logic_vector(0 to 7);
      Dbg_Capture_17 : out std_logic;
      Dbg_Shift_17 : out std_logic;
      Dbg_Update_17 : out std_logic;
      Dbg_Rst_17 : out std_logic;
      Dbg_Clk_18 : out std_logic;
      Dbg_TDI_18 : out std_logic;
      Dbg_TDO_18 : in std_logic;
      Dbg_Reg_En_18 : out std_logic_vector(0 to 7);
      Dbg_Capture_18 : out std_logic;
      Dbg_Shift_18 : out std_logic;
      Dbg_Update_18 : out std_logic;
      Dbg_Rst_18 : out std_logic;
      Dbg_Clk_19 : out std_logic;
      Dbg_TDI_19 : out std_logic;
      Dbg_TDO_19 : in std_logic;
      Dbg_Reg_En_19 : out std_logic_vector(0 to 7);
      Dbg_Capture_19 : out std_logic;
      Dbg_Shift_19 : out std_logic;
      Dbg_Update_19 : out std_logic;
      Dbg_Rst_19 : out std_logic;
      Dbg_Clk_20 : out std_logic;
      Dbg_TDI_20 : out std_logic;
      Dbg_TDO_20 : in std_logic;
      Dbg_Reg_En_20 : out std_logic_vector(0 to 7);
      Dbg_Capture_20 : out std_logic;
      Dbg_Shift_20 : out std_logic;
      Dbg_Update_20 : out std_logic;
      Dbg_Rst_20 : out std_logic;
      Dbg_Clk_21 : out std_logic;
      Dbg_TDI_21 : out std_logic;
      Dbg_TDO_21 : in std_logic;
      Dbg_Reg_En_21 : out std_logic_vector(0 to 7);
      Dbg_Capture_21 : out std_logic;
      Dbg_Shift_21 : out std_logic;
      Dbg_Update_21 : out std_logic;
      Dbg_Rst_21 : out std_logic;
      Dbg_Clk_22 : out std_logic;
      Dbg_TDI_22 : out std_logic;
      Dbg_TDO_22 : in std_logic;
      Dbg_Reg_En_22 : out std_logic_vector(0 to 7);
      Dbg_Capture_22 : out std_logic;
      Dbg_Shift_22 : out std_logic;
      Dbg_Update_22 : out std_logic;
      Dbg_Rst_22 : out std_logic;
      Dbg_Clk_23 : out std_logic;
      Dbg_TDI_23 : out std_logic;
      Dbg_TDO_23 : in std_logic;
      Dbg_Reg_En_23 : out std_logic_vector(0 to 7);
      Dbg_Capture_23 : out std_logic;
      Dbg_Shift_23 : out std_logic;
      Dbg_Update_23 : out std_logic;
      Dbg_Rst_23 : out std_logic;
      Dbg_Clk_24 : out std_logic;
      Dbg_TDI_24 : out std_logic;
      Dbg_TDO_24 : in std_logic;
      Dbg_Reg_En_24 : out std_logic_vector(0 to 7);
      Dbg_Capture_24 : out std_logic;
      Dbg_Shift_24 : out std_logic;
      Dbg_Update_24 : out std_logic;
      Dbg_Rst_24 : out std_logic;
      Dbg_Clk_25 : out std_logic;
      Dbg_TDI_25 : out std_logic;
      Dbg_TDO_25 : in std_logic;
      Dbg_Reg_En_25 : out std_logic_vector(0 to 7);
      Dbg_Capture_25 : out std_logic;
      Dbg_Shift_25 : out std_logic;
      Dbg_Update_25 : out std_logic;
      Dbg_Rst_25 : out std_logic;
      Dbg_Clk_26 : out std_logic;
      Dbg_TDI_26 : out std_logic;
      Dbg_TDO_26 : in std_logic;
      Dbg_Reg_En_26 : out std_logic_vector(0 to 7);
      Dbg_Capture_26 : out std_logic;
      Dbg_Shift_26 : out std_logic;
      Dbg_Update_26 : out std_logic;
      Dbg_Rst_26 : out std_logic;
      Dbg_Clk_27 : out std_logic;
      Dbg_TDI_27 : out std_logic;
      Dbg_TDO_27 : in std_logic;
      Dbg_Reg_En_27 : out std_logic_vector(0 to 7);
      Dbg_Capture_27 : out std_logic;
      Dbg_Shift_27 : out std_logic;
      Dbg_Update_27 : out std_logic;
      Dbg_Rst_27 : out std_logic;
      Dbg_Clk_28 : out std_logic;
      Dbg_TDI_28 : out std_logic;
      Dbg_TDO_28 : in std_logic;
      Dbg_Reg_En_28 : out std_logic_vector(0 to 7);
      Dbg_Capture_28 : out std_logic;
      Dbg_Shift_28 : out std_logic;
      Dbg_Update_28 : out std_logic;
      Dbg_Rst_28 : out std_logic;
      Dbg_Clk_29 : out std_logic;
      Dbg_TDI_29 : out std_logic;
      Dbg_TDO_29 : in std_logic;
      Dbg_Reg_En_29 : out std_logic_vector(0 to 7);
      Dbg_Capture_29 : out std_logic;
      Dbg_Shift_29 : out std_logic;
      Dbg_Update_29 : out std_logic;
      Dbg_Rst_29 : out std_logic;
      Dbg_Clk_30 : out std_logic;
      Dbg_TDI_30 : out std_logic;
      Dbg_TDO_30 : in std_logic;
      Dbg_Reg_En_30 : out std_logic_vector(0 to 7);
      Dbg_Capture_30 : out std_logic;
      Dbg_Shift_30 : out std_logic;
      Dbg_Update_30 : out std_logic;
      Dbg_Rst_30 : out std_logic;
      Dbg_Clk_31 : out std_logic;
      Dbg_TDI_31 : out std_logic;
      Dbg_TDO_31 : in std_logic;
      Dbg_Reg_En_31 : out std_logic_vector(0 to 7);
      Dbg_Capture_31 : out std_logic;
      Dbg_Shift_31 : out std_logic;
      Dbg_Update_31 : out std_logic;
      Dbg_Rst_31 : out std_logic;
      bscan_tdi : out std_logic;
      bscan_reset : out std_logic;
      bscan_shift : out std_logic;
      bscan_update : out std_logic;
      bscan_capture : out std_logic;
      bscan_sel1 : out std_logic;
      bscan_drck1 : out std_logic;
      bscan_tdo1 : in std_logic;
      bscan_ext_tdi : in std_logic;
      bscan_ext_reset : in std_logic;
      bscan_ext_shift : in std_logic;
      bscan_ext_update : in std_logic;
      bscan_ext_capture : in std_logic;
      bscan_ext_sel : in std_logic;
      bscan_ext_drck : in std_logic;
      bscan_ext_tdo : out std_logic;
      Ext_JTAG_DRCK : out std_logic;
      Ext_JTAG_RESET : out std_logic;
      Ext_JTAG_SEL : out std_logic;
      Ext_JTAG_CAPTURE : out std_logic;
      Ext_JTAG_SHIFT : out std_logic;
      Ext_JTAG_UPDATE : out std_logic;
      Ext_JTAG_TDI : out std_logic;
      Ext_JTAG_TDO : in std_logic
    );
  end component;

  component system_clock_generator_0_wrapper is
    port (
      CLKIN : in std_logic;
      CLKOUT0 : out std_logic;
      CLKOUT1 : out std_logic;
      CLKOUT2 : out std_logic;
      CLKOUT3 : out std_logic;
      CLKOUT4 : out std_logic;
      CLKOUT5 : out std_logic;
      CLKOUT6 : out std_logic;
      CLKOUT7 : out std_logic;
      CLKOUT8 : out std_logic;
      CLKOUT9 : out std_logic;
      CLKOUT10 : out std_logic;
      CLKOUT11 : out std_logic;
      CLKOUT12 : out std_logic;
      CLKOUT13 : out std_logic;
      CLKOUT14 : out std_logic;
      CLKOUT15 : out std_logic;
      CLKFBIN : in std_logic;
      CLKFBOUT : out std_logic;
      PSCLK : in std_logic;
      PSEN : in std_logic;
      PSINCDEC : in std_logic;
      PSDONE : out std_logic;
      RST : in std_logic;
      LOCKED : out std_logic
    );
  end component;

  component system_axi_timer_0_wrapper is
    port (
      CaptureTrig0 : in std_logic;
      CaptureTrig1 : in std_logic;
      GenerateOut0 : out std_logic;
      GenerateOut1 : out std_logic;
      PWM0 : out std_logic;
      Interrupt : out std_logic;
      Freeze : in std_logic;
      S_AXI_ACLK : in std_logic;
      S_AXI_ARESETN : in std_logic;
      S_AXI_AWADDR : in std_logic_vector(4 downto 0);
      S_AXI_AWVALID : in std_logic;
      S_AXI_AWREADY : out std_logic;
      S_AXI_WDATA : in std_logic_vector(31 downto 0);
      S_AXI_WSTRB : in std_logic_vector(3 downto 0);
      S_AXI_WVALID : in std_logic;
      S_AXI_WREADY : out std_logic;
      S_AXI_BRESP : out std_logic_vector(1 downto 0);
      S_AXI_BVALID : out std_logic;
      S_AXI_BREADY : in std_logic;
      S_AXI_ARADDR : in std_logic_vector(4 downto 0);
      S_AXI_ARVALID : in std_logic;
      S_AXI_ARREADY : out std_logic;
      S_AXI_RDATA : out std_logic_vector(31 downto 0);
      S_AXI_RRESP : out std_logic_vector(1 downto 0);
      S_AXI_RVALID : out std_logic;
      S_AXI_RREADY : in std_logic
    );
  end component;

  component system_axi4lite_0_wrapper is
    port (
      INTERCONNECT_ACLK : in std_logic;
      INTERCONNECT_ARESETN : in std_logic;
      S_AXI_ARESET_OUT_N : out std_logic_vector(0 to 0);
      M_AXI_ARESET_OUT_N : out std_logic_vector(7 downto 0);
      IRQ : out std_logic;
      S_AXI_ACLK : in std_logic_vector(0 to 0);
      S_AXI_AWID : in std_logic_vector(0 to 0);
      S_AXI_AWADDR : in std_logic_vector(31 downto 0);
      S_AXI_AWLEN : in std_logic_vector(7 downto 0);
      S_AXI_AWSIZE : in std_logic_vector(2 downto 0);
      S_AXI_AWBURST : in std_logic_vector(1 downto 0);
      S_AXI_AWLOCK : in std_logic_vector(1 downto 0);
      S_AXI_AWCACHE : in std_logic_vector(3 downto 0);
      S_AXI_AWPROT : in std_logic_vector(2 downto 0);
      S_AXI_AWQOS : in std_logic_vector(3 downto 0);
      S_AXI_AWUSER : in std_logic_vector(0 to 0);
      S_AXI_AWVALID : in std_logic_vector(0 to 0);
      S_AXI_AWREADY : out std_logic_vector(0 to 0);
      S_AXI_WID : in std_logic_vector(0 to 0);
      S_AXI_WDATA : in std_logic_vector(31 downto 0);
      S_AXI_WSTRB : in std_logic_vector(3 downto 0);
      S_AXI_WLAST : in std_logic_vector(0 to 0);
      S_AXI_WUSER : in std_logic_vector(0 to 0);
      S_AXI_WVALID : in std_logic_vector(0 to 0);
      S_AXI_WREADY : out std_logic_vector(0 to 0);
      S_AXI_BID : out std_logic_vector(0 to 0);
      S_AXI_BRESP : out std_logic_vector(1 downto 0);
      S_AXI_BUSER : out std_logic_vector(0 to 0);
      S_AXI_BVALID : out std_logic_vector(0 to 0);
      S_AXI_BREADY : in std_logic_vector(0 to 0);
      S_AXI_ARID : in std_logic_vector(0 to 0);
      S_AXI_ARADDR : in std_logic_vector(31 downto 0);
      S_AXI_ARLEN : in std_logic_vector(7 downto 0);
      S_AXI_ARSIZE : in std_logic_vector(2 downto 0);
      S_AXI_ARBURST : in std_logic_vector(1 downto 0);
      S_AXI_ARLOCK : in std_logic_vector(1 downto 0);
      S_AXI_ARCACHE : in std_logic_vector(3 downto 0);
      S_AXI_ARPROT : in std_logic_vector(2 downto 0);
      S_AXI_ARQOS : in std_logic_vector(3 downto 0);
      S_AXI_ARUSER : in std_logic_vector(0 to 0);
      S_AXI_ARVALID : in std_logic_vector(0 to 0);
      S_AXI_ARREADY : out std_logic_vector(0 to 0);
      S_AXI_RID : out std_logic_vector(0 to 0);
      S_AXI_RDATA : out std_logic_vector(31 downto 0);
      S_AXI_RRESP : out std_logic_vector(1 downto 0);
      S_AXI_RLAST : out std_logic_vector(0 to 0);
      S_AXI_RUSER : out std_logic_vector(0 to 0);
      S_AXI_RVALID : out std_logic_vector(0 to 0);
      S_AXI_RREADY : in std_logic_vector(0 to 0);
      M_AXI_ACLK : in std_logic_vector(7 downto 0);
      M_AXI_AWID : out std_logic_vector(7 downto 0);
      M_AXI_AWADDR : out std_logic_vector(255 downto 0);
      M_AXI_AWLEN : out std_logic_vector(63 downto 0);
      M_AXI_AWSIZE : out std_logic_vector(23 downto 0);
      M_AXI_AWBURST : out std_logic_vector(15 downto 0);
      M_AXI_AWLOCK : out std_logic_vector(15 downto 0);
      M_AXI_AWCACHE : out std_logic_vector(31 downto 0);
      M_AXI_AWPROT : out std_logic_vector(23 downto 0);
      M_AXI_AWREGION : out std_logic_vector(31 downto 0);
      M_AXI_AWQOS : out std_logic_vector(31 downto 0);
      M_AXI_AWUSER : out std_logic_vector(7 downto 0);
      M_AXI_AWVALID : out std_logic_vector(7 downto 0);
      M_AXI_AWREADY : in std_logic_vector(7 downto 0);
      M_AXI_WID : out std_logic_vector(7 downto 0);
      M_AXI_WDATA : out std_logic_vector(255 downto 0);
      M_AXI_WSTRB : out std_logic_vector(31 downto 0);
      M_AXI_WLAST : out std_logic_vector(7 downto 0);
      M_AXI_WUSER : out std_logic_vector(7 downto 0);
      M_AXI_WVALID : out std_logic_vector(7 downto 0);
      M_AXI_WREADY : in std_logic_vector(7 downto 0);
      M_AXI_BID : in std_logic_vector(7 downto 0);
      M_AXI_BRESP : in std_logic_vector(15 downto 0);
      M_AXI_BUSER : in std_logic_vector(7 downto 0);
      M_AXI_BVALID : in std_logic_vector(7 downto 0);
      M_AXI_BREADY : out std_logic_vector(7 downto 0);
      M_AXI_ARID : out std_logic_vector(7 downto 0);
      M_AXI_ARADDR : out std_logic_vector(255 downto 0);
      M_AXI_ARLEN : out std_logic_vector(63 downto 0);
      M_AXI_ARSIZE : out std_logic_vector(23 downto 0);
      M_AXI_ARBURST : out std_logic_vector(15 downto 0);
      M_AXI_ARLOCK : out std_logic_vector(15 downto 0);
      M_AXI_ARCACHE : out std_logic_vector(31 downto 0);
      M_AXI_ARPROT : out std_logic_vector(23 downto 0);
      M_AXI_ARREGION : out std_logic_vector(31 downto 0);
      M_AXI_ARQOS : out std_logic_vector(31 downto 0);
      M_AXI_ARUSER : out std_logic_vector(7 downto 0);
      M_AXI_ARVALID : out std_logic_vector(7 downto 0);
      M_AXI_ARREADY : in std_logic_vector(7 downto 0);
      M_AXI_RID : in std_logic_vector(7 downto 0);
      M_AXI_RDATA : in std_logic_vector(255 downto 0);
      M_AXI_RRESP : in std_logic_vector(15 downto 0);
      M_AXI_RLAST : in std_logic_vector(7 downto 0);
      M_AXI_RUSER : in std_logic_vector(7 downto 0);
      M_AXI_RVALID : in std_logic_vector(7 downto 0);
      M_AXI_RREADY : out std_logic_vector(7 downto 0);
      S_AXI_CTRL_AWADDR : in std_logic_vector(31 downto 0);
      S_AXI_CTRL_AWVALID : in std_logic;
      S_AXI_CTRL_AWREADY : out std_logic;
      S_AXI_CTRL_WDATA : in std_logic_vector(31 downto 0);
      S_AXI_CTRL_WVALID : in std_logic;
      S_AXI_CTRL_WREADY : out std_logic;
      S_AXI_CTRL_BRESP : out std_logic_vector(1 downto 0);
      S_AXI_CTRL_BVALID : out std_logic;
      S_AXI_CTRL_BREADY : in std_logic;
      S_AXI_CTRL_ARADDR : in std_logic_vector(31 downto 0);
      S_AXI_CTRL_ARVALID : in std_logic;
      S_AXI_CTRL_ARREADY : out std_logic;
      S_AXI_CTRL_RDATA : out std_logic_vector(31 downto 0);
      S_AXI_CTRL_RRESP : out std_logic_vector(1 downto 0);
      S_AXI_CTRL_RVALID : out std_logic;
      S_AXI_CTRL_RREADY : in std_logic;
      INTERCONNECT_ARESET_OUT_N : out std_logic;
      DEBUG_AW_TRANS_SEQ : out std_logic_vector(7 downto 0);
      DEBUG_AW_ARB_GRANT : out std_logic_vector(7 downto 0);
      DEBUG_AR_TRANS_SEQ : out std_logic_vector(7 downto 0);
      DEBUG_AR_ARB_GRANT : out std_logic_vector(7 downto 0);
      DEBUG_AW_TRANS_QUAL : out std_logic_vector(0 to 0);
      DEBUG_AW_ACCEPT_CNT : out std_logic_vector(7 downto 0);
      DEBUG_AW_ACTIVE_THREAD : out std_logic_vector(15 downto 0);
      DEBUG_AW_ACTIVE_TARGET : out std_logic_vector(7 downto 0);
      DEBUG_AW_ACTIVE_REGION : out std_logic_vector(7 downto 0);
      DEBUG_AW_ERROR : out std_logic_vector(7 downto 0);
      DEBUG_AW_TARGET : out std_logic_vector(7 downto 0);
      DEBUG_AR_TRANS_QUAL : out std_logic_vector(0 to 0);
      DEBUG_AR_ACCEPT_CNT : out std_logic_vector(7 downto 0);
      DEBUG_AR_ACTIVE_THREAD : out std_logic_vector(15 downto 0);
      DEBUG_AR_ACTIVE_TARGET : out std_logic_vector(7 downto 0);
      DEBUG_AR_ACTIVE_REGION : out std_logic_vector(7 downto 0);
      DEBUG_AR_ERROR : out std_logic_vector(7 downto 0);
      DEBUG_AR_TARGET : out std_logic_vector(7 downto 0);
      DEBUG_B_TRANS_SEQ : out std_logic_vector(7 downto 0);
      DEBUG_R_BEAT_CNT : out std_logic_vector(7 downto 0);
      DEBUG_R_TRANS_SEQ : out std_logic_vector(7 downto 0);
      DEBUG_AW_ISSUING_CNT : out std_logic_vector(7 downto 0);
      DEBUG_AR_ISSUING_CNT : out std_logic_vector(7 downto 0);
      DEBUG_W_BEAT_CNT : out std_logic_vector(7 downto 0);
      DEBUG_W_TRANS_SEQ : out std_logic_vector(7 downto 0);
      DEBUG_BID_TARGET : out std_logic_vector(7 downto 0);
      DEBUG_BID_ERROR : out std_logic;
      DEBUG_RID_TARGET : out std_logic_vector(7 downto 0);
      DEBUG_RID_ERROR : out std_logic;
      DEBUG_SR_SC_ARADDR : out std_logic_vector(31 downto 0);
      DEBUG_SR_SC_ARADDRCONTROL : out std_logic_vector(23 downto 0);
      DEBUG_SR_SC_AWADDR : out std_logic_vector(31 downto 0);
      DEBUG_SR_SC_AWADDRCONTROL : out std_logic_vector(23 downto 0);
      DEBUG_SR_SC_BRESP : out std_logic_vector(4 downto 0);
      DEBUG_SR_SC_RDATA : out std_logic_vector(31 downto 0);
      DEBUG_SR_SC_RDATACONTROL : out std_logic_vector(5 downto 0);
      DEBUG_SR_SC_WDATA : out std_logic_vector(31 downto 0);
      DEBUG_SR_SC_WDATACONTROL : out std_logic_vector(6 downto 0);
      DEBUG_SC_SF_ARADDR : out std_logic_vector(31 downto 0);
      DEBUG_SC_SF_ARADDRCONTROL : out std_logic_vector(23 downto 0);
      DEBUG_SC_SF_AWADDR : out std_logic_vector(31 downto 0);
      DEBUG_SC_SF_AWADDRCONTROL : out std_logic_vector(23 downto 0);
      DEBUG_SC_SF_BRESP : out std_logic_vector(4 downto 0);
      DEBUG_SC_SF_RDATA : out std_logic_vector(31 downto 0);
      DEBUG_SC_SF_RDATACONTROL : out std_logic_vector(5 downto 0);
      DEBUG_SC_SF_WDATA : out std_logic_vector(31 downto 0);
      DEBUG_SC_SF_WDATACONTROL : out std_logic_vector(6 downto 0);
      DEBUG_SF_CB_ARADDR : out std_logic_vector(31 downto 0);
      DEBUG_SF_CB_ARADDRCONTROL : out std_logic_vector(23 downto 0);
      DEBUG_SF_CB_AWADDR : out std_logic_vector(31 downto 0);
      DEBUG_SF_CB_AWADDRCONTROL : out std_logic_vector(23 downto 0);
      DEBUG_SF_CB_BRESP : out std_logic_vector(4 downto 0);
      DEBUG_SF_CB_RDATA : out std_logic_vector(31 downto 0);
      DEBUG_SF_CB_RDATACONTROL : out std_logic_vector(5 downto 0);
      DEBUG_SF_CB_WDATA : out std_logic_vector(31 downto 0);
      DEBUG_SF_CB_WDATACONTROL : out std_logic_vector(6 downto 0);
      DEBUG_CB_MF_ARADDR : out std_logic_vector(31 downto 0);
      DEBUG_CB_MF_ARADDRCONTROL : out std_logic_vector(23 downto 0);
      DEBUG_CB_MF_AWADDR : out std_logic_vector(31 downto 0);
      DEBUG_CB_MF_AWADDRCONTROL : out std_logic_vector(23 downto 0);
      DEBUG_CB_MF_BRESP : out std_logic_vector(4 downto 0);
      DEBUG_CB_MF_RDATA : out std_logic_vector(31 downto 0);
      DEBUG_CB_MF_RDATACONTROL : out std_logic_vector(5 downto 0);
      DEBUG_CB_MF_WDATA : out std_logic_vector(31 downto 0);
      DEBUG_CB_MF_WDATACONTROL : out std_logic_vector(6 downto 0);
      DEBUG_MF_MC_ARADDR : out std_logic_vector(31 downto 0);
      DEBUG_MF_MC_ARADDRCONTROL : out std_logic_vector(23 downto 0);
      DEBUG_MF_MC_AWADDR : out std_logic_vector(31 downto 0);
      DEBUG_MF_MC_AWADDRCONTROL : out std_logic_vector(23 downto 0);
      DEBUG_MF_MC_BRESP : out std_logic_vector(4 downto 0);
      DEBUG_MF_MC_RDATA : out std_logic_vector(31 downto 0);
      DEBUG_MF_MC_RDATACONTROL : out std_logic_vector(5 downto 0);
      DEBUG_MF_MC_WDATA : out std_logic_vector(31 downto 0);
      DEBUG_MF_MC_WDATACONTROL : out std_logic_vector(6 downto 0);
      DEBUG_MC_MP_ARADDR : out std_logic_vector(31 downto 0);
      DEBUG_MC_MP_ARADDRCONTROL : out std_logic_vector(23 downto 0);
      DEBUG_MC_MP_AWADDR : out std_logic_vector(31 downto 0);
      DEBUG_MC_MP_AWADDRCONTROL : out std_logic_vector(23 downto 0);
      DEBUG_MC_MP_BRESP : out std_logic_vector(4 downto 0);
      DEBUG_MC_MP_RDATA : out std_logic_vector(31 downto 0);
      DEBUG_MC_MP_RDATACONTROL : out std_logic_vector(5 downto 0);
      DEBUG_MC_MP_WDATA : out std_logic_vector(31 downto 0);
      DEBUG_MC_MP_WDATACONTROL : out std_logic_vector(6 downto 0);
      DEBUG_MP_MR_ARADDR : out std_logic_vector(31 downto 0);
      DEBUG_MP_MR_ARADDRCONTROL : out std_logic_vector(23 downto 0);
      DEBUG_MP_MR_AWADDR : out std_logic_vector(31 downto 0);
      DEBUG_MP_MR_AWADDRCONTROL : out std_logic_vector(23 downto 0);
      DEBUG_MP_MR_BRESP : out std_logic_vector(4 downto 0);
      DEBUG_MP_MR_RDATA : out std_logic_vector(31 downto 0);
      DEBUG_MP_MR_RDATACONTROL : out std_logic_vector(5 downto 0);
      DEBUG_MP_MR_WDATA : out std_logic_vector(31 downto 0);
      DEBUG_MP_MR_WDATACONTROL : out std_logic_vector(6 downto 0)
    );
  end component;

  component system_axi4_0_wrapper is
    port (
      INTERCONNECT_ACLK : in std_logic;
      INTERCONNECT_ARESETN : in std_logic;
      S_AXI_ARESET_OUT_N : out std_logic_vector(4 downto 0);
      M_AXI_ARESET_OUT_N : out std_logic_vector(0 to 0);
      IRQ : out std_logic;
      S_AXI_ACLK : in std_logic_vector(4 downto 0);
      S_AXI_AWID : in std_logic_vector(14 downto 0);
      S_AXI_AWADDR : in std_logic_vector(159 downto 0);
      S_AXI_AWLEN : in std_logic_vector(39 downto 0);
      S_AXI_AWSIZE : in std_logic_vector(14 downto 0);
      S_AXI_AWBURST : in std_logic_vector(9 downto 0);
      S_AXI_AWLOCK : in std_logic_vector(9 downto 0);
      S_AXI_AWCACHE : in std_logic_vector(19 downto 0);
      S_AXI_AWPROT : in std_logic_vector(14 downto 0);
      S_AXI_AWQOS : in std_logic_vector(19 downto 0);
      S_AXI_AWUSER : in std_logic_vector(24 downto 0);
      S_AXI_AWVALID : in std_logic_vector(4 downto 0);
      S_AXI_AWREADY : out std_logic_vector(4 downto 0);
      S_AXI_WID : in std_logic_vector(14 downto 0);
      S_AXI_WDATA : in std_logic_vector(159 downto 0);
      S_AXI_WSTRB : in std_logic_vector(19 downto 0);
      S_AXI_WLAST : in std_logic_vector(4 downto 0);
      S_AXI_WUSER : in std_logic_vector(4 downto 0);
      S_AXI_WVALID : in std_logic_vector(4 downto 0);
      S_AXI_WREADY : out std_logic_vector(4 downto 0);
      S_AXI_BID : out std_logic_vector(14 downto 0);
      S_AXI_BRESP : out std_logic_vector(9 downto 0);
      S_AXI_BUSER : out std_logic_vector(4 downto 0);
      S_AXI_BVALID : out std_logic_vector(4 downto 0);
      S_AXI_BREADY : in std_logic_vector(4 downto 0);
      S_AXI_ARID : in std_logic_vector(14 downto 0);
      S_AXI_ARADDR : in std_logic_vector(159 downto 0);
      S_AXI_ARLEN : in std_logic_vector(39 downto 0);
      S_AXI_ARSIZE : in std_logic_vector(14 downto 0);
      S_AXI_ARBURST : in std_logic_vector(9 downto 0);
      S_AXI_ARLOCK : in std_logic_vector(9 downto 0);
      S_AXI_ARCACHE : in std_logic_vector(19 downto 0);
      S_AXI_ARPROT : in std_logic_vector(14 downto 0);
      S_AXI_ARQOS : in std_logic_vector(19 downto 0);
      S_AXI_ARUSER : in std_logic_vector(24 downto 0);
      S_AXI_ARVALID : in std_logic_vector(4 downto 0);
      S_AXI_ARREADY : out std_logic_vector(4 downto 0);
      S_AXI_RID : out std_logic_vector(14 downto 0);
      S_AXI_RDATA : out std_logic_vector(159 downto 0);
      S_AXI_RRESP : out std_logic_vector(9 downto 0);
      S_AXI_RLAST : out std_logic_vector(4 downto 0);
      S_AXI_RUSER : out std_logic_vector(4 downto 0);
      S_AXI_RVALID : out std_logic_vector(4 downto 0);
      S_AXI_RREADY : in std_logic_vector(4 downto 0);
      M_AXI_ACLK : in std_logic_vector(0 to 0);
      M_AXI_AWID : out std_logic_vector(2 downto 0);
      M_AXI_AWADDR : out std_logic_vector(31 downto 0);
      M_AXI_AWLEN : out std_logic_vector(7 downto 0);
      M_AXI_AWSIZE : out std_logic_vector(2 downto 0);
      M_AXI_AWBURST : out std_logic_vector(1 downto 0);
      M_AXI_AWLOCK : out std_logic_vector(1 downto 0);
      M_AXI_AWCACHE : out std_logic_vector(3 downto 0);
      M_AXI_AWPROT : out std_logic_vector(2 downto 0);
      M_AXI_AWREGION : out std_logic_vector(3 downto 0);
      M_AXI_AWQOS : out std_logic_vector(3 downto 0);
      M_AXI_AWUSER : out std_logic_vector(4 downto 0);
      M_AXI_AWVALID : out std_logic_vector(0 to 0);
      M_AXI_AWREADY : in std_logic_vector(0 to 0);
      M_AXI_WID : out std_logic_vector(2 downto 0);
      M_AXI_WDATA : out std_logic_vector(31 downto 0);
      M_AXI_WSTRB : out std_logic_vector(3 downto 0);
      M_AXI_WLAST : out std_logic_vector(0 to 0);
      M_AXI_WUSER : out std_logic_vector(0 to 0);
      M_AXI_WVALID : out std_logic_vector(0 to 0);
      M_AXI_WREADY : in std_logic_vector(0 to 0);
      M_AXI_BID : in std_logic_vector(2 downto 0);
      M_AXI_BRESP : in std_logic_vector(1 downto 0);
      M_AXI_BUSER : in std_logic_vector(0 to 0);
      M_AXI_BVALID : in std_logic_vector(0 to 0);
      M_AXI_BREADY : out std_logic_vector(0 to 0);
      M_AXI_ARID : out std_logic_vector(2 downto 0);
      M_AXI_ARADDR : out std_logic_vector(31 downto 0);
      M_AXI_ARLEN : out std_logic_vector(7 downto 0);
      M_AXI_ARSIZE : out std_logic_vector(2 downto 0);
      M_AXI_ARBURST : out std_logic_vector(1 downto 0);
      M_AXI_ARLOCK : out std_logic_vector(1 downto 0);
      M_AXI_ARCACHE : out std_logic_vector(3 downto 0);
      M_AXI_ARPROT : out std_logic_vector(2 downto 0);
      M_AXI_ARREGION : out std_logic_vector(3 downto 0);
      M_AXI_ARQOS : out std_logic_vector(3 downto 0);
      M_AXI_ARUSER : out std_logic_vector(4 downto 0);
      M_AXI_ARVALID : out std_logic_vector(0 to 0);
      M_AXI_ARREADY : in std_logic_vector(0 to 0);
      M_AXI_RID : in std_logic_vector(2 downto 0);
      M_AXI_RDATA : in std_logic_vector(31 downto 0);
      M_AXI_RRESP : in std_logic_vector(1 downto 0);
      M_AXI_RLAST : in std_logic_vector(0 to 0);
      M_AXI_RUSER : in std_logic_vector(0 to 0);
      M_AXI_RVALID : in std_logic_vector(0 to 0);
      M_AXI_RREADY : out std_logic_vector(0 to 0);
      S_AXI_CTRL_AWADDR : in std_logic_vector(31 downto 0);
      S_AXI_CTRL_AWVALID : in std_logic;
      S_AXI_CTRL_AWREADY : out std_logic;
      S_AXI_CTRL_WDATA : in std_logic_vector(31 downto 0);
      S_AXI_CTRL_WVALID : in std_logic;
      S_AXI_CTRL_WREADY : out std_logic;
      S_AXI_CTRL_BRESP : out std_logic_vector(1 downto 0);
      S_AXI_CTRL_BVALID : out std_logic;
      S_AXI_CTRL_BREADY : in std_logic;
      S_AXI_CTRL_ARADDR : in std_logic_vector(31 downto 0);
      S_AXI_CTRL_ARVALID : in std_logic;
      S_AXI_CTRL_ARREADY : out std_logic;
      S_AXI_CTRL_RDATA : out std_logic_vector(31 downto 0);
      S_AXI_CTRL_RRESP : out std_logic_vector(1 downto 0);
      S_AXI_CTRL_RVALID : out std_logic;
      S_AXI_CTRL_RREADY : in std_logic;
      INTERCONNECT_ARESET_OUT_N : out std_logic;
      DEBUG_AW_TRANS_SEQ : out std_logic_vector(7 downto 0);
      DEBUG_AW_ARB_GRANT : out std_logic_vector(7 downto 0);
      DEBUG_AR_TRANS_SEQ : out std_logic_vector(7 downto 0);
      DEBUG_AR_ARB_GRANT : out std_logic_vector(7 downto 0);
      DEBUG_AW_TRANS_QUAL : out std_logic_vector(0 to 0);
      DEBUG_AW_ACCEPT_CNT : out std_logic_vector(7 downto 0);
      DEBUG_AW_ACTIVE_THREAD : out std_logic_vector(15 downto 0);
      DEBUG_AW_ACTIVE_TARGET : out std_logic_vector(7 downto 0);
      DEBUG_AW_ACTIVE_REGION : out std_logic_vector(7 downto 0);
      DEBUG_AW_ERROR : out std_logic_vector(7 downto 0);
      DEBUG_AW_TARGET : out std_logic_vector(7 downto 0);
      DEBUG_AR_TRANS_QUAL : out std_logic_vector(0 to 0);
      DEBUG_AR_ACCEPT_CNT : out std_logic_vector(7 downto 0);
      DEBUG_AR_ACTIVE_THREAD : out std_logic_vector(15 downto 0);
      DEBUG_AR_ACTIVE_TARGET : out std_logic_vector(7 downto 0);
      DEBUG_AR_ACTIVE_REGION : out std_logic_vector(7 downto 0);
      DEBUG_AR_ERROR : out std_logic_vector(7 downto 0);
      DEBUG_AR_TARGET : out std_logic_vector(7 downto 0);
      DEBUG_B_TRANS_SEQ : out std_logic_vector(7 downto 0);
      DEBUG_R_BEAT_CNT : out std_logic_vector(7 downto 0);
      DEBUG_R_TRANS_SEQ : out std_logic_vector(7 downto 0);
      DEBUG_AW_ISSUING_CNT : out std_logic_vector(7 downto 0);
      DEBUG_AR_ISSUING_CNT : out std_logic_vector(7 downto 0);
      DEBUG_W_BEAT_CNT : out std_logic_vector(7 downto 0);
      DEBUG_W_TRANS_SEQ : out std_logic_vector(7 downto 0);
      DEBUG_BID_TARGET : out std_logic_vector(7 downto 0);
      DEBUG_BID_ERROR : out std_logic;
      DEBUG_RID_TARGET : out std_logic_vector(7 downto 0);
      DEBUG_RID_ERROR : out std_logic;
      DEBUG_SR_SC_ARADDR : out std_logic_vector(31 downto 0);
      DEBUG_SR_SC_ARADDRCONTROL : out std_logic_vector(25 downto 0);
      DEBUG_SR_SC_AWADDR : out std_logic_vector(31 downto 0);
      DEBUG_SR_SC_AWADDRCONTROL : out std_logic_vector(25 downto 0);
      DEBUG_SR_SC_BRESP : out std_logic_vector(6 downto 0);
      DEBUG_SR_SC_RDATA : out std_logic_vector(31 downto 0);
      DEBUG_SR_SC_RDATACONTROL : out std_logic_vector(7 downto 0);
      DEBUG_SR_SC_WDATA : out std_logic_vector(31 downto 0);
      DEBUG_SR_SC_WDATACONTROL : out std_logic_vector(6 downto 0);
      DEBUG_SC_SF_ARADDR : out std_logic_vector(31 downto 0);
      DEBUG_SC_SF_ARADDRCONTROL : out std_logic_vector(25 downto 0);
      DEBUG_SC_SF_AWADDR : out std_logic_vector(31 downto 0);
      DEBUG_SC_SF_AWADDRCONTROL : out std_logic_vector(25 downto 0);
      DEBUG_SC_SF_BRESP : out std_logic_vector(6 downto 0);
      DEBUG_SC_SF_RDATA : out std_logic_vector(31 downto 0);
      DEBUG_SC_SF_RDATACONTROL : out std_logic_vector(7 downto 0);
      DEBUG_SC_SF_WDATA : out std_logic_vector(31 downto 0);
      DEBUG_SC_SF_WDATACONTROL : out std_logic_vector(6 downto 0);
      DEBUG_SF_CB_ARADDR : out std_logic_vector(31 downto 0);
      DEBUG_SF_CB_ARADDRCONTROL : out std_logic_vector(25 downto 0);
      DEBUG_SF_CB_AWADDR : out std_logic_vector(31 downto 0);
      DEBUG_SF_CB_AWADDRCONTROL : out std_logic_vector(25 downto 0);
      DEBUG_SF_CB_BRESP : out std_logic_vector(6 downto 0);
      DEBUG_SF_CB_RDATA : out std_logic_vector(31 downto 0);
      DEBUG_SF_CB_RDATACONTROL : out std_logic_vector(7 downto 0);
      DEBUG_SF_CB_WDATA : out std_logic_vector(31 downto 0);
      DEBUG_SF_CB_WDATACONTROL : out std_logic_vector(6 downto 0);
      DEBUG_CB_MF_ARADDR : out std_logic_vector(31 downto 0);
      DEBUG_CB_MF_ARADDRCONTROL : out std_logic_vector(25 downto 0);
      DEBUG_CB_MF_AWADDR : out std_logic_vector(31 downto 0);
      DEBUG_CB_MF_AWADDRCONTROL : out std_logic_vector(25 downto 0);
      DEBUG_CB_MF_BRESP : out std_logic_vector(6 downto 0);
      DEBUG_CB_MF_RDATA : out std_logic_vector(31 downto 0);
      DEBUG_CB_MF_RDATACONTROL : out std_logic_vector(7 downto 0);
      DEBUG_CB_MF_WDATA : out std_logic_vector(31 downto 0);
      DEBUG_CB_MF_WDATACONTROL : out std_logic_vector(6 downto 0);
      DEBUG_MF_MC_ARADDR : out std_logic_vector(31 downto 0);
      DEBUG_MF_MC_ARADDRCONTROL : out std_logic_vector(25 downto 0);
      DEBUG_MF_MC_AWADDR : out std_logic_vector(31 downto 0);
      DEBUG_MF_MC_AWADDRCONTROL : out std_logic_vector(25 downto 0);
      DEBUG_MF_MC_BRESP : out std_logic_vector(6 downto 0);
      DEBUG_MF_MC_RDATA : out std_logic_vector(31 downto 0);
      DEBUG_MF_MC_RDATACONTROL : out std_logic_vector(7 downto 0);
      DEBUG_MF_MC_WDATA : out std_logic_vector(31 downto 0);
      DEBUG_MF_MC_WDATACONTROL : out std_logic_vector(6 downto 0);
      DEBUG_MC_MP_ARADDR : out std_logic_vector(31 downto 0);
      DEBUG_MC_MP_ARADDRCONTROL : out std_logic_vector(25 downto 0);
      DEBUG_MC_MP_AWADDR : out std_logic_vector(31 downto 0);
      DEBUG_MC_MP_AWADDRCONTROL : out std_logic_vector(25 downto 0);
      DEBUG_MC_MP_BRESP : out std_logic_vector(6 downto 0);
      DEBUG_MC_MP_RDATA : out std_logic_vector(31 downto 0);
      DEBUG_MC_MP_RDATACONTROL : out std_logic_vector(7 downto 0);
      DEBUG_MC_MP_WDATA : out std_logic_vector(31 downto 0);
      DEBUG_MC_MP_WDATACONTROL : out std_logic_vector(6 downto 0);
      DEBUG_MP_MR_ARADDR : out std_logic_vector(31 downto 0);
      DEBUG_MP_MR_ARADDRCONTROL : out std_logic_vector(25 downto 0);
      DEBUG_MP_MR_AWADDR : out std_logic_vector(31 downto 0);
      DEBUG_MP_MR_AWADDRCONTROL : out std_logic_vector(25 downto 0);
      DEBUG_MP_MR_BRESP : out std_logic_vector(6 downto 0);
      DEBUG_MP_MR_RDATA : out std_logic_vector(31 downto 0);
      DEBUG_MP_MR_RDATACONTROL : out std_logic_vector(7 downto 0);
      DEBUG_MP_MR_WDATA : out std_logic_vector(31 downto 0);
      DEBUG_MP_MR_WDATACONTROL : out std_logic_vector(6 downto 0)
    );
  end component;

  component system_sysace_compactflash_wrapper is
    port (
      S_AXI_ACLK : in std_logic;
      S_AXI_ARESETN : in std_logic;
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
      SysACE_CLK : in std_logic;
      SysACE_MPIRQ : in std_logic;
      SysACE_MPD_I : in std_logic_vector(7 downto 0);
      SysACE_MPD_O : out std_logic_vector(7 downto 0);
      SysACE_MPD_T : out std_logic_vector(7 downto 0);
      SysACE_MPA : out std_logic_vector(6 downto 0);
      SysACE_CEN : out std_logic;
      SysACE_OEN : out std_logic;
      SysACE_WEN : out std_logic;
      SysACE_IRQ : out std_logic
    );
  end component;

  component system_rs232_uart_1_wrapper is
    port (
      S_AXI_ACLK : in std_logic;
      S_AXI_ARESETN : in std_logic;
      Interrupt : out std_logic;
      S_AXI_AWADDR : in std_logic_vector(3 downto 0);
      S_AXI_AWVALID : in std_logic;
      S_AXI_AWREADY : out std_logic;
      S_AXI_WDATA : in std_logic_vector(31 downto 0);
      S_AXI_WSTRB : in std_logic_vector(3 downto 0);
      S_AXI_WVALID : in std_logic;
      S_AXI_WREADY : out std_logic;
      S_AXI_BRESP : out std_logic_vector(1 downto 0);
      S_AXI_BVALID : out std_logic;
      S_AXI_BREADY : in std_logic;
      S_AXI_ARADDR : in std_logic_vector(3 downto 0);
      S_AXI_ARVALID : in std_logic;
      S_AXI_ARREADY : out std_logic;
      S_AXI_RDATA : out std_logic_vector(31 downto 0);
      S_AXI_RRESP : out std_logic_vector(1 downto 0);
      S_AXI_RVALID : out std_logic;
      S_AXI_RREADY : in std_logic;
      RX : in std_logic;
      TX : out std_logic
    );
  end component;

  component system_ethernet_dma_wrapper is
    port (
      s_axi_lite_aclk : in std_logic;
      m_axi_sg_aclk : in std_logic;
      m_axi_mm2s_aclk : in std_logic;
      m_axi_s2mm_aclk : in std_logic;
      axi_resetn : in std_logic;
      s_axi_lite_awvalid : in std_logic;
      s_axi_lite_awready : out std_logic;
      s_axi_lite_awaddr : in std_logic_vector(9 downto 0);
      s_axi_lite_wvalid : in std_logic;
      s_axi_lite_wready : out std_logic;
      s_axi_lite_wdata : in std_logic_vector(31 downto 0);
      s_axi_lite_bresp : out std_logic_vector(1 downto 0);
      s_axi_lite_bvalid : out std_logic;
      s_axi_lite_bready : in std_logic;
      s_axi_lite_arvalid : in std_logic;
      s_axi_lite_arready : out std_logic;
      s_axi_lite_araddr : in std_logic_vector(9 downto 0);
      s_axi_lite_rvalid : out std_logic;
      s_axi_lite_rready : in std_logic;
      s_axi_lite_rdata : out std_logic_vector(31 downto 0);
      s_axi_lite_rresp : out std_logic_vector(1 downto 0);
      m_axi_sg_awaddr : out std_logic_vector(31 downto 0);
      m_axi_sg_awlen : out std_logic_vector(7 downto 0);
      m_axi_sg_awsize : out std_logic_vector(2 downto 0);
      m_axi_sg_awburst : out std_logic_vector(1 downto 0);
      m_axi_sg_awprot : out std_logic_vector(2 downto 0);
      m_axi_sg_awcache : out std_logic_vector(3 downto 0);
      m_axi_sg_awuser : out std_logic_vector(3 downto 0);
      m_axi_sg_awvalid : out std_logic;
      m_axi_sg_awready : in std_logic;
      m_axi_sg_wdata : out std_logic_vector(31 downto 0);
      m_axi_sg_wstrb : out std_logic_vector(3 downto 0);
      m_axi_sg_wlast : out std_logic;
      m_axi_sg_wvalid : out std_logic;
      m_axi_sg_wready : in std_logic;
      m_axi_sg_bresp : in std_logic_vector(1 downto 0);
      m_axi_sg_bvalid : in std_logic;
      m_axi_sg_bready : out std_logic;
      m_axi_sg_araddr : out std_logic_vector(31 downto 0);
      m_axi_sg_arlen : out std_logic_vector(7 downto 0);
      m_axi_sg_arsize : out std_logic_vector(2 downto 0);
      m_axi_sg_arburst : out std_logic_vector(1 downto 0);
      m_axi_sg_arprot : out std_logic_vector(2 downto 0);
      m_axi_sg_arcache : out std_logic_vector(3 downto 0);
      m_axi_sg_aruser : out std_logic_vector(3 downto 0);
      m_axi_sg_arvalid : out std_logic;
      m_axi_sg_arready : in std_logic;
      m_axi_sg_rdata : in std_logic_vector(31 downto 0);
      m_axi_sg_rresp : in std_logic_vector(1 downto 0);
      m_axi_sg_rlast : in std_logic;
      m_axi_sg_rvalid : in std_logic;
      m_axi_sg_rready : out std_logic;
      m_axi_mm2s_araddr : out std_logic_vector(31 downto 0);
      m_axi_mm2s_arlen : out std_logic_vector(7 downto 0);
      m_axi_mm2s_arsize : out std_logic_vector(2 downto 0);
      m_axi_mm2s_arburst : out std_logic_vector(1 downto 0);
      m_axi_mm2s_arprot : out std_logic_vector(2 downto 0);
      m_axi_mm2s_arcache : out std_logic_vector(3 downto 0);
      m_axi_mm2s_aruser : out std_logic_vector(3 downto 0);
      m_axi_mm2s_arvalid : out std_logic;
      m_axi_mm2s_arready : in std_logic;
      m_axi_mm2s_rdata : in std_logic_vector(31 downto 0);
      m_axi_mm2s_rresp : in std_logic_vector(1 downto 0);
      m_axi_mm2s_rlast : in std_logic;
      m_axi_mm2s_rvalid : in std_logic;
      m_axi_mm2s_rready : out std_logic;
      mm2s_prmry_reset_out_n : out std_logic;
      m_axis_mm2s_tdata : out std_logic_vector(31 downto 0);
      m_axis_mm2s_tkeep : out std_logic_vector(3 downto 0);
      m_axis_mm2s_tvalid : out std_logic;
      m_axis_mm2s_tready : in std_logic;
      m_axis_mm2s_tlast : out std_logic;
      m_axis_mm2s_tuser : out std_logic_vector(3 downto 0);
      m_axis_mm2s_tid : out std_logic_vector(4 downto 0);
      m_axis_mm2s_tdest : out std_logic_vector(4 downto 0);
      mm2s_cntrl_reset_out_n : out std_logic;
      m_axis_mm2s_cntrl_tdata : out std_logic_vector(31 downto 0);
      m_axis_mm2s_cntrl_tkeep : out std_logic_vector(3 downto 0);
      m_axis_mm2s_cntrl_tvalid : out std_logic;
      m_axis_mm2s_cntrl_tready : in std_logic;
      m_axis_mm2s_cntrl_tlast : out std_logic;
      m_axi_s2mm_awaddr : out std_logic_vector(31 downto 0);
      m_axi_s2mm_awlen : out std_logic_vector(7 downto 0);
      m_axi_s2mm_awsize : out std_logic_vector(2 downto 0);
      m_axi_s2mm_awburst : out std_logic_vector(1 downto 0);
      m_axi_s2mm_awprot : out std_logic_vector(2 downto 0);
      m_axi_s2mm_awcache : out std_logic_vector(3 downto 0);
      m_axi_s2mm_awuser : out std_logic_vector(3 downto 0);
      m_axi_s2mm_awvalid : out std_logic;
      m_axi_s2mm_awready : in std_logic;
      m_axi_s2mm_wdata : out std_logic_vector(31 downto 0);
      m_axi_s2mm_wstrb : out std_logic_vector(3 downto 0);
      m_axi_s2mm_wlast : out std_logic;
      m_axi_s2mm_wvalid : out std_logic;
      m_axi_s2mm_wready : in std_logic;
      m_axi_s2mm_bresp : in std_logic_vector(1 downto 0);
      m_axi_s2mm_bvalid : in std_logic;
      m_axi_s2mm_bready : out std_logic;
      s2mm_prmry_reset_out_n : out std_logic;
      s_axis_s2mm_tdata : in std_logic_vector(31 downto 0);
      s_axis_s2mm_tkeep : in std_logic_vector(3 downto 0);
      s_axis_s2mm_tvalid : in std_logic;
      s_axis_s2mm_tready : out std_logic;
      s_axis_s2mm_tlast : in std_logic;
      s_axis_s2mm_tuser : in std_logic_vector(3 downto 0);
      s_axis_s2mm_tid : in std_logic_vector(4 downto 0);
      s_axis_s2mm_tdest : in std_logic_vector(4 downto 0);
      s2mm_sts_reset_out_n : out std_logic;
      s_axis_s2mm_sts_tdata : in std_logic_vector(31 downto 0);
      s_axis_s2mm_sts_tkeep : in std_logic_vector(3 downto 0);
      s_axis_s2mm_sts_tvalid : in std_logic;
      s_axis_s2mm_sts_tready : out std_logic;
      s_axis_s2mm_sts_tlast : in std_logic;
      mm2s_introut : out std_logic;
      s2mm_introut : out std_logic
    );
  end component;

  component system_ethernet_wrapper is
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
  end component;

  component system_ddr3_sdram_wrapper is
    port (
      clk : in std_logic;
      clk_mem : in std_logic;
      clk_rd_base : in std_logic;
      aresetn : in std_logic;
      clk_ref : in std_logic;
      iodelay_ctrl_rdy_i : in std_logic;
      iodelay_ctrl_rdy_o : out std_logic;
      pd_PSEN : out std_logic;
      pd_PSINCDEC : out std_logic;
      pd_PSDONE : in std_logic;
      phy_init_done : out std_logic;
      interrupt : out std_logic;
      ddr_addr : out std_logic_vector(12 downto 0);
      ddr_ba : out std_logic_vector(2 downto 0);
      ddr_cas_n : out std_logic;
      ddr_ck_p : out std_logic_vector(0 to 0);
      ddr_ck_n : out std_logic_vector(0 to 0);
      ddr_cke : out std_logic_vector(0 to 0);
      ddr_cs_n : out std_logic_vector(0 to 0);
      ddr_dm : out std_logic_vector(0 to 0);
      ddr_odt : out std_logic_vector(0 to 0);
      ddr_ras_n : out std_logic;
      ddr_reset_n : out std_logic;
      ddr_parity : out std_logic;
      ddr_we_n : out std_logic;
      ddr_dq : inout std_logic_vector(7 downto 0);
      ddr_dqs_p : inout std_logic_vector(0 to 0);
      ddr_dqs_n : inout std_logic_vector(0 to 0);
      s_axi_awid : in std_logic_vector(2 downto 0);
      s_axi_awaddr : in std_logic_vector(31 downto 0);
      s_axi_awlen : in std_logic_vector(7 downto 0);
      s_axi_awsize : in std_logic_vector(2 downto 0);
      s_axi_awburst : in std_logic_vector(1 downto 0);
      s_axi_awlock : in std_logic_vector(0 to 0);
      s_axi_awcache : in std_logic_vector(3 downto 0);
      s_axi_awprot : in std_logic_vector(2 downto 0);
      s_axi_awqos : in std_logic_vector(3 downto 0);
      s_axi_awvalid : in std_logic;
      s_axi_awready : out std_logic;
      s_axi_wdata : in std_logic_vector(31 downto 0);
      s_axi_wstrb : in std_logic_vector(3 downto 0);
      s_axi_wlast : in std_logic;
      s_axi_wvalid : in std_logic;
      s_axi_wready : out std_logic;
      s_axi_bid : out std_logic_vector(2 downto 0);
      s_axi_bresp : out std_logic_vector(1 downto 0);
      s_axi_bvalid : out std_logic;
      s_axi_bready : in std_logic;
      s_axi_arid : in std_logic_vector(2 downto 0);
      s_axi_araddr : in std_logic_vector(31 downto 0);
      s_axi_arlen : in std_logic_vector(7 downto 0);
      s_axi_arsize : in std_logic_vector(2 downto 0);
      s_axi_arburst : in std_logic_vector(1 downto 0);
      s_axi_arlock : in std_logic_vector(0 to 0);
      s_axi_arcache : in std_logic_vector(3 downto 0);
      s_axi_arprot : in std_logic_vector(2 downto 0);
      s_axi_arqos : in std_logic_vector(3 downto 0);
      s_axi_arvalid : in std_logic;
      s_axi_arready : out std_logic;
      s_axi_rid : out std_logic_vector(2 downto 0);
      s_axi_rdata : out std_logic_vector(31 downto 0);
      s_axi_rresp : out std_logic_vector(1 downto 0);
      s_axi_rlast : out std_logic;
      s_axi_rvalid : out std_logic;
      s_axi_rready : in std_logic;
      s_axi_ctrl_awaddr : in std_logic_vector(31 downto 0);
      s_axi_ctrl_awvalid : in std_logic;
      s_axi_ctrl_awready : out std_logic;
      s_axi_ctrl_wdata : in std_logic_vector(31 downto 0);
      s_axi_ctrl_wvalid : in std_logic;
      s_axi_ctrl_wready : out std_logic;
      s_axi_ctrl_bresp : out std_logic_vector(1 downto 0);
      s_axi_ctrl_bvalid : out std_logic;
      s_axi_ctrl_bready : in std_logic;
      s_axi_ctrl_araddr : in std_logic_vector(31 downto 0);
      s_axi_ctrl_arvalid : in std_logic;
      s_axi_ctrl_arready : out std_logic;
      s_axi_ctrl_rdata : out std_logic_vector(31 downto 0);
      s_axi_ctrl_rresp : out std_logic_vector(1 downto 0);
      s_axi_ctrl_rvalid : out std_logic;
      s_axi_ctrl_rready : in std_logic
    );
  end component;

  component system_controller_0_wrapper is
    port (
      S_AXI_ACLK : in std_logic;
      S_AXI_ARESETN : in std_logic;
      S_AXI_AWADDR : in std_logic_vector(31 downto 0);
      S_AXI_AWVALID : in std_logic;
      S_AXI_WDATA : in std_logic_vector(31 downto 0);
      S_AXI_WSTRB : in std_logic_vector(3 downto 0);
      S_AXI_WVALID : in std_logic;
      S_AXI_BREADY : in std_logic;
      S_AXI_ARADDR : in std_logic_vector(31 downto 0);
      S_AXI_ARVALID : in std_logic;
      S_AXI_RREADY : in std_logic;
      S_AXI_ARREADY : out std_logic;
      S_AXI_RDATA : out std_logic_vector(31 downto 0);
      S_AXI_RRESP : out std_logic_vector(1 downto 0);
      S_AXI_RVALID : out std_logic;
      S_AXI_WREADY : out std_logic;
      S_AXI_BRESP : out std_logic_vector(1 downto 0);
      S_AXI_BVALID : out std_logic;
      S_AXI_AWREADY : out std_logic
    );
  end component;

  component IOBUF is
    port (
      I : in std_logic;
      IO : inout std_logic;
      O : out std_logic;
      T : in std_logic
    );
  end component;

  component IBUFGDS is
    port (
      I : in std_logic;
      IB : in std_logic;
      O : out std_logic
    );
  end component;

  -- Internal signals

  signal AXI_STR_RXD_ARESETN : std_logic;
  signal AXI_STR_RXS_ARESETN : std_logic;
  signal AXI_STR_TXC_ARESETN : std_logic;
  signal AXI_STR_TXD_ARESETN : std_logic;
  signal CLK : std_logic;
  signal ETHERNET_INTERRUPT : std_logic;
  signal ETHERNET_MDIO_I : std_logic;
  signal ETHERNET_MDIO_O : std_logic;
  signal ETHERNET_MDIO_T : std_logic;
  signal ETHERNET_dma_mm2s_introut : std_logic;
  signal ETHERNET_dma_rxd_TDATA : std_logic_vector(31 downto 0);
  signal ETHERNET_dma_rxd_TKEEP : std_logic_vector(3 downto 0);
  signal ETHERNET_dma_rxd_TLAST : std_logic;
  signal ETHERNET_dma_rxd_TREADY : std_logic;
  signal ETHERNET_dma_rxd_TVALID : std_logic;
  signal ETHERNET_dma_rxs_TDATA : std_logic_vector(31 downto 0);
  signal ETHERNET_dma_rxs_TKEEP : std_logic_vector(3 downto 0);
  signal ETHERNET_dma_rxs_TLAST : std_logic;
  signal ETHERNET_dma_rxs_TREADY : std_logic;
  signal ETHERNET_dma_rxs_TVALID : std_logic;
  signal ETHERNET_dma_s2mm_introut : std_logic;
  signal ETHERNET_dma_txc_TDATA : std_logic_vector(31 downto 0);
  signal ETHERNET_dma_txc_TKEEP : std_logic_vector(3 downto 0);
  signal ETHERNET_dma_txc_TLAST : std_logic;
  signal ETHERNET_dma_txc_TREADY : std_logic;
  signal ETHERNET_dma_txc_TVALID : std_logic;
  signal ETHERNET_dma_txd_TDATA : std_logic_vector(31 downto 0);
  signal ETHERNET_dma_txd_TKEEP : std_logic_vector(3 downto 0);
  signal ETHERNET_dma_txd_TLAST : std_logic;
  signal ETHERNET_dma_txd_TREADY : std_logic;
  signal ETHERNET_dma_txd_TVALID : std_logic;
  signal Ext_BRK : std_logic;
  signal Ext_NM_BRK : std_logic;
  signal SysACE_MPD_I : std_logic_vector(7 downto 0);
  signal SysACE_MPD_O : std_logic_vector(7 downto 0);
  signal SysACE_MPD_T : std_logic_vector(7 downto 0);
  signal axi4_0_M_ARADDR : std_logic_vector(31 downto 0);
  signal axi4_0_M_ARBURST : std_logic_vector(1 downto 0);
  signal axi4_0_M_ARCACHE : std_logic_vector(3 downto 0);
  signal axi4_0_M_ARESETN : std_logic_vector(0 to 0);
  signal axi4_0_M_ARID : std_logic_vector(2 downto 0);
  signal axi4_0_M_ARLEN : std_logic_vector(7 downto 0);
  signal axi4_0_M_ARLOCK : std_logic_vector(1 downto 0);
  signal axi4_0_M_ARPROT : std_logic_vector(2 downto 0);
  signal axi4_0_M_ARQOS : std_logic_vector(3 downto 0);
  signal axi4_0_M_ARREADY : std_logic_vector(0 to 0);
  signal axi4_0_M_ARSIZE : std_logic_vector(2 downto 0);
  signal axi4_0_M_ARVALID : std_logic_vector(0 to 0);
  signal axi4_0_M_AWADDR : std_logic_vector(31 downto 0);
  signal axi4_0_M_AWBURST : std_logic_vector(1 downto 0);
  signal axi4_0_M_AWCACHE : std_logic_vector(3 downto 0);
  signal axi4_0_M_AWID : std_logic_vector(2 downto 0);
  signal axi4_0_M_AWLEN : std_logic_vector(7 downto 0);
  signal axi4_0_M_AWLOCK : std_logic_vector(1 downto 0);
  signal axi4_0_M_AWPROT : std_logic_vector(2 downto 0);
  signal axi4_0_M_AWQOS : std_logic_vector(3 downto 0);
  signal axi4_0_M_AWREADY : std_logic_vector(0 to 0);
  signal axi4_0_M_AWSIZE : std_logic_vector(2 downto 0);
  signal axi4_0_M_AWVALID : std_logic_vector(0 to 0);
  signal axi4_0_M_BID : std_logic_vector(2 downto 0);
  signal axi4_0_M_BREADY : std_logic_vector(0 to 0);
  signal axi4_0_M_BRESP : std_logic_vector(1 downto 0);
  signal axi4_0_M_BVALID : std_logic_vector(0 to 0);
  signal axi4_0_M_RDATA : std_logic_vector(31 downto 0);
  signal axi4_0_M_RID : std_logic_vector(2 downto 0);
  signal axi4_0_M_RLAST : std_logic_vector(0 to 0);
  signal axi4_0_M_RREADY : std_logic_vector(0 to 0);
  signal axi4_0_M_RRESP : std_logic_vector(1 downto 0);
  signal axi4_0_M_RVALID : std_logic_vector(0 to 0);
  signal axi4_0_M_WDATA : std_logic_vector(31 downto 0);
  signal axi4_0_M_WLAST : std_logic_vector(0 to 0);
  signal axi4_0_M_WREADY : std_logic_vector(0 to 0);
  signal axi4_0_M_WSTRB : std_logic_vector(3 downto 0);
  signal axi4_0_M_WVALID : std_logic_vector(0 to 0);
  signal axi4_0_S_ARADDR : std_logic_vector(159 downto 0);
  signal axi4_0_S_ARBURST : std_logic_vector(9 downto 0);
  signal axi4_0_S_ARCACHE : std_logic_vector(19 downto 0);
  signal axi4_0_S_ARID : std_logic_vector(14 downto 0);
  signal axi4_0_S_ARLEN : std_logic_vector(39 downto 0);
  signal axi4_0_S_ARLOCK : std_logic_vector(9 downto 0);
  signal axi4_0_S_ARPROT : std_logic_vector(14 downto 0);
  signal axi4_0_S_ARQOS : std_logic_vector(19 downto 0);
  signal axi4_0_S_ARREADY : std_logic_vector(4 downto 0);
  signal axi4_0_S_ARSIZE : std_logic_vector(14 downto 0);
  signal axi4_0_S_ARUSER : std_logic_vector(24 downto 0);
  signal axi4_0_S_ARVALID : std_logic_vector(4 downto 0);
  signal axi4_0_S_AWADDR : std_logic_vector(159 downto 0);
  signal axi4_0_S_AWBURST : std_logic_vector(9 downto 0);
  signal axi4_0_S_AWCACHE : std_logic_vector(19 downto 0);
  signal axi4_0_S_AWID : std_logic_vector(14 downto 0);
  signal axi4_0_S_AWLEN : std_logic_vector(39 downto 0);
  signal axi4_0_S_AWLOCK : std_logic_vector(9 downto 0);
  signal axi4_0_S_AWPROT : std_logic_vector(14 downto 0);
  signal axi4_0_S_AWQOS : std_logic_vector(19 downto 0);
  signal axi4_0_S_AWREADY : std_logic_vector(4 downto 0);
  signal axi4_0_S_AWSIZE : std_logic_vector(14 downto 0);
  signal axi4_0_S_AWUSER : std_logic_vector(24 downto 0);
  signal axi4_0_S_AWVALID : std_logic_vector(4 downto 0);
  signal axi4_0_S_BID : std_logic_vector(14 downto 0);
  signal axi4_0_S_BREADY : std_logic_vector(4 downto 0);
  signal axi4_0_S_BRESP : std_logic_vector(9 downto 0);
  signal axi4_0_S_BUSER : std_logic_vector(4 downto 0);
  signal axi4_0_S_BVALID : std_logic_vector(4 downto 0);
  signal axi4_0_S_RDATA : std_logic_vector(159 downto 0);
  signal axi4_0_S_RID : std_logic_vector(14 downto 0);
  signal axi4_0_S_RLAST : std_logic_vector(4 downto 0);
  signal axi4_0_S_RREADY : std_logic_vector(4 downto 0);
  signal axi4_0_S_RRESP : std_logic_vector(9 downto 0);
  signal axi4_0_S_RUSER : std_logic_vector(4 downto 0);
  signal axi4_0_S_RVALID : std_logic_vector(4 downto 0);
  signal axi4_0_S_WDATA : std_logic_vector(159 downto 0);
  signal axi4_0_S_WLAST : std_logic_vector(4 downto 0);
  signal axi4_0_S_WREADY : std_logic_vector(4 downto 0);
  signal axi4_0_S_WSTRB : std_logic_vector(19 downto 0);
  signal axi4_0_S_WUSER : std_logic_vector(4 downto 0);
  signal axi4_0_S_WVALID : std_logic_vector(4 downto 0);
  signal axi4lite_0_M_ARADDR : std_logic_vector(255 downto 0);
  signal axi4lite_0_M_ARESETN : std_logic_vector(7 downto 0);
  signal axi4lite_0_M_ARREADY : std_logic_vector(7 downto 0);
  signal axi4lite_0_M_ARVALID : std_logic_vector(7 downto 0);
  signal axi4lite_0_M_AWADDR : std_logic_vector(255 downto 0);
  signal axi4lite_0_M_AWREADY : std_logic_vector(7 downto 0);
  signal axi4lite_0_M_AWVALID : std_logic_vector(7 downto 0);
  signal axi4lite_0_M_BREADY : std_logic_vector(7 downto 0);
  signal axi4lite_0_M_BRESP : std_logic_vector(15 downto 0);
  signal axi4lite_0_M_BVALID : std_logic_vector(7 downto 0);
  signal axi4lite_0_M_RDATA : std_logic_vector(255 downto 0);
  signal axi4lite_0_M_RREADY : std_logic_vector(7 downto 0);
  signal axi4lite_0_M_RRESP : std_logic_vector(15 downto 0);
  signal axi4lite_0_M_RVALID : std_logic_vector(7 downto 0);
  signal axi4lite_0_M_WDATA : std_logic_vector(255 downto 0);
  signal axi4lite_0_M_WREADY : std_logic_vector(7 downto 0);
  signal axi4lite_0_M_WSTRB : std_logic_vector(31 downto 0);
  signal axi4lite_0_M_WVALID : std_logic_vector(7 downto 0);
  signal axi4lite_0_S_ARADDR : std_logic_vector(31 downto 0);
  signal axi4lite_0_S_ARBURST : std_logic_vector(1 downto 0);
  signal axi4lite_0_S_ARCACHE : std_logic_vector(3 downto 0);
  signal axi4lite_0_S_ARID : std_logic_vector(0 to 0);
  signal axi4lite_0_S_ARLEN : std_logic_vector(7 downto 0);
  signal axi4lite_0_S_ARLOCK : std_logic_vector(1 downto 0);
  signal axi4lite_0_S_ARPROT : std_logic_vector(2 downto 0);
  signal axi4lite_0_S_ARQOS : std_logic_vector(3 downto 0);
  signal axi4lite_0_S_ARREADY : std_logic_vector(0 to 0);
  signal axi4lite_0_S_ARSIZE : std_logic_vector(2 downto 0);
  signal axi4lite_0_S_ARVALID : std_logic_vector(0 to 0);
  signal axi4lite_0_S_AWADDR : std_logic_vector(31 downto 0);
  signal axi4lite_0_S_AWBURST : std_logic_vector(1 downto 0);
  signal axi4lite_0_S_AWCACHE : std_logic_vector(3 downto 0);
  signal axi4lite_0_S_AWID : std_logic_vector(0 to 0);
  signal axi4lite_0_S_AWLEN : std_logic_vector(7 downto 0);
  signal axi4lite_0_S_AWLOCK : std_logic_vector(1 downto 0);
  signal axi4lite_0_S_AWPROT : std_logic_vector(2 downto 0);
  signal axi4lite_0_S_AWQOS : std_logic_vector(3 downto 0);
  signal axi4lite_0_S_AWREADY : std_logic_vector(0 to 0);
  signal axi4lite_0_S_AWSIZE : std_logic_vector(2 downto 0);
  signal axi4lite_0_S_AWVALID : std_logic_vector(0 to 0);
  signal axi4lite_0_S_BID : std_logic_vector(0 downto 0);
  signal axi4lite_0_S_BREADY : std_logic_vector(0 to 0);
  signal axi4lite_0_S_BRESP : std_logic_vector(1 downto 0);
  signal axi4lite_0_S_BVALID : std_logic_vector(0 to 0);
  signal axi4lite_0_S_RDATA : std_logic_vector(31 downto 0);
  signal axi4lite_0_S_RID : std_logic_vector(0 downto 0);
  signal axi4lite_0_S_RLAST : std_logic_vector(0 to 0);
  signal axi4lite_0_S_RREADY : std_logic_vector(0 to 0);
  signal axi4lite_0_S_RRESP : std_logic_vector(1 downto 0);
  signal axi4lite_0_S_RVALID : std_logic_vector(0 to 0);
  signal axi4lite_0_S_WDATA : std_logic_vector(31 downto 0);
  signal axi4lite_0_S_WLAST : std_logic_vector(0 to 0);
  signal axi4lite_0_S_WREADY : std_logic_vector(0 to 0);
  signal axi4lite_0_S_WSTRB : std_logic_vector(3 downto 0);
  signal axi4lite_0_S_WVALID : std_logic_vector(0 to 0);
  signal axi_timer_0_Interrupt : std_logic;
  signal clk_100_0000MHzMMCM0 : std_logic_vector(0 to 0);
  signal clk_125_0000MHz : std_logic;
  signal clk_200_0000MHzMMCM0 : std_logic_vector(0 to 0);
  signal clk_400_0000MHzMMCM0 : std_logic;
  signal clk_400_0000MHzMMCM0_nobuf_varphase : std_logic;
  signal microblaze_0_d_bram_ctrl_2_microblaze_0_bram_block_BRAM_Addr : std_logic_vector(0 to 31);
  signal microblaze_0_d_bram_ctrl_2_microblaze_0_bram_block_BRAM_Clk : std_logic;
  signal microblaze_0_d_bram_ctrl_2_microblaze_0_bram_block_BRAM_Din : std_logic_vector(0 to 31);
  signal microblaze_0_d_bram_ctrl_2_microblaze_0_bram_block_BRAM_Dout : std_logic_vector(0 to 31);
  signal microblaze_0_d_bram_ctrl_2_microblaze_0_bram_block_BRAM_EN : std_logic;
  signal microblaze_0_d_bram_ctrl_2_microblaze_0_bram_block_BRAM_Rst : std_logic;
  signal microblaze_0_d_bram_ctrl_2_microblaze_0_bram_block_BRAM_WEN : std_logic_vector(0 to 3);
  signal microblaze_0_debug_Dbg_Capture : std_logic;
  signal microblaze_0_debug_Dbg_Clk : std_logic;
  signal microblaze_0_debug_Dbg_Reg_En : std_logic_vector(0 to 7);
  signal microblaze_0_debug_Dbg_Shift : std_logic;
  signal microblaze_0_debug_Dbg_TDI : std_logic;
  signal microblaze_0_debug_Dbg_TDO : std_logic;
  signal microblaze_0_debug_Dbg_Update : std_logic;
  signal microblaze_0_debug_Debug_Rst : std_logic;
  signal microblaze_0_dlmb_LMB_ABus : std_logic_vector(0 to 31);
  signal microblaze_0_dlmb_LMB_AddrStrobe : std_logic;
  signal microblaze_0_dlmb_LMB_BE : std_logic_vector(0 to 3);
  signal microblaze_0_dlmb_LMB_CE : std_logic;
  signal microblaze_0_dlmb_LMB_ReadDBus : std_logic_vector(0 to 31);
  signal microblaze_0_dlmb_LMB_ReadStrobe : std_logic;
  signal microblaze_0_dlmb_LMB_Ready : std_logic;
  signal microblaze_0_dlmb_LMB_Rst : std_logic;
  signal microblaze_0_dlmb_LMB_UE : std_logic;
  signal microblaze_0_dlmb_LMB_Wait : std_logic;
  signal microblaze_0_dlmb_LMB_WriteDBus : std_logic_vector(0 to 31);
  signal microblaze_0_dlmb_LMB_WriteStrobe : std_logic;
  signal microblaze_0_dlmb_M_ABus : std_logic_vector(0 to 31);
  signal microblaze_0_dlmb_M_AddrStrobe : std_logic;
  signal microblaze_0_dlmb_M_BE : std_logic_vector(0 to 3);
  signal microblaze_0_dlmb_M_DBus : std_logic_vector(0 to 31);
  signal microblaze_0_dlmb_M_ReadStrobe : std_logic;
  signal microblaze_0_dlmb_M_WriteStrobe : std_logic;
  signal microblaze_0_dlmb_Sl_CE : std_logic_vector(0 to 0);
  signal microblaze_0_dlmb_Sl_DBus : std_logic_vector(0 to 31);
  signal microblaze_0_dlmb_Sl_Ready : std_logic_vector(0 to 0);
  signal microblaze_0_dlmb_Sl_UE : std_logic_vector(0 to 0);
  signal microblaze_0_dlmb_Sl_Wait : std_logic_vector(0 to 0);
  signal microblaze_0_i_bram_ctrl_2_microblaze_0_bram_block_BRAM_Addr : std_logic_vector(0 to 31);
  signal microblaze_0_i_bram_ctrl_2_microblaze_0_bram_block_BRAM_Clk : std_logic;
  signal microblaze_0_i_bram_ctrl_2_microblaze_0_bram_block_BRAM_Din : std_logic_vector(0 to 31);
  signal microblaze_0_i_bram_ctrl_2_microblaze_0_bram_block_BRAM_Dout : std_logic_vector(0 to 31);
  signal microblaze_0_i_bram_ctrl_2_microblaze_0_bram_block_BRAM_EN : std_logic;
  signal microblaze_0_i_bram_ctrl_2_microblaze_0_bram_block_BRAM_Rst : std_logic;
  signal microblaze_0_i_bram_ctrl_2_microblaze_0_bram_block_BRAM_WEN : std_logic_vector(0 to 3);
  signal microblaze_0_ilmb_LMB_ABus : std_logic_vector(0 to 31);
  signal microblaze_0_ilmb_LMB_AddrStrobe : std_logic;
  signal microblaze_0_ilmb_LMB_BE : std_logic_vector(0 to 3);
  signal microblaze_0_ilmb_LMB_CE : std_logic;
  signal microblaze_0_ilmb_LMB_ReadDBus : std_logic_vector(0 to 31);
  signal microblaze_0_ilmb_LMB_ReadStrobe : std_logic;
  signal microblaze_0_ilmb_LMB_Ready : std_logic;
  signal microblaze_0_ilmb_LMB_Rst : std_logic;
  signal microblaze_0_ilmb_LMB_UE : std_logic;
  signal microblaze_0_ilmb_LMB_Wait : std_logic;
  signal microblaze_0_ilmb_LMB_WriteDBus : std_logic_vector(0 to 31);
  signal microblaze_0_ilmb_LMB_WriteStrobe : std_logic;
  signal microblaze_0_ilmb_M_ABus : std_logic_vector(0 to 31);
  signal microblaze_0_ilmb_M_AddrStrobe : std_logic;
  signal microblaze_0_ilmb_M_ReadStrobe : std_logic;
  signal microblaze_0_ilmb_Sl_CE : std_logic_vector(0 to 0);
  signal microblaze_0_ilmb_Sl_DBus : std_logic_vector(0 to 31);
  signal microblaze_0_ilmb_Sl_Ready : std_logic_vector(0 to 0);
  signal microblaze_0_ilmb_Sl_UE : std_logic_vector(0 to 0);
  signal microblaze_0_ilmb_Sl_Wait : std_logic_vector(0 to 0);
  signal microblaze_0_interrupt_Interrupt : std_logic;
  signal microblaze_0_interrupt_Interrupt_Ack : std_logic_vector(1 downto 0);
  signal microblaze_0_interrupt_Interrupt_Address : std_logic_vector(0 to 31);
  signal net_gnd0 : std_logic;
  signal net_gnd1 : std_logic_vector(0 to 0);
  signal net_gnd2 : std_logic_vector(0 to 1);
  signal net_gnd3 : std_logic_vector(0 to 2);
  signal net_gnd4 : std_logic_vector(0 to 3);
  signal net_gnd5 : std_logic_vector(4 downto 0);
  signal net_gnd8 : std_logic_vector(7 downto 0);
  signal net_gnd15 : std_logic_vector(14 downto 0);
  signal net_gnd16 : std_logic_vector(0 to 15);
  signal net_gnd32 : std_logic_vector(0 to 31);
  signal net_gnd4096 : std_logic_vector(0 to 4095);
  signal net_vcc0 : std_logic;
  signal net_vcc1 : std_logic_vector(0 downto 0);
  signal pgassign1 : std_logic_vector(0 to 0);
  signal pgassign2 : std_logic_vector(0 to 0);
  signal pgassign3 : std_logic_vector(0 to 0);
  signal pgassign4 : std_logic_vector(0 to 0);
  signal pgassign5 : std_logic_vector(0 to 0);
  signal pgassign6 : std_logic_vector(3 downto 0);
  signal pgassign7 : std_logic_vector(7 downto 0);
  signal pgassign8 : std_logic_vector(4 downto 0);
  signal proc_sys_reset_0_BUS_STRUCT_RESET : std_logic_vector(0 to 0);
  signal proc_sys_reset_0_Dcm_locked : std_logic;
  signal proc_sys_reset_0_Interconnect_aresetn : std_logic_vector(0 to 0);
  signal proc_sys_reset_0_MB_Debug_Sys_Rst : std_logic;
  signal proc_sys_reset_0_MB_Reset : std_logic;
  signal psdone : std_logic;
  signal psen : std_logic;
  signal psincdec : std_logic;

  attribute BUFFER_TYPE : STRING;
  attribute BOX_TYPE : STRING;
  attribute BUFFER_TYPE of SysACE_CLK : signal is "BUFGP";
  attribute BOX_TYPE of system_proc_sys_reset_0_wrapper : component is "user_black_box";
  attribute BOX_TYPE of system_microblaze_0_intc_wrapper : component is "user_black_box";
  attribute BOX_TYPE of system_microblaze_0_ilmb_wrapper : component is "user_black_box";
  attribute BOX_TYPE of system_microblaze_0_i_bram_ctrl_wrapper : component is "user_black_box";
  attribute BOX_TYPE of system_microblaze_0_dlmb_wrapper : component is "user_black_box";
  attribute BOX_TYPE of system_microblaze_0_d_bram_ctrl_wrapper : component is "user_black_box";
  attribute BOX_TYPE of system_microblaze_0_bram_block_wrapper : component is "user_black_box";
  attribute BOX_TYPE of system_microblaze_0_wrapper : component is "user_black_box";
  attribute BOX_TYPE of system_debug_module_wrapper : component is "user_black_box";
  attribute BOX_TYPE of system_clock_generator_0_wrapper : component is "user_black_box";
  attribute BOX_TYPE of system_axi_timer_0_wrapper : component is "user_black_box";
  attribute BOX_TYPE of system_axi4lite_0_wrapper : component is "user_black_box";
  attribute BOX_TYPE of system_axi4_0_wrapper : component is "user_black_box";
  attribute BOX_TYPE of system_sysace_compactflash_wrapper : component is "user_black_box";
  attribute BOX_TYPE of system_rs232_uart_1_wrapper : component is "user_black_box";
  attribute BOX_TYPE of system_ethernet_dma_wrapper : component is "user_black_box";
  attribute BOX_TYPE of system_ethernet_wrapper : component is "user_black_box";
  attribute BOX_TYPE of system_ddr3_sdram_wrapper : component is "user_black_box";
  attribute BOX_TYPE of system_controller_0_wrapper : component is "user_black_box";

begin

  -- Internal assignments

  axi4_0_S_AWID(8 downto 6) <= B"000";
  axi4_0_S_AWID(11 downto 9) <= B"000";
  axi4_0_S_AWID(14 downto 12) <= B"000";
  axi4_0_S_AWADDR(127 downto 96) <= B"00000000000000000000000000000000";
  axi4_0_S_AWLEN(31 downto 24) <= B"00000000";
  axi4_0_S_AWSIZE(11 downto 9) <= B"000";
  axi4_0_S_AWBURST(7 downto 6) <= B"00";
  axi4_0_S_AWLOCK(5 downto 4) <= B"00";
  axi4_0_S_AWLOCK(7 downto 6) <= B"00";
  axi4_0_S_AWLOCK(9 downto 8) <= B"00";
  axi4_0_S_AWCACHE(15 downto 12) <= B"0000";
  axi4_0_S_AWPROT(11 downto 9) <= B"000";
  axi4_0_S_AWQOS(11 downto 8) <= B"0000";
  axi4_0_S_AWQOS(15 downto 12) <= B"0000";
  axi4_0_S_AWQOS(19 downto 16) <= B"0000";
  axi4_0_S_AWVALID(3 downto 3) <= B"0";
  axi4_0_S_AWUSER(19 downto 15) <= B"00000";
  axi4_0_S_WDATA(127 downto 96) <= B"00000000000000000000000000000000";
  axi4_0_S_WSTRB(15 downto 12) <= B"0000";
  axi4_0_S_WLAST(3 downto 3) <= B"0";
  axi4_0_S_WVALID(3 downto 3) <= B"0";
  axi4_0_S_WUSER(2 downto 2) <= B"0";
  axi4_0_S_WUSER(3 downto 3) <= B"0";
  axi4_0_S_WUSER(4 downto 4) <= B"0";
  axi4_0_S_BREADY(3 downto 3) <= B"0";
  axi4_0_S_ARID(8 downto 6) <= B"000";
  axi4_0_S_ARID(11 downto 9) <= B"000";
  axi4_0_S_ARID(14 downto 12) <= B"000";
  axi4_0_S_ARADDR(159 downto 128) <= B"00000000000000000000000000000000";
  axi4_0_S_ARLEN(39 downto 32) <= B"00000000";
  axi4_0_S_ARSIZE(14 downto 12) <= B"000";
  axi4_0_S_ARBURST(9 downto 8) <= B"00";
  axi4_0_S_ARLOCK(5 downto 4) <= B"00";
  axi4_0_S_ARLOCK(7 downto 6) <= B"00";
  axi4_0_S_ARLOCK(9 downto 8) <= B"00";
  axi4_0_S_ARCACHE(19 downto 16) <= B"0000";
  axi4_0_S_ARPROT(14 downto 12) <= B"000";
  axi4_0_S_ARQOS(11 downto 8) <= B"0000";
  axi4_0_S_ARQOS(15 downto 12) <= B"0000";
  axi4_0_S_ARQOS(19 downto 16) <= B"0000";
  axi4_0_S_ARVALID(4 downto 4) <= B"0";
  axi4_0_S_ARUSER(24 downto 20) <= B"00000";
  axi4_0_S_RREADY(4 downto 4) <= B"0";
  ddr_memory_odt <= pgassign1(0);
  ddr_memory_cs_n <= pgassign2(0);
  ddr_memory_clk_n <= pgassign3(0);
  ddr_memory_clk <= pgassign4(0);
  ddr_memory_cke <= pgassign5(0);
  pgassign6(3) <= ETHERNET_INTERRUPT;
  pgassign6(2) <= axi_timer_0_Interrupt;
  pgassign6(1) <= ETHERNET_dma_mm2s_introut;
  pgassign6(0) <= ETHERNET_dma_s2mm_introut;
  pgassign7(7 downto 7) <= clk_100_0000MHzMMCM0(0 to 0);
  pgassign7(6 downto 6) <= clk_100_0000MHzMMCM0(0 to 0);
  pgassign7(5 downto 5) <= clk_100_0000MHzMMCM0(0 to 0);
  pgassign7(4 downto 4) <= clk_100_0000MHzMMCM0(0 to 0);
  pgassign7(3 downto 3) <= clk_100_0000MHzMMCM0(0 to 0);
  pgassign7(2 downto 2) <= clk_100_0000MHzMMCM0(0 to 0);
  pgassign7(1 downto 1) <= clk_100_0000MHzMMCM0(0 to 0);
  pgassign7(0 downto 0) <= clk_100_0000MHzMMCM0(0 to 0);
  pgassign8(4 downto 4) <= clk_100_0000MHzMMCM0(0 to 0);
  pgassign8(3 downto 3) <= clk_100_0000MHzMMCM0(0 to 0);
  pgassign8(2 downto 2) <= clk_100_0000MHzMMCM0(0 to 0);
  pgassign8(1 downto 1) <= clk_100_0000MHzMMCM0(0 to 0);
  pgassign8(0 downto 0) <= clk_100_0000MHzMMCM0(0 to 0);
  net_gnd0 <= '0';
  net_gnd1(0 to 0) <= B"0";
  net_gnd15(14 downto 0) <= B"000000000000000";
  net_gnd16(0 to 15) <= B"0000000000000000";
  net_gnd2(0 to 1) <= B"00";
  net_gnd3(0 to 2) <= B"000";
  net_gnd32(0 to 31) <= B"00000000000000000000000000000000";
  net_gnd4(0 to 3) <= B"0000";
  net_gnd4096(0 to 4095) <= X"0000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000";
  net_gnd5(4 downto 0) <= B"00000";
  net_gnd8(7 downto 0) <= B"00000000";
  net_vcc0 <= '1';
  net_vcc1(0 downto 0) <= B"1";

  proc_sys_reset_0 : system_proc_sys_reset_0_wrapper
    port map (
      Slowest_sync_clk => pgassign7(7),
      Ext_Reset_In => RESET,
      Aux_Reset_In => net_gnd0,
      MB_Debug_Sys_Rst => proc_sys_reset_0_MB_Debug_Sys_Rst,
      Core_Reset_Req_0 => net_gnd0,
      Chip_Reset_Req_0 => net_gnd0,
      System_Reset_Req_0 => net_gnd0,
      Core_Reset_Req_1 => net_gnd0,
      Chip_Reset_Req_1 => net_gnd0,
      System_Reset_Req_1 => net_gnd0,
      Dcm_locked => proc_sys_reset_0_Dcm_locked,
      RstcPPCresetcore_0 => open,
      RstcPPCresetchip_0 => open,
      RstcPPCresetsys_0 => open,
      RstcPPCresetcore_1 => open,
      RstcPPCresetchip_1 => open,
      RstcPPCresetsys_1 => open,
      MB_Reset => proc_sys_reset_0_MB_Reset,
      Bus_Struct_Reset => proc_sys_reset_0_BUS_STRUCT_RESET(0 to 0),
      Peripheral_Reset => open,
      Interconnect_aresetn => proc_sys_reset_0_Interconnect_aresetn(0 to 0),
      Peripheral_aresetn => open
    );

  microblaze_0_intc : system_microblaze_0_intc_wrapper
    port map (
      S_AXI_ACLK => pgassign7(7),
      S_AXI_ARESETN => axi4lite_0_M_ARESETN(0),
      S_AXI_AWADDR => axi4lite_0_M_AWADDR(8 downto 0),
      S_AXI_AWVALID => axi4lite_0_M_AWVALID(0),
      S_AXI_AWREADY => axi4lite_0_M_AWREADY(0),
      S_AXI_WDATA => axi4lite_0_M_WDATA(31 downto 0),
      S_AXI_WSTRB => axi4lite_0_M_WSTRB(3 downto 0),
      S_AXI_WVALID => axi4lite_0_M_WVALID(0),
      S_AXI_WREADY => axi4lite_0_M_WREADY(0),
      S_AXI_BRESP => axi4lite_0_M_BRESP(1 downto 0),
      S_AXI_BVALID => axi4lite_0_M_BVALID(0),
      S_AXI_BREADY => axi4lite_0_M_BREADY(0),
      S_AXI_ARADDR => axi4lite_0_M_ARADDR(8 downto 0),
      S_AXI_ARVALID => axi4lite_0_M_ARVALID(0),
      S_AXI_ARREADY => axi4lite_0_M_ARREADY(0),
      S_AXI_RDATA => axi4lite_0_M_RDATA(31 downto 0),
      S_AXI_RRESP => axi4lite_0_M_RRESP(1 downto 0),
      S_AXI_RVALID => axi4lite_0_M_RVALID(0),
      S_AXI_RREADY => axi4lite_0_M_RREADY(0),
      Intr => pgassign6,
      Processor_clk => net_gnd0,
      Processor_rst => net_gnd0,
      Irq => microblaze_0_interrupt_Interrupt,
      Interrupt_address => microblaze_0_interrupt_Interrupt_Address(0 to 31),
      Processor_ack => microblaze_0_interrupt_Interrupt_Ack
    );

  microblaze_0_ilmb : system_microblaze_0_ilmb_wrapper
    port map (
      LMB_Clk => pgassign7(7),
      SYS_Rst => proc_sys_reset_0_BUS_STRUCT_RESET(0),
      LMB_Rst => microblaze_0_ilmb_LMB_Rst,
      M_ABus => microblaze_0_ilmb_M_ABus,
      M_ReadStrobe => microblaze_0_ilmb_M_ReadStrobe,
      M_WriteStrobe => net_gnd0,
      M_AddrStrobe => microblaze_0_ilmb_M_AddrStrobe,
      M_DBus => net_gnd32,
      M_BE => net_gnd4,
      Sl_DBus => microblaze_0_ilmb_Sl_DBus,
      Sl_Ready => microblaze_0_ilmb_Sl_Ready(0 to 0),
      Sl_Wait => microblaze_0_ilmb_Sl_Wait(0 to 0),
      Sl_UE => microblaze_0_ilmb_Sl_UE(0 to 0),
      Sl_CE => microblaze_0_ilmb_Sl_CE(0 to 0),
      LMB_ABus => microblaze_0_ilmb_LMB_ABus,
      LMB_ReadStrobe => microblaze_0_ilmb_LMB_ReadStrobe,
      LMB_WriteStrobe => microblaze_0_ilmb_LMB_WriteStrobe,
      LMB_AddrStrobe => microblaze_0_ilmb_LMB_AddrStrobe,
      LMB_ReadDBus => microblaze_0_ilmb_LMB_ReadDBus,
      LMB_WriteDBus => microblaze_0_ilmb_LMB_WriteDBus,
      LMB_Ready => microblaze_0_ilmb_LMB_Ready,
      LMB_Wait => microblaze_0_ilmb_LMB_Wait,
      LMB_UE => microblaze_0_ilmb_LMB_UE,
      LMB_CE => microblaze_0_ilmb_LMB_CE,
      LMB_BE => microblaze_0_ilmb_LMB_BE
    );

  microblaze_0_i_bram_ctrl : system_microblaze_0_i_bram_ctrl_wrapper
    port map (
      LMB_Clk => pgassign7(7),
      LMB_Rst => microblaze_0_ilmb_LMB_Rst,
      LMB_ABus => microblaze_0_ilmb_LMB_ABus,
      LMB_WriteDBus => microblaze_0_ilmb_LMB_WriteDBus,
      LMB_AddrStrobe => microblaze_0_ilmb_LMB_AddrStrobe,
      LMB_ReadStrobe => microblaze_0_ilmb_LMB_ReadStrobe,
      LMB_WriteStrobe => microblaze_0_ilmb_LMB_WriteStrobe,
      LMB_BE => microblaze_0_ilmb_LMB_BE,
      Sl_DBus => microblaze_0_ilmb_Sl_DBus,
      Sl_Ready => microblaze_0_ilmb_Sl_Ready(0),
      Sl_Wait => microblaze_0_ilmb_Sl_Wait(0),
      Sl_UE => microblaze_0_ilmb_Sl_UE(0),
      Sl_CE => microblaze_0_ilmb_Sl_CE(0),
      LMB1_ABus => net_gnd32,
      LMB1_WriteDBus => net_gnd32,
      LMB1_AddrStrobe => net_gnd0,
      LMB1_ReadStrobe => net_gnd0,
      LMB1_WriteStrobe => net_gnd0,
      LMB1_BE => net_gnd4,
      Sl1_DBus => open,
      Sl1_Ready => open,
      Sl1_Wait => open,
      Sl1_UE => open,
      Sl1_CE => open,
      LMB2_ABus => net_gnd32,
      LMB2_WriteDBus => net_gnd32,
      LMB2_AddrStrobe => net_gnd0,
      LMB2_ReadStrobe => net_gnd0,
      LMB2_WriteStrobe => net_gnd0,
      LMB2_BE => net_gnd4,
      Sl2_DBus => open,
      Sl2_Ready => open,
      Sl2_Wait => open,
      Sl2_UE => open,
      Sl2_CE => open,
      LMB3_ABus => net_gnd32,
      LMB3_WriteDBus => net_gnd32,
      LMB3_AddrStrobe => net_gnd0,
      LMB3_ReadStrobe => net_gnd0,
      LMB3_WriteStrobe => net_gnd0,
      LMB3_BE => net_gnd4,
      Sl3_DBus => open,
      Sl3_Ready => open,
      Sl3_Wait => open,
      Sl3_UE => open,
      Sl3_CE => open,
      BRAM_Rst_A => microblaze_0_i_bram_ctrl_2_microblaze_0_bram_block_BRAM_Rst,
      BRAM_Clk_A => microblaze_0_i_bram_ctrl_2_microblaze_0_bram_block_BRAM_Clk,
      BRAM_EN_A => microblaze_0_i_bram_ctrl_2_microblaze_0_bram_block_BRAM_EN,
      BRAM_WEN_A => microblaze_0_i_bram_ctrl_2_microblaze_0_bram_block_BRAM_WEN,
      BRAM_Addr_A => microblaze_0_i_bram_ctrl_2_microblaze_0_bram_block_BRAM_Addr,
      BRAM_Din_A => microblaze_0_i_bram_ctrl_2_microblaze_0_bram_block_BRAM_Din,
      BRAM_Dout_A => microblaze_0_i_bram_ctrl_2_microblaze_0_bram_block_BRAM_Dout,
      Interrupt => open,
      UE => open,
      CE => open,
      SPLB_CTRL_PLB_ABus => net_gnd32,
      SPLB_CTRL_PLB_PAValid => net_gnd0,
      SPLB_CTRL_PLB_masterID => net_gnd1(0 to 0),
      SPLB_CTRL_PLB_RNW => net_gnd0,
      SPLB_CTRL_PLB_BE => net_gnd4,
      SPLB_CTRL_PLB_size => net_gnd4,
      SPLB_CTRL_PLB_type => net_gnd3,
      SPLB_CTRL_PLB_wrDBus => net_gnd32,
      SPLB_CTRL_Sl_addrAck => open,
      SPLB_CTRL_Sl_SSize => open,
      SPLB_CTRL_Sl_wait => open,
      SPLB_CTRL_Sl_rearbitrate => open,
      SPLB_CTRL_Sl_wrDAck => open,
      SPLB_CTRL_Sl_wrComp => open,
      SPLB_CTRL_Sl_rdDBus => open,
      SPLB_CTRL_Sl_rdDAck => open,
      SPLB_CTRL_Sl_rdComp => open,
      SPLB_CTRL_Sl_MBusy => open,
      SPLB_CTRL_Sl_MWrErr => open,
      SPLB_CTRL_Sl_MRdErr => open,
      SPLB_CTRL_PLB_UABus => net_gnd32,
      SPLB_CTRL_PLB_SAValid => net_gnd0,
      SPLB_CTRL_PLB_rdPrim => net_gnd0,
      SPLB_CTRL_PLB_wrPrim => net_gnd0,
      SPLB_CTRL_PLB_abort => net_gnd0,
      SPLB_CTRL_PLB_busLock => net_gnd0,
      SPLB_CTRL_PLB_MSize => net_gnd2,
      SPLB_CTRL_PLB_lockErr => net_gnd0,
      SPLB_CTRL_PLB_wrBurst => net_gnd0,
      SPLB_CTRL_PLB_rdBurst => net_gnd0,
      SPLB_CTRL_PLB_wrPendReq => net_gnd0,
      SPLB_CTRL_PLB_rdPendReq => net_gnd0,
      SPLB_CTRL_PLB_wrPendPri => net_gnd2,
      SPLB_CTRL_PLB_rdPendPri => net_gnd2,
      SPLB_CTRL_PLB_reqPri => net_gnd2,
      SPLB_CTRL_PLB_TAttribute => net_gnd16,
      SPLB_CTRL_Sl_wrBTerm => open,
      SPLB_CTRL_Sl_rdWdAddr => open,
      SPLB_CTRL_Sl_rdBTerm => open,
      SPLB_CTRL_Sl_MIRQ => open,
      S_AXI_CTRL_ACLK => net_vcc0,
      S_AXI_CTRL_ARESETN => net_gnd0,
      S_AXI_CTRL_AWADDR => net_gnd32(0 to 31),
      S_AXI_CTRL_AWVALID => net_gnd0,
      S_AXI_CTRL_AWREADY => open,
      S_AXI_CTRL_WDATA => net_gnd32(0 to 31),
      S_AXI_CTRL_WSTRB => net_gnd4(0 to 3),
      S_AXI_CTRL_WVALID => net_gnd0,
      S_AXI_CTRL_WREADY => open,
      S_AXI_CTRL_BRESP => open,
      S_AXI_CTRL_BVALID => open,
      S_AXI_CTRL_BREADY => net_gnd0,
      S_AXI_CTRL_ARADDR => net_gnd32(0 to 31),
      S_AXI_CTRL_ARVALID => net_gnd0,
      S_AXI_CTRL_ARREADY => open,
      S_AXI_CTRL_RDATA => open,
      S_AXI_CTRL_RRESP => open,
      S_AXI_CTRL_RVALID => open,
      S_AXI_CTRL_RREADY => net_gnd0
    );

  microblaze_0_dlmb : system_microblaze_0_dlmb_wrapper
    port map (
      LMB_Clk => pgassign7(7),
      SYS_Rst => proc_sys_reset_0_BUS_STRUCT_RESET(0),
      LMB_Rst => microblaze_0_dlmb_LMB_Rst,
      M_ABus => microblaze_0_dlmb_M_ABus,
      M_ReadStrobe => microblaze_0_dlmb_M_ReadStrobe,
      M_WriteStrobe => microblaze_0_dlmb_M_WriteStrobe,
      M_AddrStrobe => microblaze_0_dlmb_M_AddrStrobe,
      M_DBus => microblaze_0_dlmb_M_DBus,
      M_BE => microblaze_0_dlmb_M_BE,
      Sl_DBus => microblaze_0_dlmb_Sl_DBus,
      Sl_Ready => microblaze_0_dlmb_Sl_Ready(0 to 0),
      Sl_Wait => microblaze_0_dlmb_Sl_Wait(0 to 0),
      Sl_UE => microblaze_0_dlmb_Sl_UE(0 to 0),
      Sl_CE => microblaze_0_dlmb_Sl_CE(0 to 0),
      LMB_ABus => microblaze_0_dlmb_LMB_ABus,
      LMB_ReadStrobe => microblaze_0_dlmb_LMB_ReadStrobe,
      LMB_WriteStrobe => microblaze_0_dlmb_LMB_WriteStrobe,
      LMB_AddrStrobe => microblaze_0_dlmb_LMB_AddrStrobe,
      LMB_ReadDBus => microblaze_0_dlmb_LMB_ReadDBus,
      LMB_WriteDBus => microblaze_0_dlmb_LMB_WriteDBus,
      LMB_Ready => microblaze_0_dlmb_LMB_Ready,
      LMB_Wait => microblaze_0_dlmb_LMB_Wait,
      LMB_UE => microblaze_0_dlmb_LMB_UE,
      LMB_CE => microblaze_0_dlmb_LMB_CE,
      LMB_BE => microblaze_0_dlmb_LMB_BE
    );

  microblaze_0_d_bram_ctrl : system_microblaze_0_d_bram_ctrl_wrapper
    port map (
      LMB_Clk => pgassign7(7),
      LMB_Rst => microblaze_0_dlmb_LMB_Rst,
      LMB_ABus => microblaze_0_dlmb_LMB_ABus,
      LMB_WriteDBus => microblaze_0_dlmb_LMB_WriteDBus,
      LMB_AddrStrobe => microblaze_0_dlmb_LMB_AddrStrobe,
      LMB_ReadStrobe => microblaze_0_dlmb_LMB_ReadStrobe,
      LMB_WriteStrobe => microblaze_0_dlmb_LMB_WriteStrobe,
      LMB_BE => microblaze_0_dlmb_LMB_BE,
      Sl_DBus => microblaze_0_dlmb_Sl_DBus,
      Sl_Ready => microblaze_0_dlmb_Sl_Ready(0),
      Sl_Wait => microblaze_0_dlmb_Sl_Wait(0),
      Sl_UE => microblaze_0_dlmb_Sl_UE(0),
      Sl_CE => microblaze_0_dlmb_Sl_CE(0),
      LMB1_ABus => net_gnd32,
      LMB1_WriteDBus => net_gnd32,
      LMB1_AddrStrobe => net_gnd0,
      LMB1_ReadStrobe => net_gnd0,
      LMB1_WriteStrobe => net_gnd0,
      LMB1_BE => net_gnd4,
      Sl1_DBus => open,
      Sl1_Ready => open,
      Sl1_Wait => open,
      Sl1_UE => open,
      Sl1_CE => open,
      LMB2_ABus => net_gnd32,
      LMB2_WriteDBus => net_gnd32,
      LMB2_AddrStrobe => net_gnd0,
      LMB2_ReadStrobe => net_gnd0,
      LMB2_WriteStrobe => net_gnd0,
      LMB2_BE => net_gnd4,
      Sl2_DBus => open,
      Sl2_Ready => open,
      Sl2_Wait => open,
      Sl2_UE => open,
      Sl2_CE => open,
      LMB3_ABus => net_gnd32,
      LMB3_WriteDBus => net_gnd32,
      LMB3_AddrStrobe => net_gnd0,
      LMB3_ReadStrobe => net_gnd0,
      LMB3_WriteStrobe => net_gnd0,
      LMB3_BE => net_gnd4,
      Sl3_DBus => open,
      Sl3_Ready => open,
      Sl3_Wait => open,
      Sl3_UE => open,
      Sl3_CE => open,
      BRAM_Rst_A => microblaze_0_d_bram_ctrl_2_microblaze_0_bram_block_BRAM_Rst,
      BRAM_Clk_A => microblaze_0_d_bram_ctrl_2_microblaze_0_bram_block_BRAM_Clk,
      BRAM_EN_A => microblaze_0_d_bram_ctrl_2_microblaze_0_bram_block_BRAM_EN,
      BRAM_WEN_A => microblaze_0_d_bram_ctrl_2_microblaze_0_bram_block_BRAM_WEN,
      BRAM_Addr_A => microblaze_0_d_bram_ctrl_2_microblaze_0_bram_block_BRAM_Addr,
      BRAM_Din_A => microblaze_0_d_bram_ctrl_2_microblaze_0_bram_block_BRAM_Din,
      BRAM_Dout_A => microblaze_0_d_bram_ctrl_2_microblaze_0_bram_block_BRAM_Dout,
      Interrupt => open,
      UE => open,
      CE => open,
      SPLB_CTRL_PLB_ABus => net_gnd32,
      SPLB_CTRL_PLB_PAValid => net_gnd0,
      SPLB_CTRL_PLB_masterID => net_gnd1(0 to 0),
      SPLB_CTRL_PLB_RNW => net_gnd0,
      SPLB_CTRL_PLB_BE => net_gnd4,
      SPLB_CTRL_PLB_size => net_gnd4,
      SPLB_CTRL_PLB_type => net_gnd3,
      SPLB_CTRL_PLB_wrDBus => net_gnd32,
      SPLB_CTRL_Sl_addrAck => open,
      SPLB_CTRL_Sl_SSize => open,
      SPLB_CTRL_Sl_wait => open,
      SPLB_CTRL_Sl_rearbitrate => open,
      SPLB_CTRL_Sl_wrDAck => open,
      SPLB_CTRL_Sl_wrComp => open,
      SPLB_CTRL_Sl_rdDBus => open,
      SPLB_CTRL_Sl_rdDAck => open,
      SPLB_CTRL_Sl_rdComp => open,
      SPLB_CTRL_Sl_MBusy => open,
      SPLB_CTRL_Sl_MWrErr => open,
      SPLB_CTRL_Sl_MRdErr => open,
      SPLB_CTRL_PLB_UABus => net_gnd32,
      SPLB_CTRL_PLB_SAValid => net_gnd0,
      SPLB_CTRL_PLB_rdPrim => net_gnd0,
      SPLB_CTRL_PLB_wrPrim => net_gnd0,
      SPLB_CTRL_PLB_abort => net_gnd0,
      SPLB_CTRL_PLB_busLock => net_gnd0,
      SPLB_CTRL_PLB_MSize => net_gnd2,
      SPLB_CTRL_PLB_lockErr => net_gnd0,
      SPLB_CTRL_PLB_wrBurst => net_gnd0,
      SPLB_CTRL_PLB_rdBurst => net_gnd0,
      SPLB_CTRL_PLB_wrPendReq => net_gnd0,
      SPLB_CTRL_PLB_rdPendReq => net_gnd0,
      SPLB_CTRL_PLB_wrPendPri => net_gnd2,
      SPLB_CTRL_PLB_rdPendPri => net_gnd2,
      SPLB_CTRL_PLB_reqPri => net_gnd2,
      SPLB_CTRL_PLB_TAttribute => net_gnd16,
      SPLB_CTRL_Sl_wrBTerm => open,
      SPLB_CTRL_Sl_rdWdAddr => open,
      SPLB_CTRL_Sl_rdBTerm => open,
      SPLB_CTRL_Sl_MIRQ => open,
      S_AXI_CTRL_ACLK => net_vcc0,
      S_AXI_CTRL_ARESETN => net_gnd0,
      S_AXI_CTRL_AWADDR => net_gnd32(0 to 31),
      S_AXI_CTRL_AWVALID => net_gnd0,
      S_AXI_CTRL_AWREADY => open,
      S_AXI_CTRL_WDATA => net_gnd32(0 to 31),
      S_AXI_CTRL_WSTRB => net_gnd4(0 to 3),
      S_AXI_CTRL_WVALID => net_gnd0,
      S_AXI_CTRL_WREADY => open,
      S_AXI_CTRL_BRESP => open,
      S_AXI_CTRL_BVALID => open,
      S_AXI_CTRL_BREADY => net_gnd0,
      S_AXI_CTRL_ARADDR => net_gnd32(0 to 31),
      S_AXI_CTRL_ARVALID => net_gnd0,
      S_AXI_CTRL_ARREADY => open,
      S_AXI_CTRL_RDATA => open,
      S_AXI_CTRL_RRESP => open,
      S_AXI_CTRL_RVALID => open,
      S_AXI_CTRL_RREADY => net_gnd0
    );

  microblaze_0_bram_block : system_microblaze_0_bram_block_wrapper
    port map (
      BRAM_Rst_A => microblaze_0_i_bram_ctrl_2_microblaze_0_bram_block_BRAM_Rst,
      BRAM_Clk_A => microblaze_0_i_bram_ctrl_2_microblaze_0_bram_block_BRAM_Clk,
      BRAM_EN_A => microblaze_0_i_bram_ctrl_2_microblaze_0_bram_block_BRAM_EN,
      BRAM_WEN_A => microblaze_0_i_bram_ctrl_2_microblaze_0_bram_block_BRAM_WEN,
      BRAM_Addr_A => microblaze_0_i_bram_ctrl_2_microblaze_0_bram_block_BRAM_Addr,
      BRAM_Din_A => microblaze_0_i_bram_ctrl_2_microblaze_0_bram_block_BRAM_Din,
      BRAM_Dout_A => microblaze_0_i_bram_ctrl_2_microblaze_0_bram_block_BRAM_Dout,
      BRAM_Rst_B => microblaze_0_d_bram_ctrl_2_microblaze_0_bram_block_BRAM_Rst,
      BRAM_Clk_B => microblaze_0_d_bram_ctrl_2_microblaze_0_bram_block_BRAM_Clk,
      BRAM_EN_B => microblaze_0_d_bram_ctrl_2_microblaze_0_bram_block_BRAM_EN,
      BRAM_WEN_B => microblaze_0_d_bram_ctrl_2_microblaze_0_bram_block_BRAM_WEN,
      BRAM_Addr_B => microblaze_0_d_bram_ctrl_2_microblaze_0_bram_block_BRAM_Addr,
      BRAM_Din_B => microblaze_0_d_bram_ctrl_2_microblaze_0_bram_block_BRAM_Din,
      BRAM_Dout_B => microblaze_0_d_bram_ctrl_2_microblaze_0_bram_block_BRAM_Dout
    );

  microblaze_0 : system_microblaze_0_wrapper
    port map (
      CLK => pgassign7(7),
      RESET => microblaze_0_dlmb_LMB_Rst,
      MB_RESET => proc_sys_reset_0_MB_Reset,
      INTERRUPT => microblaze_0_interrupt_Interrupt,
      INTERRUPT_ADDRESS => microblaze_0_interrupt_Interrupt_Address,
      INTERRUPT_ACK => microblaze_0_interrupt_Interrupt_Ack(1 downto 0),
      EXT_BRK => Ext_BRK,
      EXT_NM_BRK => Ext_NM_BRK,
      DBG_STOP => net_gnd0,
      MB_Halted => open,
      MB_Error => open,
      WAKEUP => net_gnd2,
      SLEEP => open,
      DBG_WAKEUP => open,
      LOCKSTEP_MASTER_OUT => open,
      LOCKSTEP_SLAVE_IN => net_gnd4096,
      LOCKSTEP_OUT => open,
      INSTR => microblaze_0_ilmb_LMB_ReadDBus,
      IREADY => microblaze_0_ilmb_LMB_Ready,
      IWAIT => microblaze_0_ilmb_LMB_Wait,
      ICE => microblaze_0_ilmb_LMB_CE,
      IUE => microblaze_0_ilmb_LMB_UE,
      INSTR_ADDR => microblaze_0_ilmb_M_ABus,
      IFETCH => microblaze_0_ilmb_M_ReadStrobe,
      I_AS => microblaze_0_ilmb_M_AddrStrobe,
      IPLB_M_ABort => open,
      IPLB_M_ABus => open,
      IPLB_M_UABus => open,
      IPLB_M_BE => open,
      IPLB_M_busLock => open,
      IPLB_M_lockErr => open,
      IPLB_M_MSize => open,
      IPLB_M_priority => open,
      IPLB_M_rdBurst => open,
      IPLB_M_request => open,
      IPLB_M_RNW => open,
      IPLB_M_size => open,
      IPLB_M_TAttribute => open,
      IPLB_M_type => open,
      IPLB_M_wrBurst => open,
      IPLB_M_wrDBus => open,
      IPLB_MBusy => net_gnd0,
      IPLB_MRdErr => net_gnd0,
      IPLB_MWrErr => net_gnd0,
      IPLB_MIRQ => net_gnd0,
      IPLB_MWrBTerm => net_gnd0,
      IPLB_MWrDAck => net_gnd0,
      IPLB_MAddrAck => net_gnd0,
      IPLB_MRdBTerm => net_gnd0,
      IPLB_MRdDAck => net_gnd0,
      IPLB_MRdDBus => net_gnd32,
      IPLB_MRdWdAddr => net_gnd4,
      IPLB_MRearbitrate => net_gnd0,
      IPLB_MSSize => net_gnd2,
      IPLB_MTimeout => net_gnd0,
      DATA_READ => microblaze_0_dlmb_LMB_ReadDBus,
      DREADY => microblaze_0_dlmb_LMB_Ready,
      DWAIT => microblaze_0_dlmb_LMB_Wait,
      DCE => microblaze_0_dlmb_LMB_CE,
      DUE => microblaze_0_dlmb_LMB_UE,
      DATA_WRITE => microblaze_0_dlmb_M_DBus,
      DATA_ADDR => microblaze_0_dlmb_M_ABus,
      D_AS => microblaze_0_dlmb_M_AddrStrobe,
      READ_STROBE => microblaze_0_dlmb_M_ReadStrobe,
      WRITE_STROBE => microblaze_0_dlmb_M_WriteStrobe,
      BYTE_ENABLE => microblaze_0_dlmb_M_BE,
      DPLB_M_ABort => open,
      DPLB_M_ABus => open,
      DPLB_M_UABus => open,
      DPLB_M_BE => open,
      DPLB_M_busLock => open,
      DPLB_M_lockErr => open,
      DPLB_M_MSize => open,
      DPLB_M_priority => open,
      DPLB_M_rdBurst => open,
      DPLB_M_request => open,
      DPLB_M_RNW => open,
      DPLB_M_size => open,
      DPLB_M_TAttribute => open,
      DPLB_M_type => open,
      DPLB_M_wrBurst => open,
      DPLB_M_wrDBus => open,
      DPLB_MBusy => net_gnd0,
      DPLB_MRdErr => net_gnd0,
      DPLB_MWrErr => net_gnd0,
      DPLB_MIRQ => net_gnd0,
      DPLB_MWrBTerm => net_gnd0,
      DPLB_MWrDAck => net_gnd0,
      DPLB_MAddrAck => net_gnd0,
      DPLB_MRdBTerm => net_gnd0,
      DPLB_MRdDAck => net_gnd0,
      DPLB_MRdDBus => net_gnd32,
      DPLB_MRdWdAddr => net_gnd4,
      DPLB_MRearbitrate => net_gnd0,
      DPLB_MSSize => net_gnd2,
      DPLB_MTimeout => net_gnd0,
      M_AXI_IP_AWID => open,
      M_AXI_IP_AWADDR => open,
      M_AXI_IP_AWLEN => open,
      M_AXI_IP_AWSIZE => open,
      M_AXI_IP_AWBURST => open,
      M_AXI_IP_AWLOCK => open,
      M_AXI_IP_AWCACHE => open,
      M_AXI_IP_AWPROT => open,
      M_AXI_IP_AWQOS => open,
      M_AXI_IP_AWVALID => open,
      M_AXI_IP_AWREADY => net_gnd0,
      M_AXI_IP_WDATA => open,
      M_AXI_IP_WSTRB => open,
      M_AXI_IP_WLAST => open,
      M_AXI_IP_WVALID => open,
      M_AXI_IP_WREADY => net_gnd0,
      M_AXI_IP_BID => net_gnd1(0 to 0),
      M_AXI_IP_BRESP => net_gnd2(0 to 1),
      M_AXI_IP_BVALID => net_gnd0,
      M_AXI_IP_BREADY => open,
      M_AXI_IP_ARID => open,
      M_AXI_IP_ARADDR => open,
      M_AXI_IP_ARLEN => open,
      M_AXI_IP_ARSIZE => open,
      M_AXI_IP_ARBURST => open,
      M_AXI_IP_ARLOCK => open,
      M_AXI_IP_ARCACHE => open,
      M_AXI_IP_ARPROT => open,
      M_AXI_IP_ARQOS => open,
      M_AXI_IP_ARVALID => open,
      M_AXI_IP_ARREADY => net_gnd0,
      M_AXI_IP_RID => net_gnd1(0 to 0),
      M_AXI_IP_RDATA => net_gnd32(0 to 31),
      M_AXI_IP_RRESP => net_gnd2(0 to 1),
      M_AXI_IP_RLAST => net_gnd0,
      M_AXI_IP_RVALID => net_gnd0,
      M_AXI_IP_RREADY => open,
      M_AXI_DP_AWID => axi4lite_0_S_AWID(0 to 0),
      M_AXI_DP_AWADDR => axi4lite_0_S_AWADDR,
      M_AXI_DP_AWLEN => axi4lite_0_S_AWLEN,
      M_AXI_DP_AWSIZE => axi4lite_0_S_AWSIZE,
      M_AXI_DP_AWBURST => axi4lite_0_S_AWBURST,
      M_AXI_DP_AWLOCK => axi4lite_0_S_AWLOCK(0),
      M_AXI_DP_AWCACHE => axi4lite_0_S_AWCACHE,
      M_AXI_DP_AWPROT => axi4lite_0_S_AWPROT,
      M_AXI_DP_AWQOS => axi4lite_0_S_AWQOS,
      M_AXI_DP_AWVALID => axi4lite_0_S_AWVALID(0),
      M_AXI_DP_AWREADY => axi4lite_0_S_AWREADY(0),
      M_AXI_DP_WDATA => axi4lite_0_S_WDATA,
      M_AXI_DP_WSTRB => axi4lite_0_S_WSTRB,
      M_AXI_DP_WLAST => axi4lite_0_S_WLAST(0),
      M_AXI_DP_WVALID => axi4lite_0_S_WVALID(0),
      M_AXI_DP_WREADY => axi4lite_0_S_WREADY(0),
      M_AXI_DP_BID => axi4lite_0_S_BID(0 downto 0),
      M_AXI_DP_BRESP => axi4lite_0_S_BRESP,
      M_AXI_DP_BVALID => axi4lite_0_S_BVALID(0),
      M_AXI_DP_BREADY => axi4lite_0_S_BREADY(0),
      M_AXI_DP_ARID => axi4lite_0_S_ARID(0 to 0),
      M_AXI_DP_ARADDR => axi4lite_0_S_ARADDR,
      M_AXI_DP_ARLEN => axi4lite_0_S_ARLEN,
      M_AXI_DP_ARSIZE => axi4lite_0_S_ARSIZE,
      M_AXI_DP_ARBURST => axi4lite_0_S_ARBURST,
      M_AXI_DP_ARLOCK => axi4lite_0_S_ARLOCK(0),
      M_AXI_DP_ARCACHE => axi4lite_0_S_ARCACHE,
      M_AXI_DP_ARPROT => axi4lite_0_S_ARPROT,
      M_AXI_DP_ARQOS => axi4lite_0_S_ARQOS,
      M_AXI_DP_ARVALID => axi4lite_0_S_ARVALID(0),
      M_AXI_DP_ARREADY => axi4lite_0_S_ARREADY(0),
      M_AXI_DP_RID => axi4lite_0_S_RID(0 downto 0),
      M_AXI_DP_RDATA => axi4lite_0_S_RDATA,
      M_AXI_DP_RRESP => axi4lite_0_S_RRESP,
      M_AXI_DP_RLAST => axi4lite_0_S_RLAST(0),
      M_AXI_DP_RVALID => axi4lite_0_S_RVALID(0),
      M_AXI_DP_RREADY => axi4lite_0_S_RREADY(0),
      M_AXI_IC_AWID => axi4_0_S_AWID(3 downto 3),
      M_AXI_IC_AWADDR => axi4_0_S_AWADDR(63 downto 32),
      M_AXI_IC_AWLEN => axi4_0_S_AWLEN(15 downto 8),
      M_AXI_IC_AWSIZE => axi4_0_S_AWSIZE(5 downto 3),
      M_AXI_IC_AWBURST => axi4_0_S_AWBURST(3 downto 2),
      M_AXI_IC_AWLOCK => axi4_0_S_AWLOCK(2),
      M_AXI_IC_AWCACHE => axi4_0_S_AWCACHE(7 downto 4),
      M_AXI_IC_AWPROT => axi4_0_S_AWPROT(5 downto 3),
      M_AXI_IC_AWQOS => axi4_0_S_AWQOS(7 downto 4),
      M_AXI_IC_AWVALID => axi4_0_S_AWVALID(1),
      M_AXI_IC_AWREADY => axi4_0_S_AWREADY(1),
      M_AXI_IC_AWUSER => axi4_0_S_AWUSER(9 downto 5),
      M_AXI_IC_WDATA => axi4_0_S_WDATA(63 downto 32),
      M_AXI_IC_WSTRB => axi4_0_S_WSTRB(7 downto 4),
      M_AXI_IC_WLAST => axi4_0_S_WLAST(1),
      M_AXI_IC_WVALID => axi4_0_S_WVALID(1),
      M_AXI_IC_WREADY => axi4_0_S_WREADY(1),
      M_AXI_IC_WUSER => axi4_0_S_WUSER(1 downto 1),
      M_AXI_IC_BID => axi4_0_S_BID(3 downto 3),
      M_AXI_IC_BRESP => axi4_0_S_BRESP(3 downto 2),
      M_AXI_IC_BVALID => axi4_0_S_BVALID(1),
      M_AXI_IC_BREADY => axi4_0_S_BREADY(1),
      M_AXI_IC_BUSER => axi4_0_S_BUSER(1 downto 1),
      M_AXI_IC_ARID => axi4_0_S_ARID(3 downto 3),
      M_AXI_IC_ARADDR => axi4_0_S_ARADDR(63 downto 32),
      M_AXI_IC_ARLEN => axi4_0_S_ARLEN(15 downto 8),
      M_AXI_IC_ARSIZE => axi4_0_S_ARSIZE(5 downto 3),
      M_AXI_IC_ARBURST => axi4_0_S_ARBURST(3 downto 2),
      M_AXI_IC_ARLOCK => axi4_0_S_ARLOCK(2),
      M_AXI_IC_ARCACHE => axi4_0_S_ARCACHE(7 downto 4),
      M_AXI_IC_ARPROT => axi4_0_S_ARPROT(5 downto 3),
      M_AXI_IC_ARQOS => axi4_0_S_ARQOS(7 downto 4),
      M_AXI_IC_ARVALID => axi4_0_S_ARVALID(1),
      M_AXI_IC_ARREADY => axi4_0_S_ARREADY(1),
      M_AXI_IC_ARUSER => axi4_0_S_ARUSER(9 downto 5),
      M_AXI_IC_RID => axi4_0_S_RID(3 downto 3),
      M_AXI_IC_RDATA => axi4_0_S_RDATA(63 downto 32),
      M_AXI_IC_RRESP => axi4_0_S_RRESP(3 downto 2),
      M_AXI_IC_RLAST => axi4_0_S_RLAST(1),
      M_AXI_IC_RVALID => axi4_0_S_RVALID(1),
      M_AXI_IC_RREADY => axi4_0_S_RREADY(1),
      M_AXI_IC_RUSER => axi4_0_S_RUSER(1 downto 1),
      M_AXI_DC_AWID => axi4_0_S_AWID(0 downto 0),
      M_AXI_DC_AWADDR => axi4_0_S_AWADDR(31 downto 0),
      M_AXI_DC_AWLEN => axi4_0_S_AWLEN(7 downto 0),
      M_AXI_DC_AWSIZE => axi4_0_S_AWSIZE(2 downto 0),
      M_AXI_DC_AWBURST => axi4_0_S_AWBURST(1 downto 0),
      M_AXI_DC_AWLOCK => axi4_0_S_AWLOCK(0),
      M_AXI_DC_AWCACHE => axi4_0_S_AWCACHE(3 downto 0),
      M_AXI_DC_AWPROT => axi4_0_S_AWPROT(2 downto 0),
      M_AXI_DC_AWQOS => axi4_0_S_AWQOS(3 downto 0),
      M_AXI_DC_AWVALID => axi4_0_S_AWVALID(0),
      M_AXI_DC_AWREADY => axi4_0_S_AWREADY(0),
      M_AXI_DC_AWUSER => axi4_0_S_AWUSER(4 downto 0),
      M_AXI_DC_WDATA => axi4_0_S_WDATA(31 downto 0),
      M_AXI_DC_WSTRB => axi4_0_S_WSTRB(3 downto 0),
      M_AXI_DC_WLAST => axi4_0_S_WLAST(0),
      M_AXI_DC_WVALID => axi4_0_S_WVALID(0),
      M_AXI_DC_WREADY => axi4_0_S_WREADY(0),
      M_AXI_DC_WUSER => axi4_0_S_WUSER(0 downto 0),
      M_AXI_DC_BID => axi4_0_S_BID(0 downto 0),
      M_AXI_DC_BRESP => axi4_0_S_BRESP(1 downto 0),
      M_AXI_DC_BVALID => axi4_0_S_BVALID(0),
      M_AXI_DC_BREADY => axi4_0_S_BREADY(0),
      M_AXI_DC_BUSER => axi4_0_S_BUSER(0 downto 0),
      M_AXI_DC_ARID => axi4_0_S_ARID(0 downto 0),
      M_AXI_DC_ARADDR => axi4_0_S_ARADDR(31 downto 0),
      M_AXI_DC_ARLEN => axi4_0_S_ARLEN(7 downto 0),
      M_AXI_DC_ARSIZE => axi4_0_S_ARSIZE(2 downto 0),
      M_AXI_DC_ARBURST => axi4_0_S_ARBURST(1 downto 0),
      M_AXI_DC_ARLOCK => axi4_0_S_ARLOCK(0),
      M_AXI_DC_ARCACHE => axi4_0_S_ARCACHE(3 downto 0),
      M_AXI_DC_ARPROT => axi4_0_S_ARPROT(2 downto 0),
      M_AXI_DC_ARQOS => axi4_0_S_ARQOS(3 downto 0),
      M_AXI_DC_ARVALID => axi4_0_S_ARVALID(0),
      M_AXI_DC_ARREADY => axi4_0_S_ARREADY(0),
      M_AXI_DC_ARUSER => axi4_0_S_ARUSER(4 downto 0),
      M_AXI_DC_RID => axi4_0_S_RID(0 downto 0),
      M_AXI_DC_RDATA => axi4_0_S_RDATA(31 downto 0),
      M_AXI_DC_RRESP => axi4_0_S_RRESP(1 downto 0),
      M_AXI_DC_RLAST => axi4_0_S_RLAST(0),
      M_AXI_DC_RVALID => axi4_0_S_RVALID(0),
      M_AXI_DC_RREADY => axi4_0_S_RREADY(0),
      M_AXI_DC_RUSER => axi4_0_S_RUSER(0 downto 0),
      DBG_CLK => microblaze_0_debug_Dbg_Clk,
      DBG_TDI => microblaze_0_debug_Dbg_TDI,
      DBG_TDO => microblaze_0_debug_Dbg_TDO,
      DBG_REG_EN => microblaze_0_debug_Dbg_Reg_En,
      DBG_SHIFT => microblaze_0_debug_Dbg_Shift,
      DBG_CAPTURE => microblaze_0_debug_Dbg_Capture,
      DBG_UPDATE => microblaze_0_debug_Dbg_Update,
      DEBUG_RST => microblaze_0_debug_Debug_Rst,
      Trace_Instruction => open,
      Trace_Valid_Instr => open,
      Trace_PC => open,
      Trace_Reg_Write => open,
      Trace_Reg_Addr => open,
      Trace_MSR_Reg => open,
      Trace_PID_Reg => open,
      Trace_New_Reg_Value => open,
      Trace_Exception_Taken => open,
      Trace_Exception_Kind => open,
      Trace_Jump_Taken => open,
      Trace_Delay_Slot => open,
      Trace_Data_Address => open,
      Trace_Data_Access => open,
      Trace_Data_Read => open,
      Trace_Data_Write => open,
      Trace_Data_Write_Value => open,
      Trace_Data_Byte_Enable => open,
      Trace_DCache_Req => open,
      Trace_DCache_Hit => open,
      Trace_DCache_Rdy => open,
      Trace_DCache_Read => open,
      Trace_ICache_Req => open,
      Trace_ICache_Hit => open,
      Trace_ICache_Rdy => open,
      Trace_OF_PipeRun => open,
      Trace_EX_PipeRun => open,
      Trace_MEM_PipeRun => open,
      Trace_MB_Halted => open,
      Trace_Jump_Hit => open,
      FSL0_S_CLK => open,
      FSL0_S_READ => open,
      FSL0_S_DATA => net_gnd32,
      FSL0_S_CONTROL => net_gnd0,
      FSL0_S_EXISTS => net_gnd0,
      FSL0_M_CLK => open,
      FSL0_M_WRITE => open,
      FSL0_M_DATA => open,
      FSL0_M_CONTROL => open,
      FSL0_M_FULL => net_gnd0,
      FSL1_S_CLK => open,
      FSL1_S_READ => open,
      FSL1_S_DATA => net_gnd32,
      FSL1_S_CONTROL => net_gnd0,
      FSL1_S_EXISTS => net_gnd0,
      FSL1_M_CLK => open,
      FSL1_M_WRITE => open,
      FSL1_M_DATA => open,
      FSL1_M_CONTROL => open,
      FSL1_M_FULL => net_gnd0,
      FSL2_S_CLK => open,
      FSL2_S_READ => open,
      FSL2_S_DATA => net_gnd32,
      FSL2_S_CONTROL => net_gnd0,
      FSL2_S_EXISTS => net_gnd0,
      FSL2_M_CLK => open,
      FSL2_M_WRITE => open,
      FSL2_M_DATA => open,
      FSL2_M_CONTROL => open,
      FSL2_M_FULL => net_gnd0,
      FSL3_S_CLK => open,
      FSL3_S_READ => open,
      FSL3_S_DATA => net_gnd32,
      FSL3_S_CONTROL => net_gnd0,
      FSL3_S_EXISTS => net_gnd0,
      FSL3_M_CLK => open,
      FSL3_M_WRITE => open,
      FSL3_M_DATA => open,
      FSL3_M_CONTROL => open,
      FSL3_M_FULL => net_gnd0,
      FSL4_S_CLK => open,
      FSL4_S_READ => open,
      FSL4_S_DATA => net_gnd32,
      FSL4_S_CONTROL => net_gnd0,
      FSL4_S_EXISTS => net_gnd0,
      FSL4_M_CLK => open,
      FSL4_M_WRITE => open,
      FSL4_M_DATA => open,
      FSL4_M_CONTROL => open,
      FSL4_M_FULL => net_gnd0,
      FSL5_S_CLK => open,
      FSL5_S_READ => open,
      FSL5_S_DATA => net_gnd32,
      FSL5_S_CONTROL => net_gnd0,
      FSL5_S_EXISTS => net_gnd0,
      FSL5_M_CLK => open,
      FSL5_M_WRITE => open,
      FSL5_M_DATA => open,
      FSL5_M_CONTROL => open,
      FSL5_M_FULL => net_gnd0,
      FSL6_S_CLK => open,
      FSL6_S_READ => open,
      FSL6_S_DATA => net_gnd32,
      FSL6_S_CONTROL => net_gnd0,
      FSL6_S_EXISTS => net_gnd0,
      FSL6_M_CLK => open,
      FSL6_M_WRITE => open,
      FSL6_M_DATA => open,
      FSL6_M_CONTROL => open,
      FSL6_M_FULL => net_gnd0,
      FSL7_S_CLK => open,
      FSL7_S_READ => open,
      FSL7_S_DATA => net_gnd32,
      FSL7_S_CONTROL => net_gnd0,
      FSL7_S_EXISTS => net_gnd0,
      FSL7_M_CLK => open,
      FSL7_M_WRITE => open,
      FSL7_M_DATA => open,
      FSL7_M_CONTROL => open,
      FSL7_M_FULL => net_gnd0,
      FSL8_S_CLK => open,
      FSL8_S_READ => open,
      FSL8_S_DATA => net_gnd32,
      FSL8_S_CONTROL => net_gnd0,
      FSL8_S_EXISTS => net_gnd0,
      FSL8_M_CLK => open,
      FSL8_M_WRITE => open,
      FSL8_M_DATA => open,
      FSL8_M_CONTROL => open,
      FSL8_M_FULL => net_gnd0,
      FSL9_S_CLK => open,
      FSL9_S_READ => open,
      FSL9_S_DATA => net_gnd32,
      FSL9_S_CONTROL => net_gnd0,
      FSL9_S_EXISTS => net_gnd0,
      FSL9_M_CLK => open,
      FSL9_M_WRITE => open,
      FSL9_M_DATA => open,
      FSL9_M_CONTROL => open,
      FSL9_M_FULL => net_gnd0,
      FSL10_S_CLK => open,
      FSL10_S_READ => open,
      FSL10_S_DATA => net_gnd32,
      FSL10_S_CONTROL => net_gnd0,
      FSL10_S_EXISTS => net_gnd0,
      FSL10_M_CLK => open,
      FSL10_M_WRITE => open,
      FSL10_M_DATA => open,
      FSL10_M_CONTROL => open,
      FSL10_M_FULL => net_gnd0,
      FSL11_S_CLK => open,
      FSL11_S_READ => open,
      FSL11_S_DATA => net_gnd32,
      FSL11_S_CONTROL => net_gnd0,
      FSL11_S_EXISTS => net_gnd0,
      FSL11_M_CLK => open,
      FSL11_M_WRITE => open,
      FSL11_M_DATA => open,
      FSL11_M_CONTROL => open,
      FSL11_M_FULL => net_gnd0,
      FSL12_S_CLK => open,
      FSL12_S_READ => open,
      FSL12_S_DATA => net_gnd32,
      FSL12_S_CONTROL => net_gnd0,
      FSL12_S_EXISTS => net_gnd0,
      FSL12_M_CLK => open,
      FSL12_M_WRITE => open,
      FSL12_M_DATA => open,
      FSL12_M_CONTROL => open,
      FSL12_M_FULL => net_gnd0,
      FSL13_S_CLK => open,
      FSL13_S_READ => open,
      FSL13_S_DATA => net_gnd32,
      FSL13_S_CONTROL => net_gnd0,
      FSL13_S_EXISTS => net_gnd0,
      FSL13_M_CLK => open,
      FSL13_M_WRITE => open,
      FSL13_M_DATA => open,
      FSL13_M_CONTROL => open,
      FSL13_M_FULL => net_gnd0,
      FSL14_S_CLK => open,
      FSL14_S_READ => open,
      FSL14_S_DATA => net_gnd32,
      FSL14_S_CONTROL => net_gnd0,
      FSL14_S_EXISTS => net_gnd0,
      FSL14_M_CLK => open,
      FSL14_M_WRITE => open,
      FSL14_M_DATA => open,
      FSL14_M_CONTROL => open,
      FSL14_M_FULL => net_gnd0,
      FSL15_S_CLK => open,
      FSL15_S_READ => open,
      FSL15_S_DATA => net_gnd32,
      FSL15_S_CONTROL => net_gnd0,
      FSL15_S_EXISTS => net_gnd0,
      FSL15_M_CLK => open,
      FSL15_M_WRITE => open,
      FSL15_M_DATA => open,
      FSL15_M_CONTROL => open,
      FSL15_M_FULL => net_gnd0,
      M0_AXIS_TLAST => open,
      M0_AXIS_TDATA => open,
      M0_AXIS_TVALID => open,
      M0_AXIS_TREADY => net_gnd0,
      S0_AXIS_TLAST => net_gnd0,
      S0_AXIS_TDATA => net_gnd32(0 to 31),
      S0_AXIS_TVALID => net_gnd0,
      S0_AXIS_TREADY => open,
      M1_AXIS_TLAST => open,
      M1_AXIS_TDATA => open,
      M1_AXIS_TVALID => open,
      M1_AXIS_TREADY => net_gnd0,
      S1_AXIS_TLAST => net_gnd0,
      S1_AXIS_TDATA => net_gnd32(0 to 31),
      S1_AXIS_TVALID => net_gnd0,
      S1_AXIS_TREADY => open,
      M2_AXIS_TLAST => open,
      M2_AXIS_TDATA => open,
      M2_AXIS_TVALID => open,
      M2_AXIS_TREADY => net_gnd0,
      S2_AXIS_TLAST => net_gnd0,
      S2_AXIS_TDATA => net_gnd32(0 to 31),
      S2_AXIS_TVALID => net_gnd0,
      S2_AXIS_TREADY => open,
      M3_AXIS_TLAST => open,
      M3_AXIS_TDATA => open,
      M3_AXIS_TVALID => open,
      M3_AXIS_TREADY => net_gnd0,
      S3_AXIS_TLAST => net_gnd0,
      S3_AXIS_TDATA => net_gnd32(0 to 31),
      S3_AXIS_TVALID => net_gnd0,
      S3_AXIS_TREADY => open,
      M4_AXIS_TLAST => open,
      M4_AXIS_TDATA => open,
      M4_AXIS_TVALID => open,
      M4_AXIS_TREADY => net_gnd0,
      S4_AXIS_TLAST => net_gnd0,
      S4_AXIS_TDATA => net_gnd32(0 to 31),
      S4_AXIS_TVALID => net_gnd0,
      S4_AXIS_TREADY => open,
      M5_AXIS_TLAST => open,
      M5_AXIS_TDATA => open,
      M5_AXIS_TVALID => open,
      M5_AXIS_TREADY => net_gnd0,
      S5_AXIS_TLAST => net_gnd0,
      S5_AXIS_TDATA => net_gnd32(0 to 31),
      S5_AXIS_TVALID => net_gnd0,
      S5_AXIS_TREADY => open,
      M6_AXIS_TLAST => open,
      M6_AXIS_TDATA => open,
      M6_AXIS_TVALID => open,
      M6_AXIS_TREADY => net_gnd0,
      S6_AXIS_TLAST => net_gnd0,
      S6_AXIS_TDATA => net_gnd32(0 to 31),
      S6_AXIS_TVALID => net_gnd0,
      S6_AXIS_TREADY => open,
      M7_AXIS_TLAST => open,
      M7_AXIS_TDATA => open,
      M7_AXIS_TVALID => open,
      M7_AXIS_TREADY => net_gnd0,
      S7_AXIS_TLAST => net_gnd0,
      S7_AXIS_TDATA => net_gnd32(0 to 31),
      S7_AXIS_TVALID => net_gnd0,
      S7_AXIS_TREADY => open,
      M8_AXIS_TLAST => open,
      M8_AXIS_TDATA => open,
      M8_AXIS_TVALID => open,
      M8_AXIS_TREADY => net_gnd0,
      S8_AXIS_TLAST => net_gnd0,
      S8_AXIS_TDATA => net_gnd32(0 to 31),
      S8_AXIS_TVALID => net_gnd0,
      S8_AXIS_TREADY => open,
      M9_AXIS_TLAST => open,
      M9_AXIS_TDATA => open,
      M9_AXIS_TVALID => open,
      M9_AXIS_TREADY => net_gnd0,
      S9_AXIS_TLAST => net_gnd0,
      S9_AXIS_TDATA => net_gnd32(0 to 31),
      S9_AXIS_TVALID => net_gnd0,
      S9_AXIS_TREADY => open,
      M10_AXIS_TLAST => open,
      M10_AXIS_TDATA => open,
      M10_AXIS_TVALID => open,
      M10_AXIS_TREADY => net_gnd0,
      S10_AXIS_TLAST => net_gnd0,
      S10_AXIS_TDATA => net_gnd32(0 to 31),
      S10_AXIS_TVALID => net_gnd0,
      S10_AXIS_TREADY => open,
      M11_AXIS_TLAST => open,
      M11_AXIS_TDATA => open,
      M11_AXIS_TVALID => open,
      M11_AXIS_TREADY => net_gnd0,
      S11_AXIS_TLAST => net_gnd0,
      S11_AXIS_TDATA => net_gnd32(0 to 31),
      S11_AXIS_TVALID => net_gnd0,
      S11_AXIS_TREADY => open,
      M12_AXIS_TLAST => open,
      M12_AXIS_TDATA => open,
      M12_AXIS_TVALID => open,
      M12_AXIS_TREADY => net_gnd0,
      S12_AXIS_TLAST => net_gnd0,
      S12_AXIS_TDATA => net_gnd32(0 to 31),
      S12_AXIS_TVALID => net_gnd0,
      S12_AXIS_TREADY => open,
      M13_AXIS_TLAST => open,
      M13_AXIS_TDATA => open,
      M13_AXIS_TVALID => open,
      M13_AXIS_TREADY => net_gnd0,
      S13_AXIS_TLAST => net_gnd0,
      S13_AXIS_TDATA => net_gnd32(0 to 31),
      S13_AXIS_TVALID => net_gnd0,
      S13_AXIS_TREADY => open,
      M14_AXIS_TLAST => open,
      M14_AXIS_TDATA => open,
      M14_AXIS_TVALID => open,
      M14_AXIS_TREADY => net_gnd0,
      S14_AXIS_TLAST => net_gnd0,
      S14_AXIS_TDATA => net_gnd32(0 to 31),
      S14_AXIS_TVALID => net_gnd0,
      S14_AXIS_TREADY => open,
      M15_AXIS_TLAST => open,
      M15_AXIS_TDATA => open,
      M15_AXIS_TVALID => open,
      M15_AXIS_TREADY => net_gnd0,
      S15_AXIS_TLAST => net_gnd0,
      S15_AXIS_TDATA => net_gnd32(0 to 31),
      S15_AXIS_TVALID => net_gnd0,
      S15_AXIS_TREADY => open,
      ICACHE_FSL_IN_CLK => open,
      ICACHE_FSL_IN_READ => open,
      ICACHE_FSL_IN_DATA => net_gnd32,
      ICACHE_FSL_IN_CONTROL => net_gnd0,
      ICACHE_FSL_IN_EXISTS => net_gnd0,
      ICACHE_FSL_OUT_CLK => open,
      ICACHE_FSL_OUT_WRITE => open,
      ICACHE_FSL_OUT_DATA => open,
      ICACHE_FSL_OUT_CONTROL => open,
      ICACHE_FSL_OUT_FULL => net_gnd0,
      DCACHE_FSL_IN_CLK => open,
      DCACHE_FSL_IN_READ => open,
      DCACHE_FSL_IN_DATA => net_gnd32,
      DCACHE_FSL_IN_CONTROL => net_gnd0,
      DCACHE_FSL_IN_EXISTS => net_gnd0,
      DCACHE_FSL_OUT_CLK => open,
      DCACHE_FSL_OUT_WRITE => open,
      DCACHE_FSL_OUT_DATA => open,
      DCACHE_FSL_OUT_CONTROL => open,
      DCACHE_FSL_OUT_FULL => net_gnd0
    );

  debug_module : system_debug_module_wrapper
    port map (
      Interrupt => open,
      Debug_SYS_Rst => proc_sys_reset_0_MB_Debug_Sys_Rst,
      Ext_BRK => Ext_BRK,
      Ext_NM_BRK => Ext_NM_BRK,
      S_AXI_ACLK => pgassign7(7),
      S_AXI_ARESETN => axi4lite_0_M_ARESETN(1),
      S_AXI_AWADDR => axi4lite_0_M_AWADDR(63 downto 32),
      S_AXI_AWVALID => axi4lite_0_M_AWVALID(1),
      S_AXI_AWREADY => axi4lite_0_M_AWREADY(1),
      S_AXI_WDATA => axi4lite_0_M_WDATA(63 downto 32),
      S_AXI_WSTRB => axi4lite_0_M_WSTRB(7 downto 4),
      S_AXI_WVALID => axi4lite_0_M_WVALID(1),
      S_AXI_WREADY => axi4lite_0_M_WREADY(1),
      S_AXI_BRESP => axi4lite_0_M_BRESP(3 downto 2),
      S_AXI_BVALID => axi4lite_0_M_BVALID(1),
      S_AXI_BREADY => axi4lite_0_M_BREADY(1),
      S_AXI_ARADDR => axi4lite_0_M_ARADDR(63 downto 32),
      S_AXI_ARVALID => axi4lite_0_M_ARVALID(1),
      S_AXI_ARREADY => axi4lite_0_M_ARREADY(1),
      S_AXI_RDATA => axi4lite_0_M_RDATA(63 downto 32),
      S_AXI_RRESP => axi4lite_0_M_RRESP(3 downto 2),
      S_AXI_RVALID => axi4lite_0_M_RVALID(1),
      S_AXI_RREADY => axi4lite_0_M_RREADY(1),
      SPLB_Clk => net_gnd0,
      SPLB_Rst => net_gnd0,
      PLB_ABus => net_gnd32,
      PLB_UABus => net_gnd32,
      PLB_PAValid => net_gnd0,
      PLB_SAValid => net_gnd0,
      PLB_rdPrim => net_gnd0,
      PLB_wrPrim => net_gnd0,
      PLB_masterID => net_gnd3,
      PLB_abort => net_gnd0,
      PLB_busLock => net_gnd0,
      PLB_RNW => net_gnd0,
      PLB_BE => net_gnd4,
      PLB_MSize => net_gnd2,
      PLB_size => net_gnd4,
      PLB_type => net_gnd3,
      PLB_lockErr => net_gnd0,
      PLB_wrDBus => net_gnd32,
      PLB_wrBurst => net_gnd0,
      PLB_rdBurst => net_gnd0,
      PLB_wrPendReq => net_gnd0,
      PLB_rdPendReq => net_gnd0,
      PLB_wrPendPri => net_gnd2,
      PLB_rdPendPri => net_gnd2,
      PLB_reqPri => net_gnd2,
      PLB_TAttribute => net_gnd16,
      Sl_addrAck => open,
      Sl_SSize => open,
      Sl_wait => open,
      Sl_rearbitrate => open,
      Sl_wrDAck => open,
      Sl_wrComp => open,
      Sl_wrBTerm => open,
      Sl_rdDBus => open,
      Sl_rdWdAddr => open,
      Sl_rdDAck => open,
      Sl_rdComp => open,
      Sl_rdBTerm => open,
      Sl_MBusy => open,
      Sl_MWrErr => open,
      Sl_MRdErr => open,
      Sl_MIRQ => open,
      Dbg_Clk_0 => microblaze_0_debug_Dbg_Clk,
      Dbg_TDI_0 => microblaze_0_debug_Dbg_TDI,
      Dbg_TDO_0 => microblaze_0_debug_Dbg_TDO,
      Dbg_Reg_En_0 => microblaze_0_debug_Dbg_Reg_En,
      Dbg_Capture_0 => microblaze_0_debug_Dbg_Capture,
      Dbg_Shift_0 => microblaze_0_debug_Dbg_Shift,
      Dbg_Update_0 => microblaze_0_debug_Dbg_Update,
      Dbg_Rst_0 => microblaze_0_debug_Debug_Rst,
      Dbg_Clk_1 => open,
      Dbg_TDI_1 => open,
      Dbg_TDO_1 => net_gnd0,
      Dbg_Reg_En_1 => open,
      Dbg_Capture_1 => open,
      Dbg_Shift_1 => open,
      Dbg_Update_1 => open,
      Dbg_Rst_1 => open,
      Dbg_Clk_2 => open,
      Dbg_TDI_2 => open,
      Dbg_TDO_2 => net_gnd0,
      Dbg_Reg_En_2 => open,
      Dbg_Capture_2 => open,
      Dbg_Shift_2 => open,
      Dbg_Update_2 => open,
      Dbg_Rst_2 => open,
      Dbg_Clk_3 => open,
      Dbg_TDI_3 => open,
      Dbg_TDO_3 => net_gnd0,
      Dbg_Reg_En_3 => open,
      Dbg_Capture_3 => open,
      Dbg_Shift_3 => open,
      Dbg_Update_3 => open,
      Dbg_Rst_3 => open,
      Dbg_Clk_4 => open,
      Dbg_TDI_4 => open,
      Dbg_TDO_4 => net_gnd0,
      Dbg_Reg_En_4 => open,
      Dbg_Capture_4 => open,
      Dbg_Shift_4 => open,
      Dbg_Update_4 => open,
      Dbg_Rst_4 => open,
      Dbg_Clk_5 => open,
      Dbg_TDI_5 => open,
      Dbg_TDO_5 => net_gnd0,
      Dbg_Reg_En_5 => open,
      Dbg_Capture_5 => open,
      Dbg_Shift_5 => open,
      Dbg_Update_5 => open,
      Dbg_Rst_5 => open,
      Dbg_Clk_6 => open,
      Dbg_TDI_6 => open,
      Dbg_TDO_6 => net_gnd0,
      Dbg_Reg_En_6 => open,
      Dbg_Capture_6 => open,
      Dbg_Shift_6 => open,
      Dbg_Update_6 => open,
      Dbg_Rst_6 => open,
      Dbg_Clk_7 => open,
      Dbg_TDI_7 => open,
      Dbg_TDO_7 => net_gnd0,
      Dbg_Reg_En_7 => open,
      Dbg_Capture_7 => open,
      Dbg_Shift_7 => open,
      Dbg_Update_7 => open,
      Dbg_Rst_7 => open,
      Dbg_Clk_8 => open,
      Dbg_TDI_8 => open,
      Dbg_TDO_8 => net_gnd0,
      Dbg_Reg_En_8 => open,
      Dbg_Capture_8 => open,
      Dbg_Shift_8 => open,
      Dbg_Update_8 => open,
      Dbg_Rst_8 => open,
      Dbg_Clk_9 => open,
      Dbg_TDI_9 => open,
      Dbg_TDO_9 => net_gnd0,
      Dbg_Reg_En_9 => open,
      Dbg_Capture_9 => open,
      Dbg_Shift_9 => open,
      Dbg_Update_9 => open,
      Dbg_Rst_9 => open,
      Dbg_Clk_10 => open,
      Dbg_TDI_10 => open,
      Dbg_TDO_10 => net_gnd0,
      Dbg_Reg_En_10 => open,
      Dbg_Capture_10 => open,
      Dbg_Shift_10 => open,
      Dbg_Update_10 => open,
      Dbg_Rst_10 => open,
      Dbg_Clk_11 => open,
      Dbg_TDI_11 => open,
      Dbg_TDO_11 => net_gnd0,
      Dbg_Reg_En_11 => open,
      Dbg_Capture_11 => open,
      Dbg_Shift_11 => open,
      Dbg_Update_11 => open,
      Dbg_Rst_11 => open,
      Dbg_Clk_12 => open,
      Dbg_TDI_12 => open,
      Dbg_TDO_12 => net_gnd0,
      Dbg_Reg_En_12 => open,
      Dbg_Capture_12 => open,
      Dbg_Shift_12 => open,
      Dbg_Update_12 => open,
      Dbg_Rst_12 => open,
      Dbg_Clk_13 => open,
      Dbg_TDI_13 => open,
      Dbg_TDO_13 => net_gnd0,
      Dbg_Reg_En_13 => open,
      Dbg_Capture_13 => open,
      Dbg_Shift_13 => open,
      Dbg_Update_13 => open,
      Dbg_Rst_13 => open,
      Dbg_Clk_14 => open,
      Dbg_TDI_14 => open,
      Dbg_TDO_14 => net_gnd0,
      Dbg_Reg_En_14 => open,
      Dbg_Capture_14 => open,
      Dbg_Shift_14 => open,
      Dbg_Update_14 => open,
      Dbg_Rst_14 => open,
      Dbg_Clk_15 => open,
      Dbg_TDI_15 => open,
      Dbg_TDO_15 => net_gnd0,
      Dbg_Reg_En_15 => open,
      Dbg_Capture_15 => open,
      Dbg_Shift_15 => open,
      Dbg_Update_15 => open,
      Dbg_Rst_15 => open,
      Dbg_Clk_16 => open,
      Dbg_TDI_16 => open,
      Dbg_TDO_16 => net_gnd0,
      Dbg_Reg_En_16 => open,
      Dbg_Capture_16 => open,
      Dbg_Shift_16 => open,
      Dbg_Update_16 => open,
      Dbg_Rst_16 => open,
      Dbg_Clk_17 => open,
      Dbg_TDI_17 => open,
      Dbg_TDO_17 => net_gnd0,
      Dbg_Reg_En_17 => open,
      Dbg_Capture_17 => open,
      Dbg_Shift_17 => open,
      Dbg_Update_17 => open,
      Dbg_Rst_17 => open,
      Dbg_Clk_18 => open,
      Dbg_TDI_18 => open,
      Dbg_TDO_18 => net_gnd0,
      Dbg_Reg_En_18 => open,
      Dbg_Capture_18 => open,
      Dbg_Shift_18 => open,
      Dbg_Update_18 => open,
      Dbg_Rst_18 => open,
      Dbg_Clk_19 => open,
      Dbg_TDI_19 => open,
      Dbg_TDO_19 => net_gnd0,
      Dbg_Reg_En_19 => open,
      Dbg_Capture_19 => open,
      Dbg_Shift_19 => open,
      Dbg_Update_19 => open,
      Dbg_Rst_19 => open,
      Dbg_Clk_20 => open,
      Dbg_TDI_20 => open,
      Dbg_TDO_20 => net_gnd0,
      Dbg_Reg_En_20 => open,
      Dbg_Capture_20 => open,
      Dbg_Shift_20 => open,
      Dbg_Update_20 => open,
      Dbg_Rst_20 => open,
      Dbg_Clk_21 => open,
      Dbg_TDI_21 => open,
      Dbg_TDO_21 => net_gnd0,
      Dbg_Reg_En_21 => open,
      Dbg_Capture_21 => open,
      Dbg_Shift_21 => open,
      Dbg_Update_21 => open,
      Dbg_Rst_21 => open,
      Dbg_Clk_22 => open,
      Dbg_TDI_22 => open,
      Dbg_TDO_22 => net_gnd0,
      Dbg_Reg_En_22 => open,
      Dbg_Capture_22 => open,
      Dbg_Shift_22 => open,
      Dbg_Update_22 => open,
      Dbg_Rst_22 => open,
      Dbg_Clk_23 => open,
      Dbg_TDI_23 => open,
      Dbg_TDO_23 => net_gnd0,
      Dbg_Reg_En_23 => open,
      Dbg_Capture_23 => open,
      Dbg_Shift_23 => open,
      Dbg_Update_23 => open,
      Dbg_Rst_23 => open,
      Dbg_Clk_24 => open,
      Dbg_TDI_24 => open,
      Dbg_TDO_24 => net_gnd0,
      Dbg_Reg_En_24 => open,
      Dbg_Capture_24 => open,
      Dbg_Shift_24 => open,
      Dbg_Update_24 => open,
      Dbg_Rst_24 => open,
      Dbg_Clk_25 => open,
      Dbg_TDI_25 => open,
      Dbg_TDO_25 => net_gnd0,
      Dbg_Reg_En_25 => open,
      Dbg_Capture_25 => open,
      Dbg_Shift_25 => open,
      Dbg_Update_25 => open,
      Dbg_Rst_25 => open,
      Dbg_Clk_26 => open,
      Dbg_TDI_26 => open,
      Dbg_TDO_26 => net_gnd0,
      Dbg_Reg_En_26 => open,
      Dbg_Capture_26 => open,
      Dbg_Shift_26 => open,
      Dbg_Update_26 => open,
      Dbg_Rst_26 => open,
      Dbg_Clk_27 => open,
      Dbg_TDI_27 => open,
      Dbg_TDO_27 => net_gnd0,
      Dbg_Reg_En_27 => open,
      Dbg_Capture_27 => open,
      Dbg_Shift_27 => open,
      Dbg_Update_27 => open,
      Dbg_Rst_27 => open,
      Dbg_Clk_28 => open,
      Dbg_TDI_28 => open,
      Dbg_TDO_28 => net_gnd0,
      Dbg_Reg_En_28 => open,
      Dbg_Capture_28 => open,
      Dbg_Shift_28 => open,
      Dbg_Update_28 => open,
      Dbg_Rst_28 => open,
      Dbg_Clk_29 => open,
      Dbg_TDI_29 => open,
      Dbg_TDO_29 => net_gnd0,
      Dbg_Reg_En_29 => open,
      Dbg_Capture_29 => open,
      Dbg_Shift_29 => open,
      Dbg_Update_29 => open,
      Dbg_Rst_29 => open,
      Dbg_Clk_30 => open,
      Dbg_TDI_30 => open,
      Dbg_TDO_30 => net_gnd0,
      Dbg_Reg_En_30 => open,
      Dbg_Capture_30 => open,
      Dbg_Shift_30 => open,
      Dbg_Update_30 => open,
      Dbg_Rst_30 => open,
      Dbg_Clk_31 => open,
      Dbg_TDI_31 => open,
      Dbg_TDO_31 => net_gnd0,
      Dbg_Reg_En_31 => open,
      Dbg_Capture_31 => open,
      Dbg_Shift_31 => open,
      Dbg_Update_31 => open,
      Dbg_Rst_31 => open,
      bscan_tdi => open,
      bscan_reset => open,
      bscan_shift => open,
      bscan_update => open,
      bscan_capture => open,
      bscan_sel1 => open,
      bscan_drck1 => open,
      bscan_tdo1 => net_gnd0,
      bscan_ext_tdi => net_gnd0,
      bscan_ext_reset => net_gnd0,
      bscan_ext_shift => net_gnd0,
      bscan_ext_update => net_gnd0,
      bscan_ext_capture => net_gnd0,
      bscan_ext_sel => net_gnd0,
      bscan_ext_drck => net_gnd0,
      bscan_ext_tdo => open,
      Ext_JTAG_DRCK => open,
      Ext_JTAG_RESET => open,
      Ext_JTAG_SEL => open,
      Ext_JTAG_CAPTURE => open,
      Ext_JTAG_SHIFT => open,
      Ext_JTAG_UPDATE => open,
      Ext_JTAG_TDI => open,
      Ext_JTAG_TDO => net_gnd0
    );

  clock_generator_0 : system_clock_generator_0_wrapper
    port map (
      CLKIN => CLK,
      CLKOUT0 => clk_100_0000MHzMMCM0(0),
      CLKOUT1 => clk_125_0000MHz,
      CLKOUT2 => clk_200_0000MHzMMCM0(0),
      CLKOUT3 => clk_400_0000MHzMMCM0,
      CLKOUT4 => clk_400_0000MHzMMCM0_nobuf_varphase,
      CLKOUT5 => open,
      CLKOUT6 => open,
      CLKOUT7 => open,
      CLKOUT8 => open,
      CLKOUT9 => open,
      CLKOUT10 => open,
      CLKOUT11 => open,
      CLKOUT12 => open,
      CLKOUT13 => open,
      CLKOUT14 => open,
      CLKOUT15 => open,
      CLKFBIN => net_gnd0,
      CLKFBOUT => open,
      PSCLK => clk_200_0000MHzMMCM0(0),
      PSEN => psen,
      PSINCDEC => psincdec,
      PSDONE => psdone,
      RST => RESET,
      LOCKED => proc_sys_reset_0_Dcm_locked
    );

  axi_timer_0 : system_axi_timer_0_wrapper
    port map (
      CaptureTrig0 => net_gnd0,
      CaptureTrig1 => net_gnd0,
      GenerateOut0 => open,
      GenerateOut1 => open,
      PWM0 => open,
      Interrupt => axi_timer_0_Interrupt,
      Freeze => net_gnd0,
      S_AXI_ACLK => pgassign7(7),
      S_AXI_ARESETN => axi4lite_0_M_ARESETN(2),
      S_AXI_AWADDR => axi4lite_0_M_AWADDR(68 downto 64),
      S_AXI_AWVALID => axi4lite_0_M_AWVALID(2),
      S_AXI_AWREADY => axi4lite_0_M_AWREADY(2),
      S_AXI_WDATA => axi4lite_0_M_WDATA(95 downto 64),
      S_AXI_WSTRB => axi4lite_0_M_WSTRB(11 downto 8),
      S_AXI_WVALID => axi4lite_0_M_WVALID(2),
      S_AXI_WREADY => axi4lite_0_M_WREADY(2),
      S_AXI_BRESP => axi4lite_0_M_BRESP(5 downto 4),
      S_AXI_BVALID => axi4lite_0_M_BVALID(2),
      S_AXI_BREADY => axi4lite_0_M_BREADY(2),
      S_AXI_ARADDR => axi4lite_0_M_ARADDR(68 downto 64),
      S_AXI_ARVALID => axi4lite_0_M_ARVALID(2),
      S_AXI_ARREADY => axi4lite_0_M_ARREADY(2),
      S_AXI_RDATA => axi4lite_0_M_RDATA(95 downto 64),
      S_AXI_RRESP => axi4lite_0_M_RRESP(5 downto 4),
      S_AXI_RVALID => axi4lite_0_M_RVALID(2),
      S_AXI_RREADY => axi4lite_0_M_RREADY(2)
    );

  axi4lite_0 : system_axi4lite_0_wrapper
    port map (
      INTERCONNECT_ACLK => pgassign7(7),
      INTERCONNECT_ARESETN => proc_sys_reset_0_Interconnect_aresetn(0),
      S_AXI_ARESET_OUT_N => open,
      M_AXI_ARESET_OUT_N => axi4lite_0_M_ARESETN,
      IRQ => open,
      S_AXI_ACLK => pgassign7(7 downto 7),
      S_AXI_AWID => axi4lite_0_S_AWID(0 to 0),
      S_AXI_AWADDR => axi4lite_0_S_AWADDR,
      S_AXI_AWLEN => axi4lite_0_S_AWLEN,
      S_AXI_AWSIZE => axi4lite_0_S_AWSIZE,
      S_AXI_AWBURST => axi4lite_0_S_AWBURST,
      S_AXI_AWLOCK => axi4lite_0_S_AWLOCK,
      S_AXI_AWCACHE => axi4lite_0_S_AWCACHE,
      S_AXI_AWPROT => axi4lite_0_S_AWPROT,
      S_AXI_AWQOS => axi4lite_0_S_AWQOS,
      S_AXI_AWUSER => net_gnd1(0 to 0),
      S_AXI_AWVALID => axi4lite_0_S_AWVALID(0 to 0),
      S_AXI_AWREADY => axi4lite_0_S_AWREADY(0 to 0),
      S_AXI_WID => net_gnd1(0 to 0),
      S_AXI_WDATA => axi4lite_0_S_WDATA,
      S_AXI_WSTRB => axi4lite_0_S_WSTRB,
      S_AXI_WLAST => axi4lite_0_S_WLAST(0 to 0),
      S_AXI_WUSER => net_gnd1(0 to 0),
      S_AXI_WVALID => axi4lite_0_S_WVALID(0 to 0),
      S_AXI_WREADY => axi4lite_0_S_WREADY(0 to 0),
      S_AXI_BID => axi4lite_0_S_BID(0 downto 0),
      S_AXI_BRESP => axi4lite_0_S_BRESP,
      S_AXI_BUSER => open,
      S_AXI_BVALID => axi4lite_0_S_BVALID(0 to 0),
      S_AXI_BREADY => axi4lite_0_S_BREADY(0 to 0),
      S_AXI_ARID => axi4lite_0_S_ARID(0 to 0),
      S_AXI_ARADDR => axi4lite_0_S_ARADDR,
      S_AXI_ARLEN => axi4lite_0_S_ARLEN,
      S_AXI_ARSIZE => axi4lite_0_S_ARSIZE,
      S_AXI_ARBURST => axi4lite_0_S_ARBURST,
      S_AXI_ARLOCK => axi4lite_0_S_ARLOCK,
      S_AXI_ARCACHE => axi4lite_0_S_ARCACHE,
      S_AXI_ARPROT => axi4lite_0_S_ARPROT,
      S_AXI_ARQOS => axi4lite_0_S_ARQOS,
      S_AXI_ARUSER => net_gnd1(0 to 0),
      S_AXI_ARVALID => axi4lite_0_S_ARVALID(0 to 0),
      S_AXI_ARREADY => axi4lite_0_S_ARREADY(0 to 0),
      S_AXI_RID => axi4lite_0_S_RID(0 downto 0),
      S_AXI_RDATA => axi4lite_0_S_RDATA,
      S_AXI_RRESP => axi4lite_0_S_RRESP,
      S_AXI_RLAST => axi4lite_0_S_RLAST(0 to 0),
      S_AXI_RUSER => open,
      S_AXI_RVALID => axi4lite_0_S_RVALID(0 to 0),
      S_AXI_RREADY => axi4lite_0_S_RREADY(0 to 0),
      M_AXI_ACLK => pgassign7,
      M_AXI_AWID => open,
      M_AXI_AWADDR => axi4lite_0_M_AWADDR,
      M_AXI_AWLEN => open,
      M_AXI_AWSIZE => open,
      M_AXI_AWBURST => open,
      M_AXI_AWLOCK => open,
      M_AXI_AWCACHE => open,
      M_AXI_AWPROT => open,
      M_AXI_AWREGION => open,
      M_AXI_AWQOS => open,
      M_AXI_AWUSER => open,
      M_AXI_AWVALID => axi4lite_0_M_AWVALID,
      M_AXI_AWREADY => axi4lite_0_M_AWREADY,
      M_AXI_WID => open,
      M_AXI_WDATA => axi4lite_0_M_WDATA,
      M_AXI_WSTRB => axi4lite_0_M_WSTRB,
      M_AXI_WLAST => open,
      M_AXI_WUSER => open,
      M_AXI_WVALID => axi4lite_0_M_WVALID,
      M_AXI_WREADY => axi4lite_0_M_WREADY,
      M_AXI_BID => net_gnd8,
      M_AXI_BRESP => axi4lite_0_M_BRESP,
      M_AXI_BUSER => net_gnd8,
      M_AXI_BVALID => axi4lite_0_M_BVALID,
      M_AXI_BREADY => axi4lite_0_M_BREADY,
      M_AXI_ARID => open,
      M_AXI_ARADDR => axi4lite_0_M_ARADDR,
      M_AXI_ARLEN => open,
      M_AXI_ARSIZE => open,
      M_AXI_ARBURST => open,
      M_AXI_ARLOCK => open,
      M_AXI_ARCACHE => open,
      M_AXI_ARPROT => open,
      M_AXI_ARREGION => open,
      M_AXI_ARQOS => open,
      M_AXI_ARUSER => open,
      M_AXI_ARVALID => axi4lite_0_M_ARVALID,
      M_AXI_ARREADY => axi4lite_0_M_ARREADY,
      M_AXI_RID => net_gnd8,
      M_AXI_RDATA => axi4lite_0_M_RDATA,
      M_AXI_RRESP => axi4lite_0_M_RRESP,
      M_AXI_RLAST => net_gnd8,
      M_AXI_RUSER => net_gnd8,
      M_AXI_RVALID => axi4lite_0_M_RVALID,
      M_AXI_RREADY => axi4lite_0_M_RREADY,
      S_AXI_CTRL_AWADDR => net_gnd32(0 to 31),
      S_AXI_CTRL_AWVALID => net_gnd0,
      S_AXI_CTRL_AWREADY => open,
      S_AXI_CTRL_WDATA => net_gnd32(0 to 31),
      S_AXI_CTRL_WVALID => net_gnd0,
      S_AXI_CTRL_WREADY => open,
      S_AXI_CTRL_BRESP => open,
      S_AXI_CTRL_BVALID => open,
      S_AXI_CTRL_BREADY => net_gnd0,
      S_AXI_CTRL_ARADDR => net_gnd32(0 to 31),
      S_AXI_CTRL_ARVALID => net_gnd0,
      S_AXI_CTRL_ARREADY => open,
      S_AXI_CTRL_RDATA => open,
      S_AXI_CTRL_RRESP => open,
      S_AXI_CTRL_RVALID => open,
      S_AXI_CTRL_RREADY => net_gnd0,
      INTERCONNECT_ARESET_OUT_N => open,
      DEBUG_AW_TRANS_SEQ => open,
      DEBUG_AW_ARB_GRANT => open,
      DEBUG_AR_TRANS_SEQ => open,
      DEBUG_AR_ARB_GRANT => open,
      DEBUG_AW_TRANS_QUAL => open,
      DEBUG_AW_ACCEPT_CNT => open,
      DEBUG_AW_ACTIVE_THREAD => open,
      DEBUG_AW_ACTIVE_TARGET => open,
      DEBUG_AW_ACTIVE_REGION => open,
      DEBUG_AW_ERROR => open,
      DEBUG_AW_TARGET => open,
      DEBUG_AR_TRANS_QUAL => open,
      DEBUG_AR_ACCEPT_CNT => open,
      DEBUG_AR_ACTIVE_THREAD => open,
      DEBUG_AR_ACTIVE_TARGET => open,
      DEBUG_AR_ACTIVE_REGION => open,
      DEBUG_AR_ERROR => open,
      DEBUG_AR_TARGET => open,
      DEBUG_B_TRANS_SEQ => open,
      DEBUG_R_BEAT_CNT => open,
      DEBUG_R_TRANS_SEQ => open,
      DEBUG_AW_ISSUING_CNT => open,
      DEBUG_AR_ISSUING_CNT => open,
      DEBUG_W_BEAT_CNT => open,
      DEBUG_W_TRANS_SEQ => open,
      DEBUG_BID_TARGET => open,
      DEBUG_BID_ERROR => open,
      DEBUG_RID_TARGET => open,
      DEBUG_RID_ERROR => open,
      DEBUG_SR_SC_ARADDR => open,
      DEBUG_SR_SC_ARADDRCONTROL => open,
      DEBUG_SR_SC_AWADDR => open,
      DEBUG_SR_SC_AWADDRCONTROL => open,
      DEBUG_SR_SC_BRESP => open,
      DEBUG_SR_SC_RDATA => open,
      DEBUG_SR_SC_RDATACONTROL => open,
      DEBUG_SR_SC_WDATA => open,
      DEBUG_SR_SC_WDATACONTROL => open,
      DEBUG_SC_SF_ARADDR => open,
      DEBUG_SC_SF_ARADDRCONTROL => open,
      DEBUG_SC_SF_AWADDR => open,
      DEBUG_SC_SF_AWADDRCONTROL => open,
      DEBUG_SC_SF_BRESP => open,
      DEBUG_SC_SF_RDATA => open,
      DEBUG_SC_SF_RDATACONTROL => open,
      DEBUG_SC_SF_WDATA => open,
      DEBUG_SC_SF_WDATACONTROL => open,
      DEBUG_SF_CB_ARADDR => open,
      DEBUG_SF_CB_ARADDRCONTROL => open,
      DEBUG_SF_CB_AWADDR => open,
      DEBUG_SF_CB_AWADDRCONTROL => open,
      DEBUG_SF_CB_BRESP => open,
      DEBUG_SF_CB_RDATA => open,
      DEBUG_SF_CB_RDATACONTROL => open,
      DEBUG_SF_CB_WDATA => open,
      DEBUG_SF_CB_WDATACONTROL => open,
      DEBUG_CB_MF_ARADDR => open,
      DEBUG_CB_MF_ARADDRCONTROL => open,
      DEBUG_CB_MF_AWADDR => open,
      DEBUG_CB_MF_AWADDRCONTROL => open,
      DEBUG_CB_MF_BRESP => open,
      DEBUG_CB_MF_RDATA => open,
      DEBUG_CB_MF_RDATACONTROL => open,
      DEBUG_CB_MF_WDATA => open,
      DEBUG_CB_MF_WDATACONTROL => open,
      DEBUG_MF_MC_ARADDR => open,
      DEBUG_MF_MC_ARADDRCONTROL => open,
      DEBUG_MF_MC_AWADDR => open,
      DEBUG_MF_MC_AWADDRCONTROL => open,
      DEBUG_MF_MC_BRESP => open,
      DEBUG_MF_MC_RDATA => open,
      DEBUG_MF_MC_RDATACONTROL => open,
      DEBUG_MF_MC_WDATA => open,
      DEBUG_MF_MC_WDATACONTROL => open,
      DEBUG_MC_MP_ARADDR => open,
      DEBUG_MC_MP_ARADDRCONTROL => open,
      DEBUG_MC_MP_AWADDR => open,
      DEBUG_MC_MP_AWADDRCONTROL => open,
      DEBUG_MC_MP_BRESP => open,
      DEBUG_MC_MP_RDATA => open,
      DEBUG_MC_MP_RDATACONTROL => open,
      DEBUG_MC_MP_WDATA => open,
      DEBUG_MC_MP_WDATACONTROL => open,
      DEBUG_MP_MR_ARADDR => open,
      DEBUG_MP_MR_ARADDRCONTROL => open,
      DEBUG_MP_MR_AWADDR => open,
      DEBUG_MP_MR_AWADDRCONTROL => open,
      DEBUG_MP_MR_BRESP => open,
      DEBUG_MP_MR_RDATA => open,
      DEBUG_MP_MR_RDATACONTROL => open,
      DEBUG_MP_MR_WDATA => open,
      DEBUG_MP_MR_WDATACONTROL => open
    );

  axi4_0 : system_axi4_0_wrapper
    port map (
      INTERCONNECT_ACLK => pgassign7(7),
      INTERCONNECT_ARESETN => proc_sys_reset_0_Interconnect_aresetn(0),
      S_AXI_ARESET_OUT_N => open,
      M_AXI_ARESET_OUT_N => axi4_0_M_ARESETN(0 to 0),
      IRQ => open,
      S_AXI_ACLK => pgassign8,
      S_AXI_AWID => axi4_0_S_AWID,
      S_AXI_AWADDR => axi4_0_S_AWADDR,
      S_AXI_AWLEN => axi4_0_S_AWLEN,
      S_AXI_AWSIZE => axi4_0_S_AWSIZE,
      S_AXI_AWBURST => axi4_0_S_AWBURST,
      S_AXI_AWLOCK => axi4_0_S_AWLOCK,
      S_AXI_AWCACHE => axi4_0_S_AWCACHE,
      S_AXI_AWPROT => axi4_0_S_AWPROT,
      S_AXI_AWQOS => axi4_0_S_AWQOS,
      S_AXI_AWUSER => axi4_0_S_AWUSER,
      S_AXI_AWVALID => axi4_0_S_AWVALID,
      S_AXI_AWREADY => axi4_0_S_AWREADY,
      S_AXI_WID => net_gnd15,
      S_AXI_WDATA => axi4_0_S_WDATA,
      S_AXI_WSTRB => axi4_0_S_WSTRB,
      S_AXI_WLAST => axi4_0_S_WLAST,
      S_AXI_WUSER => axi4_0_S_WUSER,
      S_AXI_WVALID => axi4_0_S_WVALID,
      S_AXI_WREADY => axi4_0_S_WREADY,
      S_AXI_BID => axi4_0_S_BID,
      S_AXI_BRESP => axi4_0_S_BRESP,
      S_AXI_BUSER => axi4_0_S_BUSER,
      S_AXI_BVALID => axi4_0_S_BVALID,
      S_AXI_BREADY => axi4_0_S_BREADY,
      S_AXI_ARID => axi4_0_S_ARID,
      S_AXI_ARADDR => axi4_0_S_ARADDR,
      S_AXI_ARLEN => axi4_0_S_ARLEN,
      S_AXI_ARSIZE => axi4_0_S_ARSIZE,
      S_AXI_ARBURST => axi4_0_S_ARBURST,
      S_AXI_ARLOCK => axi4_0_S_ARLOCK,
      S_AXI_ARCACHE => axi4_0_S_ARCACHE,
      S_AXI_ARPROT => axi4_0_S_ARPROT,
      S_AXI_ARQOS => axi4_0_S_ARQOS,
      S_AXI_ARUSER => axi4_0_S_ARUSER,
      S_AXI_ARVALID => axi4_0_S_ARVALID,
      S_AXI_ARREADY => axi4_0_S_ARREADY,
      S_AXI_RID => axi4_0_S_RID,
      S_AXI_RDATA => axi4_0_S_RDATA,
      S_AXI_RRESP => axi4_0_S_RRESP,
      S_AXI_RLAST => axi4_0_S_RLAST,
      S_AXI_RUSER => axi4_0_S_RUSER,
      S_AXI_RVALID => axi4_0_S_RVALID,
      S_AXI_RREADY => axi4_0_S_RREADY,
      M_AXI_ACLK => clk_200_0000MHzMMCM0(0 to 0),
      M_AXI_AWID => axi4_0_M_AWID,
      M_AXI_AWADDR => axi4_0_M_AWADDR,
      M_AXI_AWLEN => axi4_0_M_AWLEN,
      M_AXI_AWSIZE => axi4_0_M_AWSIZE,
      M_AXI_AWBURST => axi4_0_M_AWBURST,
      M_AXI_AWLOCK => axi4_0_M_AWLOCK,
      M_AXI_AWCACHE => axi4_0_M_AWCACHE,
      M_AXI_AWPROT => axi4_0_M_AWPROT,
      M_AXI_AWREGION => open,
      M_AXI_AWQOS => axi4_0_M_AWQOS,
      M_AXI_AWUSER => open,
      M_AXI_AWVALID => axi4_0_M_AWVALID(0 to 0),
      M_AXI_AWREADY => axi4_0_M_AWREADY(0 to 0),
      M_AXI_WID => open,
      M_AXI_WDATA => axi4_0_M_WDATA,
      M_AXI_WSTRB => axi4_0_M_WSTRB,
      M_AXI_WLAST => axi4_0_M_WLAST(0 to 0),
      M_AXI_WUSER => open,
      M_AXI_WVALID => axi4_0_M_WVALID(0 to 0),
      M_AXI_WREADY => axi4_0_M_WREADY(0 to 0),
      M_AXI_BID => axi4_0_M_BID,
      M_AXI_BRESP => axi4_0_M_BRESP,
      M_AXI_BUSER => net_gnd1(0 to 0),
      M_AXI_BVALID => axi4_0_M_BVALID(0 to 0),
      M_AXI_BREADY => axi4_0_M_BREADY(0 to 0),
      M_AXI_ARID => axi4_0_M_ARID,
      M_AXI_ARADDR => axi4_0_M_ARADDR,
      M_AXI_ARLEN => axi4_0_M_ARLEN,
      M_AXI_ARSIZE => axi4_0_M_ARSIZE,
      M_AXI_ARBURST => axi4_0_M_ARBURST,
      M_AXI_ARLOCK => axi4_0_M_ARLOCK,
      M_AXI_ARCACHE => axi4_0_M_ARCACHE,
      M_AXI_ARPROT => axi4_0_M_ARPROT,
      M_AXI_ARREGION => open,
      M_AXI_ARQOS => axi4_0_M_ARQOS,
      M_AXI_ARUSER => open,
      M_AXI_ARVALID => axi4_0_M_ARVALID(0 to 0),
      M_AXI_ARREADY => axi4_0_M_ARREADY(0 to 0),
      M_AXI_RID => axi4_0_M_RID,
      M_AXI_RDATA => axi4_0_M_RDATA,
      M_AXI_RRESP => axi4_0_M_RRESP,
      M_AXI_RLAST => axi4_0_M_RLAST(0 to 0),
      M_AXI_RUSER => net_gnd1(0 to 0),
      M_AXI_RVALID => axi4_0_M_RVALID(0 to 0),
      M_AXI_RREADY => axi4_0_M_RREADY(0 to 0),
      S_AXI_CTRL_AWADDR => net_gnd32(0 to 31),
      S_AXI_CTRL_AWVALID => net_gnd0,
      S_AXI_CTRL_AWREADY => open,
      S_AXI_CTRL_WDATA => net_gnd32(0 to 31),
      S_AXI_CTRL_WVALID => net_gnd0,
      S_AXI_CTRL_WREADY => open,
      S_AXI_CTRL_BRESP => open,
      S_AXI_CTRL_BVALID => open,
      S_AXI_CTRL_BREADY => net_gnd0,
      S_AXI_CTRL_ARADDR => net_gnd32(0 to 31),
      S_AXI_CTRL_ARVALID => net_gnd0,
      S_AXI_CTRL_ARREADY => open,
      S_AXI_CTRL_RDATA => open,
      S_AXI_CTRL_RRESP => open,
      S_AXI_CTRL_RVALID => open,
      S_AXI_CTRL_RREADY => net_gnd0,
      INTERCONNECT_ARESET_OUT_N => open,
      DEBUG_AW_TRANS_SEQ => open,
      DEBUG_AW_ARB_GRANT => open,
      DEBUG_AR_TRANS_SEQ => open,
      DEBUG_AR_ARB_GRANT => open,
      DEBUG_AW_TRANS_QUAL => open,
      DEBUG_AW_ACCEPT_CNT => open,
      DEBUG_AW_ACTIVE_THREAD => open,
      DEBUG_AW_ACTIVE_TARGET => open,
      DEBUG_AW_ACTIVE_REGION => open,
      DEBUG_AW_ERROR => open,
      DEBUG_AW_TARGET => open,
      DEBUG_AR_TRANS_QUAL => open,
      DEBUG_AR_ACCEPT_CNT => open,
      DEBUG_AR_ACTIVE_THREAD => open,
      DEBUG_AR_ACTIVE_TARGET => open,
      DEBUG_AR_ACTIVE_REGION => open,
      DEBUG_AR_ERROR => open,
      DEBUG_AR_TARGET => open,
      DEBUG_B_TRANS_SEQ => open,
      DEBUG_R_BEAT_CNT => open,
      DEBUG_R_TRANS_SEQ => open,
      DEBUG_AW_ISSUING_CNT => open,
      DEBUG_AR_ISSUING_CNT => open,
      DEBUG_W_BEAT_CNT => open,
      DEBUG_W_TRANS_SEQ => open,
      DEBUG_BID_TARGET => open,
      DEBUG_BID_ERROR => open,
      DEBUG_RID_TARGET => open,
      DEBUG_RID_ERROR => open,
      DEBUG_SR_SC_ARADDR => open,
      DEBUG_SR_SC_ARADDRCONTROL => open,
      DEBUG_SR_SC_AWADDR => open,
      DEBUG_SR_SC_AWADDRCONTROL => open,
      DEBUG_SR_SC_BRESP => open,
      DEBUG_SR_SC_RDATA => open,
      DEBUG_SR_SC_RDATACONTROL => open,
      DEBUG_SR_SC_WDATA => open,
      DEBUG_SR_SC_WDATACONTROL => open,
      DEBUG_SC_SF_ARADDR => open,
      DEBUG_SC_SF_ARADDRCONTROL => open,
      DEBUG_SC_SF_AWADDR => open,
      DEBUG_SC_SF_AWADDRCONTROL => open,
      DEBUG_SC_SF_BRESP => open,
      DEBUG_SC_SF_RDATA => open,
      DEBUG_SC_SF_RDATACONTROL => open,
      DEBUG_SC_SF_WDATA => open,
      DEBUG_SC_SF_WDATACONTROL => open,
      DEBUG_SF_CB_ARADDR => open,
      DEBUG_SF_CB_ARADDRCONTROL => open,
      DEBUG_SF_CB_AWADDR => open,
      DEBUG_SF_CB_AWADDRCONTROL => open,
      DEBUG_SF_CB_BRESP => open,
      DEBUG_SF_CB_RDATA => open,
      DEBUG_SF_CB_RDATACONTROL => open,
      DEBUG_SF_CB_WDATA => open,
      DEBUG_SF_CB_WDATACONTROL => open,
      DEBUG_CB_MF_ARADDR => open,
      DEBUG_CB_MF_ARADDRCONTROL => open,
      DEBUG_CB_MF_AWADDR => open,
      DEBUG_CB_MF_AWADDRCONTROL => open,
      DEBUG_CB_MF_BRESP => open,
      DEBUG_CB_MF_RDATA => open,
      DEBUG_CB_MF_RDATACONTROL => open,
      DEBUG_CB_MF_WDATA => open,
      DEBUG_CB_MF_WDATACONTROL => open,
      DEBUG_MF_MC_ARADDR => open,
      DEBUG_MF_MC_ARADDRCONTROL => open,
      DEBUG_MF_MC_AWADDR => open,
      DEBUG_MF_MC_AWADDRCONTROL => open,
      DEBUG_MF_MC_BRESP => open,
      DEBUG_MF_MC_RDATA => open,
      DEBUG_MF_MC_RDATACONTROL => open,
      DEBUG_MF_MC_WDATA => open,
      DEBUG_MF_MC_WDATACONTROL => open,
      DEBUG_MC_MP_ARADDR => open,
      DEBUG_MC_MP_ARADDRCONTROL => open,
      DEBUG_MC_MP_AWADDR => open,
      DEBUG_MC_MP_AWADDRCONTROL => open,
      DEBUG_MC_MP_BRESP => open,
      DEBUG_MC_MP_RDATA => open,
      DEBUG_MC_MP_RDATACONTROL => open,
      DEBUG_MC_MP_WDATA => open,
      DEBUG_MC_MP_WDATACONTROL => open,
      DEBUG_MP_MR_ARADDR => open,
      DEBUG_MP_MR_ARADDRCONTROL => open,
      DEBUG_MP_MR_AWADDR => open,
      DEBUG_MP_MR_AWADDRCONTROL => open,
      DEBUG_MP_MR_BRESP => open,
      DEBUG_MP_MR_RDATA => open,
      DEBUG_MP_MR_RDATACONTROL => open,
      DEBUG_MP_MR_WDATA => open,
      DEBUG_MP_MR_WDATACONTROL => open
    );

  SysACE_CompactFlash : system_sysace_compactflash_wrapper
    port map (
      S_AXI_ACLK => pgassign7(7),
      S_AXI_ARESETN => axi4lite_0_M_ARESETN(3),
      S_AXI_AWADDR => axi4lite_0_M_AWADDR(127 downto 96),
      S_AXI_AWVALID => axi4lite_0_M_AWVALID(3),
      S_AXI_AWREADY => axi4lite_0_M_AWREADY(3),
      S_AXI_WDATA => axi4lite_0_M_WDATA(127 downto 96),
      S_AXI_WSTRB => axi4lite_0_M_WSTRB(15 downto 12),
      S_AXI_WVALID => axi4lite_0_M_WVALID(3),
      S_AXI_WREADY => axi4lite_0_M_WREADY(3),
      S_AXI_BRESP => axi4lite_0_M_BRESP(7 downto 6),
      S_AXI_BVALID => axi4lite_0_M_BVALID(3),
      S_AXI_BREADY => axi4lite_0_M_BREADY(3),
      S_AXI_ARADDR => axi4lite_0_M_ARADDR(127 downto 96),
      S_AXI_ARVALID => axi4lite_0_M_ARVALID(3),
      S_AXI_ARREADY => axi4lite_0_M_ARREADY(3),
      S_AXI_RDATA => axi4lite_0_M_RDATA(127 downto 96),
      S_AXI_RRESP => axi4lite_0_M_RRESP(7 downto 6),
      S_AXI_RVALID => axi4lite_0_M_RVALID(3),
      S_AXI_RREADY => axi4lite_0_M_RREADY(3),
      SysACE_CLK => SysACE_CLK,
      SysACE_MPIRQ => SysACE_MPIRQ,
      SysACE_MPD_I => SysACE_MPD_I,
      SysACE_MPD_O => SysACE_MPD_O,
      SysACE_MPD_T => SysACE_MPD_T,
      SysACE_MPA => SysACE_MPA,
      SysACE_CEN => SysACE_CEN,
      SysACE_OEN => SysACE_OEN,
      SysACE_WEN => SysACE_WEN,
      SysACE_IRQ => open
    );

  RS232_Uart_1 : system_rs232_uart_1_wrapper
    port map (
      S_AXI_ACLK => pgassign7(7),
      S_AXI_ARESETN => axi4lite_0_M_ARESETN(4),
      Interrupt => open,
      S_AXI_AWADDR => axi4lite_0_M_AWADDR(131 downto 128),
      S_AXI_AWVALID => axi4lite_0_M_AWVALID(4),
      S_AXI_AWREADY => axi4lite_0_M_AWREADY(4),
      S_AXI_WDATA => axi4lite_0_M_WDATA(159 downto 128),
      S_AXI_WSTRB => axi4lite_0_M_WSTRB(19 downto 16),
      S_AXI_WVALID => axi4lite_0_M_WVALID(4),
      S_AXI_WREADY => axi4lite_0_M_WREADY(4),
      S_AXI_BRESP => axi4lite_0_M_BRESP(9 downto 8),
      S_AXI_BVALID => axi4lite_0_M_BVALID(4),
      S_AXI_BREADY => axi4lite_0_M_BREADY(4),
      S_AXI_ARADDR => axi4lite_0_M_ARADDR(131 downto 128),
      S_AXI_ARVALID => axi4lite_0_M_ARVALID(4),
      S_AXI_ARREADY => axi4lite_0_M_ARREADY(4),
      S_AXI_RDATA => axi4lite_0_M_RDATA(159 downto 128),
      S_AXI_RRESP => axi4lite_0_M_RRESP(9 downto 8),
      S_AXI_RVALID => axi4lite_0_M_RVALID(4),
      S_AXI_RREADY => axi4lite_0_M_RREADY(4),
      RX => RS232_Uart_1_sin,
      TX => RS232_Uart_1_sout
    );

  ETHERNET_dma : system_ethernet_dma_wrapper
    port map (
      s_axi_lite_aclk => pgassign7(7),
      m_axi_sg_aclk => pgassign7(7),
      m_axi_mm2s_aclk => pgassign7(7),
      m_axi_s2mm_aclk => pgassign7(7),
      axi_resetn => axi4lite_0_M_ARESETN(5),
      s_axi_lite_awvalid => axi4lite_0_M_AWVALID(5),
      s_axi_lite_awready => axi4lite_0_M_AWREADY(5),
      s_axi_lite_awaddr => axi4lite_0_M_AWADDR(169 downto 160),
      s_axi_lite_wvalid => axi4lite_0_M_WVALID(5),
      s_axi_lite_wready => axi4lite_0_M_WREADY(5),
      s_axi_lite_wdata => axi4lite_0_M_WDATA(191 downto 160),
      s_axi_lite_bresp => axi4lite_0_M_BRESP(11 downto 10),
      s_axi_lite_bvalid => axi4lite_0_M_BVALID(5),
      s_axi_lite_bready => axi4lite_0_M_BREADY(5),
      s_axi_lite_arvalid => axi4lite_0_M_ARVALID(5),
      s_axi_lite_arready => axi4lite_0_M_ARREADY(5),
      s_axi_lite_araddr => axi4lite_0_M_ARADDR(169 downto 160),
      s_axi_lite_rvalid => axi4lite_0_M_RVALID(5),
      s_axi_lite_rready => axi4lite_0_M_RREADY(5),
      s_axi_lite_rdata => axi4lite_0_M_RDATA(191 downto 160),
      s_axi_lite_rresp => axi4lite_0_M_RRESP(11 downto 10),
      m_axi_sg_awaddr => axi4_0_S_AWADDR(95 downto 64),
      m_axi_sg_awlen => axi4_0_S_AWLEN(23 downto 16),
      m_axi_sg_awsize => axi4_0_S_AWSIZE(8 downto 6),
      m_axi_sg_awburst => axi4_0_S_AWBURST(5 downto 4),
      m_axi_sg_awprot => axi4_0_S_AWPROT(8 downto 6),
      m_axi_sg_awcache => axi4_0_S_AWCACHE(11 downto 8),
      m_axi_sg_awuser => axi4_0_S_AWUSER(13 downto 10),
      m_axi_sg_awvalid => axi4_0_S_AWVALID(2),
      m_axi_sg_awready => axi4_0_S_AWREADY(2),
      m_axi_sg_wdata => axi4_0_S_WDATA(95 downto 64),
      m_axi_sg_wstrb => axi4_0_S_WSTRB(11 downto 8),
      m_axi_sg_wlast => axi4_0_S_WLAST(2),
      m_axi_sg_wvalid => axi4_0_S_WVALID(2),
      m_axi_sg_wready => axi4_0_S_WREADY(2),
      m_axi_sg_bresp => axi4_0_S_BRESP(5 downto 4),
      m_axi_sg_bvalid => axi4_0_S_BVALID(2),
      m_axi_sg_bready => axi4_0_S_BREADY(2),
      m_axi_sg_araddr => axi4_0_S_ARADDR(95 downto 64),
      m_axi_sg_arlen => axi4_0_S_ARLEN(23 downto 16),
      m_axi_sg_arsize => axi4_0_S_ARSIZE(8 downto 6),
      m_axi_sg_arburst => axi4_0_S_ARBURST(5 downto 4),
      m_axi_sg_arprot => axi4_0_S_ARPROT(8 downto 6),
      m_axi_sg_arcache => axi4_0_S_ARCACHE(11 downto 8),
      m_axi_sg_aruser => axi4_0_S_ARUSER(13 downto 10),
      m_axi_sg_arvalid => axi4_0_S_ARVALID(2),
      m_axi_sg_arready => axi4_0_S_ARREADY(2),
      m_axi_sg_rdata => axi4_0_S_RDATA(95 downto 64),
      m_axi_sg_rresp => axi4_0_S_RRESP(5 downto 4),
      m_axi_sg_rlast => axi4_0_S_RLAST(2),
      m_axi_sg_rvalid => axi4_0_S_RVALID(2),
      m_axi_sg_rready => axi4_0_S_RREADY(2),
      m_axi_mm2s_araddr => axi4_0_S_ARADDR(127 downto 96),
      m_axi_mm2s_arlen => axi4_0_S_ARLEN(31 downto 24),
      m_axi_mm2s_arsize => axi4_0_S_ARSIZE(11 downto 9),
      m_axi_mm2s_arburst => axi4_0_S_ARBURST(7 downto 6),
      m_axi_mm2s_arprot => axi4_0_S_ARPROT(11 downto 9),
      m_axi_mm2s_arcache => axi4_0_S_ARCACHE(15 downto 12),
      m_axi_mm2s_aruser => axi4_0_S_ARUSER(18 downto 15),
      m_axi_mm2s_arvalid => axi4_0_S_ARVALID(3),
      m_axi_mm2s_arready => axi4_0_S_ARREADY(3),
      m_axi_mm2s_rdata => axi4_0_S_RDATA(127 downto 96),
      m_axi_mm2s_rresp => axi4_0_S_RRESP(7 downto 6),
      m_axi_mm2s_rlast => axi4_0_S_RLAST(3),
      m_axi_mm2s_rvalid => axi4_0_S_RVALID(3),
      m_axi_mm2s_rready => axi4_0_S_RREADY(3),
      mm2s_prmry_reset_out_n => AXI_STR_TXD_ARESETN,
      m_axis_mm2s_tdata => ETHERNET_dma_txd_TDATA,
      m_axis_mm2s_tkeep => ETHERNET_dma_txd_TKEEP,
      m_axis_mm2s_tvalid => ETHERNET_dma_txd_TVALID,
      m_axis_mm2s_tready => ETHERNET_dma_txd_TREADY,
      m_axis_mm2s_tlast => ETHERNET_dma_txd_TLAST,
      m_axis_mm2s_tuser => open,
      m_axis_mm2s_tid => open,
      m_axis_mm2s_tdest => open,
      mm2s_cntrl_reset_out_n => AXI_STR_TXC_ARESETN,
      m_axis_mm2s_cntrl_tdata => ETHERNET_dma_txc_TDATA,
      m_axis_mm2s_cntrl_tkeep => ETHERNET_dma_txc_TKEEP,
      m_axis_mm2s_cntrl_tvalid => ETHERNET_dma_txc_TVALID,
      m_axis_mm2s_cntrl_tready => ETHERNET_dma_txc_TREADY,
      m_axis_mm2s_cntrl_tlast => ETHERNET_dma_txc_TLAST,
      m_axi_s2mm_awaddr => axi4_0_S_AWADDR(159 downto 128),
      m_axi_s2mm_awlen => axi4_0_S_AWLEN(39 downto 32),
      m_axi_s2mm_awsize => axi4_0_S_AWSIZE(14 downto 12),
      m_axi_s2mm_awburst => axi4_0_S_AWBURST(9 downto 8),
      m_axi_s2mm_awprot => axi4_0_S_AWPROT(14 downto 12),
      m_axi_s2mm_awcache => axi4_0_S_AWCACHE(19 downto 16),
      m_axi_s2mm_awuser => axi4_0_S_AWUSER(23 downto 20),
      m_axi_s2mm_awvalid => axi4_0_S_AWVALID(4),
      m_axi_s2mm_awready => axi4_0_S_AWREADY(4),
      m_axi_s2mm_wdata => axi4_0_S_WDATA(159 downto 128),
      m_axi_s2mm_wstrb => axi4_0_S_WSTRB(19 downto 16),
      m_axi_s2mm_wlast => axi4_0_S_WLAST(4),
      m_axi_s2mm_wvalid => axi4_0_S_WVALID(4),
      m_axi_s2mm_wready => axi4_0_S_WREADY(4),
      m_axi_s2mm_bresp => axi4_0_S_BRESP(9 downto 8),
      m_axi_s2mm_bvalid => axi4_0_S_BVALID(4),
      m_axi_s2mm_bready => axi4_0_S_BREADY(4),
      s2mm_prmry_reset_out_n => AXI_STR_RXD_ARESETN,
      s_axis_s2mm_tdata => ETHERNET_dma_rxd_TDATA,
      s_axis_s2mm_tkeep => ETHERNET_dma_rxd_TKEEP,
      s_axis_s2mm_tvalid => ETHERNET_dma_rxd_TVALID,
      s_axis_s2mm_tready => ETHERNET_dma_rxd_TREADY,
      s_axis_s2mm_tlast => ETHERNET_dma_rxd_TLAST,
      s_axis_s2mm_tuser => net_gnd4(0 to 3),
      s_axis_s2mm_tid => net_gnd5,
      s_axis_s2mm_tdest => net_gnd5,
      s2mm_sts_reset_out_n => AXI_STR_RXS_ARESETN,
      s_axis_s2mm_sts_tdata => ETHERNET_dma_rxs_TDATA,
      s_axis_s2mm_sts_tkeep => ETHERNET_dma_rxs_TKEEP,
      s_axis_s2mm_sts_tvalid => ETHERNET_dma_rxs_TVALID,
      s_axis_s2mm_sts_tready => ETHERNET_dma_rxs_TREADY,
      s_axis_s2mm_sts_tlast => ETHERNET_dma_rxs_TLAST,
      mm2s_introut => ETHERNET_dma_mm2s_introut,
      s2mm_introut => ETHERNET_dma_s2mm_introut
    );

  ETHERNET : system_ethernet_wrapper
    port map (
      S_AXI_ACLK => pgassign7(7),
      S_AXI_ARESETN => axi4lite_0_M_ARESETN(6),
      INTERRUPT => ETHERNET_INTERRUPT,
      S_AXI_AWADDR => axi4lite_0_M_AWADDR(223 downto 192),
      S_AXI_AWVALID => axi4lite_0_M_AWVALID(6),
      S_AXI_AWREADY => axi4lite_0_M_AWREADY(6),
      S_AXI_WDATA => axi4lite_0_M_WDATA(223 downto 192),
      S_AXI_WSTRB => axi4lite_0_M_WSTRB(27 downto 24),
      S_AXI_WVALID => axi4lite_0_M_WVALID(6),
      S_AXI_WREADY => axi4lite_0_M_WREADY(6),
      S_AXI_BRESP => axi4lite_0_M_BRESP(13 downto 12),
      S_AXI_BVALID => axi4lite_0_M_BVALID(6),
      S_AXI_BREADY => axi4lite_0_M_BREADY(6),
      S_AXI_ARADDR => axi4lite_0_M_ARADDR(223 downto 192),
      S_AXI_ARVALID => axi4lite_0_M_ARVALID(6),
      S_AXI_ARREADY => axi4lite_0_M_ARREADY(6),
      S_AXI_RDATA => axi4lite_0_M_RDATA(223 downto 192),
      S_AXI_RRESP => axi4lite_0_M_RRESP(13 downto 12),
      S_AXI_RVALID => axi4lite_0_M_RVALID(6),
      S_AXI_RREADY => axi4lite_0_M_RREADY(6),
      AXI_STR_TXD_ACLK => pgassign7(7),
      AXI_STR_TXD_ARESETN => AXI_STR_TXD_ARESETN,
      AXI_STR_TXD_TVALID => ETHERNET_dma_txd_TVALID,
      AXI_STR_TXD_TREADY => ETHERNET_dma_txd_TREADY,
      AXI_STR_TXD_TLAST => ETHERNET_dma_txd_TLAST,
      AXI_STR_TXD_TKEEP => ETHERNET_dma_txd_TKEEP,
      AXI_STR_TXD_TDATA => ETHERNET_dma_txd_TDATA,
      AXI_STR_TXC_ACLK => pgassign7(7),
      AXI_STR_TXC_ARESETN => AXI_STR_TXC_ARESETN,
      AXI_STR_TXC_TVALID => ETHERNET_dma_txc_TVALID,
      AXI_STR_TXC_TREADY => ETHERNET_dma_txc_TREADY,
      AXI_STR_TXC_TLAST => ETHERNET_dma_txc_TLAST,
      AXI_STR_TXC_TKEEP => ETHERNET_dma_txc_TKEEP,
      AXI_STR_TXC_TDATA => ETHERNET_dma_txc_TDATA,
      AXI_STR_RXD_ACLK => pgassign7(7),
      AXI_STR_RXD_ARESETN => AXI_STR_RXD_ARESETN,
      AXI_STR_RXD_TVALID => ETHERNET_dma_rxd_TVALID,
      AXI_STR_RXD_TREADY => ETHERNET_dma_rxd_TREADY,
      AXI_STR_RXD_TLAST => ETHERNET_dma_rxd_TLAST,
      AXI_STR_RXD_TKEEP => ETHERNET_dma_rxd_TKEEP,
      AXI_STR_RXD_TDATA => ETHERNET_dma_rxd_TDATA,
      AXI_STR_RXS_ACLK => pgassign7(7),
      AXI_STR_RXS_ARESETN => AXI_STR_RXS_ARESETN,
      AXI_STR_RXS_TVALID => ETHERNET_dma_rxs_TVALID,
      AXI_STR_RXS_TREADY => ETHERNET_dma_rxs_TREADY,
      AXI_STR_RXS_TLAST => ETHERNET_dma_rxs_TLAST,
      AXI_STR_RXS_TKEEP => ETHERNET_dma_rxs_TKEEP,
      AXI_STR_RXS_TDATA => ETHERNET_dma_rxs_TDATA,
      PHY_RST_N => ETHERNET_PHY_RST_N,
      GTX_CLK => clk_125_0000MHz,
      MGT_CLK_P => net_gnd0,
      MGT_CLK_N => net_gnd0,
      REF_CLK => clk_200_0000MHzMMCM0(0),
      MII_TXD => open,
      MII_TX_EN => open,
      MII_TX_ER => open,
      MII_RXD => net_gnd4(0 to 3),
      MII_RX_DV => net_gnd0,
      MII_RX_ER => net_gnd0,
      MII_RX_CLK => net_gnd0,
      MII_TX_CLK => ETHERNET_MII_TX_CLK,
      MII_COL => net_gnd0,
      MII_CRS => net_gnd0,
      GMII_TXD => ETHERNET_TXD,
      GMII_TX_EN => ETHERNET_TX_EN,
      GMII_TX_ER => ETHERNET_TX_ER,
      GMII_TX_CLK => ETHERNET_TX_CLK,
      GMII_RXD => ETHERNET_RXD,
      GMII_RX_DV => ETHERNET_RX_DV,
      GMII_RX_ER => ETHERNET_RX_ER,
      GMII_RX_CLK => ETHERNET_RX_CLK,
      GMII_COL => net_gnd0,
      GMII_CRS => net_gnd0,
      TXP => open,
      TXN => open,
      RXP => net_vcc0,
      RXN => net_gnd0,
      RGMII_TXD => open,
      RGMII_TX_CTL => open,
      RGMII_TXC => open,
      RGMII_RXD => net_gnd4(0 to 3),
      RGMII_RX_CTL => net_gnd0,
      RGMII_RXC => net_gnd0,
      MDC => ETHERNET_MDC,
      MDIO_I => ETHERNET_MDIO_I,
      MDIO_O => ETHERNET_MDIO_O,
      MDIO_T => ETHERNET_MDIO_T,
      AXI_STR_AVBTX_ACLK => open,
      AXI_STR_AVBTX_ARESETN => net_gnd0,
      AXI_STR_AVBTX_TVALID => net_gnd0,
      AXI_STR_AVBTX_TREADY => open,
      AXI_STR_AVBTX_TLAST => net_gnd0,
      AXI_STR_AVBTX_TDATA => net_gnd8,
      AXI_STR_AVBTX_TUSER => net_vcc1(0 downto 0),
      AXI_STR_AVBRX_ACLK => open,
      AXI_STR_AVBRX_ARESETN => net_gnd0,
      AXI_STR_AVBRX_TVALID => open,
      AXI_STR_AVBRX_TLAST => open,
      AXI_STR_AVBRX_TDATA => open,
      AXI_STR_AVBRX_TUSER => open,
      RTC_CLK => net_gnd0,
      AV_INTERRUPT_10MS => open,
      AV_INTERRUPT_PTP_TX => open,
      AV_INTERRUPT_PTP_RX => open,
      AV_RTC_NANOSECFIELD => open,
      AV_RTC_SECFIELD => open,
      AV_CLK_8K => open,
      AV_RTC_NANOSECFIELD_1722 => open
    );

  DDR3_SDRAM : system_ddr3_sdram_wrapper
    port map (
      clk => clk_200_0000MHzMMCM0(0),
      clk_mem => clk_400_0000MHzMMCM0,
      clk_rd_base => clk_400_0000MHzMMCM0_nobuf_varphase,
      aresetn => axi4_0_M_ARESETN(0),
      clk_ref => clk_200_0000MHzMMCM0(0),
      iodelay_ctrl_rdy_i => net_vcc0,
      iodelay_ctrl_rdy_o => open,
      pd_PSEN => psen,
      pd_PSINCDEC => psincdec,
      pd_PSDONE => psdone,
      phy_init_done => open,
      interrupt => open,
      ddr_addr => ddr_memory_addr,
      ddr_ba => ddr_memory_ba,
      ddr_cas_n => ddr_memory_cas_n,
      ddr_ck_p => pgassign4(0 to 0),
      ddr_ck_n => pgassign3(0 to 0),
      ddr_cke => pgassign5(0 to 0),
      ddr_cs_n => pgassign2(0 to 0),
      ddr_dm => ddr_memory_dm(0 to 0),
      ddr_odt => pgassign1(0 to 0),
      ddr_ras_n => ddr_memory_ras_n,
      ddr_reset_n => ddr_memory_ddr3_rst,
      ddr_parity => open,
      ddr_we_n => ddr_memory_we_n,
      ddr_dq => ddr_memory_dq,
      ddr_dqs_p => ddr_memory_dqs(0 to 0),
      ddr_dqs_n => ddr_memory_dqs_n(0 to 0),
      s_axi_awid => axi4_0_M_AWID,
      s_axi_awaddr => axi4_0_M_AWADDR,
      s_axi_awlen => axi4_0_M_AWLEN,
      s_axi_awsize => axi4_0_M_AWSIZE,
      s_axi_awburst => axi4_0_M_AWBURST,
      s_axi_awlock => axi4_0_M_AWLOCK(0 downto 0),
      s_axi_awcache => axi4_0_M_AWCACHE,
      s_axi_awprot => axi4_0_M_AWPROT,
      s_axi_awqos => axi4_0_M_AWQOS,
      s_axi_awvalid => axi4_0_M_AWVALID(0),
      s_axi_awready => axi4_0_M_AWREADY(0),
      s_axi_wdata => axi4_0_M_WDATA,
      s_axi_wstrb => axi4_0_M_WSTRB,
      s_axi_wlast => axi4_0_M_WLAST(0),
      s_axi_wvalid => axi4_0_M_WVALID(0),
      s_axi_wready => axi4_0_M_WREADY(0),
      s_axi_bid => axi4_0_M_BID,
      s_axi_bresp => axi4_0_M_BRESP,
      s_axi_bvalid => axi4_0_M_BVALID(0),
      s_axi_bready => axi4_0_M_BREADY(0),
      s_axi_arid => axi4_0_M_ARID,
      s_axi_araddr => axi4_0_M_ARADDR,
      s_axi_arlen => axi4_0_M_ARLEN,
      s_axi_arsize => axi4_0_M_ARSIZE,
      s_axi_arburst => axi4_0_M_ARBURST,
      s_axi_arlock => axi4_0_M_ARLOCK(0 downto 0),
      s_axi_arcache => axi4_0_M_ARCACHE,
      s_axi_arprot => axi4_0_M_ARPROT,
      s_axi_arqos => axi4_0_M_ARQOS,
      s_axi_arvalid => axi4_0_M_ARVALID(0),
      s_axi_arready => axi4_0_M_ARREADY(0),
      s_axi_rid => axi4_0_M_RID,
      s_axi_rdata => axi4_0_M_RDATA,
      s_axi_rresp => axi4_0_M_RRESP,
      s_axi_rlast => axi4_0_M_RLAST(0),
      s_axi_rvalid => axi4_0_M_RVALID(0),
      s_axi_rready => axi4_0_M_RREADY(0),
      s_axi_ctrl_awaddr => net_gnd32(0 to 31),
      s_axi_ctrl_awvalid => net_gnd0,
      s_axi_ctrl_awready => open,
      s_axi_ctrl_wdata => net_gnd32(0 to 31),
      s_axi_ctrl_wvalid => net_gnd0,
      s_axi_ctrl_wready => open,
      s_axi_ctrl_bresp => open,
      s_axi_ctrl_bvalid => open,
      s_axi_ctrl_bready => net_gnd0,
      s_axi_ctrl_araddr => net_gnd32(0 to 31),
      s_axi_ctrl_arvalid => net_gnd0,
      s_axi_ctrl_arready => open,
      s_axi_ctrl_rdata => open,
      s_axi_ctrl_rresp => open,
      s_axi_ctrl_rvalid => open,
      s_axi_ctrl_rready => net_gnd0
    );

  controller_0 : system_controller_0_wrapper
    port map (
      S_AXI_ACLK => pgassign7(7),
      S_AXI_ARESETN => axi4lite_0_M_ARESETN(7),
      S_AXI_AWADDR => axi4lite_0_M_AWADDR(255 downto 224),
      S_AXI_AWVALID => axi4lite_0_M_AWVALID(7),
      S_AXI_WDATA => axi4lite_0_M_WDATA(255 downto 224),
      S_AXI_WSTRB => axi4lite_0_M_WSTRB(31 downto 28),
      S_AXI_WVALID => axi4lite_0_M_WVALID(7),
      S_AXI_BREADY => axi4lite_0_M_BREADY(7),
      S_AXI_ARADDR => axi4lite_0_M_ARADDR(255 downto 224),
      S_AXI_ARVALID => axi4lite_0_M_ARVALID(7),
      S_AXI_RREADY => axi4lite_0_M_RREADY(7),
      S_AXI_ARREADY => axi4lite_0_M_ARREADY(7),
      S_AXI_RDATA => axi4lite_0_M_RDATA(255 downto 224),
      S_AXI_RRESP => axi4lite_0_M_RRESP(15 downto 14),
      S_AXI_RVALID => axi4lite_0_M_RVALID(7),
      S_AXI_WREADY => axi4lite_0_M_WREADY(7),
      S_AXI_BRESP => axi4lite_0_M_BRESP(15 downto 14),
      S_AXI_BVALID => axi4lite_0_M_BVALID(7),
      S_AXI_AWREADY => axi4lite_0_M_AWREADY(7)
    );

  iobuf_0 : IOBUF
    port map (
      I => SysACE_MPD_O(7),
      IO => SysACE_MPD(7),
      O => SysACE_MPD_I(7),
      T => SysACE_MPD_T(7)
    );

  iobuf_1 : IOBUF
    port map (
      I => SysACE_MPD_O(6),
      IO => SysACE_MPD(6),
      O => SysACE_MPD_I(6),
      T => SysACE_MPD_T(6)
    );

  iobuf_2 : IOBUF
    port map (
      I => SysACE_MPD_O(5),
      IO => SysACE_MPD(5),
      O => SysACE_MPD_I(5),
      T => SysACE_MPD_T(5)
    );

  iobuf_3 : IOBUF
    port map (
      I => SysACE_MPD_O(4),
      IO => SysACE_MPD(4),
      O => SysACE_MPD_I(4),
      T => SysACE_MPD_T(4)
    );

  iobuf_4 : IOBUF
    port map (
      I => SysACE_MPD_O(3),
      IO => SysACE_MPD(3),
      O => SysACE_MPD_I(3),
      T => SysACE_MPD_T(3)
    );

  iobuf_5 : IOBUF
    port map (
      I => SysACE_MPD_O(2),
      IO => SysACE_MPD(2),
      O => SysACE_MPD_I(2),
      T => SysACE_MPD_T(2)
    );

  iobuf_6 : IOBUF
    port map (
      I => SysACE_MPD_O(1),
      IO => SysACE_MPD(1),
      O => SysACE_MPD_I(1),
      T => SysACE_MPD_T(1)
    );

  iobuf_7 : IOBUF
    port map (
      I => SysACE_MPD_O(0),
      IO => SysACE_MPD(0),
      O => SysACE_MPD_I(0),
      T => SysACE_MPD_T(0)
    );

  iobuf_8 : IOBUF
    port map (
      I => ETHERNET_MDIO_O,
      IO => ETHERNET_MDIO,
      O => ETHERNET_MDIO_I,
      T => ETHERNET_MDIO_T
    );

  ibufgds_9 : IBUFGDS
    port map (
      I => CLK_P,
      IB => CLK_N,
      O => CLK
    );

end architecture STRUCTURE;

