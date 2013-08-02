#
# pin constraints
#
set_property LOC AD11 [ get_ports CLK_N]
set_property IOSTANDARD DIFF_SSTL15 [ get_ports CLK_N]

set_property LOC AD12 [ get_ports CLK_P]
set_property IOSTANDARD DIFF_SSTL15 [ get_ports CLK_P]

set_property LOC Y29 [ get_ports DIP_Switches_TRI_I[0]]
set_property IOSTANDARD LVCMOS25 [ get_ports DIP_Switches_TRI_I[0]]

set_property LOC W29 [ get_ports DIP_Switches_TRI_I[1]]
set_property IOSTANDARD LVCMOS25 [ get_ports DIP_Switches_TRI_I[1]]

set_property LOC AA28 [ get_ports DIP_Switches_TRI_I[2]]
set_property IOSTANDARD LVCMOS25 [ get_ports DIP_Switches_TRI_I[2]]

set_property LOC Y28 [ get_ports DIP_Switches_TRI_I[3]]
set_property IOSTANDARD LVCMOS25 [ get_ports DIP_Switches_TRI_I[3]]

set_property LOC W19 [ get_ports Ethernet_Lite_COL]
set_property IOSTANDARD LVCMOS25 [ get_ports Ethernet_Lite_COL]

set_property LOC R30 [ get_ports Ethernet_Lite_CRS]
set_property IOSTANDARD LVCMOS25 [ get_ports Ethernet_Lite_CRS]

set_property LOC R23 [ get_ports Ethernet_Lite_MDC]
set_property IOSTANDARD LVCMOS25 [ get_ports Ethernet_Lite_MDC]

set_property LOC J21 [ get_ports Ethernet_Lite_MDIO]
set_property IOSTANDARD LVCMOS25 [ get_ports Ethernet_Lite_MDIO]

set_property LOC L20 [ get_ports Ethernet_Lite_PHY_RST_N]
set_property IOSTANDARD LVCMOS25 [ get_ports Ethernet_Lite_PHY_RST_N]
set_false_path -through [get_nets Ethernet_Lite_PHY_RST_N]

set_property LOC U30 [ get_ports Ethernet_Lite_RXD[0]]
set_property IOSTANDARD LVCMOS25 [ get_ports Ethernet_Lite_RXD[0]]

set_property LOC U25 [ get_ports Ethernet_Lite_RXD[1]]
set_property IOSTANDARD LVCMOS25 [ get_ports Ethernet_Lite_RXD[1]]

set_property LOC T25 [ get_ports Ethernet_Lite_RXD[2]]
set_property IOSTANDARD LVCMOS25 [ get_ports Ethernet_Lite_RXD[2]]

set_property LOC U28 [ get_ports Ethernet_Lite_RXD[3]]
set_property IOSTANDARD LVCMOS25 [ get_ports Ethernet_Lite_RXD[3]]

set_property LOC U27 [ get_ports Ethernet_Lite_RX_CLK]
set_property IOSTANDARD LVCMOS25 [ get_ports Ethernet_Lite_RX_CLK]

set_property LOC R28 [ get_ports Ethernet_Lite_RX_DV]
set_property IOSTANDARD LVCMOS25 [ get_ports Ethernet_Lite_RX_DV]

set_property LOC V26 [ get_ports Ethernet_Lite_RX_ER]
set_property IOSTANDARD LVCMOS25 [ get_ports Ethernet_Lite_RX_ER]

set_property LOC N27 [ get_ports Ethernet_Lite_TXD[0]]
set_property IOSTANDARD LVCMOS25 [ get_ports Ethernet_Lite_TXD[0]]

set_property LOC N25 [ get_ports Ethernet_Lite_TXD[1]]
set_property IOSTANDARD LVCMOS25 [ get_ports Ethernet_Lite_TXD[1]]

set_property LOC M29 [ get_ports Ethernet_Lite_TXD[2]]
set_property IOSTANDARD LVCMOS25 [ get_ports Ethernet_Lite_TXD[2]]

set_property LOC L28 [ get_ports Ethernet_Lite_TXD[3]]
set_property IOSTANDARD LVCMOS25 [ get_ports Ethernet_Lite_TXD[3]]

set_property LOC M28 [ get_ports Ethernet_Lite_TX_CLK]
set_property IOSTANDARD LVCMOS25 [ get_ports Ethernet_Lite_TX_CLK]

set_property LOC M27 [ get_ports Ethernet_Lite_TX_EN]
set_property IOSTANDARD LVCMOS25 [ get_ports Ethernet_Lite_TX_EN]

set_property LOC AB7 [ get_ports RESET]
set_property IOSTANDARD LVCMOS15 [ get_ports RESET]

set_property LOC M19 [ get_ports RS232_Uart_1_sin]
set_property IOSTANDARD LVCMOS25 [ get_ports RS232_Uart_1_sin]

set_property LOC K24 [ get_ports RS232_Uart_1_sout]
set_property IOSTANDARD LVCMOS25 [ get_ports RS232_Uart_1_sout]

set_property LOC L26 [ get_ports sm_fan_pwm_net_vcc]
set_property IOSTANDARD LVCMOS25 [ get_ports sm_fan_pwm_net_vcc]

#
# additional constraints
#
create_clock -name sys_clk_pin -period "5.0" [get_nets "CLK"]
