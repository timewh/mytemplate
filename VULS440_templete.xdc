#used as a template of S2C VULS440 platform !
#--------------------------------------------------

#global clk
#Programmable clock pair1 or JS1 clock pair1 U19.22(P)/U19.21(N) JS1.03(P)/JS1.01(N)  C42 (P) C43 (N)	Bank52	LVDS	DIFF_TERM = FALSE					
#Programmable clock pair2 or JS1 clock pair2 U20.22(P)/U20.21(N) JS1.07(P)/JS1.05(N) AK47 (P) AK48 (N)	Bank43	LVDS	DIFF_TERM = FALSE					
#Programmable clock pair3 or JS1 clock pair3 U21.22(P)/U21.21(N) JS1.11(P)/JS1.09(N) BM15 (P) BN14 (N)	Bank60	LVDS	DIFF_TERM = FALSE	 				
#Programmable clock pair4 or JS1 clock pair4 U22.22(P)/U22.21(N) JS1.15(P)/JS1.13(N) BB26 (P) BB27 (N)	Bank65	LVDS	DIFF_TERM = TRUE				
#Programmable clock pair5 or JS1 clock pair5 U23.22(P)/U23.21(N) JS1.19(P)/JS1.17(N) BC29 (P) BD29 (N)	Bank65	LVDS	DIFF_TERM = TRUE			
#Programmable clock pair6 or JS1 clock pair6 U23.18(P)/U23.17(N) JS1.23(P)/JS1.21(N)  Y47 (P) Y48 (N)	Bank45	LVDS	DIFF_TERM = FALSE

  IBUFGDS 
   #(.DIFF_TERM("FALSE"))
   u_ibufgds
    (
      .I(gth_sysclkp_i),
      .IB(gth_sysclkn_i),
      .O(gth_sysclk_i)
    );
	
	  IBUFDS #(
    .DIFF_TERM  ("FALSE"),
    .IOSTANDARD ("LVDS")
  )ibufds_userclk_inst(
    .I          (user_clk_p),
    .IB         (user_clk_n),
    .O          (user_clk_tmp)
  );
  
  BUFG bufg_user_clk(
    .I    (user_clk_tmp),
    .O    (user_clk)
  );

#reference
set_property PACKAGE_PIN C42 [get_ports user_clk_p]
set_property IOSTANDARD LVDS [get_ports user_clk_p]
create_clock -period 10.000 -name sysclk [get_ports user_clk_p]
set_clock_groups -asynchronous -group [get_clocks sysclk -include_generated_clocks]					
	

set_property DIRECTION IN [get_ports clk_i_p]
set_property IOSTANDARD LVDS [get_ports clk_i_p]
set_property DIFF_TERM TRUE [get_ports clk_i_p]
set_property EQUALIZATION EQ_NONE [get_ports clk_i_p]
set_property DIFF_TERM_ADV TERM_100 [get_ports clk_i_p]
set_property LVDS_PRE_EMPHASIS FALSE [get_ports clk_i_p]

#
set_property BITSTREAM.GENERAL.COMPRESS TRUE [current_design]
set_property BITSTREAM.CONFIG.UNUSEDPIN Pullup [current_design]
set_property CONFIG_VOLTAGE 1.8 [current_design]
set_property CFGBVS GND [current_design]
#create_clock -period 5.000 -name sys_clk [get_ports sys_clk_p]
set_property PACKAGE_PIN C42 [get_ports sys_clk_p]
set_property IOSTANDARD LVDS [get_ports sys_clk_p]
set_property DIFF_TERM_ADV TERM_NONE [get_ports sys_clk_p]
	
#global reset
#SW 	P_JS1.4 or P_SW4 or Runtime Software	BC27	Bank65	1.8V
#SW2	P_JS1.8 or P_SW5 or Runtime Software	BD28	Bank65	1.8V

#reference
set_property PACKAGE_PIN BC27 [get_ports RESETn]
set_property IOSTANDARD LVCMOS18 [get_ports RESETn]


#TEST1	AM41	Bank43	SW2	    VCC_J4
#TEST2	M41	    Bank47	SW3	    VCC_J4
#TEST3	K47	    Bank47	S2.1	VCC_J4
#TEST4	P48	    Bank47	S2.2	VCC_J4
#TEST5	C51	    Bank48	S2.3	VCC_J4
#TEST6	E53	    Bank48	S2.4	VCC_J4
#TEST7	BL52	Bank39	LED35	VCC_J8
#TEST8	BC54	Bank40	LED36	VCC_J5
#TEST9	BK53	Bank40	LED37	VCC_J5
#TEST10	BF50	Bank40	J13.1	VCC_J5
#TEST11	AW41	Bank41	J13.2	VCC_J5
#TEST12	BC49	Bank41	J13.3	VCC_J5
#TEST13	BC48	Bank41	J13.4	VCC_J5
#TEST14	AN42	Bank42	J13.5	VCC_J5
#TEST15	AP48	Bank42	J13.6	VCC_J5
#TEST16	AV49	Bank42	J13.7	VCC_J5
#GND			        J13.8	----



#UART1 _TX	F1.E33	Bank53	J14.1	 VCC_J1
#UART1 _RX	F1.D31	Bank53	J14.2	 VCC_J1
#UART2 _TX	F2.E33	Bank53	J14.1	 VCC_J1
#UART2 _RX	F2.D31	Bank53	J14.2	 VCC_J1
#UART3 _TX	F3.E33	Bank53	J14.1	 VCC_J1
#UART3 _RX	F3.D31	Bank53	J14.2	 VCC_J1
#UART4 _TX	F4.E33	Bank53	J14.1	 VCC_J1
#UART4 _RX	F4.D31	Bank53	J14.2	 VCC_J1



#TDM_RESET	    BG31	Bank66	SW1	    VCC_J6
#TDM_DONE	    AY30	Bank66	LED2	VCC_J6
#TDM_REF_CLKP	BD30	Bank65	X1.4	1.8V
#TDM_REF_CLKN	BE30	Bank65	X1.5	1.8V

#VIO
#SWITCH1	AT28	SW_V_F1.1
#SWITCH2	AV26	SW_V_F1.2
#BUTTON1	AT29	BUTTON_V_F1.1
#BUTTON2	BA29	BUTTON _V_F1.2
#LED1	    BF27	LED_V_F1.1
#LED2	    BD25	LED_V_F1.2
#LED3	    BB29	LED_V_F1.3
#LED4	    BC28	LED_V_F1.4
#UART TXD	BE27	UART_V_F1.TX
#UART RXD	AW26	UART_V_F1.RX

#All led is High Active(include the VIO):“H” turns LED On. “L” turns LED Off.
#All button is Low Active(include the VIO):Push = L Normal = H

set_property DIRECTION OUT [get_ports WPNeg_2]
set_property IOSTANDARD LVCMOS18 [get_ports WPNeg_2]
set_property DRIVE 12 [get_ports WPNeg_2]
set_property SLEW SLOW [get_ports WPNeg_2]
set_property OUTPUT_IMPEDANCE RDRV_NONE_NONE [get_ports WPNeg_2]

#-----------------------------------------------------
#brdswitch   ON = L OFF = H
#TEST3	K47	Bank47	S2.1	VCC_J4
#TEST4	P48	Bank47	S2.2	VCC_J4
#TEST5	C51	Bank48	S2.3	VCC_J4
#TEST6	E53	Bank48	S2.4	VCC_J4
set_property PACKAGE_PIN K47 [get_ports {brdswitch[0]}]
set_property PACKAGE_PIN P48 [get_ports {brdswitch[1]}]
set_property PACKAGE_PIN C51 [get_ports {brdswitch[2]}]
set_property IOSTANDARD LVCMOS18 [get_ports brdswitch[0]]
set_property IOSTANDARD LVCMOS18 [get_ports brdswitch[1]]
set_property IOSTANDARD LVCMOS18 [get_ports brdswitch[2]]
#---------------------------------------------------------
#TEST7	BL52	Bank39	LED35	VCC_J8
#TEST8	BC54	Bank40	LED36	VCC_J5
#TEST9	BK53	Bank40	LED37	VCC_J5
set_property IOSTANDARD LVCMOS18 [get_ports bled1]
set_property IOSTANDARD LVCMOS18 [get_ports bled2]
set_property IOSTANDARD LVCMOS18 [get_ports bled3]
set_property PACKAGE_PIN BL52 [get_ports {bled1}]
set_property PACKAGE_PIN BC54 [get_ports {bled2}]
set_property PACKAGE_PIN BK53 [get_ports {bled3}]



#reference
## Clock
create_clock -name fifoClk -period 10.0 [get_ports {clk}]
## False path
#set_false_path -from [get_ports {SRST_N HRST_N}] -to [get_registers *]
#set_false_path -from [get_ports {TXE_N}] -to [get_registers *]
#set_false_path -from [get_ports {R_OOB W_OOB MLTCN STREN ERDIS}] -to [get_registers *]
#set_false_path -from {mst_fifo_fsm:i1_fsm|be_oe_n} -to {DATA[*]};
#set_false_path -from {mst_fifo_fsm:i1_fsm|be_oe_n} -to {BE[*]};
#set_false_path -from {mst_fifo_fsm:i1_fsm|dt_oe_n} -to {DATA[*]}
#set_false_path -to [get_ports {STRER[*] debug_sig[*]}]
#set_false_path -to [get_ports {SIWU_N}]
## Input delay
#set_input_delay -clock [get_clocks fifoClk] -max 7.0 [get_ports {RXF_N}]
#set_input_delay -clock [get_clocks fifoClk] -max 7.0 [get_ports {BE[*] DATA[*]}]
#set_input_delay -clock [get_clocks fifoClk] -min 6.5 [get_ports {RXF_N}]
#set_input_delay -clock [get_clocks fifoClk] -min 6.5 [get_ports {BE[*] DATA[*]}]
## Output delay
#set_output_delay -clock [get_clocks fifoClk] -max 1.0 [get_ports {WR_N RD_N OE_N}]
#set_output_delay -clock [get_clocks fifoClk] -max 1.0 [get_ports {BE[*] DATA[*]}]
#set_output_delay -clock [get_clocks fifoClk] -min 4.8 [get_ports {WR_N RD_N OE_N}]
#set_output_delay -clock [get_clocks fifoClk] -min 4.8 [get_ports {BE[*] DATA[*]}]


#dfx-sub
set_property CLOCK_DEDICATED_ROUTE FALSE [get_nets clk_sub_IBUF_inst/O]
 
set_property PACKAGE_PIN BF29 [get_ports clk_sub]
set_property PACKAGE_PIN BF28 [get_ports sub_bus_en_i]
set_property PACKAGE_PIN BE28 [get_ports sub_bus_en_o]

set_property PACKAGE_PIN AU26 [get_ports {sub_bus[0]}]
set_property PACKAGE_PIN AY27 [get_ports {sub_bus[1]}]
set_property PACKAGE_PIN BA28 [get_ports {sub_bus[2]}]
set_property PACKAGE_PIN AY26 [get_ports {sub_bus[3]}]
set_property PACKAGE_PIN BB30 [get_ports {sub_bus[4]}]


set_property IOSTANDARD LVCMOS18 [get_ports clk_sub]
set_property IOSTANDARD LVCMOS18 [get_ports sub_bus_en_i]
set_property IOSTANDARD LVCMOS18 [get_ports sub_bus_en_o]
set_property IOSTANDARD LVCMOS18 [get_ports {sub_bus[4]}]
set_property IOSTANDARD LVCMOS18 [get_ports {sub_bus[3]}]
set_property IOSTANDARD LVCMOS18 [get_ports {sub_bus[2]}]
set_property IOSTANDARD LVCMOS18 [get_ports {sub_bus[1]}]
set_property IOSTANDARD LVCMOS18 [get_ports {sub_bus[0]}]
#
set_property SLEW FAST [get_ports {sub_bus[4]}]
set_property SLEW FAST [get_ports {sub_bus[3]}]
set_property SLEW FAST [get_ports {sub_bus[2]}]
set_property SLEW FAST [get_ports {sub_bus[1]}]
set_property SLEW FAST [get_ports {sub_bus[0]}]
set_property PULLUP true [get_ports {sub_bus[4]}]
set_property PULLUP true [get_ports {sub_bus[3]}]
set_property PULLUP true [get_ports {sub_bus[2]}]
set_property PULLUP true [get_ports {sub_bus[1]}]
set_property PULLUP true [get_ports {sub_bus[0]}]
set_property SLEW FAST [get_ports sub_bus_en_o]
set_property PULLDOWN true [get_ports sub_bus_en_o]
set_property PULLDOWN true [get_ports sub_bus_en_i]
set_property PULLUP true [get_ports clk_sub]
#
create_clock -period 20.000 -name clk_sub [get_ports clk_sub]





#GT
# TF2TC ep, JX1 MINISAS conntor 1,FPGA quad 225
set_property PACKAGE_PIN AK9  [get_ports clk_ref_bk225_n]
set_property PACKAGE_PIN AK10 [get_ports clk_ref_bk225_p]
set_property PACKAGE_PIN AL3  [get_ports ch0_ep_tc2tf_n]
set_property PACKAGE_PIN AL4  [get_ports ch0_ep_tc2tf_p]
set_property PACKAGE_PIN AK1  [get_ports ch1_ep_tc2tf_n]
set_property PACKAGE_PIN AK2  [get_ports ch1_ep_tc2tf_p]
set_property PACKAGE_PIN AJ3  [get_ports ch2_ep_tc2tf_n]
set_property PACKAGE_PIN AJ4  [get_ports ch2_ep_tc2tf_p]
set_property PACKAGE_PIN AH1  [get_ports ch3_ep_tc2tf_n]
set_property PACKAGE_PIN AH2  [get_ports ch3_ep_tc2tf_p]
set_property PACKAGE_PIN AL7  [get_ports ch0_ep_tf2tc_n]
set_property PACKAGE_PIN AL8  [get_ports ch0_ep_tf2tc_p]
set_property PACKAGE_PIN AK5  [get_ports ch1_ep_tf2tc_n]
set_property PACKAGE_PIN AK6  [get_ports ch1_ep_tf2tc_p]
set_property PACKAGE_PIN AJ7  [get_ports ch2_ep_tf2tc_n]
set_property PACKAGE_PIN AJ8  [get_ports ch2_ep_tf2tc_p]
set_property PACKAGE_PIN AH5  [get_ports ch3_ep_tf2tc_n]
set_property PACKAGE_PIN AH6  [get_ports ch3_ep_tf2tc_p]

# EP_PHY Q0, JX2 MINISAS conntor 2,FPGA quad 226
set_property PACKAGE_PIN AF9  [get_ports serdes_clk_ref_bk226_n]
set_property PACKAGE_PIN AF10 [get_ports serdes_clk_ref_bk226_p]
set_property PACKAGE_PIN AG3  [get_ports {phy_rxn[0]}]
set_property PACKAGE_PIN AG4  [get_ports {phy_rxp[0]}]
set_property PACKAGE_PIN AF1  [get_ports {phy_rxn[1]}]
set_property PACKAGE_PIN AF2  [get_ports {phy_rxp[1]}]
set_property PACKAGE_PIN AE3  [get_ports {phy_rxn[2]}]
set_property PACKAGE_PIN AE4  [get_ports {phy_rxp[2]}]
set_property PACKAGE_PIN AD1  [get_ports {phy_rxn[3]}]
set_property PACKAGE_PIN AD2  [get_ports {phy_rxp[3]}]
set_property PACKAGE_PIN AG7  [get_ports {phy_txn[0]}]
set_property PACKAGE_PIN AG8  [get_ports {phy_txp[0]}]
set_property PACKAGE_PIN AF5  [get_ports {phy_txn[1]}]
set_property PACKAGE_PIN AF6  [get_ports {phy_txp[1]}]
set_property PACKAGE_PIN AE7  [get_ports {phy_txn[2]}]
set_property PACKAGE_PIN AE8  [get_ports {phy_txp[2]}]
set_property PACKAGE_PIN AD5  [get_ports {phy_txn[3]}]
set_property PACKAGE_PIN AD6  [get_ports {phy_txp[3]}]

# EP_PHY Q1, JX2 MINISAS conntor 1,FPGA quad 227
set_property PACKAGE_PIN AB9  [get_ports serdes_clk_ref_bk227_n]
set_property PACKAGE_PIN AB10 [get_ports serdes_clk_ref_bk227_p]
set_property PACKAGE_PIN AC3  [get_ports {phy_rxn[4]}]
set_property PACKAGE_PIN AC4  [get_ports {phy_rxp[4]}]
set_property PACKAGE_PIN AB1  [get_ports {phy_rxn[5]}]
set_property PACKAGE_PIN AB2  [get_ports {phy_rxp[5]}]
set_property PACKAGE_PIN AA3  [get_ports {phy_rxn[6]}]
set_property PACKAGE_PIN AA4  [get_ports {phy_rxp[6]}]
set_property PACKAGE_PIN Y1   [get_ports {phy_rxn[7]}]
set_property PACKAGE_PIN Y2   [get_ports {phy_rxp[7]}]
set_property PACKAGE_PIN AC7  [get_ports {phy_txn[4]}]
set_property PACKAGE_PIN AC8  [get_ports {phy_txp[4]}]
set_property PACKAGE_PIN AB5  [get_ports {phy_txn[5]}]
set_property PACKAGE_PIN AB6  [get_ports {phy_txp[5]}]
set_property PACKAGE_PIN AA7  [get_ports {phy_txn[6]}]
set_property PACKAGE_PIN AA8  [get_ports {phy_txp[6]}]
set_property PACKAGE_PIN Y5   [get_ports {phy_txn[7]}]
set_property PACKAGE_PIN Y6   [get_ports {phy_txp[7]}]

# SAS Q0, J5 conntor,FPGA quad 219
set_property PACKAGE_PIN BL7 [get_ports serdes_clk_ref_bk219_n]
set_property PACKAGE_PIN BL8 [get_ports serdes_clk_ref_bk219_p]
set_property PACKAGE_PIN BN3 [get_ports {sas_sds_gthrxn[3]}]
set_property PACKAGE_PIN BN4 [get_ports {sas_sds_gthrxp[3]}]
set_property PACKAGE_PIN BL3 [get_ports {sas_sds_gthrxn[2]}]
set_property PACKAGE_PIN BL4 [get_ports {sas_sds_gthrxp[2]}]
set_property PACKAGE_PIN BK1 [get_ports {sas_sds_gthrxn[1]}]
set_property PACKAGE_PIN BK2 [get_ports {sas_sds_gthrxp[1]}]
set_property PACKAGE_PIN BH1 [get_ports {sas_sds_gthrxn[0]}]
set_property PACKAGE_PIN BH2 [get_ports {sas_sds_gthrxp[0]}]
set_property PACKAGE_PIN BM5 [get_ports {sas_sds_gthtxn[3]}]
set_property PACKAGE_PIN BM6 [get_ports {sas_sds_gthtxp[3]}]
set_property PACKAGE_PIN BK5 [get_ports {sas_sds_gthtxn[2]}]
set_property PACKAGE_PIN BK6 [get_ports {sas_sds_gthtxp[2]}]
set_property PACKAGE_PIN BJ3 [get_ports {sas_sds_gthtxn[1]}]
set_property PACKAGE_PIN BJ4 [get_ports {sas_sds_gthtxp[1]}]
set_property PACKAGE_PIN BH5 [get_ports {sas_sds_gthtxn[0]}]
set_property PACKAGE_PIN BH6 [get_ports {sas_sds_gthtxp[0]}]


#1
create_debug_core u_ila_0 ila
set_property ALL_PROBE_SAME_MU true [get_debug_cores u_ila_0]
set_property ALL_PROBE_SAME_MU_CNT 4 [get_debug_cores u_ila_0]
set_property C_ADV_TRIGGER true [get_debug_cores u_ila_0]
set_property C_DATA_DEPTH 1024 [get_debug_cores u_ila_0]
set_property C_EN_STRG_QUAL true [get_debug_cores u_ila_0]
set_property C_INPUT_PIPE_STAGES 0 [get_debug_cores u_ila_0]
set_property C_TRIGIN_EN false [get_debug_cores u_ila_0]
set_property C_TRIGOUT_EN false [get_debug_cores u_ila_0]
#2
set_property port_width 1 [get_debug_ports u_ila_0/clk]
connect_debug_port u_ila_0/clk [get_nets [list clk_wiz_mmcm0_ins0/inst/clk_out1]]
#3
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe0]
set_property port_width 16 [get_debug_ports u_ila_0/probe0]
connect_debug_port u_ila_0/probe0 [get_nets [list {nor_flash_ins2/sector_buffer_cnt[0]} {nor_flash_ins2/sector_buffer_cnt[1]} {nor_flash_ins2/sector_buffer_cnt[2]} {nor_flash_ins2/sector_buffer_cnt[3]} {nor_flash_ins2/sector_buffer_cnt[4]} {nor_flash_ins2/sector_buffer_cnt[5]} {nor_flash_ins2/sector_buffer_cnt[6]} {nor_flash_ins2/sector_buffer_cnt[7]} {nor_flash_ins2/sector_buffer_cnt[8]} {nor_flash_ins2/sector_buffer_cnt[9]} {nor_flash_ins2/sector_buffer_cnt[10]} {nor_flash_ins2/sector_buffer_cnt[11]} {nor_flash_ins2/sector_buffer_cnt[12]} {nor_flash_ins2/sector_buffer_cnt[13]} {nor_flash_ins2/sector_buffer_cnt[14]} {nor_flash_ins2/sector_buffer_cnt[15]}]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe1]
set_property port_width 26 [get_debug_ports u_ila_0/probe1]
connect_debug_port u_ila_0/probe1 [get_nets [list {nor_flash_ins2/sector_addr_reg[0]} {nor_flash_ins2/sector_addr_reg[1]} {nor_flash_ins2/sector_addr_reg[2]} {nor_flash_ins2/sector_addr_reg[3]} {nor_flash_ins2/sector_addr_reg[4]} {nor_flash_ins2/sector_addr_reg[5]} {nor_flash_ins2/sector_addr_reg[6]} {nor_flash_ins2/sector_addr_reg[7]} {nor_flash_ins2/sector_addr_reg[8]} {nor_flash_ins2/sector_addr_reg[9]} {nor_flash_ins2/sector_addr_reg[10]} {nor_flash_ins2/sector_addr_reg[11]} {nor_flash_ins2/sector_addr_reg[12]} {nor_flash_ins2/sector_addr_reg[13]} {nor_flash_ins2/sector_addr_reg[14]} {nor_flash_ins2/sector_addr_reg[15]} {nor_flash_ins2/sector_addr_reg[16]} {nor_flash_ins2/sector_addr_reg[17]} {nor_flash_ins2/sector_addr_reg[18]} {nor_flash_ins2/sector_addr_reg[19]} {nor_flash_ins2/sector_addr_reg[20]} {nor_flash_ins2/sector_addr_reg[21]} {nor_flash_ins2/sector_addr_reg[22]} {nor_flash_ins2/sector_addr_reg[23]} {nor_flash_ins2/sector_addr_reg[24]} {nor_flash_ins2/sector_addr_reg[25]}]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe2]
set_property port_width 16 [get_debug_ports u_ila_0/probe2]
connect_debug_port u_ila_0/probe2 [get_nets [list {nor_flash_ins2/sector_word_cnt[0]} {nor_flash_ins2/sector_word_cnt[1]} {nor_flash_ins2/sector_word_cnt[2]} {nor_flash_ins2/sector_word_cnt[3]} {nor_flash_ins2/sector_word_cnt[4]} {nor_flash_ins2/sector_word_cnt[5]} {nor_flash_ins2/sector_word_cnt[6]} {nor_flash_ins2/sector_word_cnt[7]} {nor_flash_ins2/sector_word_cnt[8]} {nor_flash_ins2/sector_word_cnt[9]} {nor_flash_ins2/sector_word_cnt[10]} {nor_flash_ins2/sector_word_cnt[11]} {nor_flash_ins2/sector_word_cnt[12]} {nor_flash_ins2/sector_word_cnt[13]} {nor_flash_ins2/sector_word_cnt[14]} {nor_flash_ins2/sector_word_cnt[15]}]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe3]
set_property port_width 4 [get_debug_ports u_ila_0/probe3]
connect_debug_port u_ila_0/probe3 [get_nets [list {nor_flash_ins2/vy_status_idx[0]} {nor_flash_ins2/vy_status_idx[1]} {nor_flash_ins2/vy_status_idx[2]} {nor_flash_ins2/vy_status_idx[3]}]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe4]
set_property port_width 5 [get_debug_ports u_ila_0/probe4]
connect_debug_port u_ila_0/probe4 [get_nets [list {nor_flash_ins2/write_buffer_cnt[0]} {nor_flash_ins2/write_buffer_cnt[1]} {nor_flash_ins2/write_buffer_cnt[2]} {nor_flash_ins2/write_buffer_cnt[3]} {nor_flash_ins2/write_buffer_cnt[4]}]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe5]
set_property port_width 26 [get_debug_ports u_ila_0/probe5]
connect_debug_port u_ila_0/probe5 [get_nets [list {A_1_OBUF[0]} {A_1_OBUF[1]} {A_1_OBUF[2]} {A_1_OBUF[3]} {A_1_OBUF[4]} {A_1_OBUF[5]} {A_1_OBUF[6]} {A_1_OBUF[7]} {A_1_OBUF[8]} {A_1_OBUF[9]} {A_1_OBUF[10]} {A_1_OBUF[11]} {A_1_OBUF[12]} {A_1_OBUF[13]} {A_1_OBUF[14]} {A_1_OBUF[15]} {A_1_OBUF[16]} {A_1_OBUF[17]} {A_1_OBUF[18]} {A_1_OBUF[19]} {A_1_OBUF[20]} {A_1_OBUF[21]} {A_1_OBUF[22]} {A_1_OBUF[23]} {A_1_OBUF[24]} {A_1_OBUF[25]}]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe6]
set_property port_width 4 [get_debug_ports u_ila_0/probe6]
connect_debug_port u_ila_0/probe6 [get_nets [list {nor_flash_ins1/vy_status_idx[0]} {nor_flash_ins1/vy_status_idx[1]} {nor_flash_ins1/vy_status_idx[2]} {nor_flash_ins1/vy_status_idx[3]}]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe7]
set_property port_width 16 [get_debug_ports u_ila_0/probe7]
connect_debug_port u_ila_0/probe7 [get_nets [list {nor_flash_ins1/sector_word_cnt[0]} {nor_flash_ins1/sector_word_cnt[1]} {nor_flash_ins1/sector_word_cnt[2]} {nor_flash_ins1/sector_word_cnt[3]} {nor_flash_ins1/sector_word_cnt[4]} {nor_flash_ins1/sector_word_cnt[5]} {nor_flash_ins1/sector_word_cnt[6]} {nor_flash_ins1/sector_word_cnt[7]} {nor_flash_ins1/sector_word_cnt[8]} {nor_flash_ins1/sector_word_cnt[9]} {nor_flash_ins1/sector_word_cnt[10]} {nor_flash_ins1/sector_word_cnt[11]} {nor_flash_ins1/sector_word_cnt[12]} {nor_flash_ins1/sector_word_cnt[13]} {nor_flash_ins1/sector_word_cnt[14]} {nor_flash_ins1/sector_word_cnt[15]}]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe8]
set_property port_width 26 [get_debug_ports u_ila_0/probe8]
connect_debug_port u_ila_0/probe8 [get_nets [list {A_2_OBUF[0]} {A_2_OBUF[1]} {A_2_OBUF[2]} {A_2_OBUF[3]} {A_2_OBUF[4]} {A_2_OBUF[5]} {A_2_OBUF[6]} {A_2_OBUF[7]} {A_2_OBUF[8]} {A_2_OBUF[9]} {A_2_OBUF[10]} {A_2_OBUF[11]} {A_2_OBUF[12]} {A_2_OBUF[13]} {A_2_OBUF[14]} {A_2_OBUF[15]} {A_2_OBUF[16]} {A_2_OBUF[17]} {A_2_OBUF[18]} {A_2_OBUF[19]} {A_2_OBUF[20]} {A_2_OBUF[21]} {A_2_OBUF[22]} {A_2_OBUF[23]} {A_2_OBUF[24]} {A_2_OBUF[25]}]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe9]
set_property port_width 8 [get_debug_ports u_ila_0/probe9]
connect_debug_port u_ila_0/probe9 [get_nets [list {vy_status1[0]} {vy_status1[1]} {vy_status1[2]} {vy_status1[3]} {vy_status1[4]} {vy_status1[5]} {vy_status1[6]} {vy_status1[7]}]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe10]
set_property port_width 16 [get_debug_ports u_ila_0/probe10]
connect_debug_port u_ila_0/probe10 [get_nets [list {nor_flash_ins1/sector_buffer_cnt[0]} {nor_flash_ins1/sector_buffer_cnt[1]} {nor_flash_ins1/sector_buffer_cnt[2]} {nor_flash_ins1/sector_buffer_cnt[3]} {nor_flash_ins1/sector_buffer_cnt[4]} {nor_flash_ins1/sector_buffer_cnt[5]} {nor_flash_ins1/sector_buffer_cnt[6]} {nor_flash_ins1/sector_buffer_cnt[7]} {nor_flash_ins1/sector_buffer_cnt[8]} {nor_flash_ins1/sector_buffer_cnt[9]} {nor_flash_ins1/sector_buffer_cnt[10]} {nor_flash_ins1/sector_buffer_cnt[11]} {nor_flash_ins1/sector_buffer_cnt[12]} {nor_flash_ins1/sector_buffer_cnt[13]} {nor_flash_ins1/sector_buffer_cnt[14]} {nor_flash_ins1/sector_buffer_cnt[15]}]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe11]
set_property port_width 5 [get_debug_ports u_ila_0/probe11]
connect_debug_port u_ila_0/probe11 [get_nets [list {nor_flash_ins1/write_buffer_cnt[0]} {nor_flash_ins1/write_buffer_cnt[1]} {nor_flash_ins1/write_buffer_cnt[2]} {nor_flash_ins1/write_buffer_cnt[3]} {nor_flash_ins1/write_buffer_cnt[4]}]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe12]
set_property port_width 8 [get_debug_ports u_ila_0/probe12]
connect_debug_port u_ila_0/probe12 [get_nets [list {vy_status2[0]} {vy_status2[1]} {vy_status2[2]} {vy_status2[3]} {vy_status2[4]} {vy_status2[5]} {vy_status2[6]} {vy_status2[7]}]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe13]
set_property port_width 26 [get_debug_ports u_ila_0/probe13]
connect_debug_port u_ila_0/probe13 [get_nets [list {nor_flash_ins1/sector_addr_reg[0]} {nor_flash_ins1/sector_addr_reg[1]} {nor_flash_ins1/sector_addr_reg[2]} {nor_flash_ins1/sector_addr_reg[3]} {nor_flash_ins1/sector_addr_reg[4]} {nor_flash_ins1/sector_addr_reg[5]} {nor_flash_ins1/sector_addr_reg[6]} {nor_flash_ins1/sector_addr_reg[7]} {nor_flash_ins1/sector_addr_reg[8]} {nor_flash_ins1/sector_addr_reg[9]} {nor_flash_ins1/sector_addr_reg[10]} {nor_flash_ins1/sector_addr_reg[11]} {nor_flash_ins1/sector_addr_reg[12]} {nor_flash_ins1/sector_addr_reg[13]} {nor_flash_ins1/sector_addr_reg[14]} {nor_flash_ins1/sector_addr_reg[15]} {nor_flash_ins1/sector_addr_reg[16]} {nor_flash_ins1/sector_addr_reg[17]} {nor_flash_ins1/sector_addr_reg[18]} {nor_flash_ins1/sector_addr_reg[19]} {nor_flash_ins1/sector_addr_reg[20]} {nor_flash_ins1/sector_addr_reg[21]} {nor_flash_ins1/sector_addr_reg[22]} {nor_flash_ins1/sector_addr_reg[23]} {nor_flash_ins1/sector_addr_reg[24]} {nor_flash_ins1/sector_addr_reg[25]}]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe14]
set_property port_width 1 [get_debug_ports u_ila_0/probe14]
connect_debug_port u_ila_0/probe14 [get_nets [list CENeg_1_OBUF]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe15]
set_property port_width 1 [get_debug_ports u_ila_0/probe15]
connect_debug_port u_ila_0/probe15 [get_nets [list CENeg_2_OBUF]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe16]
set_property port_width 1 [get_debug_ports u_ila_0/probe16]
connect_debug_port u_ila_0/probe16 [get_nets [list EN_1KHZ]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe17]
set_property port_width 1 [get_debug_ports u_ila_0/probe17]
connect_debug_port u_ila_0/probe17 [get_nets [list OENeg_1_OBUF]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe18]
set_property port_width 1 [get_debug_ports u_ila_0/probe18]
connect_debug_port u_ila_0/probe18 [get_nets [list OENeg_2_OBUF]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe19]
set_property port_width 1 [get_debug_ports u_ila_0/probe19]
connect_debug_port u_ila_0/probe19 [get_nets [list PO_1KHZ]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe20]
set_property port_width 1 [get_debug_ports u_ila_0/probe20]
connect_debug_port u_ila_0/probe20 [get_nets [list RESETNeg_1_OBUF]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe21]
set_property port_width 1 [get_debug_ports u_ila_0/probe21]
connect_debug_port u_ila_0/probe21 [get_nets [list RESETNeg_2_OBUF]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe22]
set_property port_width 1 [get_debug_ports u_ila_0/probe22]
connect_debug_port u_ila_0/probe22 [get_nets [list nor_flash_ins2/nor_flash_drv_ins0/RESETNeg_o]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe23]
set_property port_width 1 [get_debug_ports u_ila_0/probe23]
connect_debug_port u_ila_0/probe23 [get_nets [list nor_flash_ins1/nor_flash_drv_ins0/RESETNeg_o]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe24]
set_property port_width 1 [get_debug_ports u_ila_0/probe24]
connect_debug_port u_ila_0/probe24 [get_nets [list RY_1_IBUF]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe25]
set_property port_width 1 [get_debug_ports u_ila_0/probe25]
connect_debug_port u_ila_0/probe25 [get_nets [list RY_2_IBUF]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe26]
set_property port_width 1 [get_debug_ports u_ila_0/probe26]
connect_debug_port u_ila_0/probe26 [get_nets [list nor_flash_ins2/nor_flash_drv_ins0/RY_r]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe27]
set_property port_width 1 [get_debug_ports u_ila_0/probe27]
connect_debug_port u_ila_0/probe27 [get_nets [list nor_flash_ins1/nor_flash_drv_ins0/RY_r_1]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe28]
set_property port_width 1 [get_debug_ports u_ila_0/probe28]
connect_debug_port u_ila_0/probe28 [get_nets [list WENeg_1_OBUF]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe29]
set_property port_width 1 [get_debug_ports u_ila_0/probe29]
connect_debug_port u_ila_0/probe29 [get_nets [list WENeg_2_OBUF]]
#4
set_property C_CLK_INPUT_FREQ_HZ 300000000 [get_debug_cores dbg_hub]
set_property C_ENABLE_CLK_DIVIDER false [get_debug_cores dbg_hub]
set_property C_USER_SCAN_CHAIN 1 [get_debug_cores dbg_hub]
connect_debug_port dbg_hub/clk [get_nets clk]


set_case_analysis 1 [get_pins QUAD[3].u_q/u_common/u_gthe3_common/QPLL0REFCLKSEL[0]]
set_case_analysis 0 [get_pins QUAD[3].u_q/u_common/u_gthe3_common/QPLL0REFCLKSEL[1]]
set_case_analysis 0 [get_pins QUAD[3].u_q/u_common/u_gthe3_common/QPLL0REFCLKSEL[2]]


create_generated_clock -name u_jtag_proc/tck_i -source [get_pins u_jtag_proc/tck_i_reg/C] -divide_by 2 [get_pins u_jtag_proc/tck_i_reg/Q]
set_input_delay -clock [get_clocks clk_sub] -min -add_delay 5.300 [get_ports {sub_bus[*]}]
set_input_delay -clock [get_clocks clk_sub] -max -add_delay 5.300 [get_ports {sub_bus[*]}]
set_input_delay -clock [get_clocks sysclk] -min -add_delay 5.300 [get_ports RESETn]
set_input_delay -clock [get_clocks sysclk] -max -add_delay 5.300 [get_ports RESETn]
set_input_delay -clock [get_clocks clk_sub] -min -add_delay 5.300 [get_ports sub_bus_en_i]
set_input_delay -clock [get_clocks clk_sub] -max -add_delay 5.300 [get_ports sub_bus_en_i]
set_output_delay -clock [get_clocks clk_sub] -min -add_delay 4.900 [get_ports {sub_bus[*]}]
set_output_delay -clock [get_clocks clk_sub] -max -add_delay 5.300 [get_ports {sub_bus[*]}]
set_output_delay -clock [get_clocks clk_sub] -min -add_delay 4.900 [get_ports sub_bus_en_o]
set_output_delay -clock [get_clocks clk_sub] -max -add_delay 5.300 [get_ports sub_bus_en_o]


# GT X0Y55
create_clock -period 4.000 [get_pins {QUAD[11].u_q/CH[3].u_ch/u_gthe3_channel/DMONITOROUT[16]}]
set_clock_groups -asynchronous -group [get_clocks -of_objects [get_pins {QUAD[11].u_q/CH[3].u_ch/u_gthe3_channel/DMONITOROUT[16]}]]
##
## GTH Channel and Common Loc constraints
##
set_property LOC GTHE3_CHANNEL_X0Y0 [get_cells QUAD[0].u_q/CH[0].u_ch/u_gthe3_channel]
set_property LOC GTHE3_CHANNEL_X0Y1 [get_cells QUAD[0].u_q/CH[1].u_ch/u_gthe3_channel]
set_property LOC GTHE3_CHANNEL_X0Y2 [get_cells QUAD[0].u_q/CH[2].u_ch/u_gthe3_channel]
set_property LOC GTHE3_CHANNEL_X0Y3 [get_cells QUAD[0].u_q/CH[3].u_ch/u_gthe3_channel]
set_property LOC GTHE3_COMMON_X0Y0 [get_cells QUAD[0].u_q/u_common/u_gthe3_common]



set_property LOC BUFGCE_X1Y0 [get_cells QUAD[0].u_q/CH[0].u_ch/u_buf_dmonitorclk]				
set_property LOC BUFGCE_X1Y1 [get_cells QUAD[0].u_q/CH[1].u_ch/u_buf_dmonitorclk]				
set_property LOC BUFGCE_X1Y2 [get_cells QUAD[0].u_q/CH[2].u_ch/u_buf_dmonitorclk]				

  IBUFGDS 
   #(.DIFF_TERM("FALSE"))
   u_ibufgds
    (
      .I(gth_sysclkp_i),
      .IB(gth_sysclkn_i),
      .O(gth_sysclk_i)
    );
	
	    IBUFDS_GTE3 u_buf_gth_q13_clk1
      (
        .O            (gth_refclk1_i[11]),
        .ODIV2        (gth_odiv2_1_i[11]),
        .CEB          (1'b0),
        .I            (gth_refclk1p_i[11]),
        .IB           (gth_refclk1n_i[11])
      );
	  
	  ## TX/RX out clock clock constraints
##
# GT X0Y0
set_clock_groups -asynchronous -group [get_clocks -of_objects [get_pins {u_ibert_gth_core/inst/QUAD[0].u_q/CH[0].u_ch/u_gthe3_channel/RXOUTCLK}] -include_generated_clocks]
set_clock_groups -asynchronous -group [get_clocks -of_objects [get_pins {u_ibert_gth_core/inst/QUAD[0].u_q/CH[0].u_ch/u_gthe3_channel/TXOUTCLK}] -include_generated_clocks]
# GT X0Y1
set_clock_groups -asynchronous -group [get_clocks -of_objects [get_pins {u_ibert_gth_core/inst/QUAD[0].u_q/CH[1].u_ch/u_gthe3_channel/RXOUTCLK}] -include_generated_clocks]
set_clock_groups -asynchronous -group [get_clocks -of_objects [get_pins {u_ibert_gth_core/inst/QUAD[0].u_q/CH[1].u_ch/u_gthe3_channel/TXOUTCLK}] -include_generated_clocks]
# GT X0Y2

## Icon Constraints
##
create_clock -name D_CLK -period 3.333 [get_ports gth_sysclkp_i]
set_clock_groups -group [get_clocks D_CLK -include_generated_clocks] -asynchronous
set_property C_CLK_INPUT_FREQ_HZ 300000000 [get_debug_cores dbg_hub]
set_property C_ENABLE_CLK_DIVIDER true [get_debug_cores dbg_hub]

##gth_refclk lock constraints
##
set_property PACKAGE_PIN BL8 [get_ports gth_refclk0p_i[0]]
set_property PACKAGE_PIN BL7 [get_ports gth_refclk0n_i[0]]

set_property SEVERITY {Warning} [get_drc_checks NSTD-1]
set_property SEVERITY {Warning} [get_drc_checks UCIO-1]
set_property SEVERITY {Warning} [get_drc_checks RTSTAT-1]
set_property SEVERITY {Warning} [get_drc_checks AVAL-244]
set_property SEVERITY {Warning} [get_drc_checks AVAL-245]

create_generated_clock -name u_jtag_proc/tck_i -source [get_pins u_jtag_proc/tck_i_reg/C] -divide_by 2 [get_pins u_jtag_proc/tck_i_reg/Q]


# False path
set_false_path -from [get_ports GTPRESET_IN]
set_false_path -from [get_ports {RESETn}] -to [get_registers *]

set_false_path -from [get_ports {SRST_N HRST_N}] -to [get_registers *]
set_false_path -from [get_ports {TXE_N}] -to [get_registers *]
set_false_path -from [get_ports {R_OOB W_OOB MLTCN STREN ERDIS}] -to [get_registers *]
set_false_path -from {mst_fifo_fsm:i1_fsm|be_oe_n} -to {DATA[*]};
set_false_path -from {mst_fifo_fsm:i1_fsm|be_oe_n} -to {BE[*]};
set_false_path -from {mst_fifo_fsm:i1_fsm|dt_oe_n} -to {DATA[*]}
set_false_path -to [get_ports {STRER[*] debug_sig[*]}]
set_false_path -to [get_ports {SIWU_N}]

#other TCL comds
#after opt_design(or implement_debug_core)
write_debug_probes ./*.ltx

set_property mark_debug true [get_nets sine*]
(* mark_debug = "true" *) wire sine;


#




#1
create_debug_core u_ila_0 ila
set_property ALL_PROBE_SAME_MU true [get_debug_cores u_ila_0]
set_property ALL_PROBE_SAME_MU_CNT 1 [get_debug_cores u_ila_0]
set_property C_ADV_TRIGGER false [get_debug_cores u_ila_0]
set_property C_DATA_DEPTH 4096 [get_debug_cores u_ila_0]
set_property C_EN_STRG_QUAL false [get_debug_cores u_ila_0]
set_property C_INPUT_PIPE_STAGES 0 [get_debug_cores u_ila_0]
set_property C_TRIGIN_EN false [get_debug_cores u_ila_0]
set_property C_TRIGOUT_EN false [get_debug_cores u_ila_0]
#2
set_property port_width 1 [get_debug_ports u_ila_0/clk]
connect_debug_port u_ila_0/clk [get_nets [list user_clk]]
#3
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe0]
set_property port_width 32 [get_debug_ports u_ila_0/probe0]
connect_debug_port u_ila_0/probe0 [get_nets [list {u_jtag_proc/bit_count[0]} {u_jtag_proc/bit_count[1]} {u_jtag_proc/bit_count[2]} {u_jtag_proc/bit_count[3]} {u_jtag_proc/bit_count[4]} {u_jtag_proc/bit_count[5]} {u_jtag_proc/bit_count[6]} {u_jtag_proc/bit_count[7]} {u_jtag_proc/bit_count[8]} {u_jtag_proc/bit_count[9]} {u_jtag_proc/bit_count[10]} {u_jtag_proc/bit_count[11]} {u_jtag_proc/bit_count[12]} {u_jtag_proc/bit_count[13]} {u_jtag_proc/bit_count[14]} {u_jtag_proc/bit_count[15]} {u_jtag_proc/bit_count[16]} {u_jtag_proc/bit_count[17]} {u_jtag_proc/bit_count[18]} {u_jtag_proc/bit_count[19]} {u_jtag_proc/bit_count[20]} {u_jtag_proc/bit_count[21]} {u_jtag_proc/bit_count[22]} {u_jtag_proc/bit_count[23]} {u_jtag_proc/bit_count[24]} {u_jtag_proc/bit_count[25]} {u_jtag_proc/bit_count[26]} {u_jtag_proc/bit_count[27]} {u_jtag_proc/bit_count[28]} {u_jtag_proc/bit_count[29]} {u_jtag_proc/bit_count[30]} {u_jtag_proc/bit_count[31]}]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe1]
set_property port_width 3 [get_debug_ports u_ila_0/probe1]
connect_debug_port u_ila_0/probe1 [get_nets [list {u_jtag_proc/next_state[0]} {u_jtag_proc/next_state[1]} {u_jtag_proc/next_state[2]}]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe2]
set_property port_width 3 [get_debug_ports u_ila_0/probe2]
connect_debug_port u_ila_0/probe2 [get_nets [list {u_jtag_proc/state[0]} {u_jtag_proc/state[1]} {u_jtag_proc/state[2]}]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe3]
set_property port_width 8 [get_debug_ports u_ila_0/probe3]
connect_debug_port u_ila_0/probe3 [get_nets [list {u_jtag_proc/tck_count[0]} {u_jtag_proc/tck_count[1]} {u_jtag_proc/tck_count[2]} {u_jtag_proc/tck_count[3]} {u_jtag_proc/tck_count[4]} {u_jtag_proc/tck_count[5]} {u_jtag_proc/tck_count[6]} {u_jtag_proc/tck_count[7]}]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe36]
set_property port_width 1 [get_debug_ports u_ila_0/probe36]
connect_debug_port u_ila_0/probe36 [get_nets [list u_jtag_proc/done_i]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe37]
set_property port_width 1 [get_debug_ports u_ila_0/probe37]
connect_debug_port u_ila_0/probe37 [get_nets [list u_jtag_proc/enable_d]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe38]
set_property port_width 1 [get_debug_ports u_ila_0/probe38]
connect_debug_port u_ila_0/probe38 [get_nets [list u_jtag_proc/enable_red]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe39]
set_property port_width 1 [get_debug_ports u_ila_0/probe39]
connect_debug_port u_ila_0/probe39 [get_nets [list sub_ack_i]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe40]
set_property port_width 1 [get_debug_ports u_ila_0/probe40]
connect_debug_port u_ila_0/probe40 [get_nets [list sub_ack_i_ramrd]]

#4
set_property C_CLK_INPUT_FREQ_HZ 300000000 [get_debug_cores dbg_hub]
set_property C_ENABLE_CLK_DIVIDER false [get_debug_cores dbg_hub]
set_property C_USER_SCAN_CHAIN 1 [get_debug_cores dbg_hub]
connect_debug_port dbg_hub/clk [get_nets user_clk]

# Multi-cycle paths for ALU
set_multicycle_path -through [get_pins cpuEngine/or1200_cpu/or1200_alu/*] 2
set_multicycle_path -hold -through [get_pins cpuEngine/or1200_cpu/or1200_alu/*] 1


(* clock_buffer_type="NONE" *)   (* dont_touch="true" *) 


place_cell clk_sub_IBUF_BUFG_inst BUFGCE_X1Y155/BUFCE

 (* keep="true" *)   (* dont_touch="true" *) (* clock_buffer_type="NONE" *)


  BUFG u_clk_ibufg(                                                                            
    .I              (clk_sub),                                                                  
    .O              (sub_clk_int)                                                                    
  ); //6.25MHz
set_property BEL BUFCE [get_cells u_clk_ibufg]
set_property LOC BUFGCE_X1Y156 [get_cells u_clk_ibufg]

set_property STEPS.OPT_DESIGN.TCL.PRE  {/myscripts/pre_synth.tcl}  [get_runs impl_1]
set_property STEPS.WRITE_BITSTREAM.TCL.PRE *.tcl [get_runs impl_1]















