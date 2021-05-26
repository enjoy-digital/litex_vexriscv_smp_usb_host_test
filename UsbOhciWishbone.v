// Generator : SpinalHDL v1.4.4    git head : c4a387537a6ae79dcb5f81bf20579019bc65866f
// Component : UsbOhciWishbone
// Git hash  : 027d2ce0f89eb3cd8b2b90c711ec8dd2e75d0aa8


`define MainState_binary_sequential_type [1:0]
`define MainState_binary_sequential_RESET 2'b00
`define MainState_binary_sequential_RESUME 2'b01
`define MainState_binary_sequential_OPERATIONAL 2'b10
`define MainState_binary_sequential_SUSPEND 2'b11

`define FlowType_binary_sequential_type [1:0]
`define FlowType_binary_sequential_BULK 2'b00
`define FlowType_binary_sequential_CONTROL 2'b01
`define FlowType_binary_sequential_PERIODIC 2'b10

`define endpoint_Status_binary_sequential_type [0:0]
`define endpoint_Status_binary_sequential_OK 1'b0
`define endpoint_Status_binary_sequential_FRAME_TIME 1'b1

`define token_enumDefinition_binary_sequential_type [2:0]
`define token_enumDefinition_binary_sequential_token_BOOT 3'b000
`define token_enumDefinition_binary_sequential_token_INIT 3'b001
`define token_enumDefinition_binary_sequential_token_PID 3'b010
`define token_enumDefinition_binary_sequential_token_B1 3'b011
`define token_enumDefinition_binary_sequential_token_B2 3'b100
`define token_enumDefinition_binary_sequential_token_EOP 3'b101

`define dataTx_enumDefinition_binary_sequential_type [2:0]
`define dataTx_enumDefinition_binary_sequential_dataTx_BOOT 3'b000
`define dataTx_enumDefinition_binary_sequential_dataTx_PID 3'b001
`define dataTx_enumDefinition_binary_sequential_dataTx_DATA 3'b010
`define dataTx_enumDefinition_binary_sequential_dataTx_CRC_0 3'b011
`define dataTx_enumDefinition_binary_sequential_dataTx_CRC_1 3'b100
`define dataTx_enumDefinition_binary_sequential_dataTx_EOP 3'b101

`define dataRx_enumDefinition_binary_sequential_type [1:0]
`define dataRx_enumDefinition_binary_sequential_dataRx_BOOT 2'b00
`define dataRx_enumDefinition_binary_sequential_dataRx_IDLE 2'b01
`define dataRx_enumDefinition_binary_sequential_dataRx_PID 2'b10
`define dataRx_enumDefinition_binary_sequential_dataRx_DATA 2'b11

`define sof_enumDefinition_binary_sequential_type [1:0]
`define sof_enumDefinition_binary_sequential_sof_BOOT 2'b00
`define sof_enumDefinition_binary_sequential_sof_FRAME_TX 2'b01
`define sof_enumDefinition_binary_sequential_sof_FRAME_NUMBER_CMD 2'b10
`define sof_enumDefinition_binary_sequential_sof_FRAME_NUMBER_RSP 2'b11

`define endpoint_enumDefinition_binary_sequential_type [4:0]
`define endpoint_enumDefinition_binary_sequential_endpoint_BOOT 5'b00000
`define endpoint_enumDefinition_binary_sequential_endpoint_ED_READ_CMD 5'b00001
`define endpoint_enumDefinition_binary_sequential_endpoint_ED_READ_RSP 5'b00010
`define endpoint_enumDefinition_binary_sequential_endpoint_ED_ANALYSE 5'b00011
`define endpoint_enumDefinition_binary_sequential_endpoint_TD_READ_CMD 5'b00100
`define endpoint_enumDefinition_binary_sequential_endpoint_TD_READ_RSP 5'b00101
`define endpoint_enumDefinition_binary_sequential_endpoint_TD_ANALYSE 5'b00110
`define endpoint_enumDefinition_binary_sequential_endpoint_TD_CHECK_TIME 5'b00111
`define endpoint_enumDefinition_binary_sequential_endpoint_BUFFER_READ 5'b01000
`define endpoint_enumDefinition_binary_sequential_endpoint_TOKEN 5'b01001
`define endpoint_enumDefinition_binary_sequential_endpoint_DATA_TX 5'b01010
`define endpoint_enumDefinition_binary_sequential_endpoint_DATA_RX 5'b01011
`define endpoint_enumDefinition_binary_sequential_endpoint_DATA_RX_VALIDATE 5'b01100
`define endpoint_enumDefinition_binary_sequential_endpoint_ACK_RX 5'b01101
`define endpoint_enumDefinition_binary_sequential_endpoint_ACK_TX_0 5'b01110
`define endpoint_enumDefinition_binary_sequential_endpoint_ACK_TX_1 5'b01111
`define endpoint_enumDefinition_binary_sequential_endpoint_ACK_TX_EOP 5'b10000
`define endpoint_enumDefinition_binary_sequential_endpoint_DATA_RX_WAIT_DMA 5'b10001
`define endpoint_enumDefinition_binary_sequential_endpoint_UPDATE_TD_PROCESS 5'b10010
`define endpoint_enumDefinition_binary_sequential_endpoint_UPDATE_TD_CMD 5'b10011
`define endpoint_enumDefinition_binary_sequential_endpoint_UPDATE_ED_CMD 5'b10100
`define endpoint_enumDefinition_binary_sequential_endpoint_UPDATE_SYNC 5'b10101
`define endpoint_enumDefinition_binary_sequential_endpoint_ABORD 5'b10110

`define endpoint_dmaLogic_enumDefinition_binary_sequential_type [2:0]
`define endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_BOOT 3'b000
`define endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_INIT 3'b001
`define endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_TO_USB 3'b010
`define endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_FROM_USB 3'b011
`define endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_VALIDATION 3'b100
`define endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_CALC_CMD 3'b101
`define endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_READ_CMD 3'b110
`define endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_WRITE_CMD 3'b111

`define operational_enumDefinition_binary_sequential_type [2:0]
`define operational_enumDefinition_binary_sequential_operational_BOOT 3'b000
`define operational_enumDefinition_binary_sequential_operational_SOF 3'b001
`define operational_enumDefinition_binary_sequential_operational_ARBITER 3'b010
`define operational_enumDefinition_binary_sequential_operational_END_POINT 3'b011
`define operational_enumDefinition_binary_sequential_operational_PERIODIC_HEAD_CMD 3'b100
`define operational_enumDefinition_binary_sequential_operational_PERIODIC_HEAD_RSP 3'b101
`define operational_enumDefinition_binary_sequential_operational_WAIT_SOF 3'b110

`define hc_enumDefinition_binary_sequential_type [2:0]
`define hc_enumDefinition_binary_sequential_hc_BOOT 3'b000
`define hc_enumDefinition_binary_sequential_hc_RESET 3'b001
`define hc_enumDefinition_binary_sequential_hc_RESUME 3'b010
`define hc_enumDefinition_binary_sequential_hc_OPERATIONAL 3'b011
`define hc_enumDefinition_binary_sequential_hc_SUSPEND 3'b100
`define hc_enumDefinition_binary_sequential_hc_ANY_TO_RESET 3'b101
`define hc_enumDefinition_binary_sequential_hc_ANY_TO_SUSPEND 3'b110

`define txShared_frame_enumDefinition_binary_sequential_type [3:0]
`define txShared_frame_enumDefinition_binary_sequential_txShared_frame_BOOT 4'b0000
`define txShared_frame_enumDefinition_binary_sequential_txShared_frame_IDLE 4'b0001
`define txShared_frame_enumDefinition_binary_sequential_txShared_frame_TAKE_LINE 4'b0010
`define txShared_frame_enumDefinition_binary_sequential_txShared_frame_PREAMBLE_SYNC 4'b0011
`define txShared_frame_enumDefinition_binary_sequential_txShared_frame_PREAMBLE_PID 4'b0100
`define txShared_frame_enumDefinition_binary_sequential_txShared_frame_PREAMBLE_DELAY 4'b0101
`define txShared_frame_enumDefinition_binary_sequential_txShared_frame_SYNC 4'b0110
`define txShared_frame_enumDefinition_binary_sequential_txShared_frame_DATA 4'b0111
`define txShared_frame_enumDefinition_binary_sequential_txShared_frame_EOP_0 4'b1000
`define txShared_frame_enumDefinition_binary_sequential_txShared_frame_EOP_1 4'b1001
`define txShared_frame_enumDefinition_binary_sequential_txShared_frame_EOP_2 4'b1010

`define upstreamRx_enumDefinition_binary_sequential_type [1:0]
`define upstreamRx_enumDefinition_binary_sequential_upstreamRx_BOOT 2'b00
`define upstreamRx_enumDefinition_binary_sequential_upstreamRx_IDLE 2'b01
`define upstreamRx_enumDefinition_binary_sequential_upstreamRx_SUSPEND 2'b10

`define ports_0_rx_packet_enumDefinition_binary_sequential_type [1:0]
`define ports_0_rx_packet_enumDefinition_binary_sequential_ports_0_rx_packet_BOOT 2'b00
`define ports_0_rx_packet_enumDefinition_binary_sequential_ports_0_rx_packet_IDLE 2'b01
`define ports_0_rx_packet_enumDefinition_binary_sequential_ports_0_rx_packet_PACKET 2'b10
`define ports_0_rx_packet_enumDefinition_binary_sequential_ports_0_rx_packet_ERRORED 2'b11

`define ports_0_fsm_enumDefinition_binary_sequential_type [3:0]
`define ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_BOOT 4'b0000
`define ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_POWER_OFF 4'b0001
`define ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_DISCONNECTED 4'b0010
`define ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_DISABLED 4'b0011
`define ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_RESETTING 4'b0100
`define ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_RESETTING_DELAY 4'b0101
`define ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_RESETTING_SYNC 4'b0110
`define ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_ENABLED 4'b0111
`define ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_SUSPENDED 4'b1000
`define ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_RESUMING 4'b1001
`define ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_SEND_EOP_0 4'b1010
`define ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_SEND_EOP_1 4'b1011
`define ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_RESTART_S 4'b1100
`define ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_RESTART_E 4'b1101


module UsbOhciWishbone (
  output              io_dma_CYC,
  output              io_dma_STB,
  input               io_dma_ACK,
  output              io_dma_WE,
  output     [29:0]   io_dma_ADR,
  input      [31:0]   io_dma_DAT_MISO,
  output     [31:0]   io_dma_DAT_MOSI,
  output     [3:0]    io_dma_SEL,
  input               io_dma_ERR,
  output     [2:0]    io_dma_CTI,
  output     [1:0]    io_dma_BTE,
  input               io_ctrl_CYC,
  input               io_ctrl_STB,
  output              io_ctrl_ACK,
  input               io_ctrl_WE,
  input      [9:0]    io_ctrl_ADR,
  output     [31:0]   io_ctrl_DAT_MISO,
  input      [31:0]   io_ctrl_DAT_MOSI,
  input      [3:0]    io_ctrl_SEL,
  output              io_interrupt,
  input               io_usb_0_dp_read,
  output              io_usb_0_dp_write,
  output              io_usb_0_dp_writeEnable,
  input               io_usb_0_dm_read,
  output              io_usb_0_dm_write,
  output              io_usb_0_dm_writeEnable,
  input               phy_clk,
  input               phy_reset,
  input               ctrl_clk,
  input               ctrl_reset
);
  wire                _zz_1;
  wire                front_dmaBridge_io_input_cmd_ready;
  wire                front_dmaBridge_io_input_rsp_valid;
  wire                front_dmaBridge_io_input_rsp_payload_last;
  wire       [0:0]    front_dmaBridge_io_input_rsp_payload_fragment_opcode;
  wire       [31:0]   front_dmaBridge_io_input_rsp_payload_fragment_data;
  wire       [31:0]   front_dmaBridge_io_output_DAT_MOSI;
  wire       [29:0]   front_dmaBridge_io_output_ADR;
  wire                front_dmaBridge_io_output_CYC;
  wire       [3:0]    front_dmaBridge_io_output_SEL;
  wire                front_dmaBridge_io_output_STB;
  wire                front_dmaBridge_io_output_WE;
  wire       [2:0]    front_dmaBridge_io_output_CTI;
  wire       [1:0]    front_dmaBridge_io_output_BTE;
  wire       [31:0]   front_ctrlBridge_io_input_DAT_MISO;
  wire                front_ctrlBridge_io_input_ACK;
  wire                front_ctrlBridge_io_output_cmd_valid;
  wire                front_ctrlBridge_io_output_cmd_payload_last;
  wire       [0:0]    front_ctrlBridge_io_output_cmd_payload_fragment_opcode;
  wire       [11:0]   front_ctrlBridge_io_output_cmd_payload_fragment_address;
  wire       [1:0]    front_ctrlBridge_io_output_cmd_payload_fragment_length;
  wire       [31:0]   front_ctrlBridge_io_output_cmd_payload_fragment_data;
  wire       [3:0]    front_ctrlBridge_io_output_cmd_payload_fragment_mask;
  wire                front_ctrlBridge_io_output_rsp_ready;
  wire                front_ohci_io_ctrl_cmd_ready;
  wire                front_ohci_io_ctrl_rsp_valid;
  wire                front_ohci_io_ctrl_rsp_payload_last;
  wire       [0:0]    front_ohci_io_ctrl_rsp_payload_fragment_opcode;
  wire       [31:0]   front_ohci_io_ctrl_rsp_payload_fragment_data;
  wire                front_ohci_io_phy_lowSpeed;
  wire                front_ohci_io_phy_usbReset;
  wire                front_ohci_io_phy_usbResume;
  wire                front_ohci_io_phy_tx_valid;
  wire                front_ohci_io_phy_tx_payload_last;
  wire       [7:0]    front_ohci_io_phy_tx_payload_fragment;
  wire                front_ohci_io_phy_ports_0_removable;
  wire                front_ohci_io_phy_ports_0_power;
  wire                front_ohci_io_phy_ports_0_reset_valid;
  wire                front_ohci_io_phy_ports_0_suspend_valid;
  wire                front_ohci_io_phy_ports_0_resume_valid;
  wire                front_ohci_io_phy_ports_0_disable_valid;
  wire                front_ohci_io_dma_cmd_valid;
  wire                front_ohci_io_dma_cmd_payload_last;
  wire       [0:0]    front_ohci_io_dma_cmd_payload_fragment_opcode;
  wire       [31:0]   front_ohci_io_dma_cmd_payload_fragment_address;
  wire       [5:0]    front_ohci_io_dma_cmd_payload_fragment_length;
  wire       [31:0]   front_ohci_io_dma_cmd_payload_fragment_data;
  wire       [3:0]    front_ohci_io_dma_cmd_payload_fragment_mask;
  wire                front_ohci_io_dma_rsp_ready;
  wire                front_ohci_io_interrupt;
  wire                front_ohci_io_interruptBios;
  wire                back_phy_io_ctrl_overcurrent;
  wire                back_phy_io_ctrl_tick;
  wire                back_phy_io_ctrl_tx_ready;
  wire                back_phy_io_ctrl_txEop;
  wire                back_phy_io_ctrl_rx_flow_valid;
  wire                back_phy_io_ctrl_rx_flow_payload_stuffingError;
  wire       [7:0]    back_phy_io_ctrl_rx_flow_payload_data;
  wire                back_phy_io_ctrl_rx_active;
  wire                back_phy_io_ctrl_ports_0_reset_ready;
  wire                back_phy_io_ctrl_ports_0_suspend_ready;
  wire                back_phy_io_ctrl_ports_0_resume_ready;
  wire                back_phy_io_ctrl_ports_0_disable_ready;
  wire                back_phy_io_ctrl_ports_0_connect;
  wire                back_phy_io_ctrl_ports_0_disconnect;
  wire                back_phy_io_ctrl_ports_0_overcurrent;
  wire                back_phy_io_ctrl_ports_0_lowSpeed;
  wire                back_phy_io_ctrl_ports_0_remoteResume;
  wire                back_phy_io_usb_0_tx_enable;
  wire                back_phy_io_usb_0_tx_data;
  wire                back_phy_io_usb_0_tx_se0;
  wire                back_phy_io_usb_0_power;
  wire                cc_input_overcurrent;
  wire                cc_input_tick;
  wire                cc_input_tx_ready;
  wire                cc_input_txEop;
  wire                cc_input_rx_flow_valid;
  wire                cc_input_rx_flow_payload_stuffingError;
  wire       [7:0]    cc_input_rx_flow_payload_data;
  wire                cc_input_rx_active;
  wire                cc_input_ports_0_reset_ready;
  wire                cc_input_ports_0_suspend_ready;
  wire                cc_input_ports_0_resume_ready;
  wire                cc_input_ports_0_disable_ready;
  wire                cc_input_ports_0_connect;
  wire                cc_input_ports_0_disconnect;
  wire                cc_input_ports_0_overcurrent;
  wire                cc_input_ports_0_lowSpeed;
  wire                cc_input_ports_0_remoteResume;
  wire                cc_output_lowSpeed;
  wire                cc_output_usbReset;
  wire                cc_output_usbResume;
  wire                cc_output_tx_valid;
  wire                cc_output_tx_payload_last;
  wire       [7:0]    cc_output_tx_payload_fragment;
  wire                cc_output_ports_0_removable;
  wire                cc_output_ports_0_power;
  wire                cc_output_ports_0_reset_valid;
  wire                cc_output_ports_0_suspend_valid;
  wire                cc_output_ports_0_resume_valid;
  wire                cc_output_ports_0_disable_valid;
  wire                back_native_0_dp_read;
  wire                back_native_0_dp_write;
  wire                back_native_0_dp_writeEnable;
  wire                back_native_0_dm_read;
  wire                back_native_0_dm_write;
  wire                back_native_0_dm_writeEnable;
  wire                back_buffer_0_dp_read;
  wire                back_buffer_0_dp_write;
  wire                back_buffer_0_dp_writeEnable;
  wire                back_buffer_0_dm_read;
  wire                back_buffer_0_dm_write;
  wire                back_buffer_0_dm_writeEnable;
  wire                back_native_0_dp_stage_read;
  wire                back_native_0_dp_stage_write;
  wire                back_native_0_dp_stage_writeEnable;
  reg                 back_native_0_dp_writeEnable_regNext;
  reg                 back_native_0_dp_write_regNext;
  reg                 back_native_0_dp_stage_read_regNext;
  wire                back_native_0_dm_stage_read;
  wire                back_native_0_dm_stage_write;
  wire                back_native_0_dm_stage_writeEnable;
  reg                 back_native_0_dm_writeEnable_regNext;
  reg                 back_native_0_dm_write_regNext;
  reg                 back_native_0_dm_stage_read_regNext;
  wire                back_buffer_0_dp_stage_read;
  wire                back_buffer_0_dp_stage_write;
  wire                back_buffer_0_dp_stage_writeEnable;
  reg                 back_buffer_0_dp_writeEnable_regNext;
  reg                 back_buffer_0_dp_write_regNext;
  reg                 back_buffer_0_dp_stage_read_regNext;
  wire                back_buffer_0_dm_stage_read;
  wire                back_buffer_0_dm_stage_write;
  wire                back_buffer_0_dm_stage_writeEnable;
  reg                 back_buffer_0_dm_writeEnable_regNext;
  reg                 back_buffer_0_dm_write_regNext;
  reg                 back_buffer_0_dm_stage_read_regNext;

  UsbOhciWishbone_BmbToWishbone front_dmaBridge (
    .io_input_cmd_valid                       (front_ohci_io_dma_cmd_valid                               ), //i
    .io_input_cmd_ready                       (front_dmaBridge_io_input_cmd_ready                        ), //o
    .io_input_cmd_payload_last                (front_ohci_io_dma_cmd_payload_last                        ), //i
    .io_input_cmd_payload_fragment_opcode     (front_ohci_io_dma_cmd_payload_fragment_opcode             ), //i
    .io_input_cmd_payload_fragment_address    (front_ohci_io_dma_cmd_payload_fragment_address[31:0]      ), //i
    .io_input_cmd_payload_fragment_length     (front_ohci_io_dma_cmd_payload_fragment_length[5:0]        ), //i
    .io_input_cmd_payload_fragment_data       (front_ohci_io_dma_cmd_payload_fragment_data[31:0]         ), //i
    .io_input_cmd_payload_fragment_mask       (front_ohci_io_dma_cmd_payload_fragment_mask[3:0]          ), //i
    .io_input_rsp_valid                       (front_dmaBridge_io_input_rsp_valid                        ), //o
    .io_input_rsp_ready                       (front_ohci_io_dma_rsp_ready                               ), //i
    .io_input_rsp_payload_last                (front_dmaBridge_io_input_rsp_payload_last                 ), //o
    .io_input_rsp_payload_fragment_opcode     (front_dmaBridge_io_input_rsp_payload_fragment_opcode      ), //o
    .io_input_rsp_payload_fragment_data       (front_dmaBridge_io_input_rsp_payload_fragment_data[31:0]  ), //o
    .io_output_CYC                            (front_dmaBridge_io_output_CYC                             ), //o
    .io_output_STB                            (front_dmaBridge_io_output_STB                             ), //o
    .io_output_ACK                            (io_dma_ACK                                                ), //i
    .io_output_WE                             (front_dmaBridge_io_output_WE                              ), //o
    .io_output_ADR                            (front_dmaBridge_io_output_ADR[29:0]                       ), //o
    .io_output_DAT_MISO                       (io_dma_DAT_MISO[31:0]                                     ), //i
    .io_output_DAT_MOSI                       (front_dmaBridge_io_output_DAT_MOSI[31:0]                  ), //o
    .io_output_SEL                            (front_dmaBridge_io_output_SEL[3:0]                        ), //o
    .io_output_ERR                            (io_dma_ERR                                                ), //i
    .io_output_CTI                            (front_dmaBridge_io_output_CTI[2:0]                        ), //o
    .io_output_BTE                            (front_dmaBridge_io_output_BTE[1:0]                        ), //o
    .ctrl_clk                                 (ctrl_clk                                                  ), //i
    .ctrl_reset                               (ctrl_reset                                                )  //i
  );
  UsbOhciWishbone_WishboneToBmb front_ctrlBridge (
    .io_input_CYC                              (io_ctrl_CYC                                                    ), //i
    .io_input_STB                              (io_ctrl_STB                                                    ), //i
    .io_input_ACK                              (front_ctrlBridge_io_input_ACK                                  ), //o
    .io_input_WE                               (io_ctrl_WE                                                     ), //i
    .io_input_ADR                              (io_ctrl_ADR[9:0]                                               ), //i
    .io_input_DAT_MISO                         (front_ctrlBridge_io_input_DAT_MISO[31:0]                       ), //o
    .io_input_DAT_MOSI                         (io_ctrl_DAT_MOSI[31:0]                                         ), //i
    .io_input_SEL                              (io_ctrl_SEL[3:0]                                               ), //i
    .io_output_cmd_valid                       (front_ctrlBridge_io_output_cmd_valid                           ), //o
    .io_output_cmd_ready                       (front_ohci_io_ctrl_cmd_ready                                   ), //i
    .io_output_cmd_payload_last                (front_ctrlBridge_io_output_cmd_payload_last                    ), //o
    .io_output_cmd_payload_fragment_opcode     (front_ctrlBridge_io_output_cmd_payload_fragment_opcode         ), //o
    .io_output_cmd_payload_fragment_address    (front_ctrlBridge_io_output_cmd_payload_fragment_address[11:0]  ), //o
    .io_output_cmd_payload_fragment_length     (front_ctrlBridge_io_output_cmd_payload_fragment_length[1:0]    ), //o
    .io_output_cmd_payload_fragment_data       (front_ctrlBridge_io_output_cmd_payload_fragment_data[31:0]     ), //o
    .io_output_cmd_payload_fragment_mask       (front_ctrlBridge_io_output_cmd_payload_fragment_mask[3:0]      ), //o
    .io_output_rsp_valid                       (front_ohci_io_ctrl_rsp_valid                                   ), //i
    .io_output_rsp_ready                       (front_ctrlBridge_io_output_rsp_ready                           ), //o
    .io_output_rsp_payload_last                (front_ohci_io_ctrl_rsp_payload_last                            ), //i
    .io_output_rsp_payload_fragment_opcode     (front_ohci_io_ctrl_rsp_payload_fragment_opcode                 ), //i
    .io_output_rsp_payload_fragment_data       (front_ohci_io_ctrl_rsp_payload_fragment_data[31:0]             ), //i
    .ctrl_clk                                  (ctrl_clk                                                       ), //i
    .ctrl_reset                                (ctrl_reset                                                     )  //i
  );
  UsbOhciWishbone_UsbOhci front_ohci (
    .io_ctrl_cmd_valid                       (front_ctrlBridge_io_output_cmd_valid                           ), //i
    .io_ctrl_cmd_ready                       (front_ohci_io_ctrl_cmd_ready                                   ), //o
    .io_ctrl_cmd_payload_last                (front_ctrlBridge_io_output_cmd_payload_last                    ), //i
    .io_ctrl_cmd_payload_fragment_opcode     (front_ctrlBridge_io_output_cmd_payload_fragment_opcode         ), //i
    .io_ctrl_cmd_payload_fragment_address    (front_ctrlBridge_io_output_cmd_payload_fragment_address[11:0]  ), //i
    .io_ctrl_cmd_payload_fragment_length     (front_ctrlBridge_io_output_cmd_payload_fragment_length[1:0]    ), //i
    .io_ctrl_cmd_payload_fragment_data       (front_ctrlBridge_io_output_cmd_payload_fragment_data[31:0]     ), //i
    .io_ctrl_cmd_payload_fragment_mask       (front_ctrlBridge_io_output_cmd_payload_fragment_mask[3:0]      ), //i
    .io_ctrl_rsp_valid                       (front_ohci_io_ctrl_rsp_valid                                   ), //o
    .io_ctrl_rsp_ready                       (front_ctrlBridge_io_output_rsp_ready                           ), //i
    .io_ctrl_rsp_payload_last                (front_ohci_io_ctrl_rsp_payload_last                            ), //o
    .io_ctrl_rsp_payload_fragment_opcode     (front_ohci_io_ctrl_rsp_payload_fragment_opcode                 ), //o
    .io_ctrl_rsp_payload_fragment_data       (front_ohci_io_ctrl_rsp_payload_fragment_data[31:0]             ), //o
    .io_phy_lowSpeed                         (front_ohci_io_phy_lowSpeed                                     ), //o
    .io_phy_tx_valid                         (front_ohci_io_phy_tx_valid                                     ), //o
    .io_phy_tx_ready                         (cc_input_tx_ready                                              ), //i
    .io_phy_tx_payload_last                  (front_ohci_io_phy_tx_payload_last                              ), //o
    .io_phy_tx_payload_fragment              (front_ohci_io_phy_tx_payload_fragment[7:0]                     ), //o
    .io_phy_txEop                            (cc_input_txEop                                                 ), //i
    .io_phy_rx_flow_valid                    (cc_input_rx_flow_valid                                         ), //i
    .io_phy_rx_flow_payload_stuffingError    (cc_input_rx_flow_payload_stuffingError                         ), //i
    .io_phy_rx_flow_payload_data             (cc_input_rx_flow_payload_data[7:0]                             ), //i
    .io_phy_rx_active                        (cc_input_rx_active                                             ), //i
    .io_phy_usbReset                         (front_ohci_io_phy_usbReset                                     ), //o
    .io_phy_usbResume                        (front_ohci_io_phy_usbResume                                    ), //o
    .io_phy_overcurrent                      (cc_input_overcurrent                                           ), //i
    .io_phy_tick                             (cc_input_tick                                                  ), //i
    .io_phy_ports_0_disable_valid            (front_ohci_io_phy_ports_0_disable_valid                        ), //o
    .io_phy_ports_0_disable_ready            (cc_input_ports_0_disable_ready                                 ), //i
    .io_phy_ports_0_removable                (front_ohci_io_phy_ports_0_removable                            ), //o
    .io_phy_ports_0_power                    (front_ohci_io_phy_ports_0_power                                ), //o
    .io_phy_ports_0_reset_valid              (front_ohci_io_phy_ports_0_reset_valid                          ), //o
    .io_phy_ports_0_reset_ready              (cc_input_ports_0_reset_ready                                   ), //i
    .io_phy_ports_0_suspend_valid            (front_ohci_io_phy_ports_0_suspend_valid                        ), //o
    .io_phy_ports_0_suspend_ready            (cc_input_ports_0_suspend_ready                                 ), //i
    .io_phy_ports_0_resume_valid             (front_ohci_io_phy_ports_0_resume_valid                         ), //o
    .io_phy_ports_0_resume_ready             (cc_input_ports_0_resume_ready                                  ), //i
    .io_phy_ports_0_connect                  (cc_input_ports_0_connect                                       ), //i
    .io_phy_ports_0_disconnect               (cc_input_ports_0_disconnect                                    ), //i
    .io_phy_ports_0_overcurrent              (cc_input_ports_0_overcurrent                                   ), //i
    .io_phy_ports_0_remoteResume             (cc_input_ports_0_remoteResume                                  ), //i
    .io_phy_ports_0_lowSpeed                 (cc_input_ports_0_lowSpeed                                      ), //i
    .io_dma_cmd_valid                        (front_ohci_io_dma_cmd_valid                                    ), //o
    .io_dma_cmd_ready                        (front_dmaBridge_io_input_cmd_ready                             ), //i
    .io_dma_cmd_payload_last                 (front_ohci_io_dma_cmd_payload_last                             ), //o
    .io_dma_cmd_payload_fragment_opcode      (front_ohci_io_dma_cmd_payload_fragment_opcode                  ), //o
    .io_dma_cmd_payload_fragment_address     (front_ohci_io_dma_cmd_payload_fragment_address[31:0]           ), //o
    .io_dma_cmd_payload_fragment_length      (front_ohci_io_dma_cmd_payload_fragment_length[5:0]             ), //o
    .io_dma_cmd_payload_fragment_data        (front_ohci_io_dma_cmd_payload_fragment_data[31:0]              ), //o
    .io_dma_cmd_payload_fragment_mask        (front_ohci_io_dma_cmd_payload_fragment_mask[3:0]               ), //o
    .io_dma_rsp_valid                        (front_dmaBridge_io_input_rsp_valid                             ), //i
    .io_dma_rsp_ready                        (front_ohci_io_dma_rsp_ready                                    ), //o
    .io_dma_rsp_payload_last                 (front_dmaBridge_io_input_rsp_payload_last                      ), //i
    .io_dma_rsp_payload_fragment_opcode      (front_dmaBridge_io_input_rsp_payload_fragment_opcode           ), //i
    .io_dma_rsp_payload_fragment_data        (front_dmaBridge_io_input_rsp_payload_fragment_data[31:0]       ), //i
    .io_interrupt                            (front_ohci_io_interrupt                                        ), //o
    .io_interruptBios                        (front_ohci_io_interruptBios                                    ), //o
    .ctrl_clk                                (ctrl_clk                                                       ), //i
    .ctrl_reset                              (ctrl_reset                                                     )  //i
  );
  UsbOhciWishbone_UsbLsFsPhy back_phy (
    .io_ctrl_lowSpeed                         (cc_output_lowSpeed                              ), //i
    .io_ctrl_tx_valid                         (cc_output_tx_valid                              ), //i
    .io_ctrl_tx_ready                         (back_phy_io_ctrl_tx_ready                       ), //o
    .io_ctrl_tx_payload_last                  (cc_output_tx_payload_last                       ), //i
    .io_ctrl_tx_payload_fragment              (cc_output_tx_payload_fragment[7:0]              ), //i
    .io_ctrl_txEop                            (back_phy_io_ctrl_txEop                          ), //o
    .io_ctrl_rx_flow_valid                    (back_phy_io_ctrl_rx_flow_valid                  ), //o
    .io_ctrl_rx_flow_payload_stuffingError    (back_phy_io_ctrl_rx_flow_payload_stuffingError  ), //o
    .io_ctrl_rx_flow_payload_data             (back_phy_io_ctrl_rx_flow_payload_data[7:0]      ), //o
    .io_ctrl_rx_active                        (back_phy_io_ctrl_rx_active                      ), //o
    .io_ctrl_usbReset                         (cc_output_usbReset                              ), //i
    .io_ctrl_usbResume                        (cc_output_usbResume                             ), //i
    .io_ctrl_overcurrent                      (back_phy_io_ctrl_overcurrent                    ), //o
    .io_ctrl_tick                             (back_phy_io_ctrl_tick                           ), //o
    .io_ctrl_ports_0_disable_valid            (cc_output_ports_0_disable_valid                 ), //i
    .io_ctrl_ports_0_disable_ready            (back_phy_io_ctrl_ports_0_disable_ready          ), //o
    .io_ctrl_ports_0_removable                (cc_output_ports_0_removable                     ), //i
    .io_ctrl_ports_0_power                    (cc_output_ports_0_power                         ), //i
    .io_ctrl_ports_0_reset_valid              (cc_output_ports_0_reset_valid                   ), //i
    .io_ctrl_ports_0_reset_ready              (back_phy_io_ctrl_ports_0_reset_ready            ), //o
    .io_ctrl_ports_0_suspend_valid            (cc_output_ports_0_suspend_valid                 ), //i
    .io_ctrl_ports_0_suspend_ready            (back_phy_io_ctrl_ports_0_suspend_ready          ), //o
    .io_ctrl_ports_0_resume_valid             (cc_output_ports_0_resume_valid                  ), //i
    .io_ctrl_ports_0_resume_ready             (back_phy_io_ctrl_ports_0_resume_ready           ), //o
    .io_ctrl_ports_0_connect                  (back_phy_io_ctrl_ports_0_connect                ), //o
    .io_ctrl_ports_0_disconnect               (back_phy_io_ctrl_ports_0_disconnect             ), //o
    .io_ctrl_ports_0_overcurrent              (back_phy_io_ctrl_ports_0_overcurrent            ), //o
    .io_ctrl_ports_0_remoteResume             (back_phy_io_ctrl_ports_0_remoteResume           ), //o
    .io_ctrl_ports_0_lowSpeed                 (back_phy_io_ctrl_ports_0_lowSpeed               ), //o
    .io_usb_0_tx_enable                       (back_phy_io_usb_0_tx_enable                     ), //o
    .io_usb_0_tx_data                         (back_phy_io_usb_0_tx_data                       ), //o
    .io_usb_0_tx_se0                          (back_phy_io_usb_0_tx_se0                        ), //o
    .io_usb_0_rx_dp                           (back_native_0_dp_read                           ), //i
    .io_usb_0_rx_dm                           (back_native_0_dm_read                           ), //i
    .io_usb_0_overcurrent                     (_zz_1                                           ), //i
    .io_usb_0_power                           (back_phy_io_usb_0_power                         ), //o
    .phy_clk                                  (phy_clk                                         ), //i
    .phy_reset                                (phy_reset                                       )  //i
  );
  UsbOhciWishbone_CtrlCc cc (
    .input_lowSpeed                          (front_ohci_io_phy_lowSpeed                      ), //i
    .input_tx_valid                          (front_ohci_io_phy_tx_valid                      ), //i
    .input_tx_ready                          (cc_input_tx_ready                               ), //o
    .input_tx_payload_last                   (front_ohci_io_phy_tx_payload_last               ), //i
    .input_tx_payload_fragment               (front_ohci_io_phy_tx_payload_fragment[7:0]      ), //i
    .input_txEop                             (cc_input_txEop                                  ), //o
    .input_rx_flow_valid                     (cc_input_rx_flow_valid                          ), //o
    .input_rx_flow_payload_stuffingError     (cc_input_rx_flow_payload_stuffingError          ), //o
    .input_rx_flow_payload_data              (cc_input_rx_flow_payload_data[7:0]              ), //o
    .input_rx_active                         (cc_input_rx_active                              ), //o
    .input_usbReset                          (front_ohci_io_phy_usbReset                      ), //i
    .input_usbResume                         (front_ohci_io_phy_usbResume                     ), //i
    .input_overcurrent                       (cc_input_overcurrent                            ), //o
    .input_tick                              (cc_input_tick                                   ), //o
    .input_ports_0_disable_valid             (front_ohci_io_phy_ports_0_disable_valid         ), //i
    .input_ports_0_disable_ready             (cc_input_ports_0_disable_ready                  ), //o
    .input_ports_0_removable                 (front_ohci_io_phy_ports_0_removable             ), //i
    .input_ports_0_power                     (front_ohci_io_phy_ports_0_power                 ), //i
    .input_ports_0_reset_valid               (front_ohci_io_phy_ports_0_reset_valid           ), //i
    .input_ports_0_reset_ready               (cc_input_ports_0_reset_ready                    ), //o
    .input_ports_0_suspend_valid             (front_ohci_io_phy_ports_0_suspend_valid         ), //i
    .input_ports_0_suspend_ready             (cc_input_ports_0_suspend_ready                  ), //o
    .input_ports_0_resume_valid              (front_ohci_io_phy_ports_0_resume_valid          ), //i
    .input_ports_0_resume_ready              (cc_input_ports_0_resume_ready                   ), //o
    .input_ports_0_connect                   (cc_input_ports_0_connect                        ), //o
    .input_ports_0_disconnect                (cc_input_ports_0_disconnect                     ), //o
    .input_ports_0_overcurrent               (cc_input_ports_0_overcurrent                    ), //o
    .input_ports_0_remoteResume              (cc_input_ports_0_remoteResume                   ), //o
    .input_ports_0_lowSpeed                  (cc_input_ports_0_lowSpeed                       ), //o
    .output_lowSpeed                         (cc_output_lowSpeed                              ), //o
    .output_tx_valid                         (cc_output_tx_valid                              ), //o
    .output_tx_ready                         (back_phy_io_ctrl_tx_ready                       ), //i
    .output_tx_payload_last                  (cc_output_tx_payload_last                       ), //o
    .output_tx_payload_fragment              (cc_output_tx_payload_fragment[7:0]              ), //o
    .output_txEop                            (back_phy_io_ctrl_txEop                          ), //i
    .output_rx_flow_valid                    (back_phy_io_ctrl_rx_flow_valid                  ), //i
    .output_rx_flow_payload_stuffingError    (back_phy_io_ctrl_rx_flow_payload_stuffingError  ), //i
    .output_rx_flow_payload_data             (back_phy_io_ctrl_rx_flow_payload_data[7:0]      ), //i
    .output_rx_active                        (back_phy_io_ctrl_rx_active                      ), //i
    .output_usbReset                         (cc_output_usbReset                              ), //o
    .output_usbResume                        (cc_output_usbResume                             ), //o
    .output_overcurrent                      (back_phy_io_ctrl_overcurrent                    ), //i
    .output_tick                             (back_phy_io_ctrl_tick                           ), //i
    .output_ports_0_disable_valid            (cc_output_ports_0_disable_valid                 ), //o
    .output_ports_0_disable_ready            (back_phy_io_ctrl_ports_0_disable_ready          ), //i
    .output_ports_0_removable                (cc_output_ports_0_removable                     ), //o
    .output_ports_0_power                    (cc_output_ports_0_power                         ), //o
    .output_ports_0_reset_valid              (cc_output_ports_0_reset_valid                   ), //o
    .output_ports_0_reset_ready              (back_phy_io_ctrl_ports_0_reset_ready            ), //i
    .output_ports_0_suspend_valid            (cc_output_ports_0_suspend_valid                 ), //o
    .output_ports_0_suspend_ready            (back_phy_io_ctrl_ports_0_suspend_ready          ), //i
    .output_ports_0_resume_valid             (cc_output_ports_0_resume_valid                  ), //o
    .output_ports_0_resume_ready             (back_phy_io_ctrl_ports_0_resume_ready           ), //i
    .output_ports_0_connect                  (back_phy_io_ctrl_ports_0_connect                ), //i
    .output_ports_0_disconnect               (back_phy_io_ctrl_ports_0_disconnect             ), //i
    .output_ports_0_overcurrent              (back_phy_io_ctrl_ports_0_overcurrent            ), //i
    .output_ports_0_remoteResume             (back_phy_io_ctrl_ports_0_remoteResume           ), //i
    .output_ports_0_lowSpeed                 (back_phy_io_ctrl_ports_0_lowSpeed               ), //i
    .phy_clk                                 (phy_clk                                         ), //i
    .phy_reset                               (phy_reset                                       ), //i
    .ctrl_clk                                (ctrl_clk                                        ), //i
    .ctrl_reset                              (ctrl_reset                                      )  //i
  );
  assign io_dma_CYC = front_dmaBridge_io_output_CYC;
  assign io_dma_STB = front_dmaBridge_io_output_STB;
  assign io_dma_WE = front_dmaBridge_io_output_WE;
  assign io_dma_ADR = front_dmaBridge_io_output_ADR;
  assign io_dma_DAT_MOSI = front_dmaBridge_io_output_DAT_MOSI;
  assign io_dma_SEL = front_dmaBridge_io_output_SEL;
  assign io_dma_CTI = front_dmaBridge_io_output_CTI;
  assign io_dma_BTE = front_dmaBridge_io_output_BTE;
  assign io_ctrl_ACK = front_ctrlBridge_io_input_ACK;
  assign io_ctrl_DAT_MISO = front_ctrlBridge_io_input_DAT_MISO;
  assign io_interrupt = front_ohci_io_interrupt;
  assign _zz_1 = 1'b0;
  assign back_native_0_dp_writeEnable = back_phy_io_usb_0_tx_enable;
  assign back_native_0_dm_writeEnable = back_phy_io_usb_0_tx_enable;
  assign back_native_0_dp_write = ((! back_phy_io_usb_0_tx_se0) && back_phy_io_usb_0_tx_data);
  assign back_native_0_dm_write = ((! back_phy_io_usb_0_tx_se0) && (! back_phy_io_usb_0_tx_data));
  assign back_native_0_dp_stage_writeEnable = back_native_0_dp_writeEnable_regNext;
  assign back_native_0_dp_stage_write = back_native_0_dp_write_regNext;
  assign back_native_0_dp_read = back_native_0_dp_stage_read_regNext;
  assign back_buffer_0_dp_writeEnable = back_native_0_dp_stage_writeEnable;
  assign back_buffer_0_dp_write = back_native_0_dp_stage_write;
  assign back_native_0_dp_stage_read = back_buffer_0_dp_read;
  assign back_native_0_dm_stage_writeEnable = back_native_0_dm_writeEnable_regNext;
  assign back_native_0_dm_stage_write = back_native_0_dm_write_regNext;
  assign back_native_0_dm_read = back_native_0_dm_stage_read_regNext;
  assign back_buffer_0_dm_writeEnable = back_native_0_dm_stage_writeEnable;
  assign back_buffer_0_dm_write = back_native_0_dm_stage_write;
  assign back_native_0_dm_stage_read = back_buffer_0_dm_read;
  assign back_buffer_0_dp_stage_writeEnable = back_buffer_0_dp_writeEnable_regNext;
  assign back_buffer_0_dp_stage_write = back_buffer_0_dp_write_regNext;
  assign back_buffer_0_dp_read = back_buffer_0_dp_stage_read_regNext;
  assign back_buffer_0_dp_stage_read = io_usb_0_dp_read;
  assign back_buffer_0_dm_stage_writeEnable = back_buffer_0_dm_writeEnable_regNext;
  assign back_buffer_0_dm_stage_write = back_buffer_0_dm_write_regNext;
  assign back_buffer_0_dm_read = back_buffer_0_dm_stage_read_regNext;
  assign back_buffer_0_dm_stage_read = io_usb_0_dm_read;
  assign io_usb_0_dp_write = back_buffer_0_dp_stage_write;
  assign io_usb_0_dp_writeEnable = back_buffer_0_dp_stage_writeEnable;
  assign io_usb_0_dm_write = back_buffer_0_dm_stage_write;
  assign io_usb_0_dm_writeEnable = back_buffer_0_dm_stage_writeEnable;
  always @ (posedge phy_clk) begin
    back_native_0_dp_writeEnable_regNext <= back_native_0_dp_writeEnable;
    back_native_0_dp_write_regNext <= back_native_0_dp_write;
    back_native_0_dp_stage_read_regNext <= back_native_0_dp_stage_read;
    back_native_0_dm_writeEnable_regNext <= back_native_0_dm_writeEnable;
    back_native_0_dm_write_regNext <= back_native_0_dm_write;
    back_native_0_dm_stage_read_regNext <= back_native_0_dm_stage_read;
    back_buffer_0_dp_writeEnable_regNext <= back_buffer_0_dp_writeEnable;
    back_buffer_0_dp_write_regNext <= back_buffer_0_dp_write;
    back_buffer_0_dp_stage_read_regNext <= back_buffer_0_dp_stage_read;
    back_buffer_0_dm_writeEnable_regNext <= back_buffer_0_dm_writeEnable;
    back_buffer_0_dm_write_regNext <= back_buffer_0_dm_write;
    back_buffer_0_dm_stage_read_regNext <= back_buffer_0_dm_stage_read;
  end


endmodule

module UsbOhciWishbone_CtrlCc (
  input               input_lowSpeed,
  input               input_tx_valid,
  output              input_tx_ready,
  input               input_tx_payload_last,
  input      [7:0]    input_tx_payload_fragment,
  output              input_txEop,
  output              input_rx_flow_valid,
  output              input_rx_flow_payload_stuffingError,
  output     [7:0]    input_rx_flow_payload_data,
  output              input_rx_active,
  input               input_usbReset,
  input               input_usbResume,
  output              input_overcurrent,
  output              input_tick,
  input               input_ports_0_disable_valid,
  output              input_ports_0_disable_ready,
  input               input_ports_0_removable,
  input               input_ports_0_power,
  input               input_ports_0_reset_valid,
  output              input_ports_0_reset_ready,
  input               input_ports_0_suspend_valid,
  output              input_ports_0_suspend_ready,
  input               input_ports_0_resume_valid,
  output              input_ports_0_resume_ready,
  output              input_ports_0_connect,
  output              input_ports_0_disconnect,
  output              input_ports_0_overcurrent,
  output              input_ports_0_remoteResume,
  output              input_ports_0_lowSpeed,
  output              output_lowSpeed,
  output              output_tx_valid,
  input               output_tx_ready,
  output              output_tx_payload_last,
  output     [7:0]    output_tx_payload_fragment,
  input               output_txEop,
  input               output_rx_flow_valid,
  input               output_rx_flow_payload_stuffingError,
  input      [7:0]    output_rx_flow_payload_data,
  input               output_rx_active,
  output              output_usbReset,
  output              output_usbResume,
  input               output_overcurrent,
  input               output_tick,
  output              output_ports_0_disable_valid,
  input               output_ports_0_disable_ready,
  output              output_ports_0_removable,
  output              output_ports_0_power,
  output              output_ports_0_reset_valid,
  input               output_ports_0_reset_ready,
  output              output_ports_0_suspend_valid,
  input               output_ports_0_suspend_ready,
  output              output_ports_0_resume_valid,
  input               output_ports_0_resume_ready,
  input               output_ports_0_connect,
  input               output_ports_0_disconnect,
  input               output_ports_0_overcurrent,
  input               output_ports_0_remoteResume,
  input               output_ports_0_lowSpeed,
  input               phy_clk,
  input               phy_reset,
  input               ctrl_clk,
  input               ctrl_reset
);
  wire                _zz_1;
  wire                input_lowSpeed_buffercc_io_dataOut;
  wire                input_usbReset_buffercc_io_dataOut;
  wire                input_usbResume_buffercc_io_dataOut;
  wire                output_overcurrent_buffercc_io_dataOut;
  wire                input_tx_ccToggle_io_input_ready;
  wire                input_tx_ccToggle_io_output_valid;
  wire                input_tx_ccToggle_io_output_payload_last;
  wire       [7:0]    input_tx_ccToggle_io_output_payload_fragment;
  wire                pulseCCByToggle_io_pulseOut;
  wire                output_rx_flow_ccToggle_io_output_valid;
  wire                output_rx_flow_ccToggle_io_output_payload_stuffingError;
  wire       [7:0]    output_rx_flow_ccToggle_io_output_payload_data;
  wire                output_rx_active_buffercc_io_dataOut;
  wire                pulseCCByToggle_1_io_pulseOut;
  wire                input_ports_0_removable_buffercc_io_dataOut;
  wire                input_ports_0_power_buffercc_io_dataOut;
  wire                output_ports_0_lowSpeed_buffercc_io_dataOut;
  wire                output_ports_0_overcurrent_buffercc_io_dataOut;
  wire                pulseCCByToggle_2_io_pulseOut;
  wire                pulseCCByToggle_3_io_pulseOut;
  wire                pulseCCByToggle_4_io_pulseOut;
  wire                input_ports_0_reset_ccToggle_io_input_ready;
  wire                input_ports_0_reset_ccToggle_io_output_valid;
  wire                input_ports_0_suspend_ccToggle_io_input_ready;
  wire                input_ports_0_suspend_ccToggle_io_output_valid;
  wire                input_ports_0_resume_ccToggle_io_input_ready;
  wire                input_ports_0_resume_ccToggle_io_output_valid;
  wire                input_ports_0_disable_ccToggle_io_input_ready;
  wire                input_ports_0_disable_ccToggle_io_output_valid;
  wire                input_tx_ccToggle_io_output_m2sPipe_valid;
  wire                input_tx_ccToggle_io_output_m2sPipe_ready;
  wire                input_tx_ccToggle_io_output_m2sPipe_payload_last;
  wire       [7:0]    input_tx_ccToggle_io_output_m2sPipe_payload_fragment;
  reg                 input_tx_ccToggle_io_output_m2sPipe_rValid;
  reg                 input_tx_ccToggle_io_output_m2sPipe_rData_last;
  reg        [7:0]    input_tx_ccToggle_io_output_m2sPipe_rData_fragment;

  UsbOhciWishbone_BufferCC_16 input_lowSpeed_buffercc (
    .io_dataIn     (input_lowSpeed                      ), //i
    .io_dataOut    (input_lowSpeed_buffercc_io_dataOut  ), //o
    .phy_clk       (phy_clk                             ), //i
    .phy_reset     (phy_reset                           )  //i
  );
  UsbOhciWishbone_BufferCC_16 input_usbReset_buffercc (
    .io_dataIn     (input_usbReset                      ), //i
    .io_dataOut    (input_usbReset_buffercc_io_dataOut  ), //o
    .phy_clk       (phy_clk                             ), //i
    .phy_reset     (phy_reset                           )  //i
  );
  UsbOhciWishbone_BufferCC_16 input_usbResume_buffercc (
    .io_dataIn     (input_usbResume                      ), //i
    .io_dataOut    (input_usbResume_buffercc_io_dataOut  ), //o
    .phy_clk       (phy_clk                              ), //i
    .phy_reset     (phy_reset                            )  //i
  );
  UsbOhciWishbone_BufferCC_19 output_overcurrent_buffercc (
    .io_dataIn     (output_overcurrent                      ), //i
    .io_dataOut    (output_overcurrent_buffercc_io_dataOut  ), //o
    .ctrl_clk      (ctrl_clk                                ), //i
    .ctrl_reset    (ctrl_reset                              )  //i
  );
  UsbOhciWishbone_StreamCCByToggle input_tx_ccToggle (
    .io_input_valid                (input_tx_valid                                     ), //i
    .io_input_ready                (input_tx_ccToggle_io_input_ready                   ), //o
    .io_input_payload_last         (input_tx_payload_last                              ), //i
    .io_input_payload_fragment     (input_tx_payload_fragment[7:0]                     ), //i
    .io_output_valid               (input_tx_ccToggle_io_output_valid                  ), //o
    .io_output_ready               (_zz_1                                              ), //i
    .io_output_payload_last        (input_tx_ccToggle_io_output_payload_last           ), //o
    .io_output_payload_fragment    (input_tx_ccToggle_io_output_payload_fragment[7:0]  ), //o
    .ctrl_clk                      (ctrl_clk                                           ), //i
    .ctrl_reset                    (ctrl_reset                                         ), //i
    .phy_clk                       (phy_clk                                            ), //i
    .phy_reset                     (phy_reset                                          )  //i
  );
  UsbOhciWishbone_PulseCCByToggle pulseCCByToggle (
    .io_pulseIn     (output_txEop                 ), //i
    .io_pulseOut    (pulseCCByToggle_io_pulseOut  ), //o
    .phy_clk        (phy_clk                      ), //i
    .phy_reset      (phy_reset                    ), //i
    .ctrl_clk       (ctrl_clk                     ), //i
    .ctrl_reset     (ctrl_reset                   )  //i
  );
  UsbOhciWishbone_FlowCCByToggle output_rx_flow_ccToggle (
    .io_input_valid                     (output_rx_flow_valid                                     ), //i
    .io_input_payload_stuffingError     (output_rx_flow_payload_stuffingError                     ), //i
    .io_input_payload_data              (output_rx_flow_payload_data[7:0]                         ), //i
    .io_output_valid                    (output_rx_flow_ccToggle_io_output_valid                  ), //o
    .io_output_payload_stuffingError    (output_rx_flow_ccToggle_io_output_payload_stuffingError  ), //o
    .io_output_payload_data             (output_rx_flow_ccToggle_io_output_payload_data[7:0]      ), //o
    .phy_clk                            (phy_clk                                                  ), //i
    .phy_reset                          (phy_reset                                                ), //i
    .ctrl_clk                           (ctrl_clk                                                 ), //i
    .ctrl_reset                         (ctrl_reset                                               )  //i
  );
  UsbOhciWishbone_BufferCC_19 output_rx_active_buffercc (
    .io_dataIn     (output_rx_active                      ), //i
    .io_dataOut    (output_rx_active_buffercc_io_dataOut  ), //o
    .ctrl_clk      (ctrl_clk                              ), //i
    .ctrl_reset    (ctrl_reset                            )  //i
  );
  UsbOhciWishbone_PulseCCByToggle pulseCCByToggle_1 (
    .io_pulseIn     (output_tick                    ), //i
    .io_pulseOut    (pulseCCByToggle_1_io_pulseOut  ), //o
    .phy_clk        (phy_clk                        ), //i
    .phy_reset      (phy_reset                      ), //i
    .ctrl_clk       (ctrl_clk                       ), //i
    .ctrl_reset     (ctrl_reset                     )  //i
  );
  UsbOhciWishbone_BufferCC_16 input_ports_0_removable_buffercc (
    .io_dataIn     (input_ports_0_removable                      ), //i
    .io_dataOut    (input_ports_0_removable_buffercc_io_dataOut  ), //o
    .phy_clk       (phy_clk                                      ), //i
    .phy_reset     (phy_reset                                    )  //i
  );
  UsbOhciWishbone_BufferCC_16 input_ports_0_power_buffercc (
    .io_dataIn     (input_ports_0_power                      ), //i
    .io_dataOut    (input_ports_0_power_buffercc_io_dataOut  ), //o
    .phy_clk       (phy_clk                                  ), //i
    .phy_reset     (phy_reset                                )  //i
  );
  UsbOhciWishbone_BufferCC_19 output_ports_0_lowSpeed_buffercc (
    .io_dataIn     (output_ports_0_lowSpeed                      ), //i
    .io_dataOut    (output_ports_0_lowSpeed_buffercc_io_dataOut  ), //o
    .ctrl_clk      (ctrl_clk                                     ), //i
    .ctrl_reset    (ctrl_reset                                   )  //i
  );
  UsbOhciWishbone_BufferCC_19 output_ports_0_overcurrent_buffercc (
    .io_dataIn     (output_ports_0_overcurrent                      ), //i
    .io_dataOut    (output_ports_0_overcurrent_buffercc_io_dataOut  ), //o
    .ctrl_clk      (ctrl_clk                                        ), //i
    .ctrl_reset    (ctrl_reset                                      )  //i
  );
  UsbOhciWishbone_PulseCCByToggle pulseCCByToggle_2 (
    .io_pulseIn     (output_ports_0_connect         ), //i
    .io_pulseOut    (pulseCCByToggle_2_io_pulseOut  ), //o
    .phy_clk        (phy_clk                        ), //i
    .phy_reset      (phy_reset                      ), //i
    .ctrl_clk       (ctrl_clk                       ), //i
    .ctrl_reset     (ctrl_reset                     )  //i
  );
  UsbOhciWishbone_PulseCCByToggle pulseCCByToggle_3 (
    .io_pulseIn     (output_ports_0_disconnect      ), //i
    .io_pulseOut    (pulseCCByToggle_3_io_pulseOut  ), //o
    .phy_clk        (phy_clk                        ), //i
    .phy_reset      (phy_reset                      ), //i
    .ctrl_clk       (ctrl_clk                       ), //i
    .ctrl_reset     (ctrl_reset                     )  //i
  );
  UsbOhciWishbone_PulseCCByToggle pulseCCByToggle_4 (
    .io_pulseIn     (output_ports_0_remoteResume    ), //i
    .io_pulseOut    (pulseCCByToggle_4_io_pulseOut  ), //o
    .phy_clk        (phy_clk                        ), //i
    .phy_reset      (phy_reset                      ), //i
    .ctrl_clk       (ctrl_clk                       ), //i
    .ctrl_reset     (ctrl_reset                     )  //i
  );
  UsbOhciWishbone_StreamCCByToggleWithoutBuffer input_ports_0_reset_ccToggle (
    .io_input_valid     (input_ports_0_reset_valid                     ), //i
    .io_input_ready     (input_ports_0_reset_ccToggle_io_input_ready   ), //o
    .io_output_valid    (input_ports_0_reset_ccToggle_io_output_valid  ), //o
    .io_output_ready    (output_ports_0_reset_ready                    ), //i
    .ctrl_clk           (ctrl_clk                                      ), //i
    .ctrl_reset         (ctrl_reset                                    ), //i
    .phy_clk            (phy_clk                                       ), //i
    .phy_reset          (phy_reset                                     )  //i
  );
  UsbOhciWishbone_StreamCCByToggleWithoutBuffer input_ports_0_suspend_ccToggle (
    .io_input_valid     (input_ports_0_suspend_valid                     ), //i
    .io_input_ready     (input_ports_0_suspend_ccToggle_io_input_ready   ), //o
    .io_output_valid    (input_ports_0_suspend_ccToggle_io_output_valid  ), //o
    .io_output_ready    (output_ports_0_suspend_ready                    ), //i
    .ctrl_clk           (ctrl_clk                                        ), //i
    .ctrl_reset         (ctrl_reset                                      ), //i
    .phy_clk            (phy_clk                                         ), //i
    .phy_reset          (phy_reset                                       )  //i
  );
  UsbOhciWishbone_StreamCCByToggleWithoutBuffer input_ports_0_resume_ccToggle (
    .io_input_valid     (input_ports_0_resume_valid                     ), //i
    .io_input_ready     (input_ports_0_resume_ccToggle_io_input_ready   ), //o
    .io_output_valid    (input_ports_0_resume_ccToggle_io_output_valid  ), //o
    .io_output_ready    (output_ports_0_resume_ready                    ), //i
    .ctrl_clk           (ctrl_clk                                       ), //i
    .ctrl_reset         (ctrl_reset                                     ), //i
    .phy_clk            (phy_clk                                        ), //i
    .phy_reset          (phy_reset                                      )  //i
  );
  UsbOhciWishbone_StreamCCByToggleWithoutBuffer input_ports_0_disable_ccToggle (
    .io_input_valid     (input_ports_0_disable_valid                     ), //i
    .io_input_ready     (input_ports_0_disable_ccToggle_io_input_ready   ), //o
    .io_output_valid    (input_ports_0_disable_ccToggle_io_output_valid  ), //o
    .io_output_ready    (output_ports_0_disable_ready                    ), //i
    .ctrl_clk           (ctrl_clk                                        ), //i
    .ctrl_reset         (ctrl_reset                                      ), //i
    .phy_clk            (phy_clk                                         ), //i
    .phy_reset          (phy_reset                                       )  //i
  );
  assign output_lowSpeed = input_lowSpeed_buffercc_io_dataOut;
  assign output_usbReset = input_usbReset_buffercc_io_dataOut;
  assign output_usbResume = input_usbResume_buffercc_io_dataOut;
  assign input_overcurrent = output_overcurrent_buffercc_io_dataOut;
  assign input_tx_ready = input_tx_ccToggle_io_input_ready;
  assign _zz_1 = ((1'b1 && (! input_tx_ccToggle_io_output_m2sPipe_valid)) || input_tx_ccToggle_io_output_m2sPipe_ready);
  assign input_tx_ccToggle_io_output_m2sPipe_valid = input_tx_ccToggle_io_output_m2sPipe_rValid;
  assign input_tx_ccToggle_io_output_m2sPipe_payload_last = input_tx_ccToggle_io_output_m2sPipe_rData_last;
  assign input_tx_ccToggle_io_output_m2sPipe_payload_fragment = input_tx_ccToggle_io_output_m2sPipe_rData_fragment;
  assign output_tx_valid = input_tx_ccToggle_io_output_m2sPipe_valid;
  assign input_tx_ccToggle_io_output_m2sPipe_ready = output_tx_ready;
  assign output_tx_payload_last = input_tx_ccToggle_io_output_m2sPipe_payload_last;
  assign output_tx_payload_fragment = input_tx_ccToggle_io_output_m2sPipe_payload_fragment;
  assign input_txEop = pulseCCByToggle_io_pulseOut;
  assign input_rx_flow_valid = output_rx_flow_ccToggle_io_output_valid;
  assign input_rx_flow_payload_stuffingError = output_rx_flow_ccToggle_io_output_payload_stuffingError;
  assign input_rx_flow_payload_data = output_rx_flow_ccToggle_io_output_payload_data;
  assign input_rx_active = output_rx_active_buffercc_io_dataOut;
  assign input_tick = pulseCCByToggle_1_io_pulseOut;
  assign output_ports_0_removable = input_ports_0_removable_buffercc_io_dataOut;
  assign output_ports_0_power = input_ports_0_power_buffercc_io_dataOut;
  assign input_ports_0_lowSpeed = output_ports_0_lowSpeed_buffercc_io_dataOut;
  assign input_ports_0_overcurrent = output_ports_0_overcurrent_buffercc_io_dataOut;
  assign input_ports_0_connect = pulseCCByToggle_2_io_pulseOut;
  assign input_ports_0_disconnect = pulseCCByToggle_3_io_pulseOut;
  assign input_ports_0_remoteResume = pulseCCByToggle_4_io_pulseOut;
  assign input_ports_0_reset_ready = input_ports_0_reset_ccToggle_io_input_ready;
  assign output_ports_0_reset_valid = input_ports_0_reset_ccToggle_io_output_valid;
  assign input_ports_0_suspend_ready = input_ports_0_suspend_ccToggle_io_input_ready;
  assign output_ports_0_suspend_valid = input_ports_0_suspend_ccToggle_io_output_valid;
  assign input_ports_0_resume_ready = input_ports_0_resume_ccToggle_io_input_ready;
  assign output_ports_0_resume_valid = input_ports_0_resume_ccToggle_io_output_valid;
  assign input_ports_0_disable_ready = input_ports_0_disable_ccToggle_io_input_ready;
  assign output_ports_0_disable_valid = input_ports_0_disable_ccToggle_io_output_valid;
  always @ (posedge phy_clk or posedge phy_reset) begin
    if (phy_reset) begin
      input_tx_ccToggle_io_output_m2sPipe_rValid <= 1'b0;
    end else begin
      if(_zz_1)begin
        input_tx_ccToggle_io_output_m2sPipe_rValid <= input_tx_ccToggle_io_output_valid;
      end
    end
  end

  always @ (posedge phy_clk) begin
    if(_zz_1)begin
      input_tx_ccToggle_io_output_m2sPipe_rData_last <= input_tx_ccToggle_io_output_payload_last;
      input_tx_ccToggle_io_output_m2sPipe_rData_fragment <= input_tx_ccToggle_io_output_payload_fragment;
    end
  end


endmodule

module UsbOhciWishbone_UsbLsFsPhy (
  input               io_ctrl_lowSpeed,
  input               io_ctrl_tx_valid,
  output reg          io_ctrl_tx_ready,
  input               io_ctrl_tx_payload_last,
  input      [7:0]    io_ctrl_tx_payload_fragment,
  output reg          io_ctrl_txEop,
  output reg          io_ctrl_rx_flow_valid,
  output reg          io_ctrl_rx_flow_payload_stuffingError,
  output reg [7:0]    io_ctrl_rx_flow_payload_data,
  output reg          io_ctrl_rx_active,
  input               io_ctrl_usbReset,
  input               io_ctrl_usbResume,
  output              io_ctrl_overcurrent,
  output              io_ctrl_tick,
  input               io_ctrl_ports_0_disable_valid,
  output              io_ctrl_ports_0_disable_ready,
  input               io_ctrl_ports_0_removable,
  input               io_ctrl_ports_0_power,
  input               io_ctrl_ports_0_reset_valid,
  output reg          io_ctrl_ports_0_reset_ready,
  input               io_ctrl_ports_0_suspend_valid,
  output              io_ctrl_ports_0_suspend_ready,
  input               io_ctrl_ports_0_resume_valid,
  output              io_ctrl_ports_0_resume_ready,
  output reg          io_ctrl_ports_0_connect,
  output              io_ctrl_ports_0_disconnect,
  output              io_ctrl_ports_0_overcurrent,
  output              io_ctrl_ports_0_remoteResume,
  output              io_ctrl_ports_0_lowSpeed,
  output reg          io_usb_0_tx_enable,
  output reg          io_usb_0_tx_data,
  output reg          io_usb_0_tx_se0,
  input               io_usb_0_rx_dp,
  input               io_usb_0_rx_dm,
  input               io_usb_0_overcurrent,
  output              io_usb_0_power,
  input               phy_clk,
  input               phy_reset
);
  wire                ports_0_filter_io_filtred_dp;
  wire                ports_0_filter_io_filtred_dm;
  wire                ports_0_filter_io_filtred_d;
  wire                ports_0_filter_io_filtred_se0;
  wire                ports_0_filter_io_filtred_sample;
  wire                _zz_9;
  wire                _zz_10;
  wire                _zz_11;
  wire                _zz_12;
  wire                _zz_13;
  wire                _zz_14;
  wire                _zz_15;
  wire                _zz_16;
  wire                _zz_17;
  wire                _zz_18;
  wire                _zz_19;
  wire                _zz_20;
  wire       [0:0]    _zz_21;
  wire       [1:0]    _zz_22;
  wire       [4:0]    _zz_23;
  wire       [9:0]    _zz_24;
  wire       [5:0]    _zz_25;
  wire       [9:0]    _zz_26;
  wire       [7:0]    _zz_27;
  wire       [9:0]    _zz_28;
  wire       [6:0]    _zz_29;
  wire       [8:0]    _zz_30;
  wire       [0:0]    _zz_31;
  wire       [1:0]    _zz_32;
  wire       [6:0]    _zz_33;
  wire       [9:0]    _zz_34;
  wire       [11:0]   _zz_35;
  wire       [0:0]    _zz_36;
  wire       [6:0]    _zz_37;
  wire       [4:0]    _zz_38;
  wire       [23:0]   _zz_39;
  wire       [5:0]    _zz_40;
  wire       [23:0]   _zz_41;
  wire                tickTimer_counter_willIncrement;
  wire                tickTimer_counter_willClear;
  reg        [1:0]    tickTimer_counter_valueNext;
  reg        [1:0]    tickTimer_counter_value;
  wire                tickTimer_counter_willOverflowIfInc;
  wire                tickTimer_counter_willOverflow;
  wire                tickTimer_tick;
  reg                 txShared_timer_lowSpeed;
  reg        [9:0]    txShared_timer_counter;
  reg                 txShared_timer_clear;
  wire                txShared_timer_oneCycle;
  wire                txShared_timer_twoCycle;
  wire                txShared_timer_fourCycle;
  reg                 txShared_rxToTxDelay_lowSpeed;
  reg        [8:0]    txShared_rxToTxDelay_counter;
  reg                 txShared_rxToTxDelay_clear;
  wire                txShared_rxToTxDelay_twoCycle;
  reg                 txShared_rxToTxDelay_active;
  reg                 txShared_encoder_input_valid;
  reg                 txShared_encoder_input_ready;
  reg                 txShared_encoder_input_data;
  reg                 txShared_encoder_input_lowSpeed;
  reg                 txShared_encoder_output_valid;
  reg                 txShared_encoder_output_se0;
  reg                 txShared_encoder_output_lowSpeed;
  reg                 txShared_encoder_output_data;
  reg        [2:0]    txShared_encoder_counter;
  reg                 txShared_encoder_state;
  reg                 txShared_serialiser_input_valid;
  reg                 txShared_serialiser_input_ready;
  reg        [7:0]    txShared_serialiser_input_data;
  reg                 txShared_serialiser_input_lowSpeed;
  reg        [2:0]    txShared_serialiser_bitCounter;
  reg        [4:0]    txShared_lowSpeedSof_timer;
  reg        [1:0]    txShared_lowSpeedSof_state;
  reg                 txShared_lowSpeedSof_increment;
  reg                 txShared_lowSpeedSof_overrideEncoder;
  reg                 txShared_encoder_output_valid_regNext;
  reg                 io_ctrl_tx_payload_first;
  wire                txShared_lowSpeedSof_valid;
  wire                txShared_lowSpeedSof_data;
  wire                txShared_lowSpeedSof_se0;
  wire                txShared_frame_wantExit;
  reg                 txShared_frame_wantStart;
  wire                txShared_frame_wantKill;
  wire                txShared_frame_busy;
  reg                 txShared_frame_wasLowSpeed;
  wire                upstreamRx_wantExit;
  reg                 upstreamRx_wantStart;
  wire                upstreamRx_wantKill;
  wire                upstreamRx_timer_lowSpeed;
  reg        [19:0]   upstreamRx_timer_counter;
  reg                 upstreamRx_timer_clear;
  wire                upstreamRx_timer_IDLE_EOI;
  wire                Rx_Suspend;
  reg                 resumeFromPort;
  reg                 ports_0_portLowSpeed;
  reg                 ports_0_rx_enablePackets;
  wire                ports_0_rx_j;
  wire                ports_0_rx_k;
  reg                 ports_0_rx_stuffingError;
  reg                 ports_0_rx_waitSync;
  reg                 ports_0_rx_decoder_state;
  reg                 ports_0_rx_decoder_output_valid;
  reg                 ports_0_rx_decoder_output_payload;
  reg        [2:0]    ports_0_rx_destuffer_counter;
  wire                ports_0_rx_destuffer_unstuffNext;
  wire                ports_0_rx_destuffer_output_valid;
  wire                ports_0_rx_destuffer_output_payload;
  wire                ports_0_rx_history_updated;
  wire                _zz_1;
  reg                 _zz_2;
  reg                 _zz_3;
  reg                 _zz_4;
  reg                 _zz_5;
  reg                 _zz_6;
  reg                 _zz_7;
  reg                 _zz_8;
  wire       [7:0]    ports_0_rx_history_value;
  wire                ports_0_rx_history_sync_hit;
  wire       [6:0]    ports_0_rx_eop_maxThreshold;
  wire       [5:0]    ports_0_rx_eop_minThreshold;
  reg        [6:0]    ports_0_rx_eop_counter;
  wire                ports_0_rx_eop_maxHit;
  reg                 ports_0_rx_eop_hit;
  wire                ports_0_rx_packet_wantExit;
  reg                 ports_0_rx_packet_wantStart;
  wire                ports_0_rx_packet_wantKill;
  reg        [2:0]    ports_0_rx_packet_counter;
  wire                ports_0_rx_packet_errorTimeout_lowSpeed;
  reg        [11:0]   ports_0_rx_packet_errorTimeout_counter;
  reg                 ports_0_rx_packet_errorTimeout_clear;
  wire                ports_0_rx_packet_errorTimeout_trigger;
  reg                 ports_0_rx_packet_errorTimeout_p;
  reg                 ports_0_rx_packet_errorTimeout_n;
  reg        [6:0]    ports_0_rx_disconnect_counter;
  reg                 ports_0_rx_disconnect_clear;
  wire                ports_0_rx_disconnect_hit;
  reg                 ports_0_rx_disconnect_hitLast;
  wire                ports_0_rx_disconnect_event;
  wire                ports_0_fsm_wantExit;
  reg                 ports_0_fsm_wantStart;
  wire                ports_0_fsm_wantKill;
  reg                 ports_0_fsm_timer_lowSpeed;
  reg        [23:0]   ports_0_fsm_timer_counter;
  reg                 ports_0_fsm_timer_clear;
  wire                ports_0_fsm_timer_DISCONNECTED_EOI;
  wire                ports_0_fsm_timer_RESET_DELAY;
  wire                ports_0_fsm_timer_RESET_EOI;
  wire                ports_0_fsm_timer_RESUME_EOI;
  wire                ports_0_fsm_timer_RESTART_EOI;
  wire                ports_0_fsm_timer_ONE_BIT;
  wire                ports_0_fsm_timer_TWO_BIT;
  reg                 ports_0_fsm_resetInProgress;
  reg                 ports_0_fsm_lowSpeedEop;
  wire                ports_0_fsm_forceJ;
  reg        `txShared_frame_enumDefinition_binary_sequential_type txShared_frame_stateReg;
  reg        `txShared_frame_enumDefinition_binary_sequential_type txShared_frame_stateNext;
  reg        `upstreamRx_enumDefinition_binary_sequential_type upstreamRx_stateReg;
  reg        `upstreamRx_enumDefinition_binary_sequential_type upstreamRx_stateNext;
  reg        `ports_0_rx_packet_enumDefinition_binary_sequential_type ports_0_rx_packet_stateReg;
  reg        `ports_0_rx_packet_enumDefinition_binary_sequential_type ports_0_rx_packet_stateNext;
  reg        `ports_0_fsm_enumDefinition_binary_sequential_type ports_0_fsm_stateReg;
  reg        `ports_0_fsm_enumDefinition_binary_sequential_type ports_0_fsm_stateNext;
  `ifndef SYNTHESIS
  reg [231:0] txShared_frame_stateReg_string;
  reg [231:0] txShared_frame_stateNext_string;
  reg [143:0] upstreamRx_stateReg_string;
  reg [143:0] upstreamRx_stateNext_string;
  reg [199:0] ports_0_rx_packet_stateReg_string;
  reg [199:0] ports_0_rx_packet_stateNext_string;
  reg [215:0] ports_0_fsm_stateReg_string;
  reg [215:0] ports_0_fsm_stateNext_string;
  `endif


  assign _zz_9 = (txShared_encoder_counter == 3'b110);
  assign _zz_10 = (txShared_lowSpeedSof_state == 2'b00);
  assign _zz_11 = ((io_ctrl_tx_valid && io_ctrl_tx_payload_first) && (io_ctrl_tx_payload_fragment == 8'ha5));
  assign _zz_12 = (ports_0_rx_packet_counter == 3'b111);
  assign _zz_13 = ((! ports_0_filter_io_filtred_se0) && ((! ports_0_filter_io_filtred_d) ^ ports_0_portLowSpeed));
  assign _zz_14 = ((! ports_0_filter_io_filtred_se0) && ((! ports_0_filter_io_filtred_d) ^ ports_0_portLowSpeed));
  assign _zz_15 = ((ports_0_rx_decoder_state ^ ports_0_filter_io_filtred_d) ^ ports_0_portLowSpeed);
  assign _zz_16 = (! txShared_encoder_output_valid);
  assign _zz_17 = (ports_0_portLowSpeed && txShared_lowSpeedSof_overrideEncoder);
  assign _zz_18 = ((! io_ctrl_ports_0_power) || io_ctrl_usbReset);
  assign _zz_19 = (! ports_0_fsm_resetInProgress);
  assign _zz_20 = (ports_0_filter_io_filtred_dm != ports_0_filter_io_filtred_dp);
  assign _zz_21 = tickTimer_counter_willIncrement;
  assign _zz_22 = {1'd0, _zz_21};
  assign _zz_23 = (txShared_timer_lowSpeed ? 5'h1f : 5'h03);
  assign _zz_24 = {5'd0, _zz_23};
  assign _zz_25 = (txShared_timer_lowSpeed ? 6'h3f : 6'h07);
  assign _zz_26 = {4'd0, _zz_25};
  assign _zz_27 = (txShared_timer_lowSpeed ? 8'h9f : 8'h13);
  assign _zz_28 = {2'd0, _zz_27};
  assign _zz_29 = (txShared_rxToTxDelay_lowSpeed ? 7'h7f : 7'h0f);
  assign _zz_30 = {2'd0, _zz_29};
  assign _zz_31 = txShared_lowSpeedSof_increment;
  assign _zz_32 = {1'd0, _zz_31};
  assign _zz_33 = {1'd0, ports_0_rx_eop_minThreshold};
  assign _zz_34 = (ports_0_rx_packet_errorTimeout_lowSpeed ? 10'h27f : 10'h04f);
  assign _zz_35 = {2'd0, _zz_34};
  assign _zz_36 = (! ports_0_rx_disconnect_hit);
  assign _zz_37 = {6'd0, _zz_36};
  assign _zz_38 = (ports_0_fsm_timer_lowSpeed ? 5'h1f : 5'h03);
  assign _zz_39 = {19'd0, _zz_38};
  assign _zz_40 = (ports_0_fsm_timer_lowSpeed ? 6'h3f : 6'h07);
  assign _zz_41 = {18'd0, _zz_40};
  UsbOhciWishbone_UsbLsFsPhyFilter ports_0_filter (
    .io_lowSpeed          (io_ctrl_lowSpeed                  ), //i
    .io_usb_dp            (io_usb_0_rx_dp                    ), //i
    .io_usb_dm            (io_usb_0_rx_dm                    ), //i
    .io_filtred_dp        (ports_0_filter_io_filtred_dp      ), //o
    .io_filtred_dm        (ports_0_filter_io_filtred_dm      ), //o
    .io_filtred_d         (ports_0_filter_io_filtred_d       ), //o
    .io_filtred_se0       (ports_0_filter_io_filtred_se0     ), //o
    .io_filtred_sample    (ports_0_filter_io_filtred_sample  ), //o
    .phy_clk              (phy_clk                           ), //i
    .phy_reset            (phy_reset                         )  //i
  );
  `ifndef SYNTHESIS
  always @(*) begin
    case(txShared_frame_stateReg)
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_BOOT : txShared_frame_stateReg_string = "txShared_frame_BOOT          ";
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_IDLE : txShared_frame_stateReg_string = "txShared_frame_IDLE          ";
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_TAKE_LINE : txShared_frame_stateReg_string = "txShared_frame_TAKE_LINE     ";
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_PREAMBLE_SYNC : txShared_frame_stateReg_string = "txShared_frame_PREAMBLE_SYNC ";
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_PREAMBLE_PID : txShared_frame_stateReg_string = "txShared_frame_PREAMBLE_PID  ";
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_PREAMBLE_DELAY : txShared_frame_stateReg_string = "txShared_frame_PREAMBLE_DELAY";
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_SYNC : txShared_frame_stateReg_string = "txShared_frame_SYNC          ";
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_DATA : txShared_frame_stateReg_string = "txShared_frame_DATA          ";
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_EOP_0 : txShared_frame_stateReg_string = "txShared_frame_EOP_0         ";
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_EOP_1 : txShared_frame_stateReg_string = "txShared_frame_EOP_1         ";
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_EOP_2 : txShared_frame_stateReg_string = "txShared_frame_EOP_2         ";
      default : txShared_frame_stateReg_string = "?????????????????????????????";
    endcase
  end
  always @(*) begin
    case(txShared_frame_stateNext)
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_BOOT : txShared_frame_stateNext_string = "txShared_frame_BOOT          ";
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_IDLE : txShared_frame_stateNext_string = "txShared_frame_IDLE          ";
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_TAKE_LINE : txShared_frame_stateNext_string = "txShared_frame_TAKE_LINE     ";
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_PREAMBLE_SYNC : txShared_frame_stateNext_string = "txShared_frame_PREAMBLE_SYNC ";
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_PREAMBLE_PID : txShared_frame_stateNext_string = "txShared_frame_PREAMBLE_PID  ";
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_PREAMBLE_DELAY : txShared_frame_stateNext_string = "txShared_frame_PREAMBLE_DELAY";
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_SYNC : txShared_frame_stateNext_string = "txShared_frame_SYNC          ";
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_DATA : txShared_frame_stateNext_string = "txShared_frame_DATA          ";
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_EOP_0 : txShared_frame_stateNext_string = "txShared_frame_EOP_0         ";
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_EOP_1 : txShared_frame_stateNext_string = "txShared_frame_EOP_1         ";
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_EOP_2 : txShared_frame_stateNext_string = "txShared_frame_EOP_2         ";
      default : txShared_frame_stateNext_string = "?????????????????????????????";
    endcase
  end
  always @(*) begin
    case(upstreamRx_stateReg)
      `upstreamRx_enumDefinition_binary_sequential_upstreamRx_BOOT : upstreamRx_stateReg_string = "upstreamRx_BOOT   ";
      `upstreamRx_enumDefinition_binary_sequential_upstreamRx_IDLE : upstreamRx_stateReg_string = "upstreamRx_IDLE   ";
      `upstreamRx_enumDefinition_binary_sequential_upstreamRx_SUSPEND : upstreamRx_stateReg_string = "upstreamRx_SUSPEND";
      default : upstreamRx_stateReg_string = "??????????????????";
    endcase
  end
  always @(*) begin
    case(upstreamRx_stateNext)
      `upstreamRx_enumDefinition_binary_sequential_upstreamRx_BOOT : upstreamRx_stateNext_string = "upstreamRx_BOOT   ";
      `upstreamRx_enumDefinition_binary_sequential_upstreamRx_IDLE : upstreamRx_stateNext_string = "upstreamRx_IDLE   ";
      `upstreamRx_enumDefinition_binary_sequential_upstreamRx_SUSPEND : upstreamRx_stateNext_string = "upstreamRx_SUSPEND";
      default : upstreamRx_stateNext_string = "??????????????????";
    endcase
  end
  always @(*) begin
    case(ports_0_rx_packet_stateReg)
      `ports_0_rx_packet_enumDefinition_binary_sequential_ports_0_rx_packet_BOOT : ports_0_rx_packet_stateReg_string = "ports_0_rx_packet_BOOT   ";
      `ports_0_rx_packet_enumDefinition_binary_sequential_ports_0_rx_packet_IDLE : ports_0_rx_packet_stateReg_string = "ports_0_rx_packet_IDLE   ";
      `ports_0_rx_packet_enumDefinition_binary_sequential_ports_0_rx_packet_PACKET : ports_0_rx_packet_stateReg_string = "ports_0_rx_packet_PACKET ";
      `ports_0_rx_packet_enumDefinition_binary_sequential_ports_0_rx_packet_ERRORED : ports_0_rx_packet_stateReg_string = "ports_0_rx_packet_ERRORED";
      default : ports_0_rx_packet_stateReg_string = "?????????????????????????";
    endcase
  end
  always @(*) begin
    case(ports_0_rx_packet_stateNext)
      `ports_0_rx_packet_enumDefinition_binary_sequential_ports_0_rx_packet_BOOT : ports_0_rx_packet_stateNext_string = "ports_0_rx_packet_BOOT   ";
      `ports_0_rx_packet_enumDefinition_binary_sequential_ports_0_rx_packet_IDLE : ports_0_rx_packet_stateNext_string = "ports_0_rx_packet_IDLE   ";
      `ports_0_rx_packet_enumDefinition_binary_sequential_ports_0_rx_packet_PACKET : ports_0_rx_packet_stateNext_string = "ports_0_rx_packet_PACKET ";
      `ports_0_rx_packet_enumDefinition_binary_sequential_ports_0_rx_packet_ERRORED : ports_0_rx_packet_stateNext_string = "ports_0_rx_packet_ERRORED";
      default : ports_0_rx_packet_stateNext_string = "?????????????????????????";
    endcase
  end
  always @(*) begin
    case(ports_0_fsm_stateReg)
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_BOOT : ports_0_fsm_stateReg_string = "ports_0_fsm_BOOT           ";
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_POWER_OFF : ports_0_fsm_stateReg_string = "ports_0_fsm_POWER_OFF      ";
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_DISCONNECTED : ports_0_fsm_stateReg_string = "ports_0_fsm_DISCONNECTED   ";
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_DISABLED : ports_0_fsm_stateReg_string = "ports_0_fsm_DISABLED       ";
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_RESETTING : ports_0_fsm_stateReg_string = "ports_0_fsm_RESETTING      ";
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_RESETTING_DELAY : ports_0_fsm_stateReg_string = "ports_0_fsm_RESETTING_DELAY";
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_RESETTING_SYNC : ports_0_fsm_stateReg_string = "ports_0_fsm_RESETTING_SYNC ";
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_ENABLED : ports_0_fsm_stateReg_string = "ports_0_fsm_ENABLED        ";
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_SUSPENDED : ports_0_fsm_stateReg_string = "ports_0_fsm_SUSPENDED      ";
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_RESUMING : ports_0_fsm_stateReg_string = "ports_0_fsm_RESUMING       ";
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_SEND_EOP_0 : ports_0_fsm_stateReg_string = "ports_0_fsm_SEND_EOP_0     ";
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_SEND_EOP_1 : ports_0_fsm_stateReg_string = "ports_0_fsm_SEND_EOP_1     ";
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_RESTART_S : ports_0_fsm_stateReg_string = "ports_0_fsm_RESTART_S      ";
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_RESTART_E : ports_0_fsm_stateReg_string = "ports_0_fsm_RESTART_E      ";
      default : ports_0_fsm_stateReg_string = "???????????????????????????";
    endcase
  end
  always @(*) begin
    case(ports_0_fsm_stateNext)
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_BOOT : ports_0_fsm_stateNext_string = "ports_0_fsm_BOOT           ";
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_POWER_OFF : ports_0_fsm_stateNext_string = "ports_0_fsm_POWER_OFF      ";
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_DISCONNECTED : ports_0_fsm_stateNext_string = "ports_0_fsm_DISCONNECTED   ";
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_DISABLED : ports_0_fsm_stateNext_string = "ports_0_fsm_DISABLED       ";
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_RESETTING : ports_0_fsm_stateNext_string = "ports_0_fsm_RESETTING      ";
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_RESETTING_DELAY : ports_0_fsm_stateNext_string = "ports_0_fsm_RESETTING_DELAY";
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_RESETTING_SYNC : ports_0_fsm_stateNext_string = "ports_0_fsm_RESETTING_SYNC ";
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_ENABLED : ports_0_fsm_stateNext_string = "ports_0_fsm_ENABLED        ";
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_SUSPENDED : ports_0_fsm_stateNext_string = "ports_0_fsm_SUSPENDED      ";
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_RESUMING : ports_0_fsm_stateNext_string = "ports_0_fsm_RESUMING       ";
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_SEND_EOP_0 : ports_0_fsm_stateNext_string = "ports_0_fsm_SEND_EOP_0     ";
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_SEND_EOP_1 : ports_0_fsm_stateNext_string = "ports_0_fsm_SEND_EOP_1     ";
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_RESTART_S : ports_0_fsm_stateNext_string = "ports_0_fsm_RESTART_S      ";
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_RESTART_E : ports_0_fsm_stateNext_string = "ports_0_fsm_RESTART_E      ";
      default : ports_0_fsm_stateNext_string = "???????????????????????????";
    endcase
  end
  `endif

  assign tickTimer_counter_willClear = 1'b0;
  assign tickTimer_counter_willOverflowIfInc = (tickTimer_counter_value == 2'b11);
  assign tickTimer_counter_willOverflow = (tickTimer_counter_willOverflowIfInc && tickTimer_counter_willIncrement);
  always @ (*) begin
    tickTimer_counter_valueNext = (tickTimer_counter_value + _zz_22);
    if(tickTimer_counter_willClear)begin
      tickTimer_counter_valueNext = 2'b00;
    end
  end

  assign tickTimer_counter_willIncrement = 1'b1;
  assign tickTimer_tick = (tickTimer_counter_willOverflow == 1'b1);
  assign io_ctrl_tick = tickTimer_tick;
  always @ (*) begin
    txShared_timer_clear = 1'b0;
    if(txShared_encoder_input_valid)begin
      if(_zz_9)begin
        if(txShared_timer_oneCycle)begin
          txShared_timer_clear = 1'b1;
        end
      end
    end
    if(txShared_encoder_input_ready)begin
      txShared_timer_clear = 1'b1;
    end
    case(txShared_frame_stateReg)
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_IDLE : begin
        txShared_timer_clear = 1'b1;
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_TAKE_LINE : begin
        if(txShared_timer_oneCycle)begin
          txShared_timer_clear = 1'b1;
        end
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_PREAMBLE_SYNC : begin
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_PREAMBLE_PID : begin
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_PREAMBLE_DELAY : begin
        if(txShared_timer_fourCycle)begin
          txShared_timer_clear = 1'b1;
        end
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_SYNC : begin
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_DATA : begin
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_EOP_0 : begin
        if(txShared_timer_twoCycle)begin
          txShared_timer_clear = 1'b1;
        end
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_EOP_1 : begin
        if(txShared_timer_oneCycle)begin
          txShared_timer_clear = 1'b1;
        end
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_EOP_2 : begin
        if(txShared_timer_twoCycle)begin
          txShared_timer_clear = 1'b1;
        end
      end
      default : begin
      end
    endcase
  end

  assign txShared_timer_oneCycle = (txShared_timer_counter == _zz_24);
  assign txShared_timer_twoCycle = (txShared_timer_counter == _zz_26);
  assign txShared_timer_fourCycle = (txShared_timer_counter == _zz_28);
  always @ (*) begin
    txShared_timer_lowSpeed = 1'b0;
    if(txShared_encoder_input_valid)begin
      txShared_timer_lowSpeed = txShared_encoder_input_lowSpeed;
    end
    case(txShared_frame_stateReg)
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_IDLE : begin
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_TAKE_LINE : begin
        txShared_timer_lowSpeed = 1'b0;
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_PREAMBLE_SYNC : begin
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_PREAMBLE_PID : begin
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_PREAMBLE_DELAY : begin
        txShared_timer_lowSpeed = 1'b0;
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_SYNC : begin
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_DATA : begin
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_EOP_0 : begin
        txShared_timer_lowSpeed = txShared_frame_wasLowSpeed;
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_EOP_1 : begin
        txShared_timer_lowSpeed = txShared_frame_wasLowSpeed;
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_EOP_2 : begin
        txShared_timer_lowSpeed = txShared_frame_wasLowSpeed;
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    txShared_rxToTxDelay_clear = 1'b0;
    if(ports_0_rx_eop_hit)begin
      txShared_rxToTxDelay_clear = 1'b1;
    end
  end

  assign txShared_rxToTxDelay_twoCycle = (txShared_rxToTxDelay_counter == _zz_30);
  always @ (*) begin
    txShared_encoder_input_valid = 1'b0;
    if(txShared_serialiser_input_valid)begin
      txShared_encoder_input_valid = 1'b1;
    end
  end

  always @ (*) begin
    txShared_encoder_input_ready = 1'b0;
    if(txShared_encoder_input_valid)begin
      if(! _zz_9) begin
        if(txShared_encoder_input_data)begin
          if(txShared_timer_oneCycle)begin
            txShared_encoder_input_ready = 1'b1;
          end
        end else begin
          if(txShared_timer_oneCycle)begin
            txShared_encoder_input_ready = 1'b1;
          end
        end
      end
    end
  end

  always @ (*) begin
    txShared_encoder_input_data = 1'bx;
    if(txShared_serialiser_input_valid)begin
      txShared_encoder_input_data = txShared_serialiser_input_data[txShared_serialiser_bitCounter];
    end
  end

  always @ (*) begin
    txShared_encoder_input_lowSpeed = 1'bx;
    if(txShared_serialiser_input_valid)begin
      txShared_encoder_input_lowSpeed = txShared_serialiser_input_lowSpeed;
    end
  end

  always @ (*) begin
    txShared_encoder_output_valid = 1'b0;
    if(txShared_encoder_input_valid)begin
      txShared_encoder_output_valid = txShared_encoder_input_valid;
    end
    case(txShared_frame_stateReg)
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_IDLE : begin
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_TAKE_LINE : begin
        txShared_encoder_output_valid = 1'b1;
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_PREAMBLE_SYNC : begin
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_PREAMBLE_PID : begin
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_PREAMBLE_DELAY : begin
        txShared_encoder_output_valid = 1'b1;
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_SYNC : begin
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_DATA : begin
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_EOP_0 : begin
        txShared_encoder_output_valid = 1'b1;
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_EOP_1 : begin
        txShared_encoder_output_valid = 1'b1;
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_EOP_2 : begin
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    txShared_encoder_output_se0 = 1'b0;
    case(txShared_frame_stateReg)
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_IDLE : begin
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_TAKE_LINE : begin
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_PREAMBLE_SYNC : begin
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_PREAMBLE_PID : begin
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_PREAMBLE_DELAY : begin
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_SYNC : begin
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_DATA : begin
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_EOP_0 : begin
        txShared_encoder_output_se0 = 1'b1;
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_EOP_1 : begin
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_EOP_2 : begin
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    txShared_encoder_output_lowSpeed = 1'bx;
    if(txShared_encoder_input_valid)begin
      txShared_encoder_output_lowSpeed = txShared_encoder_input_lowSpeed;
    end
    case(txShared_frame_stateReg)
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_IDLE : begin
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_TAKE_LINE : begin
        txShared_encoder_output_lowSpeed = 1'b0;
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_PREAMBLE_SYNC : begin
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_PREAMBLE_PID : begin
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_PREAMBLE_DELAY : begin
        txShared_encoder_output_lowSpeed = 1'b0;
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_SYNC : begin
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_DATA : begin
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_EOP_0 : begin
        txShared_encoder_output_lowSpeed = txShared_frame_wasLowSpeed;
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_EOP_1 : begin
        txShared_encoder_output_lowSpeed = txShared_frame_wasLowSpeed;
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_EOP_2 : begin
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    txShared_encoder_output_data = 1'bx;
    if(txShared_encoder_input_valid)begin
      if(_zz_9)begin
        txShared_encoder_output_data = (! txShared_encoder_state);
      end else begin
        if(txShared_encoder_input_data)begin
          txShared_encoder_output_data = txShared_encoder_state;
        end else begin
          txShared_encoder_output_data = (! txShared_encoder_state);
        end
      end
    end
    case(txShared_frame_stateReg)
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_IDLE : begin
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_TAKE_LINE : begin
        txShared_encoder_output_data = 1'b1;
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_PREAMBLE_SYNC : begin
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_PREAMBLE_PID : begin
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_PREAMBLE_DELAY : begin
        txShared_encoder_output_data = 1'b1;
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_SYNC : begin
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_DATA : begin
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_EOP_0 : begin
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_EOP_1 : begin
        txShared_encoder_output_data = 1'b1;
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_EOP_2 : begin
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    txShared_serialiser_input_valid = 1'b0;
    case(txShared_frame_stateReg)
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_IDLE : begin
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_TAKE_LINE : begin
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_PREAMBLE_SYNC : begin
        txShared_serialiser_input_valid = 1'b1;
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_PREAMBLE_PID : begin
        txShared_serialiser_input_valid = 1'b1;
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_PREAMBLE_DELAY : begin
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_SYNC : begin
        txShared_serialiser_input_valid = 1'b1;
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_DATA : begin
        txShared_serialiser_input_valid = 1'b1;
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_EOP_0 : begin
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_EOP_1 : begin
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_EOP_2 : begin
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    txShared_serialiser_input_ready = 1'b0;
    if(txShared_serialiser_input_valid)begin
      if(txShared_encoder_input_ready)begin
        if((txShared_serialiser_bitCounter == 3'b111))begin
          txShared_serialiser_input_ready = 1'b1;
        end
      end
    end
  end

  always @ (*) begin
    txShared_serialiser_input_data = 8'bxxxxxxxx;
    case(txShared_frame_stateReg)
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_IDLE : begin
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_TAKE_LINE : begin
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_PREAMBLE_SYNC : begin
        txShared_serialiser_input_data = 8'h80;
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_PREAMBLE_PID : begin
        txShared_serialiser_input_data = 8'h3c;
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_PREAMBLE_DELAY : begin
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_SYNC : begin
        txShared_serialiser_input_data = 8'h80;
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_DATA : begin
        txShared_serialiser_input_data = io_ctrl_tx_payload_fragment;
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_EOP_0 : begin
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_EOP_1 : begin
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_EOP_2 : begin
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    txShared_serialiser_input_lowSpeed = 1'bx;
    case(txShared_frame_stateReg)
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_IDLE : begin
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_TAKE_LINE : begin
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_PREAMBLE_SYNC : begin
        txShared_serialiser_input_lowSpeed = 1'b0;
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_PREAMBLE_PID : begin
        txShared_serialiser_input_lowSpeed = 1'b0;
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_PREAMBLE_DELAY : begin
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_SYNC : begin
        txShared_serialiser_input_lowSpeed = txShared_frame_wasLowSpeed;
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_DATA : begin
        txShared_serialiser_input_lowSpeed = txShared_frame_wasLowSpeed;
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_EOP_0 : begin
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_EOP_1 : begin
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_EOP_2 : begin
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    txShared_lowSpeedSof_increment = 1'b0;
    if(_zz_10)begin
      if(_zz_11)begin
        txShared_lowSpeedSof_increment = 1'b1;
      end
    end
  end

  assign txShared_lowSpeedSof_valid = (txShared_lowSpeedSof_state != 2'b00);
  assign txShared_lowSpeedSof_data = 1'b0;
  assign txShared_lowSpeedSof_se0 = (txShared_lowSpeedSof_state != 2'b11);
  assign txShared_frame_wantExit = 1'b0;
  always @ (*) begin
    txShared_frame_wantStart = 1'b0;
    case(txShared_frame_stateReg)
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_IDLE : begin
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_TAKE_LINE : begin
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_PREAMBLE_SYNC : begin
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_PREAMBLE_PID : begin
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_PREAMBLE_DELAY : begin
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_SYNC : begin
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_DATA : begin
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_EOP_0 : begin
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_EOP_1 : begin
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_EOP_2 : begin
      end
      default : begin
        txShared_frame_wantStart = 1'b1;
      end
    endcase
  end

  assign txShared_frame_wantKill = 1'b0;
  assign txShared_frame_busy = (! (txShared_frame_stateReg == `txShared_frame_enumDefinition_binary_sequential_txShared_frame_BOOT));
  always @ (*) begin
    io_ctrl_tx_ready = 1'b0;
    case(txShared_frame_stateReg)
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_IDLE : begin
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_TAKE_LINE : begin
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_PREAMBLE_SYNC : begin
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_PREAMBLE_PID : begin
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_PREAMBLE_DELAY : begin
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_SYNC : begin
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_DATA : begin
        if(txShared_serialiser_input_ready)begin
          io_ctrl_tx_ready = 1'b1;
        end
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_EOP_0 : begin
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_EOP_1 : begin
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_EOP_2 : begin
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    io_ctrl_txEop = 1'b0;
    case(txShared_frame_stateReg)
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_IDLE : begin
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_TAKE_LINE : begin
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_PREAMBLE_SYNC : begin
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_PREAMBLE_PID : begin
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_PREAMBLE_DELAY : begin
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_SYNC : begin
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_DATA : begin
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_EOP_0 : begin
        if(txShared_timer_twoCycle)begin
          io_ctrl_txEop = 1'b1;
        end
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_EOP_1 : begin
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_EOP_2 : begin
      end
      default : begin
      end
    endcase
  end

  assign upstreamRx_wantExit = 1'b0;
  always @ (*) begin
    upstreamRx_wantStart = 1'b0;
    case(upstreamRx_stateReg)
      `upstreamRx_enumDefinition_binary_sequential_upstreamRx_IDLE : begin
      end
      `upstreamRx_enumDefinition_binary_sequential_upstreamRx_SUSPEND : begin
      end
      default : begin
        upstreamRx_wantStart = 1'b1;
      end
    endcase
  end

  assign upstreamRx_wantKill = 1'b0;
  always @ (*) begin
    upstreamRx_timer_clear = 1'b0;
    if(txShared_encoder_output_valid)begin
      upstreamRx_timer_clear = 1'b1;
    end
  end

  assign upstreamRx_timer_IDLE_EOI = (upstreamRx_timer_counter == 20'h2327f);
  assign io_ctrl_overcurrent = 1'b0;
  always @ (*) begin
    io_ctrl_rx_flow_valid = 1'b0;
    case(ports_0_rx_packet_stateReg)
      `ports_0_rx_packet_enumDefinition_binary_sequential_ports_0_rx_packet_IDLE : begin
      end
      `ports_0_rx_packet_enumDefinition_binary_sequential_ports_0_rx_packet_PACKET : begin
        if(ports_0_rx_destuffer_output_valid)begin
          if(_zz_12)begin
            io_ctrl_rx_flow_valid = ports_0_rx_enablePackets;
          end
        end
      end
      `ports_0_rx_packet_enumDefinition_binary_sequential_ports_0_rx_packet_ERRORED : begin
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    io_ctrl_rx_active = 1'b0;
    case(ports_0_rx_packet_stateReg)
      `ports_0_rx_packet_enumDefinition_binary_sequential_ports_0_rx_packet_IDLE : begin
      end
      `ports_0_rx_packet_enumDefinition_binary_sequential_ports_0_rx_packet_PACKET : begin
        io_ctrl_rx_active = 1'b1;
      end
      `ports_0_rx_packet_enumDefinition_binary_sequential_ports_0_rx_packet_ERRORED : begin
        io_ctrl_rx_active = 1'b1;
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    io_ctrl_rx_flow_payload_stuffingError = 1'b0;
    case(ports_0_rx_packet_stateReg)
      `ports_0_rx_packet_enumDefinition_binary_sequential_ports_0_rx_packet_IDLE : begin
      end
      `ports_0_rx_packet_enumDefinition_binary_sequential_ports_0_rx_packet_PACKET : begin
        io_ctrl_rx_flow_payload_stuffingError = ports_0_rx_stuffingError;
      end
      `ports_0_rx_packet_enumDefinition_binary_sequential_ports_0_rx_packet_ERRORED : begin
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    io_ctrl_rx_flow_payload_data = 8'bxxxxxxxx;
    case(ports_0_rx_packet_stateReg)
      `ports_0_rx_packet_enumDefinition_binary_sequential_ports_0_rx_packet_IDLE : begin
      end
      `ports_0_rx_packet_enumDefinition_binary_sequential_ports_0_rx_packet_PACKET : begin
        io_ctrl_rx_flow_payload_data = ports_0_rx_history_value;
      end
      `ports_0_rx_packet_enumDefinition_binary_sequential_ports_0_rx_packet_ERRORED : begin
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    resumeFromPort = 1'b0;
    case(ports_0_fsm_stateReg)
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_POWER_OFF : begin
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_DISCONNECTED : begin
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_DISABLED : begin
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_RESETTING : begin
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_RESETTING_DELAY : begin
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_RESETTING_SYNC : begin
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_ENABLED : begin
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_SUSPENDED : begin
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_RESUMING : begin
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_SEND_EOP_0 : begin
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_SEND_EOP_1 : begin
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_RESTART_S : begin
        if(_zz_13)begin
          resumeFromPort = 1'b1;
        end
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_RESTART_E : begin
        if(_zz_14)begin
          resumeFromPort = 1'b1;
        end
      end
      default : begin
      end
    endcase
  end

  assign io_ctrl_ports_0_lowSpeed = ports_0_portLowSpeed;
  assign io_ctrl_ports_0_remoteResume = 1'b0;
  always @ (*) begin
    ports_0_rx_enablePackets = 1'b0;
    case(ports_0_fsm_stateReg)
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_POWER_OFF : begin
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_DISCONNECTED : begin
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_DISABLED : begin
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_RESETTING : begin
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_RESETTING_DELAY : begin
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_RESETTING_SYNC : begin
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_ENABLED : begin
        ports_0_rx_enablePackets = 1'b1;
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_SUSPENDED : begin
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_RESUMING : begin
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_SEND_EOP_0 : begin
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_SEND_EOP_1 : begin
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_RESTART_S : begin
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_RESTART_E : begin
      end
      default : begin
      end
    endcase
  end

  assign ports_0_rx_j = ((ports_0_filter_io_filtred_dp == (! ports_0_portLowSpeed)) && (ports_0_filter_io_filtred_dm == ports_0_portLowSpeed));
  assign ports_0_rx_k = ((ports_0_filter_io_filtred_dp == ports_0_portLowSpeed) && (ports_0_filter_io_filtred_dm == (! ports_0_portLowSpeed)));
  assign io_usb_0_power = io_ctrl_ports_0_power;
  assign io_ctrl_ports_0_overcurrent = io_usb_0_overcurrent;
  always @ (*) begin
    ports_0_rx_waitSync = 1'b0;
    case(ports_0_rx_packet_stateReg)
      `ports_0_rx_packet_enumDefinition_binary_sequential_ports_0_rx_packet_IDLE : begin
        ports_0_rx_waitSync = 1'b1;
      end
      `ports_0_rx_packet_enumDefinition_binary_sequential_ports_0_rx_packet_PACKET : begin
      end
      `ports_0_rx_packet_enumDefinition_binary_sequential_ports_0_rx_packet_ERRORED : begin
      end
      default : begin
      end
    endcase
    if(((! (ports_0_rx_packet_stateReg == `ports_0_rx_packet_enumDefinition_binary_sequential_ports_0_rx_packet_IDLE)) && (ports_0_rx_packet_stateNext == `ports_0_rx_packet_enumDefinition_binary_sequential_ports_0_rx_packet_IDLE)))begin
      ports_0_rx_waitSync = 1'b1;
    end
  end

  always @ (*) begin
    ports_0_rx_decoder_output_valid = 1'b0;
    if(ports_0_filter_io_filtred_sample)begin
      ports_0_rx_decoder_output_valid = 1'b1;
    end
  end

  always @ (*) begin
    ports_0_rx_decoder_output_payload = 1'bx;
    if(ports_0_filter_io_filtred_sample)begin
      if(_zz_15)begin
        ports_0_rx_decoder_output_payload = 1'b0;
      end else begin
        ports_0_rx_decoder_output_payload = 1'b1;
      end
    end
  end

  assign ports_0_rx_destuffer_unstuffNext = (ports_0_rx_destuffer_counter == 3'b110);
  assign ports_0_rx_destuffer_output_valid = (ports_0_rx_decoder_output_valid && (! ports_0_rx_destuffer_unstuffNext));
  assign ports_0_rx_destuffer_output_payload = ports_0_rx_decoder_output_payload;
  assign ports_0_rx_history_updated = ports_0_rx_destuffer_output_valid;
  assign _zz_1 = ports_0_rx_destuffer_output_payload;
  assign ports_0_rx_history_value = {_zz_1,{_zz_2,{_zz_3,{_zz_4,{_zz_5,{_zz_6,{_zz_7,_zz_8}}}}}}};
  assign ports_0_rx_history_sync_hit = (ports_0_rx_history_updated && (ports_0_rx_history_value == 8'hd5));
  assign ports_0_rx_eop_maxThreshold = (io_ctrl_lowSpeed ? 7'h60 : 7'h0c);
  assign ports_0_rx_eop_minThreshold = (io_ctrl_lowSpeed ? 6'h2a : 6'h05);
  assign ports_0_rx_eop_maxHit = (ports_0_rx_eop_counter == ports_0_rx_eop_maxThreshold);
  always @ (*) begin
    ports_0_rx_eop_hit = 1'b0;
    if(ports_0_rx_j)begin
      if(((_zz_33 <= ports_0_rx_eop_counter) && (! ports_0_rx_eop_maxHit)))begin
        ports_0_rx_eop_hit = 1'b1;
      end
    end
  end

  assign ports_0_rx_packet_wantExit = 1'b0;
  always @ (*) begin
    ports_0_rx_packet_wantStart = 1'b0;
    case(ports_0_rx_packet_stateReg)
      `ports_0_rx_packet_enumDefinition_binary_sequential_ports_0_rx_packet_IDLE : begin
      end
      `ports_0_rx_packet_enumDefinition_binary_sequential_ports_0_rx_packet_PACKET : begin
      end
      `ports_0_rx_packet_enumDefinition_binary_sequential_ports_0_rx_packet_ERRORED : begin
      end
      default : begin
        ports_0_rx_packet_wantStart = 1'b1;
      end
    endcase
  end

  assign ports_0_rx_packet_wantKill = 1'b0;
  always @ (*) begin
    ports_0_rx_packet_errorTimeout_clear = 1'b0;
    case(ports_0_rx_packet_stateReg)
      `ports_0_rx_packet_enumDefinition_binary_sequential_ports_0_rx_packet_IDLE : begin
      end
      `ports_0_rx_packet_enumDefinition_binary_sequential_ports_0_rx_packet_PACKET : begin
      end
      `ports_0_rx_packet_enumDefinition_binary_sequential_ports_0_rx_packet_ERRORED : begin
        if(((ports_0_rx_packet_errorTimeout_p != ports_0_filter_io_filtred_dp) || (ports_0_rx_packet_errorTimeout_n != ports_0_filter_io_filtred_dm)))begin
          ports_0_rx_packet_errorTimeout_clear = 1'b1;
        end
      end
      default : begin
      end
    endcase
    if(((! (ports_0_rx_packet_stateReg == `ports_0_rx_packet_enumDefinition_binary_sequential_ports_0_rx_packet_ERRORED)) && (ports_0_rx_packet_stateNext == `ports_0_rx_packet_enumDefinition_binary_sequential_ports_0_rx_packet_ERRORED)))begin
      ports_0_rx_packet_errorTimeout_clear = 1'b1;
    end
  end

  assign ports_0_rx_packet_errorTimeout_lowSpeed = io_ctrl_lowSpeed;
  assign ports_0_rx_packet_errorTimeout_trigger = (ports_0_rx_packet_errorTimeout_counter == _zz_35);
  always @ (*) begin
    ports_0_rx_disconnect_clear = 1'b0;
    if(((! ports_0_filter_io_filtred_se0) || io_usb_0_tx_enable))begin
      ports_0_rx_disconnect_clear = 1'b1;
    end
    if((((! (ports_0_fsm_stateReg == `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_ENABLED)) && (! (ports_0_fsm_stateReg == `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_SUSPENDED))) && (! (ports_0_fsm_stateReg == `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_DISABLED))))begin
      ports_0_rx_disconnect_clear = 1'b1;
    end
  end

  assign ports_0_rx_disconnect_hit = (ports_0_rx_disconnect_counter == 7'h68);
  assign ports_0_rx_disconnect_event = (ports_0_rx_disconnect_hit && (! ports_0_rx_disconnect_hitLast));
  assign io_ctrl_ports_0_disconnect = ports_0_rx_disconnect_event;
  assign ports_0_fsm_wantExit = 1'b0;
  always @ (*) begin
    ports_0_fsm_wantStart = 1'b0;
    case(ports_0_fsm_stateReg)
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_POWER_OFF : begin
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_DISCONNECTED : begin
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_DISABLED : begin
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_RESETTING : begin
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_RESETTING_DELAY : begin
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_RESETTING_SYNC : begin
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_ENABLED : begin
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_SUSPENDED : begin
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_RESUMING : begin
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_SEND_EOP_0 : begin
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_SEND_EOP_1 : begin
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_RESTART_S : begin
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_RESTART_E : begin
      end
      default : begin
        ports_0_fsm_wantStart = 1'b1;
      end
    endcase
  end

  assign ports_0_fsm_wantKill = 1'b0;
  always @ (*) begin
    ports_0_fsm_timer_clear = 1'b0;
    case(ports_0_fsm_stateReg)
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_POWER_OFF : begin
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_DISCONNECTED : begin
        if(((! ports_0_filter_io_filtred_dp) && (! ports_0_filter_io_filtred_dm)))begin
          ports_0_fsm_timer_clear = 1'b1;
        end
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_DISABLED : begin
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_RESETTING : begin
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_RESETTING_DELAY : begin
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_RESETTING_SYNC : begin
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_ENABLED : begin
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_SUSPENDED : begin
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_RESUMING : begin
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_SEND_EOP_0 : begin
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_SEND_EOP_1 : begin
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_RESTART_S : begin
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_RESTART_E : begin
      end
      default : begin
      end
    endcase
    if(((! (ports_0_fsm_stateReg == `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_DISCONNECTED)) && (ports_0_fsm_stateNext == `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_DISCONNECTED)))begin
      ports_0_fsm_timer_clear = 1'b1;
    end
    if(((! (ports_0_fsm_stateReg == `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_RESETTING)) && (ports_0_fsm_stateNext == `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_RESETTING)))begin
      ports_0_fsm_timer_clear = 1'b1;
    end
    if(((! (ports_0_fsm_stateReg == `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_RESETTING_DELAY)) && (ports_0_fsm_stateNext == `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_RESETTING_DELAY)))begin
      ports_0_fsm_timer_clear = 1'b1;
    end
    if(((! (ports_0_fsm_stateReg == `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_RESUMING)) && (ports_0_fsm_stateNext == `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_RESUMING)))begin
      ports_0_fsm_timer_clear = 1'b1;
    end
    if(((! (ports_0_fsm_stateReg == `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_SEND_EOP_0)) && (ports_0_fsm_stateNext == `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_SEND_EOP_0)))begin
      ports_0_fsm_timer_clear = 1'b1;
    end
    if(((! (ports_0_fsm_stateReg == `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_SEND_EOP_1)) && (ports_0_fsm_stateNext == `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_SEND_EOP_1)))begin
      ports_0_fsm_timer_clear = 1'b1;
    end
    if(((! (ports_0_fsm_stateReg == `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_RESTART_S)) && (ports_0_fsm_stateNext == `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_RESTART_S)))begin
      ports_0_fsm_timer_clear = 1'b1;
    end
    if(((! (ports_0_fsm_stateReg == `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_RESTART_E)) && (ports_0_fsm_stateNext == `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_RESTART_E)))begin
      ports_0_fsm_timer_clear = 1'b1;
    end
  end

  assign ports_0_fsm_timer_DISCONNECTED_EOI = (ports_0_fsm_timer_counter == 24'h005dbf);
  assign ports_0_fsm_timer_RESET_DELAY = (ports_0_fsm_timer_counter == 24'h00095f);
  assign ports_0_fsm_timer_RESET_EOI = (ports_0_fsm_timer_counter == 24'h249eff);
  assign ports_0_fsm_timer_RESUME_EOI = (ports_0_fsm_timer_counter == 24'h0f617f);
  assign ports_0_fsm_timer_RESTART_EOI = (ports_0_fsm_timer_counter == 24'h0012bf);
  assign ports_0_fsm_timer_ONE_BIT = (ports_0_fsm_timer_counter == _zz_39);
  assign ports_0_fsm_timer_TWO_BIT = (ports_0_fsm_timer_counter == _zz_41);
  always @ (*) begin
    ports_0_fsm_timer_lowSpeed = ports_0_portLowSpeed;
    case(ports_0_fsm_stateReg)
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_POWER_OFF : begin
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_DISCONNECTED : begin
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_DISABLED : begin
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_RESETTING : begin
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_RESETTING_DELAY : begin
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_RESETTING_SYNC : begin
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_ENABLED : begin
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_SUSPENDED : begin
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_RESUMING : begin
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_SEND_EOP_0 : begin
        if(ports_0_fsm_lowSpeedEop)begin
          ports_0_fsm_timer_lowSpeed = 1'b1;
        end
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_SEND_EOP_1 : begin
        if(ports_0_fsm_lowSpeedEop)begin
          ports_0_fsm_timer_lowSpeed = 1'b1;
        end
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_RESTART_S : begin
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_RESTART_E : begin
      end
      default : begin
      end
    endcase
  end

  assign io_ctrl_ports_0_disable_ready = 1'b1;
  always @ (*) begin
    io_ctrl_ports_0_reset_ready = 1'b0;
    case(ports_0_fsm_stateReg)
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_POWER_OFF : begin
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_DISCONNECTED : begin
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_DISABLED : begin
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_RESETTING : begin
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_RESETTING_DELAY : begin
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_RESETTING_SYNC : begin
        if(_zz_16)begin
          io_ctrl_ports_0_reset_ready = 1'b1;
        end
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_ENABLED : begin
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_SUSPENDED : begin
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_RESUMING : begin
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_SEND_EOP_0 : begin
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_SEND_EOP_1 : begin
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_RESTART_S : begin
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_RESTART_E : begin
      end
      default : begin
      end
    endcase
  end

  assign io_ctrl_ports_0_resume_ready = 1'b1;
  assign io_ctrl_ports_0_suspend_ready = 1'b1;
  always @ (*) begin
    io_ctrl_ports_0_connect = 1'b0;
    case(ports_0_fsm_stateReg)
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_POWER_OFF : begin
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_DISCONNECTED : begin
        if(ports_0_fsm_timer_DISCONNECTED_EOI)begin
          io_ctrl_ports_0_connect = 1'b1;
        end
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_DISABLED : begin
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_RESETTING : begin
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_RESETTING_DELAY : begin
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_RESETTING_SYNC : begin
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_ENABLED : begin
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_SUSPENDED : begin
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_RESUMING : begin
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_SEND_EOP_0 : begin
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_SEND_EOP_1 : begin
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_RESTART_S : begin
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_RESTART_E : begin
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    io_usb_0_tx_enable = 1'b0;
    case(ports_0_fsm_stateReg)
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_POWER_OFF : begin
        io_usb_0_tx_enable = 1'b1;
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_DISCONNECTED : begin
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_DISABLED : begin
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_RESETTING : begin
        io_usb_0_tx_enable = 1'b1;
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_RESETTING_DELAY : begin
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_RESETTING_SYNC : begin
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_ENABLED : begin
        io_usb_0_tx_enable = txShared_encoder_output_valid;
        if(_zz_17)begin
          io_usb_0_tx_enable = txShared_lowSpeedSof_valid;
        end
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_SUSPENDED : begin
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_RESUMING : begin
        io_usb_0_tx_enable = 1'b1;
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_SEND_EOP_0 : begin
        io_usb_0_tx_enable = 1'b1;
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_SEND_EOP_1 : begin
        io_usb_0_tx_enable = 1'b1;
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_RESTART_S : begin
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_RESTART_E : begin
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    io_usb_0_tx_data = 1'bx;
    case(ports_0_fsm_stateReg)
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_POWER_OFF : begin
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_DISCONNECTED : begin
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_DISABLED : begin
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_RESETTING : begin
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_RESETTING_DELAY : begin
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_RESETTING_SYNC : begin
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_ENABLED : begin
        io_usb_0_tx_data = ((txShared_encoder_output_data || ports_0_fsm_forceJ) ^ ports_0_portLowSpeed);
        if(_zz_17)begin
          io_usb_0_tx_data = txShared_lowSpeedSof_data;
        end
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_SUSPENDED : begin
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_RESUMING : begin
        io_usb_0_tx_data = ports_0_portLowSpeed;
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_SEND_EOP_0 : begin
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_SEND_EOP_1 : begin
        io_usb_0_tx_data = (! ports_0_portLowSpeed);
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_RESTART_S : begin
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_RESTART_E : begin
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    io_usb_0_tx_se0 = 1'bx;
    case(ports_0_fsm_stateReg)
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_POWER_OFF : begin
        io_usb_0_tx_se0 = 1'b1;
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_DISCONNECTED : begin
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_DISABLED : begin
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_RESETTING : begin
        io_usb_0_tx_se0 = 1'b1;
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_RESETTING_DELAY : begin
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_RESETTING_SYNC : begin
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_ENABLED : begin
        io_usb_0_tx_se0 = (txShared_encoder_output_se0 && (! ports_0_fsm_forceJ));
        if(_zz_17)begin
          io_usb_0_tx_se0 = txShared_lowSpeedSof_se0;
        end
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_SUSPENDED : begin
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_RESUMING : begin
        io_usb_0_tx_se0 = 1'b0;
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_SEND_EOP_0 : begin
        io_usb_0_tx_se0 = 1'b1;
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_SEND_EOP_1 : begin
        io_usb_0_tx_se0 = 1'b0;
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_RESTART_S : begin
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_RESTART_E : begin
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    ports_0_fsm_resetInProgress = 1'b0;
    case(ports_0_fsm_stateReg)
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_POWER_OFF : begin
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_DISCONNECTED : begin
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_DISABLED : begin
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_RESETTING : begin
        ports_0_fsm_resetInProgress = 1'b1;
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_RESETTING_DELAY : begin
        ports_0_fsm_resetInProgress = 1'b1;
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_RESETTING_SYNC : begin
        ports_0_fsm_resetInProgress = 1'b1;
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_ENABLED : begin
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_SUSPENDED : begin
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_RESUMING : begin
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_SEND_EOP_0 : begin
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_SEND_EOP_1 : begin
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_RESTART_S : begin
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_RESTART_E : begin
      end
      default : begin
      end
    endcase
  end

  assign ports_0_fsm_forceJ = (ports_0_portLowSpeed && (! txShared_encoder_output_lowSpeed));
  always @ (*) begin
    txShared_frame_stateNext = txShared_frame_stateReg;
    case(txShared_frame_stateReg)
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_IDLE : begin
        if((io_ctrl_tx_valid && (! txShared_rxToTxDelay_active)))begin
          txShared_frame_stateNext = `txShared_frame_enumDefinition_binary_sequential_txShared_frame_TAKE_LINE;
        end
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_TAKE_LINE : begin
        if(txShared_timer_oneCycle)begin
          if(io_ctrl_lowSpeed)begin
            txShared_frame_stateNext = `txShared_frame_enumDefinition_binary_sequential_txShared_frame_PREAMBLE_SYNC;
          end else begin
            txShared_frame_stateNext = `txShared_frame_enumDefinition_binary_sequential_txShared_frame_SYNC;
          end
        end
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_PREAMBLE_SYNC : begin
        if(txShared_serialiser_input_ready)begin
          txShared_frame_stateNext = `txShared_frame_enumDefinition_binary_sequential_txShared_frame_PREAMBLE_PID;
        end
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_PREAMBLE_PID : begin
        if(txShared_serialiser_input_ready)begin
          txShared_frame_stateNext = `txShared_frame_enumDefinition_binary_sequential_txShared_frame_PREAMBLE_DELAY;
        end
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_PREAMBLE_DELAY : begin
        if(txShared_timer_fourCycle)begin
          txShared_frame_stateNext = `txShared_frame_enumDefinition_binary_sequential_txShared_frame_SYNC;
        end
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_SYNC : begin
        if(txShared_serialiser_input_ready)begin
          txShared_frame_stateNext = `txShared_frame_enumDefinition_binary_sequential_txShared_frame_DATA;
        end
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_DATA : begin
        if(txShared_serialiser_input_ready)begin
          if(io_ctrl_tx_payload_last)begin
            txShared_frame_stateNext = `txShared_frame_enumDefinition_binary_sequential_txShared_frame_EOP_0;
          end
        end
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_EOP_0 : begin
        if(txShared_timer_twoCycle)begin
          txShared_frame_stateNext = `txShared_frame_enumDefinition_binary_sequential_txShared_frame_EOP_1;
        end
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_EOP_1 : begin
        if(txShared_timer_oneCycle)begin
          txShared_frame_stateNext = `txShared_frame_enumDefinition_binary_sequential_txShared_frame_EOP_2;
        end
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_EOP_2 : begin
        if(txShared_timer_twoCycle)begin
          txShared_frame_stateNext = `txShared_frame_enumDefinition_binary_sequential_txShared_frame_IDLE;
        end
      end
      default : begin
      end
    endcase
    if(txShared_frame_wantStart)begin
      txShared_frame_stateNext = `txShared_frame_enumDefinition_binary_sequential_txShared_frame_IDLE;
    end
    if(txShared_frame_wantKill)begin
      txShared_frame_stateNext = `txShared_frame_enumDefinition_binary_sequential_txShared_frame_BOOT;
    end
  end

  always @ (*) begin
    upstreamRx_stateNext = upstreamRx_stateReg;
    case(upstreamRx_stateReg)
      `upstreamRx_enumDefinition_binary_sequential_upstreamRx_IDLE : begin
        if(upstreamRx_timer_IDLE_EOI)begin
          upstreamRx_stateNext = `upstreamRx_enumDefinition_binary_sequential_upstreamRx_SUSPEND;
        end
      end
      `upstreamRx_enumDefinition_binary_sequential_upstreamRx_SUSPEND : begin
        if(txShared_encoder_output_valid)begin
          upstreamRx_stateNext = `upstreamRx_enumDefinition_binary_sequential_upstreamRx_IDLE;
        end
      end
      default : begin
      end
    endcase
    if(upstreamRx_wantStart)begin
      upstreamRx_stateNext = `upstreamRx_enumDefinition_binary_sequential_upstreamRx_IDLE;
    end
    if(upstreamRx_wantKill)begin
      upstreamRx_stateNext = `upstreamRx_enumDefinition_binary_sequential_upstreamRx_BOOT;
    end
  end

  assign Rx_Suspend = (upstreamRx_stateReg == `upstreamRx_enumDefinition_binary_sequential_upstreamRx_SUSPEND);
  always @ (*) begin
    ports_0_rx_packet_stateNext = ports_0_rx_packet_stateReg;
    case(ports_0_rx_packet_stateReg)
      `ports_0_rx_packet_enumDefinition_binary_sequential_ports_0_rx_packet_IDLE : begin
        if(ports_0_rx_history_sync_hit)begin
          ports_0_rx_packet_stateNext = `ports_0_rx_packet_enumDefinition_binary_sequential_ports_0_rx_packet_PACKET;
        end
      end
      `ports_0_rx_packet_enumDefinition_binary_sequential_ports_0_rx_packet_PACKET : begin
        if(ports_0_rx_destuffer_output_valid)begin
          if(_zz_12)begin
            if(ports_0_rx_stuffingError)begin
              ports_0_rx_packet_stateNext = `ports_0_rx_packet_enumDefinition_binary_sequential_ports_0_rx_packet_ERRORED;
            end
          end
        end
      end
      `ports_0_rx_packet_enumDefinition_binary_sequential_ports_0_rx_packet_ERRORED : begin
        if(ports_0_rx_packet_errorTimeout_trigger)begin
          ports_0_rx_packet_stateNext = `ports_0_rx_packet_enumDefinition_binary_sequential_ports_0_rx_packet_IDLE;
        end
      end
      default : begin
      end
    endcase
    if(ports_0_rx_eop_hit)begin
      ports_0_rx_packet_stateNext = `ports_0_rx_packet_enumDefinition_binary_sequential_ports_0_rx_packet_IDLE;
    end
    if(txShared_encoder_output_valid)begin
      ports_0_rx_packet_stateNext = `ports_0_rx_packet_enumDefinition_binary_sequential_ports_0_rx_packet_IDLE;
    end
    if(ports_0_rx_packet_wantStart)begin
      ports_0_rx_packet_stateNext = `ports_0_rx_packet_enumDefinition_binary_sequential_ports_0_rx_packet_IDLE;
    end
    if(ports_0_rx_packet_wantKill)begin
      ports_0_rx_packet_stateNext = `ports_0_rx_packet_enumDefinition_binary_sequential_ports_0_rx_packet_BOOT;
    end
  end

  always @ (*) begin
    ports_0_fsm_stateNext = ports_0_fsm_stateReg;
    case(ports_0_fsm_stateReg)
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_POWER_OFF : begin
        if(io_ctrl_ports_0_power)begin
          ports_0_fsm_stateNext = `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_DISCONNECTED;
        end
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_DISCONNECTED : begin
        if(ports_0_fsm_timer_DISCONNECTED_EOI)begin
          ports_0_fsm_stateNext = `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_DISABLED;
        end
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_DISABLED : begin
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_RESETTING : begin
        if(ports_0_fsm_timer_RESET_EOI)begin
          ports_0_fsm_stateNext = `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_RESETTING_DELAY;
        end
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_RESETTING_DELAY : begin
        if(ports_0_fsm_timer_RESET_DELAY)begin
          ports_0_fsm_stateNext = `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_RESETTING_SYNC;
        end
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_RESETTING_SYNC : begin
        if(_zz_16)begin
          ports_0_fsm_stateNext = `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_ENABLED;
        end
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_ENABLED : begin
        if(io_ctrl_ports_0_suspend_valid)begin
          ports_0_fsm_stateNext = `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_SUSPENDED;
        end else begin
          if((Rx_Suspend && (ports_0_filter_io_filtred_se0 || ((! ports_0_filter_io_filtred_se0) && ((! ports_0_filter_io_filtred_d) ^ ports_0_portLowSpeed)))))begin
            ports_0_fsm_stateNext = `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_RESTART_E;
          end else begin
            if(io_ctrl_usbResume)begin
              ports_0_fsm_stateNext = `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_RESUMING;
            end
          end
        end
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_SUSPENDED : begin
        if((io_ctrl_ports_0_resume_valid || ((! Rx_Suspend) && ((! ports_0_filter_io_filtred_se0) && ((! ports_0_filter_io_filtred_d) ^ ports_0_portLowSpeed)))))begin
          ports_0_fsm_stateNext = `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_RESUMING;
        end else begin
          if((Rx_Suspend && (ports_0_filter_io_filtred_se0 || ((! ports_0_filter_io_filtred_se0) && ((! ports_0_filter_io_filtred_d) ^ ports_0_portLowSpeed)))))begin
            ports_0_fsm_stateNext = `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_RESTART_S;
          end
        end
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_RESUMING : begin
        if(ports_0_fsm_timer_RESUME_EOI)begin
          ports_0_fsm_stateNext = `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_SEND_EOP_0;
        end
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_SEND_EOP_0 : begin
        if(ports_0_fsm_timer_TWO_BIT)begin
          ports_0_fsm_stateNext = `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_SEND_EOP_1;
        end
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_SEND_EOP_1 : begin
        if(ports_0_fsm_timer_ONE_BIT)begin
          ports_0_fsm_stateNext = `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_ENABLED;
        end
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_RESTART_S : begin
        if(_zz_13)begin
          ports_0_fsm_stateNext = `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_RESUMING;
        end
        if(ports_0_fsm_timer_RESTART_EOI)begin
          ports_0_fsm_stateNext = `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_DISCONNECTED;
        end
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_RESTART_E : begin
        if(_zz_14)begin
          ports_0_fsm_stateNext = `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_RESUMING;
        end
        if(ports_0_fsm_timer_RESTART_EOI)begin
          ports_0_fsm_stateNext = `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_DISCONNECTED;
        end
      end
      default : begin
      end
    endcase
    if(_zz_18)begin
      ports_0_fsm_stateNext = `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_POWER_OFF;
    end else begin
      if(ports_0_rx_disconnect_event)begin
        ports_0_fsm_stateNext = `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_DISCONNECTED;
      end else begin
        if(io_ctrl_ports_0_disable_valid)begin
          ports_0_fsm_stateNext = `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_DISABLED;
        end else begin
          if(io_ctrl_ports_0_reset_valid)begin
            if(_zz_19)begin
              if(_zz_20)begin
                ports_0_fsm_stateNext = `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_RESETTING;
              end
            end
          end
        end
      end
    end
    if(ports_0_fsm_wantStart)begin
      ports_0_fsm_stateNext = `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_POWER_OFF;
    end
    if(ports_0_fsm_wantKill)begin
      ports_0_fsm_stateNext = `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_BOOT;
    end
  end

  always @ (posedge phy_clk or posedge phy_reset) begin
    if (phy_reset) begin
      tickTimer_counter_value <= 2'b00;
      txShared_rxToTxDelay_active <= 1'b0;
      txShared_lowSpeedSof_state <= 2'b00;
      txShared_lowSpeedSof_overrideEncoder <= 1'b0;
      ports_0_rx_eop_counter <= 7'h0;
      ports_0_rx_disconnect_counter <= 7'h0;
      txShared_frame_stateReg <= `txShared_frame_enumDefinition_binary_sequential_txShared_frame_BOOT;
      upstreamRx_stateReg <= `upstreamRx_enumDefinition_binary_sequential_upstreamRx_BOOT;
      ports_0_rx_packet_stateReg <= `ports_0_rx_packet_enumDefinition_binary_sequential_ports_0_rx_packet_BOOT;
      ports_0_fsm_stateReg <= `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_BOOT;
    end else begin
      tickTimer_counter_value <= tickTimer_counter_valueNext;
      if(txShared_rxToTxDelay_twoCycle)begin
        txShared_rxToTxDelay_active <= 1'b0;
      end
      if(((! txShared_encoder_output_valid) && txShared_encoder_output_valid_regNext))begin
        txShared_lowSpeedSof_overrideEncoder <= 1'b0;
      end
      txShared_lowSpeedSof_state <= (txShared_lowSpeedSof_state + _zz_32);
      if(_zz_10)begin
        if(_zz_11)begin
          txShared_lowSpeedSof_overrideEncoder <= 1'b1;
        end
      end else begin
        if((txShared_lowSpeedSof_timer == 5'h1f))begin
          txShared_lowSpeedSof_state <= (txShared_lowSpeedSof_state + 2'b01);
        end
      end
      if(((! ports_0_filter_io_filtred_dp) && (! ports_0_filter_io_filtred_dm)))begin
        if((! ports_0_rx_eop_maxHit))begin
          ports_0_rx_eop_counter <= (ports_0_rx_eop_counter + 7'h01);
        end
      end else begin
        ports_0_rx_eop_counter <= 7'h0;
      end
      ports_0_rx_disconnect_counter <= (ports_0_rx_disconnect_counter + _zz_37);
      if(ports_0_rx_disconnect_clear)begin
        ports_0_rx_disconnect_counter <= 7'h0;
      end
      txShared_frame_stateReg <= txShared_frame_stateNext;
      upstreamRx_stateReg <= upstreamRx_stateNext;
      ports_0_rx_packet_stateReg <= ports_0_rx_packet_stateNext;
      if(ports_0_rx_eop_hit)begin
        txShared_rxToTxDelay_active <= 1'b1;
      end
      ports_0_fsm_stateReg <= ports_0_fsm_stateNext;
    end
  end

  always @ (posedge phy_clk) begin
    txShared_timer_counter <= (txShared_timer_counter + 10'h001);
    if(txShared_timer_clear)begin
      txShared_timer_counter <= 10'h0;
    end
    txShared_rxToTxDelay_counter <= (txShared_rxToTxDelay_counter + 9'h001);
    if(txShared_rxToTxDelay_clear)begin
      txShared_rxToTxDelay_counter <= 9'h0;
    end
    if(txShared_encoder_input_valid)begin
      if(_zz_9)begin
        if(txShared_timer_oneCycle)begin
          txShared_encoder_counter <= 3'b000;
          txShared_encoder_state <= (! txShared_encoder_state);
        end
      end else begin
        if(txShared_encoder_input_data)begin
          if(txShared_timer_oneCycle)begin
            txShared_encoder_counter <= (txShared_encoder_counter + 3'b001);
          end
        end else begin
          if(txShared_timer_oneCycle)begin
            txShared_encoder_counter <= 3'b000;
            txShared_encoder_state <= (! txShared_encoder_state);
          end
        end
      end
    end
    if((! txShared_encoder_input_valid))begin
      txShared_encoder_counter <= 3'b000;
      txShared_encoder_state <= 1'b1;
    end
    if(txShared_serialiser_input_valid)begin
      if(txShared_encoder_input_ready)begin
        txShared_serialiser_bitCounter <= (txShared_serialiser_bitCounter + 3'b001);
      end
    end
    if(((! txShared_serialiser_input_valid) || txShared_serialiser_input_ready))begin
      txShared_serialiser_bitCounter <= 3'b000;
    end
    txShared_encoder_output_valid_regNext <= txShared_encoder_output_valid;
    if(_zz_10)begin
      if(_zz_11)begin
        txShared_lowSpeedSof_timer <= 5'h0;
      end
    end else begin
      txShared_lowSpeedSof_timer <= (txShared_lowSpeedSof_timer + 5'h01);
    end
    upstreamRx_timer_counter <= (upstreamRx_timer_counter + 20'h00001);
    if(upstreamRx_timer_clear)begin
      upstreamRx_timer_counter <= 20'h0;
    end
    if(ports_0_filter_io_filtred_sample)begin
      if(_zz_15)begin
        ports_0_rx_decoder_state <= (! ports_0_rx_decoder_state);
      end
    end
    if(ports_0_rx_waitSync)begin
      ports_0_rx_decoder_state <= 1'b0;
    end
    if(ports_0_rx_decoder_output_valid)begin
      ports_0_rx_destuffer_counter <= (ports_0_rx_destuffer_counter + 3'b001);
      if(((! ports_0_rx_decoder_output_payload) || ports_0_rx_destuffer_unstuffNext))begin
        ports_0_rx_destuffer_counter <= 3'b000;
        if(ports_0_rx_decoder_output_payload)begin
          ports_0_rx_stuffingError <= 1'b1;
        end
      end
    end
    if(ports_0_rx_waitSync)begin
      ports_0_rx_destuffer_counter <= 3'b000;
    end
    if(ports_0_rx_history_updated)begin
      _zz_2 <= _zz_1;
    end
    if(ports_0_rx_history_updated)begin
      _zz_3 <= _zz_2;
    end
    if(ports_0_rx_history_updated)begin
      _zz_4 <= _zz_3;
    end
    if(ports_0_rx_history_updated)begin
      _zz_5 <= _zz_4;
    end
    if(ports_0_rx_history_updated)begin
      _zz_6 <= _zz_5;
    end
    if(ports_0_rx_history_updated)begin
      _zz_7 <= _zz_6;
    end
    if(ports_0_rx_history_updated)begin
      _zz_8 <= _zz_7;
    end
    ports_0_rx_packet_errorTimeout_counter <= (ports_0_rx_packet_errorTimeout_counter + 12'h001);
    if(ports_0_rx_packet_errorTimeout_clear)begin
      ports_0_rx_packet_errorTimeout_counter <= 12'h0;
    end
    ports_0_rx_disconnect_hitLast <= ports_0_rx_disconnect_hit;
    ports_0_fsm_timer_counter <= (ports_0_fsm_timer_counter + 24'h000001);
    if(ports_0_fsm_timer_clear)begin
      ports_0_fsm_timer_counter <= 24'h0;
    end
    case(txShared_frame_stateReg)
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_IDLE : begin
        txShared_frame_wasLowSpeed <= io_ctrl_lowSpeed;
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_TAKE_LINE : begin
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_PREAMBLE_SYNC : begin
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_PREAMBLE_PID : begin
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_PREAMBLE_DELAY : begin
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_SYNC : begin
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_DATA : begin
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_EOP_0 : begin
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_EOP_1 : begin
      end
      `txShared_frame_enumDefinition_binary_sequential_txShared_frame_EOP_2 : begin
      end
      default : begin
      end
    endcase
    case(ports_0_rx_packet_stateReg)
      `ports_0_rx_packet_enumDefinition_binary_sequential_ports_0_rx_packet_IDLE : begin
        ports_0_rx_packet_counter <= 3'b000;
        ports_0_rx_stuffingError <= 1'b0;
      end
      `ports_0_rx_packet_enumDefinition_binary_sequential_ports_0_rx_packet_PACKET : begin
        if(ports_0_rx_destuffer_output_valid)begin
          ports_0_rx_packet_counter <= (ports_0_rx_packet_counter + 3'b001);
        end
      end
      `ports_0_rx_packet_enumDefinition_binary_sequential_ports_0_rx_packet_ERRORED : begin
        ports_0_rx_packet_errorTimeout_p <= ports_0_filter_io_filtred_dp;
        ports_0_rx_packet_errorTimeout_n <= ports_0_filter_io_filtred_dm;
      end
      default : begin
      end
    endcase
    if(ports_0_rx_eop_hit)begin
      txShared_rxToTxDelay_lowSpeed <= io_ctrl_lowSpeed;
    end
    case(ports_0_fsm_stateReg)
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_POWER_OFF : begin
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_DISCONNECTED : begin
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_DISABLED : begin
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_RESETTING : begin
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_RESETTING_DELAY : begin
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_RESETTING_SYNC : begin
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_ENABLED : begin
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_SUSPENDED : begin
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_RESUMING : begin
        if(ports_0_fsm_timer_RESUME_EOI)begin
          ports_0_fsm_lowSpeedEop <= 1'b1;
        end
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_SEND_EOP_0 : begin
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_SEND_EOP_1 : begin
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_RESTART_S : begin
      end
      `ports_0_fsm_enumDefinition_binary_sequential_ports_0_fsm_RESTART_E : begin
      end
      default : begin
      end
    endcase
    if(! _zz_18) begin
      if(! ports_0_rx_disconnect_event) begin
        if(! io_ctrl_ports_0_disable_valid) begin
          if(io_ctrl_ports_0_reset_valid)begin
            if(_zz_19)begin
              if(_zz_20)begin
                ports_0_portLowSpeed <= (! ports_0_filter_io_filtred_d);
              end
            end
          end
        end
      end
    end
  end

  always @ (posedge phy_clk or posedge phy_reset) begin
    if (phy_reset) begin
      io_ctrl_tx_payload_first <= 1'b1;
    end else begin
      if((io_ctrl_tx_valid && io_ctrl_tx_ready))begin
        io_ctrl_tx_payload_first <= io_ctrl_tx_payload_last;
      end
    end
  end


endmodule

module UsbOhciWishbone_UsbOhci (
  input               io_ctrl_cmd_valid,
  output              io_ctrl_cmd_ready,
  input               io_ctrl_cmd_payload_last,
  input      [0:0]    io_ctrl_cmd_payload_fragment_opcode,
  input      [11:0]   io_ctrl_cmd_payload_fragment_address,
  input      [1:0]    io_ctrl_cmd_payload_fragment_length,
  input      [31:0]   io_ctrl_cmd_payload_fragment_data,
  input      [3:0]    io_ctrl_cmd_payload_fragment_mask,
  output              io_ctrl_rsp_valid,
  input               io_ctrl_rsp_ready,
  output              io_ctrl_rsp_payload_last,
  output     [0:0]    io_ctrl_rsp_payload_fragment_opcode,
  output     [31:0]   io_ctrl_rsp_payload_fragment_data,
  output reg          io_phy_lowSpeed,
  output reg          io_phy_tx_valid,
  input               io_phy_tx_ready,
  output reg          io_phy_tx_payload_last,
  output reg [7:0]    io_phy_tx_payload_fragment,
  input               io_phy_txEop,
  input               io_phy_rx_flow_valid,
  input               io_phy_rx_flow_payload_stuffingError,
  input      [7:0]    io_phy_rx_flow_payload_data,
  input               io_phy_rx_active,
  output              io_phy_usbReset,
  output              io_phy_usbResume,
  input               io_phy_overcurrent,
  input               io_phy_tick,
  output              io_phy_ports_0_disable_valid,
  input               io_phy_ports_0_disable_ready,
  output              io_phy_ports_0_removable,
  output              io_phy_ports_0_power,
  output              io_phy_ports_0_reset_valid,
  input               io_phy_ports_0_reset_ready,
  output              io_phy_ports_0_suspend_valid,
  input               io_phy_ports_0_suspend_ready,
  output              io_phy_ports_0_resume_valid,
  input               io_phy_ports_0_resume_ready,
  input               io_phy_ports_0_connect,
  input               io_phy_ports_0_disconnect,
  input               io_phy_ports_0_overcurrent,
  input               io_phy_ports_0_remoteResume,
  input               io_phy_ports_0_lowSpeed,
  output              io_dma_cmd_valid,
  input               io_dma_cmd_ready,
  output              io_dma_cmd_payload_last,
  output     [0:0]    io_dma_cmd_payload_fragment_opcode,
  output     [31:0]   io_dma_cmd_payload_fragment_address,
  output     [5:0]    io_dma_cmd_payload_fragment_length,
  output     [31:0]   io_dma_cmd_payload_fragment_data,
  output     [3:0]    io_dma_cmd_payload_fragment_mask,
  input               io_dma_rsp_valid,
  output              io_dma_rsp_ready,
  input               io_dma_rsp_payload_last,
  input      [0:0]    io_dma_rsp_payload_fragment_opcode,
  input      [31:0]   io_dma_rsp_payload_fragment_data,
  output              io_interrupt,
  output              io_interruptBios,
  input               ctrl_clk,
  input               ctrl_reset
);
  reg                 _zz_66;
  reg        [31:0]   _zz_67;
  reg                 _zz_68;
  reg                 _zz_69;
  reg                 _zz_70;
  reg                 _zz_71;
  reg                 _zz_72;
  wire                _zz_73;
  reg                 _zz_74;
  reg                 _zz_75;
  reg        [7:0]    _zz_76;
  wire                fifo_io_push_ready;
  wire                fifo_io_pop_valid;
  wire       [31:0]   fifo_io_pop_payload;
  wire       [9:0]    fifo_io_occupancy;
  wire       [9:0]    fifo_io_availability;
  wire       [4:0]    token_crc5_io_result;
  wire       [4:0]    token_crc5_io_resultNext;
  wire       [15:0]   dataTx_crc16_io_result;
  wire       [15:0]   dataTx_crc16_io_resultNext;
  wire       [15:0]   dataRx_crc16_io_result;
  wire       [15:0]   dataRx_crc16_io_resultNext;
  wire                _zz_77;
  wire                _zz_78;
  wire                _zz_79;
  wire                _zz_80;
  wire                _zz_81;
  wire                _zz_82;
  wire                _zz_83;
  wire                _zz_84;
  wire                _zz_85;
  wire                _zz_86;
  wire                _zz_87;
  wire                _zz_88;
  wire                _zz_89;
  wire                _zz_90;
  wire                _zz_91;
  wire                _zz_92;
  wire                _zz_93;
  wire                _zz_94;
  wire                _zz_95;
  wire                _zz_96;
  wire                _zz_97;
  wire                _zz_98;
  wire                _zz_99;
  wire                _zz_100;
  wire                _zz_101;
  wire                _zz_102;
  wire                _zz_103;
  wire                _zz_104;
  wire                _zz_105;
  wire                _zz_106;
  wire                _zz_107;
  wire                _zz_108;
  wire                _zz_109;
  wire                _zz_110;
  wire                _zz_111;
  wire                _zz_112;
  wire                _zz_113;
  wire                _zz_114;
  wire                _zz_115;
  wire                _zz_116;
  wire                _zz_117;
  wire                _zz_118;
  wire                _zz_119;
  wire       [3:0]    _zz_120;
  wire       [0:0]    _zz_121;
  wire       [3:0]    _zz_122;
  wire       [0:0]    _zz_123;
  wire       [3:0]    _zz_124;
  wire       [0:0]    _zz_125;
  wire       [0:0]    _zz_126;
  wire       [0:0]    _zz_127;
  wire       [0:0]    _zz_128;
  wire       [0:0]    _zz_129;
  wire       [0:0]    _zz_130;
  wire       [0:0]    _zz_131;
  wire       [0:0]    _zz_132;
  wire       [0:0]    _zz_133;
  wire       [0:0]    _zz_134;
  wire       [0:0]    _zz_135;
  wire       [0:0]    _zz_136;
  wire       [0:0]    _zz_137;
  wire       [0:0]    _zz_138;
  wire       [0:0]    _zz_139;
  wire       [0:0]    _zz_140;
  wire       [0:0]    _zz_141;
  wire       [0:0]    _zz_142;
  wire       [0:0]    _zz_143;
  wire       [0:0]    _zz_144;
  wire       [0:0]    _zz_145;
  wire       [0:0]    _zz_146;
  wire       [0:0]    _zz_147;
  wire       [0:0]    _zz_148;
  wire       [0:0]    _zz_149;
  wire       [0:0]    _zz_150;
  wire       [0:0]    _zz_151;
  wire       [0:0]    _zz_152;
  wire       [0:0]    _zz_153;
  wire       [0:0]    _zz_154;
  wire       [13:0]   _zz_155;
  wire       [0:0]    _zz_156;
  wire       [0:0]    _zz_157;
  wire       [0:0]    _zz_158;
  wire       [0:0]    _zz_159;
  wire       [0:0]    _zz_160;
  wire       [0:0]    _zz_161;
  wire       [0:0]    _zz_162;
  wire       [0:0]    _zz_163;
  wire       [0:0]    _zz_164;
  wire       [0:0]    _zz_165;
  wire       [0:0]    _zz_166;
  wire       [0:0]    _zz_167;
  wire       [0:0]    _zz_168;
  wire       [0:0]    _zz_169;
  wire       [0:0]    _zz_170;
  wire       [0:0]    _zz_171;
  wire       [0:0]    _zz_172;
  wire       [3:0]    _zz_173;
  wire       [7:0]    _zz_174;
  wire       [15:0]   _zz_175;
  wire       [11:0]   _zz_176;
  wire       [12:0]   _zz_177;
  wire       [12:0]   _zz_178;
  wire       [0:0]    _zz_179;
  wire       [12:0]   _zz_180;
  wire       [13:0]   _zz_181;
  wire       [13:0]   _zz_182;
  wire       [5:0]    _zz_183;
  wire       [13:0]   _zz_184;
  wire       [13:0]   _zz_185;
  wire       [13:0]   _zz_186;
  wire       [6:0]    _zz_187;
  wire       [1:0]    _zz_188;
  wire       [6:0]    _zz_189;
  wire       [6:0]    _zz_190;
  wire       [13:0]   _zz_191;
  wire       [13:0]   _zz_192;
  wire       [13:0]   _zz_193;
  wire       [13:0]   _zz_194;
  wire       [13:0]   _zz_195;
  wire       [13:0]   _zz_196;
  wire       [13:0]   _zz_197;
  wire       [13:0]   _zz_198;
  wire       [5:0]    _zz_199;
  wire       [13:0]   _zz_200;
  wire       [13:0]   _zz_201;
  wire       [16:0]   _zz_202;
  wire       [16:0]   _zz_203;
  wire       [15:0]   _zz_204;
  wire       [4:0]    _zz_205;
  wire       [13:0]   _zz_206;
  wire       [11:0]   _zz_207;
  wire       [13:0]   _zz_208;
  wire       [13:0]   _zz_209;
  wire       [13:0]   _zz_210;
  wire       [13:0]   _zz_211;
  wire       [13:0]   _zz_212;
  wire       [13:0]   _zz_213;
  wire       [1:0]    _zz_214;
  wire       [4:0]    _zz_215;
  wire       [2:0]    _zz_216;
  wire       [3:0]    _zz_217;
  wire       [13:0]   _zz_218;
  wire       [11:0]   _zz_219;
  wire       [13:0]   _zz_220;
  wire       [13:0]   _zz_221;
  wire       [13:0]   _zz_222;
  wire       [13:0]   _zz_223;
  wire       [13:0]   _zz_224;
  wire       [13:0]   _zz_225;
  wire       [13:0]   _zz_226;
  wire       [0:0]    _zz_227;
  wire       [10:0]   _zz_228;
  wire       [13:0]   _zz_229;
  wire       [13:0]   _zz_230;
  wire       [13:0]   _zz_231;
  wire       [13:0]   _zz_232;
  wire       [6:0]    _zz_233;
  wire       [31:0]   _zz_234;
  reg                 unscheduleAll_valid;
  reg                 unscheduleAll_ready;
  reg                 ioDma_cmd_valid;
  wire                ioDma_cmd_ready;
  reg                 ioDma_cmd_payload_last;
  reg        [0:0]    ioDma_cmd_payload_fragment_opcode;
  reg        [31:0]   ioDma_cmd_payload_fragment_address;
  reg        [5:0]    ioDma_cmd_payload_fragment_length;
  reg        [31:0]   ioDma_cmd_payload_fragment_data;
  reg        [3:0]    ioDma_cmd_payload_fragment_mask;
  wire                ioDma_rsp_valid;
  wire                ioDma_rsp_ready;
  wire                ioDma_rsp_payload_last;
  wire       [0:0]    ioDma_rsp_payload_fragment_opcode;
  wire       [31:0]   ioDma_rsp_payload_fragment_data;
  reg        [3:0]    dmaCtx_pendingCounter;
  wire                dmaCtx_pendingFull;
  wire                dmaCtx_pendingEmpty;
  reg        [5:0]    dmaCtx_beatCounter;
  reg                 io_dma_cmd_payload_first;
  wire                _zz_1;
  wire       [31:0]   dmaRspMux_vec_0;
  wire       [31:0]   dmaRspMux_data;
  reg        [3:0]    dmaReadCtx_counter;
  reg        [3:0]    dmaWriteCtx_counter;
  reg                 ctrlHalt;
  wire                ctrl_readHaltTrigger;
  reg                 ctrl_writeHaltTrigger;
  wire                ctrl_rsp_valid;
  wire                ctrl_rsp_ready;
  wire                ctrl_rsp_payload_last;
  wire       [0:0]    ctrl_rsp_payload_fragment_opcode;
  reg        [31:0]   ctrl_rsp_payload_fragment_data;
  wire                _zz_2;
  wire                _zz_3;
  wire                _zz_4;
  reg                 _zz_5;
  reg                 _zz_6;
  reg        [0:0]    _zz_7;
  reg        [31:0]   _zz_8;
  wire                ctrl_askWrite;
  wire                ctrl_askRead;
  wire                ctrl_doWrite;
  wire                ctrl_doRead;
  reg                 doUnschedule;
  reg                 doSoftReset;
  wire       [4:0]    reg_hcRevision_REV;
  reg        [1:0]    reg_hcControl_CBSR;
  reg                 reg_hcControl_PLE;
  reg                 reg_hcControl_IE;
  reg                 reg_hcControl_CLE;
  reg                 reg_hcControl_BLE;
  reg        `MainState_binary_sequential_type reg_hcControl_HCFS;
  reg                 reg_hcControl_IR;
  reg                 reg_hcControl_RWC;
  reg                 reg_hcControl_RWE;
  reg                 reg_hcControl_HCFSWrite_valid;
  wire       `MainState_binary_sequential_type reg_hcControl_HCFSWrite_payload;
  reg                 reg_hcCommandStatus_startSoftReset;
  reg                 _zz_9;
  reg                 reg_hcCommandStatus_CLF;
  reg                 _zz_10;
  reg                 reg_hcCommandStatus_BLF;
  reg                 _zz_11;
  reg                 reg_hcCommandStatus_OCR;
  reg                 _zz_12;
  reg        [1:0]    reg_hcCommandStatus_SOC;
  reg                 reg_hcInterrupt_unmaskedPending;
  reg                 reg_hcInterrupt_MIE;
  reg                 _zz_13;
  reg                 _zz_14;
  reg                 reg_hcInterrupt_SO_status;
  reg                 _zz_15;
  reg                 reg_hcInterrupt_SO_enable;
  reg                 _zz_16;
  reg                 _zz_17;
  reg                 reg_hcInterrupt_WDH_status;
  reg                 _zz_18;
  reg                 reg_hcInterrupt_WDH_enable;
  reg                 _zz_19;
  reg                 _zz_20;
  reg                 reg_hcInterrupt_SF_status;
  reg                 _zz_21;
  reg                 reg_hcInterrupt_SF_enable;
  reg                 _zz_22;
  reg                 _zz_23;
  reg                 reg_hcInterrupt_RD_status;
  reg                 _zz_24;
  reg                 reg_hcInterrupt_RD_enable;
  reg                 _zz_25;
  reg                 _zz_26;
  reg                 reg_hcInterrupt_UE_status;
  reg                 _zz_27;
  reg                 reg_hcInterrupt_UE_enable;
  reg                 _zz_28;
  reg                 _zz_29;
  reg                 reg_hcInterrupt_FNO_status;
  reg                 _zz_30;
  reg                 reg_hcInterrupt_FNO_enable;
  reg                 _zz_31;
  reg                 _zz_32;
  reg                 reg_hcInterrupt_RHSC_status;
  reg                 _zz_33;
  reg                 reg_hcInterrupt_RHSC_enable;
  reg                 _zz_34;
  reg                 _zz_35;
  reg                 reg_hcInterrupt_OC_status;
  reg                 _zz_36;
  reg                 reg_hcInterrupt_OC_enable;
  reg                 _zz_37;
  reg                 _zz_38;
  wire                reg_hcInterrupt_doIrq;
  wire       [31:0]   reg_hcHCCA_HCCA_address;
  reg        [23:0]   reg_hcHCCA_HCCA_reg;
  wire       [31:0]   reg_hcPeriodCurrentED_PCED_address;
  reg        [27:0]   reg_hcPeriodCurrentED_PCED_reg;
  wire                reg_hcPeriodCurrentED_isZero;
  wire       [31:0]   reg_hcControlHeadED_CHED_address;
  reg        [27:0]   reg_hcControlHeadED_CHED_reg;
  wire       [31:0]   reg_hcControlCurrentED_CCED_address;
  reg        [27:0]   reg_hcControlCurrentED_CCED_reg;
  wire                reg_hcControlCurrentED_isZero;
  wire       [31:0]   reg_hcBulkHeadED_BHED_address;
  reg        [27:0]   reg_hcBulkHeadED_BHED_reg;
  wire       [31:0]   reg_hcBulkCurrentED_BCED_address;
  reg        [27:0]   reg_hcBulkCurrentED_BCED_reg;
  wire                reg_hcBulkCurrentED_isZero;
  wire       [31:0]   reg_hcDoneHead_DH_address;
  reg        [27:0]   reg_hcDoneHead_DH_reg;
  reg        [13:0]   reg_hcFmInterval_FI;
  reg        [14:0]   reg_hcFmInterval_FSMPS;
  reg                 reg_hcFmInterval_FIT;
  reg        [13:0]   reg_hcFmRemaining_FR;
  reg                 reg_hcFmRemaining_FRT;
  reg        [15:0]   reg_hcFmNumber_FN;
  reg                 reg_hcFmNumber_overflow;
  wire       [15:0]   reg_hcFmNumber_FNp1;
  reg        [13:0]   reg_hcPeriodicStart_PS;
  reg        [11:0]   reg_hcLSThreshold_LST;
  wire                reg_hcLSThreshold_hit;
  wire       [7:0]    reg_hcRhDescriptorA_NDP;
  reg                 reg_hcRhDescriptorA_PSM;
  reg                 reg_hcRhDescriptorA_NPS;
  reg                 reg_hcRhDescriptorA_OCPM;
  reg                 reg_hcRhDescriptorA_NOCP;
  reg        [7:0]    reg_hcRhDescriptorA_POTPGT;
  reg        [0:0]    reg_hcRhDescriptorB_DR;
  reg        [0:0]    reg_hcRhDescriptorB_PPCM;
  reg                 reg_hcRhStatus_DRWE;
  reg                 reg_hcRhStatus_CCIC;
  reg                 _zz_39;
  reg                 io_phy_overcurrent_regNext;
  reg                 reg_hcRhStatus_clearGlobalPower;
  reg                 _zz_40;
  reg                 reg_hcRhStatus_setRemoteWakeupEnable;
  reg                 _zz_41;
  reg                 reg_hcRhStatus_setGlobalPower;
  reg                 _zz_42;
  reg                 reg_hcRhStatus_clearRemoteWakeupEnable;
  reg                 _zz_43;
  reg                 reg_hcRhPortStatus_0_clearPortEnable;
  reg                 _zz_44;
  reg                 reg_hcRhPortStatus_0_setPortEnable;
  reg                 _zz_45;
  reg                 reg_hcRhPortStatus_0_setPortSuspend;
  reg                 _zz_46;
  reg                 reg_hcRhPortStatus_0_clearSuspendStatus;
  reg                 _zz_47;
  reg                 reg_hcRhPortStatus_0_setPortReset;
  reg                 _zz_48;
  reg                 reg_hcRhPortStatus_0_setPortPower;
  reg                 _zz_49;
  reg                 reg_hcRhPortStatus_0_clearPortPower;
  reg                 _zz_50;
  reg                 reg_hcRhPortStatus_0_resume;
  reg                 reg_hcRhPortStatus_0_reset;
  reg                 reg_hcRhPortStatus_0_suspend;
  reg                 reg_hcRhPortStatus_0_connected;
  reg                 reg_hcRhPortStatus_0_PSS;
  reg                 reg_hcRhPortStatus_0_PPS;
  wire                reg_hcRhPortStatus_0_CCS;
  reg                 reg_hcRhPortStatus_0_PES;
  wire                reg_hcRhPortStatus_0_CSC_set;
  reg                 reg_hcRhPortStatus_0_CSC_clear;
  reg                 reg_hcRhPortStatus_0_CSC_reg;
  reg                 _zz_51;
  wire                reg_hcRhPortStatus_0_PESC_set;
  reg                 reg_hcRhPortStatus_0_PESC_clear;
  reg                 reg_hcRhPortStatus_0_PESC_reg;
  reg                 _zz_52;
  wire                reg_hcRhPortStatus_0_PSSC_set;
  reg                 reg_hcRhPortStatus_0_PSSC_clear;
  reg                 reg_hcRhPortStatus_0_PSSC_reg;
  reg                 _zz_53;
  wire                reg_hcRhPortStatus_0_OCIC_set;
  reg                 reg_hcRhPortStatus_0_OCIC_clear;
  reg                 reg_hcRhPortStatus_0_OCIC_reg;
  reg                 _zz_54;
  wire                reg_hcRhPortStatus_0_PRSC_set;
  reg                 reg_hcRhPortStatus_0_PRSC_clear;
  reg                 reg_hcRhPortStatus_0_PRSC_reg;
  reg                 _zz_55;
  reg                 reg_hcRhPortStatus_0_CCS_regNext;
  reg                 frame_run;
  reg                 frame_reload;
  wire                frame_overflow;
  reg                 frame_tick;
  wire                frame_section1;
  reg        [14:0]   frame_limitCounter;
  wire                frame_limitHit;
  reg        [2:0]    frame_decrementTimer;
  wire                frame_decrementTimerOverflow;
  reg                 token_wantExit;
  reg                 token_wantStart;
  reg                 token_wantKill;
  reg        [3:0]    token_pid;
  reg        [10:0]   token_data;
  reg                 dataTx_wantExit;
  reg                 dataTx_wantStart;
  reg                 dataTx_wantKill;
  reg        [3:0]    dataTx_pid;
  reg                 dataTx_data_valid;
  reg                 dataTx_data_ready;
  reg                 dataTx_data_payload_last;
  reg        [7:0]    dataTx_data_payload_fragment;
  wire                rxTimer_lowSpeed;
  reg        [7:0]    rxTimer_counter;
  reg                 rxTimer_clear;
  wire                rxTimer_rxTimeout;
  wire                rxTimer_ackTx;
  wire                rxPidOk;
  reg                 dataRx_wantExit;
  reg                 dataRx_wantStart;
  reg                 dataRx_wantKill;
  reg        [3:0]    dataRx_pid;
  reg                 dataRx_data_valid;
  wire       [7:0]    dataRx_data_payload;
  wire       [7:0]    dataRx_history_0;
  wire       [7:0]    dataRx_history_1;
  reg        [7:0]    _zz_56;
  reg        [7:0]    _zz_57;
  reg        [1:0]    dataRx_valids;
  reg                 dataRx_notResponding;
  reg                 dataRx_stuffingError;
  reg                 dataRx_pidError;
  reg                 dataRx_crcError;
  reg                 sof_wantExit;
  reg                 sof_wantStart;
  reg                 sof_wantKill;
  reg                 sof_doInterruptDelay;
  reg                 priority_bulk;
  reg        [1:0]    priority_counter;
  reg                 priority_tick;
  reg                 priority_skip;
  reg        [2:0]    interruptDelay_counter;
  reg                 interruptDelay_tick;
  wire                interruptDelay_done;
  wire                interruptDelay_disabled;
  reg                 interruptDelay_disable;
  reg                 interruptDelay_load_valid;
  reg        [2:0]    interruptDelay_load_payload;
  reg                 endpoint_wantExit;
  reg                 endpoint_wantStart;
  reg                 endpoint_wantKill;
  reg        `FlowType_binary_sequential_type endpoint_flowType;
  reg        `endpoint_Status_binary_sequential_type endpoint_status_1;
  reg                 endpoint_dataPhase;
  reg        [31:0]   endpoint_ED_address;
  reg        [31:0]   endpoint_ED_words_0;
  reg        [31:0]   endpoint_ED_words_1;
  reg        [31:0]   endpoint_ED_words_2;
  reg        [31:0]   endpoint_ED_words_3;
  wire       [6:0]    endpoint_ED_FA;
  wire       [3:0]    endpoint_ED_EN;
  wire       [1:0]    endpoint_ED_D;
  wire                endpoint_ED_S;
  wire                endpoint_ED_K;
  wire                endpoint_ED_F;
  wire       [10:0]   endpoint_ED_MPS;
  wire       [27:0]   endpoint_ED_tailP;
  wire                endpoint_ED_H;
  wire                endpoint_ED_C;
  wire       [27:0]   endpoint_ED_headP;
  wire       [27:0]   endpoint_ED_nextED;
  wire                endpoint_ED_tdEmpty;
  wire                endpoint_ED_isFs;
  wire                endpoint_ED_isoOut;
  wire       [31:0]   endpoint_TD_address;
  reg        [31:0]   endpoint_TD_words_0;
  reg        [31:0]   endpoint_TD_words_1;
  reg        [31:0]   endpoint_TD_words_2;
  reg        [31:0]   endpoint_TD_words_3;
  wire       [3:0]    endpoint_TD_CC;
  wire       [1:0]    endpoint_TD_EC;
  wire       [1:0]    endpoint_TD_T;
  wire       [2:0]    endpoint_TD_DI;
  wire       [1:0]    endpoint_TD_DP;
  wire                endpoint_TD_R;
  wire       [31:0]   endpoint_TD_CBP;
  wire       [27:0]   endpoint_TD_nextTD;
  wire       [31:0]   endpoint_TD_BE;
  wire       [2:0]    endpoint_TD_FC;
  wire       [15:0]   endpoint_TD_SF;
  wire       [15:0]   endpoint_TD_isoRelativeFrameNumber;
  wire                endpoint_TD_tooEarly;
  wire       [2:0]    endpoint_TD_isoFrameNumber;
  wire                endpoint_TD_isoOverrun;
  reg                 endpoint_TD_isoOverrunReg;
  wire                endpoint_TD_isoLast;
  reg        [12:0]   endpoint_TD_isoBase;
  reg        [12:0]   endpoint_TD_isoBaseNext;
  wire                endpoint_TD_isoZero;
  wire                endpoint_TD_isSinglePage;
  wire       [12:0]   endpoint_TD_firstOffset;
  wire       [12:0]   endpoint_TD_lastOffset;
  wire                endpoint_TD_allowRounding;
  reg                 endpoint_TD_retire;
  reg                 endpoint_TD_upateCBP;
  reg                 endpoint_TD_noUpdate;
  reg                 endpoint_TD_dataPhaseUpdate;
  wire       [1:0]    endpoint_TD_TNext;
  wire                endpoint_TD_dataPhaseNext;
  wire       [3:0]    endpoint_TD_dataPid;
  wire       [3:0]    endpoint_TD_dataPidWrong;
  reg                 endpoint_TD_clear;
  wire       [1:0]    endpoint_tockenType;
  wire                endpoint_isIn;
  reg                 endpoint_applyNextED;
  reg        [13:0]   endpoint_currentAddress;
  wire       [31:0]   endpoint_currentAddressFull;
  reg        [31:0]   _zz_58;
  wire       [31:0]   endpoint_currentAddressBmb;
  reg        [12:0]   endpoint_lastAddress;
  wire       [13:0]   endpoint_transactionSizeMinusOne;
  wire       [13:0]   endpoint_transactionSize;
  reg                 endpoint_zeroLength;
  wire                endpoint_dataDone;
  reg                 endpoint_dmaLogic_wantExit;
  reg                 endpoint_dmaLogic_wantStart;
  reg                 endpoint_dmaLogic_wantKill;
  reg                 endpoint_dmaLogic_validated;
  reg        [5:0]    endpoint_dmaLogic_length;
  wire       [5:0]    endpoint_dmaLogic_lengthMax;
  wire       [5:0]    endpoint_dmaLogic_lengthCalc;
  wire       [4:0]    endpoint_dmaLogic_beatCount;
  wire       [5:0]    endpoint_dmaLogic_lengthBmb;
  reg        [10:0]   endpoint_dmaLogic_fromUsbCounter;
  reg                 endpoint_dmaLogic_overflow;
  reg                 endpoint_dmaLogic_underflow;
  wire                endpoint_dmaLogic_underflowError;
  reg        [12:0]   endpoint_dmaLogic_byteCtx_counter;
  wire                endpoint_dmaLogic_byteCtx_last;
  wire       [1:0]    endpoint_dmaLogic_byteCtx_sel;
  reg                 endpoint_dmaLogic_byteCtx_increment;
  wire       [3:0]    endpoint_dmaLogic_headMask;
  wire       [3:0]    endpoint_dmaLogic_lastMask;
  wire       [3:0]    endpoint_dmaLogic_fullMask;
  wire                endpoint_dmaLogic_beatLast;
  reg        [31:0]   endpoint_dmaLogic_buffer;
  reg                 endpoint_dmaLogic_push;
  wire                endpoint_dmaLogic_fsmStopped;
  wire       [13:0]   endpoint_byteCountCalc;
  wire                endpoint_fsTimeCheck;
  wire                endpoint_timeCheck;
  reg                 endpoint_ackRxFired;
  reg                 endpoint_ackRxActivated;
  reg                 endpoint_ackRxPidFailure;
  reg                 endpoint_ackRxStuffing;
  reg        [3:0]    endpoint_ackRxPid;
  wire       [31:0]   endpoint_tdUpdateAddress;
  reg                 operational_wantExit;
  reg                 operational_wantStart;
  reg                 operational_wantKill;
  reg                 operational_periodicHeadFetched;
  reg                 operational_periodicDone;
  reg                 operational_allowBulk;
  reg                 operational_allowControl;
  reg                 operational_allowPeriodic;
  reg                 operational_allowIsochronous;
  reg                 operational_askExit;
  wire                hc_wantExit;
  reg                 hc_wantStart;
  wire                hc_wantKill;
  reg                 hc_error;
  wire                hc_operationalIsDone;
  wire       `MainState_binary_sequential_type _zz_59;
  reg                 _zz_60;
  reg        `token_enumDefinition_binary_sequential_type token_stateReg;
  reg        `token_enumDefinition_binary_sequential_type token_stateNext;
  reg        `dataTx_enumDefinition_binary_sequential_type dataTx_stateReg;
  reg        `dataTx_enumDefinition_binary_sequential_type dataTx_stateNext;
  reg        `dataRx_enumDefinition_binary_sequential_type dataRx_stateReg;
  reg        `dataRx_enumDefinition_binary_sequential_type dataRx_stateNext;
  reg        `sof_enumDefinition_binary_sequential_type sof_stateReg;
  reg        `sof_enumDefinition_binary_sequential_type sof_stateNext;
  reg        `endpoint_enumDefinition_binary_sequential_type endpoint_stateReg;
  reg        `endpoint_enumDefinition_binary_sequential_type endpoint_stateNext;
  wire       [13:0]   _zz_61;
  reg                 _zz_62;
  wire       [15:0]   _zz_63;
  reg        `endpoint_dmaLogic_enumDefinition_binary_sequential_type endpoint_dmaLogic_stateReg;
  reg        `endpoint_dmaLogic_enumDefinition_binary_sequential_type endpoint_dmaLogic_stateNext;
  wire                _zz_64;
  wire       [3:0]    _zz_65;
  reg                 ioDma_cmd_payload_first;
  reg        `operational_enumDefinition_binary_sequential_type operational_stateReg;
  reg        `operational_enumDefinition_binary_sequential_type operational_stateNext;
  reg        `hc_enumDefinition_binary_sequential_type hc_stateReg;
  reg        `hc_enumDefinition_binary_sequential_type hc_stateNext;
  `ifndef SYNTHESIS
  reg [87:0] reg_hcControl_HCFS_string;
  reg [87:0] reg_hcControl_HCFSWrite_payload_string;
  reg [63:0] endpoint_flowType_string;
  reg [79:0] endpoint_status_1_string;
  reg [87:0] _zz_59_string;
  reg [79:0] token_stateReg_string;
  reg [79:0] token_stateNext_string;
  reg [95:0] dataTx_stateReg_string;
  reg [95:0] dataTx_stateNext_string;
  reg [87:0] dataRx_stateReg_string;
  reg [87:0] dataRx_stateNext_string;
  reg [159:0] sof_stateReg_string;
  reg [159:0] sof_stateNext_string;
  reg [207:0] endpoint_stateReg_string;
  reg [207:0] endpoint_stateNext_string;
  reg [223:0] endpoint_dmaLogic_stateReg_string;
  reg [223:0] endpoint_dmaLogic_stateNext_string;
  reg [231:0] operational_stateReg_string;
  reg [231:0] operational_stateNext_string;
  reg [135:0] hc_stateReg_string;
  reg [135:0] hc_stateNext_string;
  `endif

  function [31:0] zz__zz_58(input dummy);
    begin
      zz__zz_58 = 32'hffffffff;
      zz__zz_58[1 : 0] = 2'b00;
    end
  endfunction
  wire [31:0] _zz_235;

  assign _zz_77 = (dmaWriteCtx_counter == 4'b0000);
  assign _zz_78 = (dmaWriteCtx_counter == 4'b0001);
  assign _zz_79 = (dmaWriteCtx_counter == 4'b0000);
  assign _zz_80 = (dmaWriteCtx_counter == 4'b0000);
  assign _zz_81 = (endpoint_TD_isoFrameNumber == 3'b000);
  assign _zz_82 = (dmaWriteCtx_counter == 4'b0100);
  assign _zz_83 = (endpoint_TD_isoFrameNumber == 3'b001);
  assign _zz_84 = (dmaWriteCtx_counter == 4'b0100);
  assign _zz_85 = (endpoint_TD_isoFrameNumber == 3'b010);
  assign _zz_86 = (dmaWriteCtx_counter == 4'b0101);
  assign _zz_87 = (endpoint_TD_isoFrameNumber == 3'b011);
  assign _zz_88 = (dmaWriteCtx_counter == 4'b0101);
  assign _zz_89 = (endpoint_TD_isoFrameNumber == 3'b100);
  assign _zz_90 = (dmaWriteCtx_counter == 4'b0110);
  assign _zz_91 = (endpoint_TD_isoFrameNumber == 3'b101);
  assign _zz_92 = (dmaWriteCtx_counter == 4'b0110);
  assign _zz_93 = (endpoint_TD_isoFrameNumber == 3'b110);
  assign _zz_94 = (dmaWriteCtx_counter == 4'b0111);
  assign _zz_95 = (endpoint_TD_isoFrameNumber == 3'b111);
  assign _zz_96 = (dmaWriteCtx_counter == 4'b0111);
  assign _zz_97 = (dmaWriteCtx_counter == 4'b0000);
  assign _zz_98 = (dmaWriteCtx_counter == 4'b0001);
  assign _zz_99 = (dmaWriteCtx_counter == 4'b0010);
  assign _zz_100 = (dmaWriteCtx_counter == 4'b0010);
  assign _zz_101 = (((! (endpoint_dmaLogic_stateReg == `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_BOOT)) && (! endpoint_isIn)) && ioDma_rsp_valid);
  assign _zz_102 = (frame_run && io_phy_tick);
  assign _zz_103 = ((! (hc_stateReg == `hc_enumDefinition_binary_sequential_hc_OPERATIONAL)) && (hc_stateNext == `hc_enumDefinition_binary_sequential_hc_OPERATIONAL));
  assign _zz_104 = ((! (dataRx_stateReg == `dataRx_enumDefinition_binary_sequential_dataRx_IDLE)) && (dataRx_stateNext == `dataRx_enumDefinition_binary_sequential_dataRx_IDLE));
  assign _zz_105 = ((! (endpoint_stateReg == `endpoint_enumDefinition_binary_sequential_endpoint_ACK_RX)) && (endpoint_stateNext == `endpoint_enumDefinition_binary_sequential_endpoint_ACK_RX));
  assign _zz_106 = (! io_phy_rx_active);
  assign _zz_107 = (! io_phy_rx_active);
  assign _zz_108 = ((! (endpoint_stateReg == `endpoint_enumDefinition_binary_sequential_endpoint_DATA_RX)) && (endpoint_stateNext == `endpoint_enumDefinition_binary_sequential_endpoint_DATA_RX));
  assign _zz_109 = ((operational_allowPeriodic && (! operational_periodicDone)) && (! frame_section1));
  assign _zz_110 = ((operational_stateReg == `operational_enumDefinition_binary_sequential_operational_BOOT) && (! (operational_stateNext == `operational_enumDefinition_binary_sequential_operational_BOOT)));
  assign _zz_111 = ((endpoint_ED_H || endpoint_ED_K) || endpoint_ED_tdEmpty);
  assign _zz_112 = (! operational_periodicHeadFetched);
  assign _zz_113 = (endpoint_dmaLogic_fromUsbCounter == 11'h0);
  assign _zz_114 = (endpoint_isIn || endpoint_zeroLength);
  assign _zz_115 = (endpoint_dmaLogic_stateReg == `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_TO_USB);
  assign _zz_116 = (dataRx_pid == endpoint_TD_dataPidWrong);
  assign _zz_117 = ((! io_phy_rx_active) && endpoint_ackRxActivated);
  assign _zz_118 = (! endpoint_ackRxFired);
  assign _zz_119 = (reg_hcRhStatus_DRWE && (reg_hcRhPortStatus_0_CSC_reg != 1'b0));
  assign _zz_120 = (dmaCtx_pendingCounter + _zz_122);
  assign _zz_121 = ((ioDma_cmd_valid && ioDma_cmd_ready) && ioDma_cmd_payload_last);
  assign _zz_122 = {3'd0, _zz_121};
  assign _zz_123 = ((ioDma_rsp_valid && ioDma_rsp_ready) && ioDma_rsp_payload_last);
  assign _zz_124 = {3'd0, _zz_123};
  assign _zz_125 = 1'b1;
  assign _zz_126 = 1'b1;
  assign _zz_127 = 1'b1;
  assign _zz_128 = 1'b1;
  assign _zz_129 = 1'b1;
  assign _zz_130 = 1'b0;
  assign _zz_131 = 1'b0;
  assign _zz_132 = 1'b1;
  assign _zz_133 = 1'b0;
  assign _zz_134 = 1'b0;
  assign _zz_135 = 1'b1;
  assign _zz_136 = 1'b0;
  assign _zz_137 = 1'b0;
  assign _zz_138 = 1'b1;
  assign _zz_139 = 1'b0;
  assign _zz_140 = 1'b0;
  assign _zz_141 = 1'b1;
  assign _zz_142 = 1'b0;
  assign _zz_143 = 1'b0;
  assign _zz_144 = 1'b1;
  assign _zz_145 = 1'b0;
  assign _zz_146 = 1'b0;
  assign _zz_147 = 1'b1;
  assign _zz_148 = 1'b0;
  assign _zz_149 = 1'b0;
  assign _zz_150 = 1'b1;
  assign _zz_151 = 1'b0;
  assign _zz_152 = 1'b0;
  assign _zz_153 = 1'b1;
  assign _zz_154 = 1'b0;
  assign _zz_155 = {2'd0, reg_hcLSThreshold_LST};
  assign _zz_156 = 1'b0;
  assign _zz_157 = 1'b1;
  assign _zz_158 = 1'b1;
  assign _zz_159 = 1'b1;
  assign _zz_160 = 1'b1;
  assign _zz_161 = 1'b1;
  assign _zz_162 = 1'b1;
  assign _zz_163 = 1'b1;
  assign _zz_164 = 1'b1;
  assign _zz_165 = 1'b1;
  assign _zz_166 = 1'b1;
  assign _zz_167 = 1'b1;
  assign _zz_168 = 1'b1;
  assign _zz_169 = 1'b1;
  assign _zz_170 = 1'b1;
  assign _zz_171 = 1'b1;
  assign _zz_172 = 1'b1;
  assign _zz_173 = (rxTimer_lowSpeed ? 4'b1111 : 4'b0001);
  assign _zz_174 = {4'd0, _zz_173};
  assign _zz_175 = {13'd0, endpoint_TD_FC};
  assign _zz_176 = endpoint_TD_CBP[11 : 0];
  assign _zz_177 = {1'd0, _zz_176};
  assign _zz_178 = (endpoint_TD_isoBaseNext - _zz_180);
  assign _zz_179 = (! endpoint_TD_isoLast);
  assign _zz_180 = {12'd0, _zz_179};
  assign _zz_181 = {1'd0, endpoint_lastAddress};
  assign _zz_182 = {1'd0, endpoint_lastAddress};
  assign _zz_183 = endpoint_currentAddress[5:0];
  assign _zz_184 = ((endpoint_transactionSizeMinusOne < _zz_185) ? endpoint_transactionSizeMinusOne : _zz_186);
  assign _zz_185 = {8'd0, endpoint_dmaLogic_lengthMax};
  assign _zz_186 = {8'd0, endpoint_dmaLogic_lengthMax};
  assign _zz_187 = ({1'b0,endpoint_dmaLogic_length} + _zz_189);
  assign _zz_188 = endpoint_currentAddressFull[1 : 0];
  assign _zz_189 = {5'd0, _zz_188};
  assign _zz_190 = {endpoint_dmaLogic_beatCount,2'b11};
  assign _zz_191 = (endpoint_currentAddress + _zz_192);
  assign _zz_192 = {8'd0, endpoint_dmaLogic_length};
  assign _zz_193 = (endpoint_currentAddress + _zz_194);
  assign _zz_194 = {8'd0, endpoint_dmaLogic_length};
  assign _zz_195 = (endpoint_currentAddress + _zz_196);
  assign _zz_196 = {8'd0, endpoint_dmaLogic_length};
  assign _zz_197 = (endpoint_currentAddress + _zz_198);
  assign _zz_198 = {8'd0, endpoint_dmaLogic_length};
  assign _zz_199 = {1'd0, endpoint_dmaLogic_beatCount};
  assign _zz_200 = (_zz_201 - endpoint_currentAddress);
  assign _zz_201 = {1'd0, endpoint_lastAddress};
  assign _zz_202 = {2'd0, frame_limitCounter};
  assign _zz_203 = ({3'd0,endpoint_byteCountCalc} <<< 3);
  assign _zz_204 = reg_hcFmNumber_FN;
  assign _zz_205 = (endpoint_ED_F ? 5'h1f : 5'h0f);
  assign _zz_206 = ({1'b0,endpoint_TD_firstOffset} + _zz_208);
  assign _zz_207 = {1'b0,endpoint_ED_MPS};
  assign _zz_208 = {2'd0, _zz_207};
  assign _zz_209 = (endpoint_ED_F ? _zz_210 : ((_zz_211 < _zz_61) ? _zz_212 : _zz_61));
  assign _zz_210 = {1'd0, endpoint_TD_lastOffset};
  assign _zz_211 = {1'd0, endpoint_TD_lastOffset};
  assign _zz_212 = {1'd0, endpoint_TD_lastOffset};
  assign _zz_213 = {1'd0, endpoint_TD_lastOffset};
  assign _zz_214 = (endpoint_TD_EC + 2'b01);
  assign _zz_215 = (endpoint_ED_F ? 5'h1f : 5'h0f);
  assign _zz_216 = (endpoint_ED_F ? 3'b111 : 3'b011);
  assign _zz_217 = {1'd0, _zz_216};
  assign _zz_218 = (endpoint_ED_isoOut ? 14'h0 : _zz_220);
  assign _zz_219 = _zz_218[11:0];
  assign _zz_220 = (endpoint_currentAddress - _zz_221);
  assign _zz_221 = {1'd0, endpoint_TD_isoBase};
  assign _zz_222 = {3'd0, endpoint_dmaLogic_fromUsbCounter};
  assign _zz_223 = {3'd0, endpoint_dmaLogic_fromUsbCounter};
  assign _zz_224 = (_zz_225 - 14'h0001);
  assign _zz_225 = (endpoint_currentAddress + _zz_226);
  assign _zz_226 = {3'd0, endpoint_dmaLogic_fromUsbCounter};
  assign _zz_227 = (! endpoint_dmaLogic_fromUsbCounter[10]);
  assign _zz_228 = {10'd0, _zz_227};
  assign _zz_229 = (endpoint_currentAddress + _zz_230);
  assign _zz_230 = {8'd0, endpoint_dmaLogic_length};
  assign _zz_231 = (endpoint_currentAddress + _zz_232);
  assign _zz_232 = {8'd0, endpoint_dmaLogic_length};
  assign _zz_233 = ({2'd0,reg_hcFmNumber_FN[4 : 0]} <<< 2);
  assign _zz_234 = {25'd0, _zz_233};
  UsbOhciWishbone_StreamFifo fifo (
    .io_push_valid      (_zz_66                     ), //i
    .io_push_ready      (fifo_io_push_ready         ), //o
    .io_push_payload    (_zz_67[31:0]               ), //i
    .io_pop_valid       (fifo_io_pop_valid          ), //o
    .io_pop_ready       (_zz_68                     ), //i
    .io_pop_payload     (fifo_io_pop_payload[31:0]  ), //o
    .io_flush           (_zz_69                     ), //i
    .io_occupancy       (fifo_io_occupancy[9:0]     ), //o
    .io_availability    (fifo_io_availability[9:0]  ), //o
    .ctrl_clk           (ctrl_clk                   ), //i
    .ctrl_reset         (ctrl_reset                 )  //i
  );
  UsbOhciWishbone_Crc token_crc5 (
    .io_flush            (_zz_70                         ), //i
    .io_input_valid      (_zz_71                         ), //i
    .io_input_payload    (token_data[10:0]               ), //i
    .io_result           (token_crc5_io_result[4:0]      ), //o
    .io_resultNext       (token_crc5_io_resultNext[4:0]  ), //o
    .ctrl_clk            (ctrl_clk                       ), //i
    .ctrl_reset          (ctrl_reset                     )  //i
  );
  UsbOhciWishbone_Crc_1 dataTx_crc16 (
    .io_flush            (_zz_72                             ), //i
    .io_input_valid      (_zz_73                             ), //i
    .io_input_payload    (dataTx_data_payload_fragment[7:0]  ), //i
    .io_result           (dataTx_crc16_io_result[15:0]       ), //o
    .io_resultNext       (dataTx_crc16_io_resultNext[15:0]   ), //o
    .ctrl_clk            (ctrl_clk                           ), //i
    .ctrl_reset          (ctrl_reset                         )  //i
  );
  UsbOhciWishbone_Crc_2 dataRx_crc16 (
    .io_flush            (_zz_74                            ), //i
    .io_input_valid      (_zz_75                            ), //i
    .io_input_payload    (io_phy_rx_flow_payload_data[7:0]  ), //i
    .io_result           (dataRx_crc16_io_result[15:0]      ), //o
    .io_resultNext       (dataRx_crc16_io_resultNext[15:0]  ), //o
    .ctrl_clk            (ctrl_clk                          ), //i
    .ctrl_reset          (ctrl_reset                        )  //i
  );
  always @(*) begin
    case(endpoint_dmaLogic_byteCtx_sel)
      2'b00 : begin
        _zz_76 = fifo_io_pop_payload[7 : 0];
      end
      2'b01 : begin
        _zz_76 = fifo_io_pop_payload[15 : 8];
      end
      2'b10 : begin
        _zz_76 = fifo_io_pop_payload[23 : 16];
      end
      default : begin
        _zz_76 = fifo_io_pop_payload[31 : 24];
      end
    endcase
  end

  `ifndef SYNTHESIS
  always @(*) begin
    case(reg_hcControl_HCFS)
      `MainState_binary_sequential_RESET : reg_hcControl_HCFS_string = "RESET      ";
      `MainState_binary_sequential_RESUME : reg_hcControl_HCFS_string = "RESUME     ";
      `MainState_binary_sequential_OPERATIONAL : reg_hcControl_HCFS_string = "OPERATIONAL";
      `MainState_binary_sequential_SUSPEND : reg_hcControl_HCFS_string = "SUSPEND    ";
      default : reg_hcControl_HCFS_string = "???????????";
    endcase
  end
  always @(*) begin
    case(reg_hcControl_HCFSWrite_payload)
      `MainState_binary_sequential_RESET : reg_hcControl_HCFSWrite_payload_string = "RESET      ";
      `MainState_binary_sequential_RESUME : reg_hcControl_HCFSWrite_payload_string = "RESUME     ";
      `MainState_binary_sequential_OPERATIONAL : reg_hcControl_HCFSWrite_payload_string = "OPERATIONAL";
      `MainState_binary_sequential_SUSPEND : reg_hcControl_HCFSWrite_payload_string = "SUSPEND    ";
      default : reg_hcControl_HCFSWrite_payload_string = "???????????";
    endcase
  end
  always @(*) begin
    case(endpoint_flowType)
      `FlowType_binary_sequential_BULK : endpoint_flowType_string = "BULK    ";
      `FlowType_binary_sequential_CONTROL : endpoint_flowType_string = "CONTROL ";
      `FlowType_binary_sequential_PERIODIC : endpoint_flowType_string = "PERIODIC";
      default : endpoint_flowType_string = "????????";
    endcase
  end
  always @(*) begin
    case(endpoint_status_1)
      `endpoint_Status_binary_sequential_OK : endpoint_status_1_string = "OK        ";
      `endpoint_Status_binary_sequential_FRAME_TIME : endpoint_status_1_string = "FRAME_TIME";
      default : endpoint_status_1_string = "??????????";
    endcase
  end
  always @(*) begin
    case(_zz_59)
      `MainState_binary_sequential_RESET : _zz_59_string = "RESET      ";
      `MainState_binary_sequential_RESUME : _zz_59_string = "RESUME     ";
      `MainState_binary_sequential_OPERATIONAL : _zz_59_string = "OPERATIONAL";
      `MainState_binary_sequential_SUSPEND : _zz_59_string = "SUSPEND    ";
      default : _zz_59_string = "???????????";
    endcase
  end
  always @(*) begin
    case(token_stateReg)
      `token_enumDefinition_binary_sequential_token_BOOT : token_stateReg_string = "token_BOOT";
      `token_enumDefinition_binary_sequential_token_INIT : token_stateReg_string = "token_INIT";
      `token_enumDefinition_binary_sequential_token_PID : token_stateReg_string = "token_PID ";
      `token_enumDefinition_binary_sequential_token_B1 : token_stateReg_string = "token_B1  ";
      `token_enumDefinition_binary_sequential_token_B2 : token_stateReg_string = "token_B2  ";
      `token_enumDefinition_binary_sequential_token_EOP : token_stateReg_string = "token_EOP ";
      default : token_stateReg_string = "??????????";
    endcase
  end
  always @(*) begin
    case(token_stateNext)
      `token_enumDefinition_binary_sequential_token_BOOT : token_stateNext_string = "token_BOOT";
      `token_enumDefinition_binary_sequential_token_INIT : token_stateNext_string = "token_INIT";
      `token_enumDefinition_binary_sequential_token_PID : token_stateNext_string = "token_PID ";
      `token_enumDefinition_binary_sequential_token_B1 : token_stateNext_string = "token_B1  ";
      `token_enumDefinition_binary_sequential_token_B2 : token_stateNext_string = "token_B2  ";
      `token_enumDefinition_binary_sequential_token_EOP : token_stateNext_string = "token_EOP ";
      default : token_stateNext_string = "??????????";
    endcase
  end
  always @(*) begin
    case(dataTx_stateReg)
      `dataTx_enumDefinition_binary_sequential_dataTx_BOOT : dataTx_stateReg_string = "dataTx_BOOT ";
      `dataTx_enumDefinition_binary_sequential_dataTx_PID : dataTx_stateReg_string = "dataTx_PID  ";
      `dataTx_enumDefinition_binary_sequential_dataTx_DATA : dataTx_stateReg_string = "dataTx_DATA ";
      `dataTx_enumDefinition_binary_sequential_dataTx_CRC_0 : dataTx_stateReg_string = "dataTx_CRC_0";
      `dataTx_enumDefinition_binary_sequential_dataTx_CRC_1 : dataTx_stateReg_string = "dataTx_CRC_1";
      `dataTx_enumDefinition_binary_sequential_dataTx_EOP : dataTx_stateReg_string = "dataTx_EOP  ";
      default : dataTx_stateReg_string = "????????????";
    endcase
  end
  always @(*) begin
    case(dataTx_stateNext)
      `dataTx_enumDefinition_binary_sequential_dataTx_BOOT : dataTx_stateNext_string = "dataTx_BOOT ";
      `dataTx_enumDefinition_binary_sequential_dataTx_PID : dataTx_stateNext_string = "dataTx_PID  ";
      `dataTx_enumDefinition_binary_sequential_dataTx_DATA : dataTx_stateNext_string = "dataTx_DATA ";
      `dataTx_enumDefinition_binary_sequential_dataTx_CRC_0 : dataTx_stateNext_string = "dataTx_CRC_0";
      `dataTx_enumDefinition_binary_sequential_dataTx_CRC_1 : dataTx_stateNext_string = "dataTx_CRC_1";
      `dataTx_enumDefinition_binary_sequential_dataTx_EOP : dataTx_stateNext_string = "dataTx_EOP  ";
      default : dataTx_stateNext_string = "????????????";
    endcase
  end
  always @(*) begin
    case(dataRx_stateReg)
      `dataRx_enumDefinition_binary_sequential_dataRx_BOOT : dataRx_stateReg_string = "dataRx_BOOT";
      `dataRx_enumDefinition_binary_sequential_dataRx_IDLE : dataRx_stateReg_string = "dataRx_IDLE";
      `dataRx_enumDefinition_binary_sequential_dataRx_PID : dataRx_stateReg_string = "dataRx_PID ";
      `dataRx_enumDefinition_binary_sequential_dataRx_DATA : dataRx_stateReg_string = "dataRx_DATA";
      default : dataRx_stateReg_string = "???????????";
    endcase
  end
  always @(*) begin
    case(dataRx_stateNext)
      `dataRx_enumDefinition_binary_sequential_dataRx_BOOT : dataRx_stateNext_string = "dataRx_BOOT";
      `dataRx_enumDefinition_binary_sequential_dataRx_IDLE : dataRx_stateNext_string = "dataRx_IDLE";
      `dataRx_enumDefinition_binary_sequential_dataRx_PID : dataRx_stateNext_string = "dataRx_PID ";
      `dataRx_enumDefinition_binary_sequential_dataRx_DATA : dataRx_stateNext_string = "dataRx_DATA";
      default : dataRx_stateNext_string = "???????????";
    endcase
  end
  always @(*) begin
    case(sof_stateReg)
      `sof_enumDefinition_binary_sequential_sof_BOOT : sof_stateReg_string = "sof_BOOT            ";
      `sof_enumDefinition_binary_sequential_sof_FRAME_TX : sof_stateReg_string = "sof_FRAME_TX        ";
      `sof_enumDefinition_binary_sequential_sof_FRAME_NUMBER_CMD : sof_stateReg_string = "sof_FRAME_NUMBER_CMD";
      `sof_enumDefinition_binary_sequential_sof_FRAME_NUMBER_RSP : sof_stateReg_string = "sof_FRAME_NUMBER_RSP";
      default : sof_stateReg_string = "????????????????????";
    endcase
  end
  always @(*) begin
    case(sof_stateNext)
      `sof_enumDefinition_binary_sequential_sof_BOOT : sof_stateNext_string = "sof_BOOT            ";
      `sof_enumDefinition_binary_sequential_sof_FRAME_TX : sof_stateNext_string = "sof_FRAME_TX        ";
      `sof_enumDefinition_binary_sequential_sof_FRAME_NUMBER_CMD : sof_stateNext_string = "sof_FRAME_NUMBER_CMD";
      `sof_enumDefinition_binary_sequential_sof_FRAME_NUMBER_RSP : sof_stateNext_string = "sof_FRAME_NUMBER_RSP";
      default : sof_stateNext_string = "????????????????????";
    endcase
  end
  always @(*) begin
    case(endpoint_stateReg)
      `endpoint_enumDefinition_binary_sequential_endpoint_BOOT : endpoint_stateReg_string = "endpoint_BOOT             ";
      `endpoint_enumDefinition_binary_sequential_endpoint_ED_READ_CMD : endpoint_stateReg_string = "endpoint_ED_READ_CMD      ";
      `endpoint_enumDefinition_binary_sequential_endpoint_ED_READ_RSP : endpoint_stateReg_string = "endpoint_ED_READ_RSP      ";
      `endpoint_enumDefinition_binary_sequential_endpoint_ED_ANALYSE : endpoint_stateReg_string = "endpoint_ED_ANALYSE       ";
      `endpoint_enumDefinition_binary_sequential_endpoint_TD_READ_CMD : endpoint_stateReg_string = "endpoint_TD_READ_CMD      ";
      `endpoint_enumDefinition_binary_sequential_endpoint_TD_READ_RSP : endpoint_stateReg_string = "endpoint_TD_READ_RSP      ";
      `endpoint_enumDefinition_binary_sequential_endpoint_TD_ANALYSE : endpoint_stateReg_string = "endpoint_TD_ANALYSE       ";
      `endpoint_enumDefinition_binary_sequential_endpoint_TD_CHECK_TIME : endpoint_stateReg_string = "endpoint_TD_CHECK_TIME    ";
      `endpoint_enumDefinition_binary_sequential_endpoint_BUFFER_READ : endpoint_stateReg_string = "endpoint_BUFFER_READ      ";
      `endpoint_enumDefinition_binary_sequential_endpoint_TOKEN : endpoint_stateReg_string = "endpoint_TOKEN            ";
      `endpoint_enumDefinition_binary_sequential_endpoint_DATA_TX : endpoint_stateReg_string = "endpoint_DATA_TX          ";
      `endpoint_enumDefinition_binary_sequential_endpoint_DATA_RX : endpoint_stateReg_string = "endpoint_DATA_RX          ";
      `endpoint_enumDefinition_binary_sequential_endpoint_DATA_RX_VALIDATE : endpoint_stateReg_string = "endpoint_DATA_RX_VALIDATE ";
      `endpoint_enumDefinition_binary_sequential_endpoint_ACK_RX : endpoint_stateReg_string = "endpoint_ACK_RX           ";
      `endpoint_enumDefinition_binary_sequential_endpoint_ACK_TX_0 : endpoint_stateReg_string = "endpoint_ACK_TX_0         ";
      `endpoint_enumDefinition_binary_sequential_endpoint_ACK_TX_1 : endpoint_stateReg_string = "endpoint_ACK_TX_1         ";
      `endpoint_enumDefinition_binary_sequential_endpoint_ACK_TX_EOP : endpoint_stateReg_string = "endpoint_ACK_TX_EOP       ";
      `endpoint_enumDefinition_binary_sequential_endpoint_DATA_RX_WAIT_DMA : endpoint_stateReg_string = "endpoint_DATA_RX_WAIT_DMA ";
      `endpoint_enumDefinition_binary_sequential_endpoint_UPDATE_TD_PROCESS : endpoint_stateReg_string = "endpoint_UPDATE_TD_PROCESS";
      `endpoint_enumDefinition_binary_sequential_endpoint_UPDATE_TD_CMD : endpoint_stateReg_string = "endpoint_UPDATE_TD_CMD    ";
      `endpoint_enumDefinition_binary_sequential_endpoint_UPDATE_ED_CMD : endpoint_stateReg_string = "endpoint_UPDATE_ED_CMD    ";
      `endpoint_enumDefinition_binary_sequential_endpoint_UPDATE_SYNC : endpoint_stateReg_string = "endpoint_UPDATE_SYNC      ";
      `endpoint_enumDefinition_binary_sequential_endpoint_ABORD : endpoint_stateReg_string = "endpoint_ABORD            ";
      default : endpoint_stateReg_string = "??????????????????????????";
    endcase
  end
  always @(*) begin
    case(endpoint_stateNext)
      `endpoint_enumDefinition_binary_sequential_endpoint_BOOT : endpoint_stateNext_string = "endpoint_BOOT             ";
      `endpoint_enumDefinition_binary_sequential_endpoint_ED_READ_CMD : endpoint_stateNext_string = "endpoint_ED_READ_CMD      ";
      `endpoint_enumDefinition_binary_sequential_endpoint_ED_READ_RSP : endpoint_stateNext_string = "endpoint_ED_READ_RSP      ";
      `endpoint_enumDefinition_binary_sequential_endpoint_ED_ANALYSE : endpoint_stateNext_string = "endpoint_ED_ANALYSE       ";
      `endpoint_enumDefinition_binary_sequential_endpoint_TD_READ_CMD : endpoint_stateNext_string = "endpoint_TD_READ_CMD      ";
      `endpoint_enumDefinition_binary_sequential_endpoint_TD_READ_RSP : endpoint_stateNext_string = "endpoint_TD_READ_RSP      ";
      `endpoint_enumDefinition_binary_sequential_endpoint_TD_ANALYSE : endpoint_stateNext_string = "endpoint_TD_ANALYSE       ";
      `endpoint_enumDefinition_binary_sequential_endpoint_TD_CHECK_TIME : endpoint_stateNext_string = "endpoint_TD_CHECK_TIME    ";
      `endpoint_enumDefinition_binary_sequential_endpoint_BUFFER_READ : endpoint_stateNext_string = "endpoint_BUFFER_READ      ";
      `endpoint_enumDefinition_binary_sequential_endpoint_TOKEN : endpoint_stateNext_string = "endpoint_TOKEN            ";
      `endpoint_enumDefinition_binary_sequential_endpoint_DATA_TX : endpoint_stateNext_string = "endpoint_DATA_TX          ";
      `endpoint_enumDefinition_binary_sequential_endpoint_DATA_RX : endpoint_stateNext_string = "endpoint_DATA_RX          ";
      `endpoint_enumDefinition_binary_sequential_endpoint_DATA_RX_VALIDATE : endpoint_stateNext_string = "endpoint_DATA_RX_VALIDATE ";
      `endpoint_enumDefinition_binary_sequential_endpoint_ACK_RX : endpoint_stateNext_string = "endpoint_ACK_RX           ";
      `endpoint_enumDefinition_binary_sequential_endpoint_ACK_TX_0 : endpoint_stateNext_string = "endpoint_ACK_TX_0         ";
      `endpoint_enumDefinition_binary_sequential_endpoint_ACK_TX_1 : endpoint_stateNext_string = "endpoint_ACK_TX_1         ";
      `endpoint_enumDefinition_binary_sequential_endpoint_ACK_TX_EOP : endpoint_stateNext_string = "endpoint_ACK_TX_EOP       ";
      `endpoint_enumDefinition_binary_sequential_endpoint_DATA_RX_WAIT_DMA : endpoint_stateNext_string = "endpoint_DATA_RX_WAIT_DMA ";
      `endpoint_enumDefinition_binary_sequential_endpoint_UPDATE_TD_PROCESS : endpoint_stateNext_string = "endpoint_UPDATE_TD_PROCESS";
      `endpoint_enumDefinition_binary_sequential_endpoint_UPDATE_TD_CMD : endpoint_stateNext_string = "endpoint_UPDATE_TD_CMD    ";
      `endpoint_enumDefinition_binary_sequential_endpoint_UPDATE_ED_CMD : endpoint_stateNext_string = "endpoint_UPDATE_ED_CMD    ";
      `endpoint_enumDefinition_binary_sequential_endpoint_UPDATE_SYNC : endpoint_stateNext_string = "endpoint_UPDATE_SYNC      ";
      `endpoint_enumDefinition_binary_sequential_endpoint_ABORD : endpoint_stateNext_string = "endpoint_ABORD            ";
      default : endpoint_stateNext_string = "??????????????????????????";
    endcase
  end
  always @(*) begin
    case(endpoint_dmaLogic_stateReg)
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_BOOT : endpoint_dmaLogic_stateReg_string = "endpoint_dmaLogic_BOOT      ";
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_INIT : endpoint_dmaLogic_stateReg_string = "endpoint_dmaLogic_INIT      ";
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_TO_USB : endpoint_dmaLogic_stateReg_string = "endpoint_dmaLogic_TO_USB    ";
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_FROM_USB : endpoint_dmaLogic_stateReg_string = "endpoint_dmaLogic_FROM_USB  ";
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_VALIDATION : endpoint_dmaLogic_stateReg_string = "endpoint_dmaLogic_VALIDATION";
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_CALC_CMD : endpoint_dmaLogic_stateReg_string = "endpoint_dmaLogic_CALC_CMD  ";
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_READ_CMD : endpoint_dmaLogic_stateReg_string = "endpoint_dmaLogic_READ_CMD  ";
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_WRITE_CMD : endpoint_dmaLogic_stateReg_string = "endpoint_dmaLogic_WRITE_CMD ";
      default : endpoint_dmaLogic_stateReg_string = "????????????????????????????";
    endcase
  end
  always @(*) begin
    case(endpoint_dmaLogic_stateNext)
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_BOOT : endpoint_dmaLogic_stateNext_string = "endpoint_dmaLogic_BOOT      ";
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_INIT : endpoint_dmaLogic_stateNext_string = "endpoint_dmaLogic_INIT      ";
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_TO_USB : endpoint_dmaLogic_stateNext_string = "endpoint_dmaLogic_TO_USB    ";
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_FROM_USB : endpoint_dmaLogic_stateNext_string = "endpoint_dmaLogic_FROM_USB  ";
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_VALIDATION : endpoint_dmaLogic_stateNext_string = "endpoint_dmaLogic_VALIDATION";
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_CALC_CMD : endpoint_dmaLogic_stateNext_string = "endpoint_dmaLogic_CALC_CMD  ";
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_READ_CMD : endpoint_dmaLogic_stateNext_string = "endpoint_dmaLogic_READ_CMD  ";
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_WRITE_CMD : endpoint_dmaLogic_stateNext_string = "endpoint_dmaLogic_WRITE_CMD ";
      default : endpoint_dmaLogic_stateNext_string = "????????????????????????????";
    endcase
  end
  always @(*) begin
    case(operational_stateReg)
      `operational_enumDefinition_binary_sequential_operational_BOOT : operational_stateReg_string = "operational_BOOT             ";
      `operational_enumDefinition_binary_sequential_operational_SOF : operational_stateReg_string = "operational_SOF              ";
      `operational_enumDefinition_binary_sequential_operational_ARBITER : operational_stateReg_string = "operational_ARBITER          ";
      `operational_enumDefinition_binary_sequential_operational_END_POINT : operational_stateReg_string = "operational_END_POINT        ";
      `operational_enumDefinition_binary_sequential_operational_PERIODIC_HEAD_CMD : operational_stateReg_string = "operational_PERIODIC_HEAD_CMD";
      `operational_enumDefinition_binary_sequential_operational_PERIODIC_HEAD_RSP : operational_stateReg_string = "operational_PERIODIC_HEAD_RSP";
      `operational_enumDefinition_binary_sequential_operational_WAIT_SOF : operational_stateReg_string = "operational_WAIT_SOF         ";
      default : operational_stateReg_string = "?????????????????????????????";
    endcase
  end
  always @(*) begin
    case(operational_stateNext)
      `operational_enumDefinition_binary_sequential_operational_BOOT : operational_stateNext_string = "operational_BOOT             ";
      `operational_enumDefinition_binary_sequential_operational_SOF : operational_stateNext_string = "operational_SOF              ";
      `operational_enumDefinition_binary_sequential_operational_ARBITER : operational_stateNext_string = "operational_ARBITER          ";
      `operational_enumDefinition_binary_sequential_operational_END_POINT : operational_stateNext_string = "operational_END_POINT        ";
      `operational_enumDefinition_binary_sequential_operational_PERIODIC_HEAD_CMD : operational_stateNext_string = "operational_PERIODIC_HEAD_CMD";
      `operational_enumDefinition_binary_sequential_operational_PERIODIC_HEAD_RSP : operational_stateNext_string = "operational_PERIODIC_HEAD_RSP";
      `operational_enumDefinition_binary_sequential_operational_WAIT_SOF : operational_stateNext_string = "operational_WAIT_SOF         ";
      default : operational_stateNext_string = "?????????????????????????????";
    endcase
  end
  always @(*) begin
    case(hc_stateReg)
      `hc_enumDefinition_binary_sequential_hc_BOOT : hc_stateReg_string = "hc_BOOT          ";
      `hc_enumDefinition_binary_sequential_hc_RESET : hc_stateReg_string = "hc_RESET         ";
      `hc_enumDefinition_binary_sequential_hc_RESUME : hc_stateReg_string = "hc_RESUME        ";
      `hc_enumDefinition_binary_sequential_hc_OPERATIONAL : hc_stateReg_string = "hc_OPERATIONAL   ";
      `hc_enumDefinition_binary_sequential_hc_SUSPEND : hc_stateReg_string = "hc_SUSPEND       ";
      `hc_enumDefinition_binary_sequential_hc_ANY_TO_RESET : hc_stateReg_string = "hc_ANY_TO_RESET  ";
      `hc_enumDefinition_binary_sequential_hc_ANY_TO_SUSPEND : hc_stateReg_string = "hc_ANY_TO_SUSPEND";
      default : hc_stateReg_string = "?????????????????";
    endcase
  end
  always @(*) begin
    case(hc_stateNext)
      `hc_enumDefinition_binary_sequential_hc_BOOT : hc_stateNext_string = "hc_BOOT          ";
      `hc_enumDefinition_binary_sequential_hc_RESET : hc_stateNext_string = "hc_RESET         ";
      `hc_enumDefinition_binary_sequential_hc_RESUME : hc_stateNext_string = "hc_RESUME        ";
      `hc_enumDefinition_binary_sequential_hc_OPERATIONAL : hc_stateNext_string = "hc_OPERATIONAL   ";
      `hc_enumDefinition_binary_sequential_hc_SUSPEND : hc_stateNext_string = "hc_SUSPEND       ";
      `hc_enumDefinition_binary_sequential_hc_ANY_TO_RESET : hc_stateNext_string = "hc_ANY_TO_RESET  ";
      `hc_enumDefinition_binary_sequential_hc_ANY_TO_SUSPEND : hc_stateNext_string = "hc_ANY_TO_SUSPEND";
      default : hc_stateNext_string = "?????????????????";
    endcase
  end
  `endif

  always @ (*) begin
    io_phy_lowSpeed = 1'b0;
    if((! (endpoint_stateReg == `endpoint_enumDefinition_binary_sequential_endpoint_BOOT)))begin
      io_phy_lowSpeed = endpoint_ED_S;
    end
  end

  always @ (*) begin
    unscheduleAll_valid = 1'b0;
    if(doUnschedule)begin
      unscheduleAll_valid = 1'b1;
    end
  end

  always @ (*) begin
    unscheduleAll_ready = 1'b1;
    if((! dmaCtx_pendingEmpty))begin
      unscheduleAll_ready = 1'b0;
    end
  end

  assign dmaCtx_pendingFull = dmaCtx_pendingCounter[3];
  assign dmaCtx_pendingEmpty = (dmaCtx_pendingCounter == 4'b0000);
  assign _zz_1 = (! (dmaCtx_pendingFull || (unscheduleAll_valid && io_dma_cmd_payload_first)));
  assign ioDma_cmd_ready = (io_dma_cmd_ready && _zz_1);
  assign io_dma_cmd_valid = (ioDma_cmd_valid && _zz_1);
  assign io_dma_cmd_payload_last = ioDma_cmd_payload_last;
  assign io_dma_cmd_payload_fragment_opcode = ioDma_cmd_payload_fragment_opcode;
  assign io_dma_cmd_payload_fragment_address = ioDma_cmd_payload_fragment_address;
  assign io_dma_cmd_payload_fragment_length = ioDma_cmd_payload_fragment_length;
  assign io_dma_cmd_payload_fragment_data = ioDma_cmd_payload_fragment_data;
  assign io_dma_cmd_payload_fragment_mask = ioDma_cmd_payload_fragment_mask;
  assign ioDma_rsp_valid = io_dma_rsp_valid;
  assign io_dma_rsp_ready = ioDma_rsp_ready;
  assign ioDma_rsp_payload_last = io_dma_rsp_payload_last;
  assign ioDma_rsp_payload_fragment_opcode = io_dma_rsp_payload_fragment_opcode;
  assign ioDma_rsp_payload_fragment_data = io_dma_rsp_payload_fragment_data;
  always @ (*) begin
    ioDma_cmd_valid = 1'b0;
    case(sof_stateReg)
      `sof_enumDefinition_binary_sequential_sof_FRAME_TX : begin
      end
      `sof_enumDefinition_binary_sequential_sof_FRAME_NUMBER_CMD : begin
        ioDma_cmd_valid = 1'b1;
      end
      `sof_enumDefinition_binary_sequential_sof_FRAME_NUMBER_RSP : begin
      end
      default : begin
      end
    endcase
    case(endpoint_stateReg)
      `endpoint_enumDefinition_binary_sequential_endpoint_ED_READ_CMD : begin
        ioDma_cmd_valid = 1'b1;
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ED_READ_RSP : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ED_ANALYSE : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TD_READ_CMD : begin
        ioDma_cmd_valid = 1'b1;
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TD_READ_RSP : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TD_ANALYSE : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TD_CHECK_TIME : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_BUFFER_READ : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TOKEN : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_DATA_TX : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_DATA_RX : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_DATA_RX_VALIDATE : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ACK_RX : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ACK_TX_0 : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ACK_TX_1 : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ACK_TX_EOP : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_DATA_RX_WAIT_DMA : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_UPDATE_TD_PROCESS : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_UPDATE_TD_CMD : begin
        ioDma_cmd_valid = 1'b1;
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_UPDATE_ED_CMD : begin
        ioDma_cmd_valid = 1'b1;
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_UPDATE_SYNC : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ABORD : begin
      end
      default : begin
      end
    endcase
    case(endpoint_dmaLogic_stateReg)
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_INIT : begin
      end
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_TO_USB : begin
      end
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_FROM_USB : begin
      end
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_VALIDATION : begin
      end
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_CALC_CMD : begin
      end
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_READ_CMD : begin
        ioDma_cmd_valid = 1'b1;
      end
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_WRITE_CMD : begin
        ioDma_cmd_valid = 1'b1;
      end
      default : begin
      end
    endcase
    case(operational_stateReg)
      `operational_enumDefinition_binary_sequential_operational_SOF : begin
      end
      `operational_enumDefinition_binary_sequential_operational_ARBITER : begin
      end
      `operational_enumDefinition_binary_sequential_operational_END_POINT : begin
      end
      `operational_enumDefinition_binary_sequential_operational_PERIODIC_HEAD_CMD : begin
        ioDma_cmd_valid = 1'b1;
      end
      `operational_enumDefinition_binary_sequential_operational_PERIODIC_HEAD_RSP : begin
      end
      `operational_enumDefinition_binary_sequential_operational_WAIT_SOF : begin
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    ioDma_cmd_payload_last = 1'bx;
    case(sof_stateReg)
      `sof_enumDefinition_binary_sequential_sof_FRAME_TX : begin
      end
      `sof_enumDefinition_binary_sequential_sof_FRAME_NUMBER_CMD : begin
        ioDma_cmd_payload_last = (dmaWriteCtx_counter == 4'b0001);
      end
      `sof_enumDefinition_binary_sequential_sof_FRAME_NUMBER_RSP : begin
      end
      default : begin
      end
    endcase
    case(endpoint_stateReg)
      `endpoint_enumDefinition_binary_sequential_endpoint_ED_READ_CMD : begin
        ioDma_cmd_payload_last = 1'b1;
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ED_READ_RSP : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ED_ANALYSE : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TD_READ_CMD : begin
        ioDma_cmd_payload_last = 1'b1;
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TD_READ_RSP : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TD_ANALYSE : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TD_CHECK_TIME : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_BUFFER_READ : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TOKEN : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_DATA_TX : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_DATA_RX : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_DATA_RX_VALIDATE : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ACK_RX : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ACK_TX_0 : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ACK_TX_1 : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ACK_TX_EOP : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_DATA_RX_WAIT_DMA : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_UPDATE_TD_PROCESS : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_UPDATE_TD_CMD : begin
        ioDma_cmd_payload_last = (dmaWriteCtx_counter == _zz_217);
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_UPDATE_ED_CMD : begin
        ioDma_cmd_payload_last = (dmaWriteCtx_counter == 4'b0011);
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_UPDATE_SYNC : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ABORD : begin
      end
      default : begin
      end
    endcase
    case(endpoint_dmaLogic_stateReg)
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_INIT : begin
      end
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_TO_USB : begin
      end
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_FROM_USB : begin
      end
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_VALIDATION : begin
      end
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_CALC_CMD : begin
      end
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_READ_CMD : begin
        ioDma_cmd_payload_last = 1'b1;
      end
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_WRITE_CMD : begin
        ioDma_cmd_payload_last = endpoint_dmaLogic_beatLast;
      end
      default : begin
      end
    endcase
    case(operational_stateReg)
      `operational_enumDefinition_binary_sequential_operational_SOF : begin
      end
      `operational_enumDefinition_binary_sequential_operational_ARBITER : begin
      end
      `operational_enumDefinition_binary_sequential_operational_END_POINT : begin
      end
      `operational_enumDefinition_binary_sequential_operational_PERIODIC_HEAD_CMD : begin
        ioDma_cmd_payload_last = 1'b1;
      end
      `operational_enumDefinition_binary_sequential_operational_PERIODIC_HEAD_RSP : begin
      end
      `operational_enumDefinition_binary_sequential_operational_WAIT_SOF : begin
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    ioDma_cmd_payload_fragment_opcode = 1'bx;
    case(sof_stateReg)
      `sof_enumDefinition_binary_sequential_sof_FRAME_TX : begin
      end
      `sof_enumDefinition_binary_sequential_sof_FRAME_NUMBER_CMD : begin
        ioDma_cmd_payload_fragment_opcode = 1'b1;
      end
      `sof_enumDefinition_binary_sequential_sof_FRAME_NUMBER_RSP : begin
      end
      default : begin
      end
    endcase
    case(endpoint_stateReg)
      `endpoint_enumDefinition_binary_sequential_endpoint_ED_READ_CMD : begin
        ioDma_cmd_payload_fragment_opcode = 1'b0;
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ED_READ_RSP : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ED_ANALYSE : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TD_READ_CMD : begin
        ioDma_cmd_payload_fragment_opcode = 1'b0;
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TD_READ_RSP : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TD_ANALYSE : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TD_CHECK_TIME : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_BUFFER_READ : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TOKEN : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_DATA_TX : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_DATA_RX : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_DATA_RX_VALIDATE : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ACK_RX : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ACK_TX_0 : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ACK_TX_1 : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ACK_TX_EOP : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_DATA_RX_WAIT_DMA : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_UPDATE_TD_PROCESS : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_UPDATE_TD_CMD : begin
        ioDma_cmd_payload_fragment_opcode = 1'b1;
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_UPDATE_ED_CMD : begin
        ioDma_cmd_payload_fragment_opcode = 1'b1;
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_UPDATE_SYNC : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ABORD : begin
      end
      default : begin
      end
    endcase
    case(endpoint_dmaLogic_stateReg)
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_INIT : begin
      end
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_TO_USB : begin
      end
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_FROM_USB : begin
      end
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_VALIDATION : begin
      end
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_CALC_CMD : begin
      end
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_READ_CMD : begin
        ioDma_cmd_payload_fragment_opcode = 1'b0;
      end
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_WRITE_CMD : begin
        ioDma_cmd_payload_fragment_opcode = 1'b1;
      end
      default : begin
      end
    endcase
    case(operational_stateReg)
      `operational_enumDefinition_binary_sequential_operational_SOF : begin
      end
      `operational_enumDefinition_binary_sequential_operational_ARBITER : begin
      end
      `operational_enumDefinition_binary_sequential_operational_END_POINT : begin
      end
      `operational_enumDefinition_binary_sequential_operational_PERIODIC_HEAD_CMD : begin
        ioDma_cmd_payload_fragment_opcode = 1'b0;
      end
      `operational_enumDefinition_binary_sequential_operational_PERIODIC_HEAD_RSP : begin
      end
      `operational_enumDefinition_binary_sequential_operational_WAIT_SOF : begin
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    ioDma_cmd_payload_fragment_address = 32'bxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx;
    case(sof_stateReg)
      `sof_enumDefinition_binary_sequential_sof_FRAME_TX : begin
      end
      `sof_enumDefinition_binary_sequential_sof_FRAME_NUMBER_CMD : begin
        ioDma_cmd_payload_fragment_address = (reg_hcHCCA_HCCA_address | 32'h00000080);
      end
      `sof_enumDefinition_binary_sequential_sof_FRAME_NUMBER_RSP : begin
      end
      default : begin
      end
    endcase
    case(endpoint_stateReg)
      `endpoint_enumDefinition_binary_sequential_endpoint_ED_READ_CMD : begin
        ioDma_cmd_payload_fragment_address = endpoint_ED_address;
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ED_READ_RSP : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ED_ANALYSE : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TD_READ_CMD : begin
        ioDma_cmd_payload_fragment_address = endpoint_TD_address;
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TD_READ_RSP : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TD_ANALYSE : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TD_CHECK_TIME : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_BUFFER_READ : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TOKEN : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_DATA_TX : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_DATA_RX : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_DATA_RX_VALIDATE : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ACK_RX : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ACK_TX_0 : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ACK_TX_1 : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ACK_TX_EOP : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_DATA_RX_WAIT_DMA : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_UPDATE_TD_PROCESS : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_UPDATE_TD_CMD : begin
        ioDma_cmd_payload_fragment_address = endpoint_TD_address;
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_UPDATE_ED_CMD : begin
        ioDma_cmd_payload_fragment_address = endpoint_ED_address;
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_UPDATE_SYNC : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ABORD : begin
      end
      default : begin
      end
    endcase
    case(endpoint_dmaLogic_stateReg)
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_INIT : begin
      end
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_TO_USB : begin
      end
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_FROM_USB : begin
      end
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_VALIDATION : begin
      end
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_CALC_CMD : begin
      end
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_READ_CMD : begin
        ioDma_cmd_payload_fragment_address = endpoint_currentAddressBmb;
      end
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_WRITE_CMD : begin
        ioDma_cmd_payload_fragment_address = endpoint_currentAddressBmb;
      end
      default : begin
      end
    endcase
    case(operational_stateReg)
      `operational_enumDefinition_binary_sequential_operational_SOF : begin
      end
      `operational_enumDefinition_binary_sequential_operational_ARBITER : begin
      end
      `operational_enumDefinition_binary_sequential_operational_END_POINT : begin
      end
      `operational_enumDefinition_binary_sequential_operational_PERIODIC_HEAD_CMD : begin
        ioDma_cmd_payload_fragment_address = (reg_hcHCCA_HCCA_address | _zz_234);
      end
      `operational_enumDefinition_binary_sequential_operational_PERIODIC_HEAD_RSP : begin
      end
      `operational_enumDefinition_binary_sequential_operational_WAIT_SOF : begin
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    ioDma_cmd_payload_fragment_length = 6'bxxxxxx;
    case(sof_stateReg)
      `sof_enumDefinition_binary_sequential_sof_FRAME_TX : begin
      end
      `sof_enumDefinition_binary_sequential_sof_FRAME_NUMBER_CMD : begin
        ioDma_cmd_payload_fragment_length = 6'h07;
      end
      `sof_enumDefinition_binary_sequential_sof_FRAME_NUMBER_RSP : begin
      end
      default : begin
      end
    endcase
    case(endpoint_stateReg)
      `endpoint_enumDefinition_binary_sequential_endpoint_ED_READ_CMD : begin
        ioDma_cmd_payload_fragment_length = 6'h0f;
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ED_READ_RSP : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ED_ANALYSE : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TD_READ_CMD : begin
        ioDma_cmd_payload_fragment_length = {1'd0, _zz_205};
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TD_READ_RSP : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TD_ANALYSE : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TD_CHECK_TIME : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_BUFFER_READ : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TOKEN : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_DATA_TX : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_DATA_RX : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_DATA_RX_VALIDATE : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ACK_RX : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ACK_TX_0 : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ACK_TX_1 : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ACK_TX_EOP : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_DATA_RX_WAIT_DMA : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_UPDATE_TD_PROCESS : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_UPDATE_TD_CMD : begin
        ioDma_cmd_payload_fragment_length = {1'd0, _zz_215};
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_UPDATE_ED_CMD : begin
        ioDma_cmd_payload_fragment_length = 6'h0f;
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_UPDATE_SYNC : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ABORD : begin
      end
      default : begin
      end
    endcase
    case(endpoint_dmaLogic_stateReg)
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_INIT : begin
      end
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_TO_USB : begin
      end
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_FROM_USB : begin
      end
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_VALIDATION : begin
      end
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_CALC_CMD : begin
      end
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_READ_CMD : begin
        ioDma_cmd_payload_fragment_length = endpoint_dmaLogic_lengthBmb;
      end
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_WRITE_CMD : begin
        ioDma_cmd_payload_fragment_length = endpoint_dmaLogic_lengthBmb;
      end
      default : begin
      end
    endcase
    case(operational_stateReg)
      `operational_enumDefinition_binary_sequential_operational_SOF : begin
      end
      `operational_enumDefinition_binary_sequential_operational_ARBITER : begin
      end
      `operational_enumDefinition_binary_sequential_operational_END_POINT : begin
      end
      `operational_enumDefinition_binary_sequential_operational_PERIODIC_HEAD_CMD : begin
        ioDma_cmd_payload_fragment_length = 6'h03;
      end
      `operational_enumDefinition_binary_sequential_operational_PERIODIC_HEAD_RSP : begin
      end
      `operational_enumDefinition_binary_sequential_operational_WAIT_SOF : begin
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    ioDma_cmd_payload_fragment_data = 32'h0;
    case(sof_stateReg)
      `sof_enumDefinition_binary_sequential_sof_FRAME_TX : begin
      end
      `sof_enumDefinition_binary_sequential_sof_FRAME_NUMBER_CMD : begin
        if(_zz_77)begin
          ioDma_cmd_payload_fragment_data[31 : 0] = {16'h0,reg_hcFmNumber_FN};
        end
        if(sof_doInterruptDelay)begin
          if(_zz_78)begin
            ioDma_cmd_payload_fragment_data[31 : 0] = {reg_hcDoneHead_DH_address[31 : 1],reg_hcInterrupt_unmaskedPending};
          end
        end
      end
      `sof_enumDefinition_binary_sequential_sof_FRAME_NUMBER_RSP : begin
      end
      default : begin
      end
    endcase
    case(endpoint_stateReg)
      `endpoint_enumDefinition_binary_sequential_endpoint_ED_READ_CMD : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ED_READ_RSP : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ED_ANALYSE : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TD_READ_CMD : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TD_READ_RSP : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TD_ANALYSE : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TD_CHECK_TIME : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_BUFFER_READ : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TOKEN : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_DATA_TX : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_DATA_RX : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_DATA_RX_VALIDATE : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ACK_RX : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ACK_TX_0 : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ACK_TX_1 : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ACK_TX_EOP : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_DATA_RX_WAIT_DMA : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_UPDATE_TD_PROCESS : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_UPDATE_TD_CMD : begin
        if(endpoint_ED_F)begin
          if(endpoint_TD_isoOverrunReg)begin
            if(_zz_79)begin
              ioDma_cmd_payload_fragment_data[31 : 24] = {{4'b1000,endpoint_TD_words_0[27]},endpoint_TD_FC};
            end
          end else begin
            if(endpoint_TD_isoLast)begin
              if(_zz_80)begin
                ioDma_cmd_payload_fragment_data[31 : 24] = {{4'b0000,endpoint_TD_words_0[27]},endpoint_TD_FC};
              end
            end
            if(_zz_81)begin
              if(_zz_82)begin
                ioDma_cmd_payload_fragment_data[15 : 0] = _zz_63;
              end
            end
            if(_zz_83)begin
              if(_zz_84)begin
                ioDma_cmd_payload_fragment_data[31 : 16] = _zz_63;
              end
            end
            if(_zz_85)begin
              if(_zz_86)begin
                ioDma_cmd_payload_fragment_data[15 : 0] = _zz_63;
              end
            end
            if(_zz_87)begin
              if(_zz_88)begin
                ioDma_cmd_payload_fragment_data[31 : 16] = _zz_63;
              end
            end
            if(_zz_89)begin
              if(_zz_90)begin
                ioDma_cmd_payload_fragment_data[15 : 0] = _zz_63;
              end
            end
            if(_zz_91)begin
              if(_zz_92)begin
                ioDma_cmd_payload_fragment_data[31 : 16] = _zz_63;
              end
            end
            if(_zz_93)begin
              if(_zz_94)begin
                ioDma_cmd_payload_fragment_data[15 : 0] = _zz_63;
              end
            end
            if(_zz_95)begin
              if(_zz_96)begin
                ioDma_cmd_payload_fragment_data[31 : 16] = _zz_63;
              end
            end
          end
        end else begin
          if(_zz_97)begin
            ioDma_cmd_payload_fragment_data[31 : 24] = {{endpoint_TD_CC,endpoint_TD_EC},endpoint_TD_TNext};
          end
          if(endpoint_TD_upateCBP)begin
            if(_zz_98)begin
              ioDma_cmd_payload_fragment_data[31 : 0] = endpoint_tdUpdateAddress;
            end
          end
        end
        if(endpoint_TD_retire)begin
          if(_zz_99)begin
            ioDma_cmd_payload_fragment_data[31 : 0] = reg_hcDoneHead_DH_address;
          end
        end
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_UPDATE_ED_CMD : begin
        if(endpoint_TD_retire)begin
          if(_zz_100)begin
            ioDma_cmd_payload_fragment_data[31 : 0] = {{{endpoint_TD_nextTD,2'b00},endpoint_TD_dataPhaseNext},endpoint_ED_H};
          end
        end
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_UPDATE_SYNC : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ABORD : begin
      end
      default : begin
      end
    endcase
    case(endpoint_dmaLogic_stateReg)
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_INIT : begin
      end
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_TO_USB : begin
      end
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_FROM_USB : begin
      end
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_VALIDATION : begin
      end
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_CALC_CMD : begin
      end
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_READ_CMD : begin
      end
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_WRITE_CMD : begin
        ioDma_cmd_payload_fragment_data = fifo_io_pop_payload;
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    ioDma_cmd_payload_fragment_mask = 4'b0000;
    case(sof_stateReg)
      `sof_enumDefinition_binary_sequential_sof_FRAME_TX : begin
      end
      `sof_enumDefinition_binary_sequential_sof_FRAME_NUMBER_CMD : begin
        if(_zz_77)begin
          ioDma_cmd_payload_fragment_mask[3 : 0] = 4'b1111;
        end
        if(sof_doInterruptDelay)begin
          if(_zz_78)begin
            ioDma_cmd_payload_fragment_mask[3 : 0] = 4'b1111;
          end
        end
      end
      `sof_enumDefinition_binary_sequential_sof_FRAME_NUMBER_RSP : begin
      end
      default : begin
      end
    endcase
    case(endpoint_stateReg)
      `endpoint_enumDefinition_binary_sequential_endpoint_ED_READ_CMD : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ED_READ_RSP : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ED_ANALYSE : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TD_READ_CMD : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TD_READ_RSP : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TD_ANALYSE : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TD_CHECK_TIME : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_BUFFER_READ : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TOKEN : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_DATA_TX : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_DATA_RX : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_DATA_RX_VALIDATE : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ACK_RX : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ACK_TX_0 : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ACK_TX_1 : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ACK_TX_EOP : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_DATA_RX_WAIT_DMA : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_UPDATE_TD_PROCESS : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_UPDATE_TD_CMD : begin
        if(endpoint_ED_F)begin
          if(endpoint_TD_isoOverrunReg)begin
            if(_zz_79)begin
              ioDma_cmd_payload_fragment_mask[3 : 3] = 1'b1;
            end
          end else begin
            if(endpoint_TD_isoLast)begin
              if(_zz_80)begin
                ioDma_cmd_payload_fragment_mask[3 : 3] = 1'b1;
              end
            end
            if(_zz_81)begin
              if(_zz_82)begin
                ioDma_cmd_payload_fragment_mask[1 : 0] = 2'b11;
              end
            end
            if(_zz_83)begin
              if(_zz_84)begin
                ioDma_cmd_payload_fragment_mask[3 : 2] = 2'b11;
              end
            end
            if(_zz_85)begin
              if(_zz_86)begin
                ioDma_cmd_payload_fragment_mask[1 : 0] = 2'b11;
              end
            end
            if(_zz_87)begin
              if(_zz_88)begin
                ioDma_cmd_payload_fragment_mask[3 : 2] = 2'b11;
              end
            end
            if(_zz_89)begin
              if(_zz_90)begin
                ioDma_cmd_payload_fragment_mask[1 : 0] = 2'b11;
              end
            end
            if(_zz_91)begin
              if(_zz_92)begin
                ioDma_cmd_payload_fragment_mask[3 : 2] = 2'b11;
              end
            end
            if(_zz_93)begin
              if(_zz_94)begin
                ioDma_cmd_payload_fragment_mask[1 : 0] = 2'b11;
              end
            end
            if(_zz_95)begin
              if(_zz_96)begin
                ioDma_cmd_payload_fragment_mask[3 : 2] = 2'b11;
              end
            end
          end
        end else begin
          if(_zz_97)begin
            ioDma_cmd_payload_fragment_mask[3 : 3] = 1'b1;
          end
          if(endpoint_TD_upateCBP)begin
            if(_zz_98)begin
              ioDma_cmd_payload_fragment_mask[3 : 0] = 4'b1111;
            end
          end
        end
        if(endpoint_TD_retire)begin
          if(_zz_99)begin
            ioDma_cmd_payload_fragment_mask[3 : 0] = 4'b1111;
          end
        end
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_UPDATE_ED_CMD : begin
        if(endpoint_TD_retire)begin
          if(_zz_100)begin
            ioDma_cmd_payload_fragment_mask[3 : 0] = 4'b1111;
          end
        end
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_UPDATE_SYNC : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ABORD : begin
      end
      default : begin
      end
    endcase
    case(endpoint_dmaLogic_stateReg)
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_INIT : begin
      end
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_TO_USB : begin
      end
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_FROM_USB : begin
      end
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_VALIDATION : begin
      end
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_CALC_CMD : begin
      end
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_READ_CMD : begin
      end
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_WRITE_CMD : begin
        ioDma_cmd_payload_fragment_mask = ((endpoint_dmaLogic_fullMask & (ioDma_cmd_payload_first ? endpoint_dmaLogic_headMask : endpoint_dmaLogic_fullMask)) & (ioDma_cmd_payload_last ? endpoint_dmaLogic_lastMask : endpoint_dmaLogic_fullMask));
      end
      default : begin
      end
    endcase
  end

  assign ioDma_rsp_ready = 1'b1;
  assign dmaRspMux_vec_0 = ioDma_rsp_payload_fragment_data[31 : 0];
  assign dmaRspMux_data = dmaRspMux_vec_0;
  always @ (*) begin
    _zz_66 = 1'b0;
    if(_zz_101)begin
      _zz_66 = 1'b1;
    end
    if(endpoint_dmaLogic_push)begin
      _zz_66 = 1'b1;
    end
  end

  always @ (*) begin
    _zz_67 = 32'bxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx;
    if(_zz_101)begin
      _zz_67 = ioDma_rsp_payload_fragment_data;
    end
    if(endpoint_dmaLogic_push)begin
      _zz_67 = endpoint_dmaLogic_buffer;
    end
  end

  always @ (*) begin
    _zz_68 = 1'b0;
    case(endpoint_dmaLogic_stateReg)
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_INIT : begin
      end
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_TO_USB : begin
        if(dataTx_data_ready)begin
          if((endpoint_dmaLogic_byteCtx_sel == 2'b11))begin
            _zz_68 = 1'b1;
          end
        end
      end
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_FROM_USB : begin
      end
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_VALIDATION : begin
      end
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_CALC_CMD : begin
      end
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_READ_CMD : begin
      end
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_WRITE_CMD : begin
        if(ioDma_cmd_ready)begin
          _zz_68 = 1'b1;
        end
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    _zz_69 = 1'b0;
    case(endpoint_dmaLogic_stateReg)
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_INIT : begin
        _zz_69 = 1'b1;
      end
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_TO_USB : begin
      end
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_FROM_USB : begin
      end
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_VALIDATION : begin
      end
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_CALC_CMD : begin
      end
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_READ_CMD : begin
      end
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_WRITE_CMD : begin
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    io_phy_tx_valid = 1'b0;
    case(token_stateReg)
      `token_enumDefinition_binary_sequential_token_INIT : begin
      end
      `token_enumDefinition_binary_sequential_token_PID : begin
        io_phy_tx_valid = 1'b1;
      end
      `token_enumDefinition_binary_sequential_token_B1 : begin
        io_phy_tx_valid = 1'b1;
      end
      `token_enumDefinition_binary_sequential_token_B2 : begin
        io_phy_tx_valid = 1'b1;
      end
      `token_enumDefinition_binary_sequential_token_EOP : begin
      end
      default : begin
      end
    endcase
    case(dataTx_stateReg)
      `dataTx_enumDefinition_binary_sequential_dataTx_PID : begin
        io_phy_tx_valid = 1'b1;
      end
      `dataTx_enumDefinition_binary_sequential_dataTx_DATA : begin
        io_phy_tx_valid = 1'b1;
      end
      `dataTx_enumDefinition_binary_sequential_dataTx_CRC_0 : begin
        io_phy_tx_valid = 1'b1;
      end
      `dataTx_enumDefinition_binary_sequential_dataTx_CRC_1 : begin
        io_phy_tx_valid = 1'b1;
      end
      `dataTx_enumDefinition_binary_sequential_dataTx_EOP : begin
      end
      default : begin
      end
    endcase
    case(endpoint_stateReg)
      `endpoint_enumDefinition_binary_sequential_endpoint_ED_READ_CMD : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ED_READ_RSP : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ED_ANALYSE : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TD_READ_CMD : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TD_READ_RSP : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TD_ANALYSE : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TD_CHECK_TIME : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_BUFFER_READ : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TOKEN : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_DATA_TX : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_DATA_RX : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_DATA_RX_VALIDATE : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ACK_RX : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ACK_TX_0 : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ACK_TX_1 : begin
        io_phy_tx_valid = 1'b1;
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ACK_TX_EOP : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_DATA_RX_WAIT_DMA : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_UPDATE_TD_PROCESS : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_UPDATE_TD_CMD : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_UPDATE_ED_CMD : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_UPDATE_SYNC : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ABORD : begin
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    io_phy_tx_payload_fragment = 8'bxxxxxxxx;
    case(token_stateReg)
      `token_enumDefinition_binary_sequential_token_INIT : begin
      end
      `token_enumDefinition_binary_sequential_token_PID : begin
        io_phy_tx_payload_fragment = {(~ token_pid),token_pid};
      end
      `token_enumDefinition_binary_sequential_token_B1 : begin
        io_phy_tx_payload_fragment = token_data[7 : 0];
      end
      `token_enumDefinition_binary_sequential_token_B2 : begin
        io_phy_tx_payload_fragment = {token_crc5_io_result,token_data[10 : 8]};
      end
      `token_enumDefinition_binary_sequential_token_EOP : begin
      end
      default : begin
      end
    endcase
    case(dataTx_stateReg)
      `dataTx_enumDefinition_binary_sequential_dataTx_PID : begin
        io_phy_tx_payload_fragment = {(~ dataTx_pid),dataTx_pid};
      end
      `dataTx_enumDefinition_binary_sequential_dataTx_DATA : begin
        io_phy_tx_payload_fragment = dataTx_data_payload_fragment;
      end
      `dataTx_enumDefinition_binary_sequential_dataTx_CRC_0 : begin
        io_phy_tx_payload_fragment = dataTx_crc16_io_result[7 : 0];
      end
      `dataTx_enumDefinition_binary_sequential_dataTx_CRC_1 : begin
        io_phy_tx_payload_fragment = dataTx_crc16_io_result[15 : 8];
      end
      `dataTx_enumDefinition_binary_sequential_dataTx_EOP : begin
      end
      default : begin
      end
    endcase
    case(endpoint_stateReg)
      `endpoint_enumDefinition_binary_sequential_endpoint_ED_READ_CMD : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ED_READ_RSP : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ED_ANALYSE : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TD_READ_CMD : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TD_READ_RSP : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TD_ANALYSE : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TD_CHECK_TIME : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_BUFFER_READ : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TOKEN : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_DATA_TX : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_DATA_RX : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_DATA_RX_VALIDATE : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ACK_RX : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ACK_TX_0 : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ACK_TX_1 : begin
        io_phy_tx_payload_fragment = 8'hd2;
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ACK_TX_EOP : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_DATA_RX_WAIT_DMA : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_UPDATE_TD_PROCESS : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_UPDATE_TD_CMD : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_UPDATE_ED_CMD : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_UPDATE_SYNC : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ABORD : begin
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    io_phy_tx_payload_last = 1'bx;
    case(token_stateReg)
      `token_enumDefinition_binary_sequential_token_INIT : begin
      end
      `token_enumDefinition_binary_sequential_token_PID : begin
        io_phy_tx_payload_last = 1'b0;
      end
      `token_enumDefinition_binary_sequential_token_B1 : begin
        io_phy_tx_payload_last = 1'b0;
      end
      `token_enumDefinition_binary_sequential_token_B2 : begin
        io_phy_tx_payload_last = 1'b1;
      end
      `token_enumDefinition_binary_sequential_token_EOP : begin
      end
      default : begin
      end
    endcase
    case(dataTx_stateReg)
      `dataTx_enumDefinition_binary_sequential_dataTx_PID : begin
        io_phy_tx_payload_last = 1'b0;
      end
      `dataTx_enumDefinition_binary_sequential_dataTx_DATA : begin
        io_phy_tx_payload_last = 1'b0;
      end
      `dataTx_enumDefinition_binary_sequential_dataTx_CRC_0 : begin
        io_phy_tx_payload_last = 1'b0;
      end
      `dataTx_enumDefinition_binary_sequential_dataTx_CRC_1 : begin
        io_phy_tx_payload_last = 1'b1;
      end
      `dataTx_enumDefinition_binary_sequential_dataTx_EOP : begin
      end
      default : begin
      end
    endcase
    case(endpoint_stateReg)
      `endpoint_enumDefinition_binary_sequential_endpoint_ED_READ_CMD : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ED_READ_RSP : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ED_ANALYSE : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TD_READ_CMD : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TD_READ_RSP : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TD_ANALYSE : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TD_CHECK_TIME : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_BUFFER_READ : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TOKEN : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_DATA_TX : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_DATA_RX : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_DATA_RX_VALIDATE : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ACK_RX : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ACK_TX_0 : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ACK_TX_1 : begin
        io_phy_tx_payload_last = 1'b1;
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ACK_TX_EOP : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_DATA_RX_WAIT_DMA : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_UPDATE_TD_PROCESS : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_UPDATE_TD_CMD : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_UPDATE_ED_CMD : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_UPDATE_SYNC : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ABORD : begin
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    ctrlHalt = 1'b0;
    case(hc_stateReg)
      `hc_enumDefinition_binary_sequential_hc_RESET : begin
      end
      `hc_enumDefinition_binary_sequential_hc_RESUME : begin
      end
      `hc_enumDefinition_binary_sequential_hc_OPERATIONAL : begin
      end
      `hc_enumDefinition_binary_sequential_hc_SUSPEND : begin
      end
      `hc_enumDefinition_binary_sequential_hc_ANY_TO_RESET : begin
        ctrlHalt = 1'b1;
      end
      `hc_enumDefinition_binary_sequential_hc_ANY_TO_SUSPEND : begin
        ctrlHalt = 1'b1;
      end
      default : begin
      end
    endcase
  end

  assign ctrl_readHaltTrigger = 1'b0;
  always @ (*) begin
    ctrl_writeHaltTrigger = 1'b0;
    if(ctrlHalt)begin
      ctrl_writeHaltTrigger = 1'b1;
    end
  end

  assign _zz_2 = (! (ctrl_readHaltTrigger || ctrl_writeHaltTrigger));
  assign ctrl_rsp_ready = (_zz_3 && _zz_2);
  assign _zz_3 = ((1'b1 && (! _zz_4)) || io_ctrl_rsp_ready);
  assign _zz_4 = _zz_5;
  assign io_ctrl_rsp_valid = _zz_4;
  assign io_ctrl_rsp_payload_last = _zz_6;
  assign io_ctrl_rsp_payload_fragment_opcode = _zz_7;
  assign io_ctrl_rsp_payload_fragment_data = _zz_8;
  assign ctrl_askWrite = (io_ctrl_cmd_valid && (io_ctrl_cmd_payload_fragment_opcode == 1'b1));
  assign ctrl_askRead = (io_ctrl_cmd_valid && (io_ctrl_cmd_payload_fragment_opcode == 1'b0));
  assign ctrl_doWrite = ((io_ctrl_cmd_valid && io_ctrl_cmd_ready) && (io_ctrl_cmd_payload_fragment_opcode == 1'b1));
  assign ctrl_doRead = ((io_ctrl_cmd_valid && io_ctrl_cmd_ready) && (io_ctrl_cmd_payload_fragment_opcode == 1'b0));
  assign ctrl_rsp_valid = io_ctrl_cmd_valid;
  assign io_ctrl_cmd_ready = ctrl_rsp_ready;
  assign ctrl_rsp_payload_last = 1'b1;
  assign ctrl_rsp_payload_fragment_opcode = 1'b0;
  always @ (*) begin
    ctrl_rsp_payload_fragment_data = 32'h0;
    case(io_ctrl_cmd_payload_fragment_address)
      12'h0 : begin
        ctrl_rsp_payload_fragment_data[4 : 0] = reg_hcRevision_REV;
      end
      12'h004 : begin
        ctrl_rsp_payload_fragment_data[1 : 0] = reg_hcControl_CBSR;
        ctrl_rsp_payload_fragment_data[2 : 2] = reg_hcControl_PLE;
        ctrl_rsp_payload_fragment_data[3 : 3] = reg_hcControl_IE;
        ctrl_rsp_payload_fragment_data[4 : 4] = reg_hcControl_CLE;
        ctrl_rsp_payload_fragment_data[5 : 5] = reg_hcControl_BLE;
        ctrl_rsp_payload_fragment_data[7 : 6] = reg_hcControl_HCFS;
        ctrl_rsp_payload_fragment_data[8 : 8] = reg_hcControl_IR;
        ctrl_rsp_payload_fragment_data[9 : 9] = reg_hcControl_RWC;
        ctrl_rsp_payload_fragment_data[10 : 10] = reg_hcControl_RWE;
      end
      12'h008 : begin
        ctrl_rsp_payload_fragment_data[0 : 0] = doSoftReset;
        ctrl_rsp_payload_fragment_data[1 : 1] = reg_hcCommandStatus_CLF;
        ctrl_rsp_payload_fragment_data[2 : 2] = reg_hcCommandStatus_BLF;
        ctrl_rsp_payload_fragment_data[3 : 3] = reg_hcCommandStatus_OCR;
        ctrl_rsp_payload_fragment_data[17 : 16] = reg_hcCommandStatus_SOC;
      end
      12'h010 : begin
        ctrl_rsp_payload_fragment_data[31 : 31] = reg_hcInterrupt_MIE;
        ctrl_rsp_payload_fragment_data[0 : 0] = reg_hcInterrupt_SO_enable;
        ctrl_rsp_payload_fragment_data[1 : 1] = reg_hcInterrupt_WDH_enable;
        ctrl_rsp_payload_fragment_data[2 : 2] = reg_hcInterrupt_SF_enable;
        ctrl_rsp_payload_fragment_data[3 : 3] = reg_hcInterrupt_RD_enable;
        ctrl_rsp_payload_fragment_data[4 : 4] = reg_hcInterrupt_UE_enable;
        ctrl_rsp_payload_fragment_data[5 : 5] = reg_hcInterrupt_FNO_enable;
        ctrl_rsp_payload_fragment_data[6 : 6] = reg_hcInterrupt_RHSC_enable;
        ctrl_rsp_payload_fragment_data[30 : 30] = reg_hcInterrupt_OC_enable;
      end
      12'h014 : begin
        ctrl_rsp_payload_fragment_data[31 : 31] = reg_hcInterrupt_MIE;
        ctrl_rsp_payload_fragment_data[0 : 0] = reg_hcInterrupt_SO_enable;
        ctrl_rsp_payload_fragment_data[1 : 1] = reg_hcInterrupt_WDH_enable;
        ctrl_rsp_payload_fragment_data[2 : 2] = reg_hcInterrupt_SF_enable;
        ctrl_rsp_payload_fragment_data[3 : 3] = reg_hcInterrupt_RD_enable;
        ctrl_rsp_payload_fragment_data[4 : 4] = reg_hcInterrupt_UE_enable;
        ctrl_rsp_payload_fragment_data[5 : 5] = reg_hcInterrupt_FNO_enable;
        ctrl_rsp_payload_fragment_data[6 : 6] = reg_hcInterrupt_RHSC_enable;
        ctrl_rsp_payload_fragment_data[30 : 30] = reg_hcInterrupt_OC_enable;
      end
      12'h00c : begin
        ctrl_rsp_payload_fragment_data[0 : 0] = reg_hcInterrupt_SO_status;
        ctrl_rsp_payload_fragment_data[1 : 1] = reg_hcInterrupt_WDH_status;
        ctrl_rsp_payload_fragment_data[2 : 2] = reg_hcInterrupt_SF_status;
        ctrl_rsp_payload_fragment_data[3 : 3] = reg_hcInterrupt_RD_status;
        ctrl_rsp_payload_fragment_data[4 : 4] = reg_hcInterrupt_UE_status;
        ctrl_rsp_payload_fragment_data[5 : 5] = reg_hcInterrupt_FNO_status;
        ctrl_rsp_payload_fragment_data[6 : 6] = reg_hcInterrupt_RHSC_status;
        ctrl_rsp_payload_fragment_data[30 : 30] = reg_hcInterrupt_OC_status;
      end
      12'h018 : begin
        ctrl_rsp_payload_fragment_data[31 : 8] = reg_hcHCCA_HCCA_reg;
      end
      12'h01c : begin
        ctrl_rsp_payload_fragment_data[31 : 4] = reg_hcPeriodCurrentED_PCED_reg;
      end
      12'h020 : begin
        ctrl_rsp_payload_fragment_data[31 : 4] = reg_hcControlHeadED_CHED_reg;
      end
      12'h024 : begin
        ctrl_rsp_payload_fragment_data[31 : 4] = reg_hcControlCurrentED_CCED_reg;
      end
      12'h028 : begin
        ctrl_rsp_payload_fragment_data[31 : 4] = reg_hcBulkHeadED_BHED_reg;
      end
      12'h02c : begin
        ctrl_rsp_payload_fragment_data[31 : 4] = reg_hcBulkCurrentED_BCED_reg;
      end
      12'h030 : begin
        ctrl_rsp_payload_fragment_data[31 : 4] = reg_hcDoneHead_DH_reg;
      end
      12'h034 : begin
        ctrl_rsp_payload_fragment_data[13 : 0] = reg_hcFmInterval_FI;
        ctrl_rsp_payload_fragment_data[30 : 16] = reg_hcFmInterval_FSMPS;
        ctrl_rsp_payload_fragment_data[31 : 31] = reg_hcFmInterval_FIT;
      end
      12'h038 : begin
        ctrl_rsp_payload_fragment_data[13 : 0] = reg_hcFmRemaining_FR;
        ctrl_rsp_payload_fragment_data[31 : 31] = reg_hcFmRemaining_FRT;
      end
      12'h03c : begin
        ctrl_rsp_payload_fragment_data[15 : 0] = reg_hcFmNumber_FN;
      end
      12'h040 : begin
        ctrl_rsp_payload_fragment_data[13 : 0] = reg_hcPeriodicStart_PS;
      end
      12'h044 : begin
        ctrl_rsp_payload_fragment_data[11 : 0] = reg_hcLSThreshold_LST;
      end
      12'h048 : begin
        ctrl_rsp_payload_fragment_data[7 : 0] = reg_hcRhDescriptorA_NDP;
        ctrl_rsp_payload_fragment_data[8 : 8] = reg_hcRhDescriptorA_PSM;
        ctrl_rsp_payload_fragment_data[9 : 9] = reg_hcRhDescriptorA_NPS;
        ctrl_rsp_payload_fragment_data[11 : 11] = reg_hcRhDescriptorA_OCPM;
        ctrl_rsp_payload_fragment_data[12 : 12] = reg_hcRhDescriptorA_NOCP;
        ctrl_rsp_payload_fragment_data[31 : 24] = reg_hcRhDescriptorA_POTPGT;
      end
      12'h04c : begin
        ctrl_rsp_payload_fragment_data[1 : 1] = reg_hcRhDescriptorB_DR;
        ctrl_rsp_payload_fragment_data[17 : 17] = reg_hcRhDescriptorB_PPCM;
      end
      12'h050 : begin
        ctrl_rsp_payload_fragment_data[1 : 1] = io_phy_overcurrent;
        ctrl_rsp_payload_fragment_data[15 : 15] = reg_hcRhStatus_DRWE;
        ctrl_rsp_payload_fragment_data[17 : 17] = reg_hcRhStatus_CCIC;
      end
      12'h054 : begin
        ctrl_rsp_payload_fragment_data[2 : 2] = reg_hcRhPortStatus_0_PSS;
        ctrl_rsp_payload_fragment_data[8 : 8] = reg_hcRhPortStatus_0_PPS;
        ctrl_rsp_payload_fragment_data[0 : 0] = reg_hcRhPortStatus_0_CCS;
        ctrl_rsp_payload_fragment_data[1 : 1] = reg_hcRhPortStatus_0_PES;
        ctrl_rsp_payload_fragment_data[3 : 3] = io_phy_ports_0_overcurrent;
        ctrl_rsp_payload_fragment_data[4 : 4] = reg_hcRhPortStatus_0_reset;
        ctrl_rsp_payload_fragment_data[9 : 9] = io_phy_ports_0_lowSpeed;
        ctrl_rsp_payload_fragment_data[16 : 16] = reg_hcRhPortStatus_0_CSC_reg;
        ctrl_rsp_payload_fragment_data[17 : 17] = reg_hcRhPortStatus_0_PESC_reg;
        ctrl_rsp_payload_fragment_data[18 : 18] = reg_hcRhPortStatus_0_PSSC_reg;
        ctrl_rsp_payload_fragment_data[19 : 19] = reg_hcRhPortStatus_0_OCIC_reg;
        ctrl_rsp_payload_fragment_data[20 : 20] = reg_hcRhPortStatus_0_PRSC_reg;
      end
      default : begin
      end
    endcase
  end

  assign reg_hcRevision_REV = 5'h10;
  always @ (*) begin
    reg_hcControl_HCFSWrite_valid = 1'b0;
    case(io_ctrl_cmd_payload_fragment_address)
      12'h004 : begin
        if(ctrl_doWrite)begin
          reg_hcControl_HCFSWrite_valid = 1'b1;
        end
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    reg_hcCommandStatus_startSoftReset = 1'b0;
    if(_zz_9)begin
      if(io_ctrl_cmd_payload_fragment_data[0])begin
        reg_hcCommandStatus_startSoftReset = _zz_125[0];
      end
    end
  end

  always @ (*) begin
    _zz_9 = 1'b0;
    case(io_ctrl_cmd_payload_fragment_address)
      12'h008 : begin
        if(ctrl_doWrite)begin
          _zz_9 = 1'b1;
        end
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    _zz_10 = 1'b0;
    case(io_ctrl_cmd_payload_fragment_address)
      12'h008 : begin
        if(ctrl_doWrite)begin
          _zz_10 = 1'b1;
        end
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    _zz_11 = 1'b0;
    case(io_ctrl_cmd_payload_fragment_address)
      12'h008 : begin
        if(ctrl_doWrite)begin
          _zz_11 = 1'b1;
        end
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    _zz_12 = 1'b0;
    case(io_ctrl_cmd_payload_fragment_address)
      12'h008 : begin
        if(ctrl_doWrite)begin
          _zz_12 = 1'b1;
        end
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    reg_hcInterrupt_unmaskedPending = 1'b0;
    if((reg_hcInterrupt_SO_status && reg_hcInterrupt_SO_enable))begin
      reg_hcInterrupt_unmaskedPending = 1'b1;
    end
    if((reg_hcInterrupt_WDH_status && reg_hcInterrupt_WDH_enable))begin
      reg_hcInterrupt_unmaskedPending = 1'b1;
    end
    if((reg_hcInterrupt_SF_status && reg_hcInterrupt_SF_enable))begin
      reg_hcInterrupt_unmaskedPending = 1'b1;
    end
    if((reg_hcInterrupt_RD_status && reg_hcInterrupt_RD_enable))begin
      reg_hcInterrupt_unmaskedPending = 1'b1;
    end
    if((reg_hcInterrupt_UE_status && reg_hcInterrupt_UE_enable))begin
      reg_hcInterrupt_unmaskedPending = 1'b1;
    end
    if((reg_hcInterrupt_FNO_status && reg_hcInterrupt_FNO_enable))begin
      reg_hcInterrupt_unmaskedPending = 1'b1;
    end
    if((reg_hcInterrupt_RHSC_status && reg_hcInterrupt_RHSC_enable))begin
      reg_hcInterrupt_unmaskedPending = 1'b1;
    end
  end

  always @ (*) begin
    _zz_13 = 1'b0;
    case(io_ctrl_cmd_payload_fragment_address)
      12'h010 : begin
        if(ctrl_doWrite)begin
          _zz_13 = 1'b1;
        end
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    _zz_14 = 1'b0;
    case(io_ctrl_cmd_payload_fragment_address)
      12'h014 : begin
        if(ctrl_doWrite)begin
          _zz_14 = 1'b1;
        end
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    _zz_15 = 1'b0;
    case(io_ctrl_cmd_payload_fragment_address)
      12'h00c : begin
        if(ctrl_doWrite)begin
          _zz_15 = 1'b1;
        end
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    _zz_16 = 1'b0;
    case(io_ctrl_cmd_payload_fragment_address)
      12'h010 : begin
        if(ctrl_doWrite)begin
          _zz_16 = 1'b1;
        end
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    _zz_17 = 1'b0;
    case(io_ctrl_cmd_payload_fragment_address)
      12'h014 : begin
        if(ctrl_doWrite)begin
          _zz_17 = 1'b1;
        end
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    _zz_18 = 1'b0;
    case(io_ctrl_cmd_payload_fragment_address)
      12'h00c : begin
        if(ctrl_doWrite)begin
          _zz_18 = 1'b1;
        end
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    _zz_19 = 1'b0;
    case(io_ctrl_cmd_payload_fragment_address)
      12'h010 : begin
        if(ctrl_doWrite)begin
          _zz_19 = 1'b1;
        end
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    _zz_20 = 1'b0;
    case(io_ctrl_cmd_payload_fragment_address)
      12'h014 : begin
        if(ctrl_doWrite)begin
          _zz_20 = 1'b1;
        end
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    _zz_21 = 1'b0;
    case(io_ctrl_cmd_payload_fragment_address)
      12'h00c : begin
        if(ctrl_doWrite)begin
          _zz_21 = 1'b1;
        end
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    _zz_22 = 1'b0;
    case(io_ctrl_cmd_payload_fragment_address)
      12'h010 : begin
        if(ctrl_doWrite)begin
          _zz_22 = 1'b1;
        end
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    _zz_23 = 1'b0;
    case(io_ctrl_cmd_payload_fragment_address)
      12'h014 : begin
        if(ctrl_doWrite)begin
          _zz_23 = 1'b1;
        end
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    _zz_24 = 1'b0;
    case(io_ctrl_cmd_payload_fragment_address)
      12'h00c : begin
        if(ctrl_doWrite)begin
          _zz_24 = 1'b1;
        end
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    _zz_25 = 1'b0;
    case(io_ctrl_cmd_payload_fragment_address)
      12'h010 : begin
        if(ctrl_doWrite)begin
          _zz_25 = 1'b1;
        end
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    _zz_26 = 1'b0;
    case(io_ctrl_cmd_payload_fragment_address)
      12'h014 : begin
        if(ctrl_doWrite)begin
          _zz_26 = 1'b1;
        end
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    _zz_27 = 1'b0;
    case(io_ctrl_cmd_payload_fragment_address)
      12'h00c : begin
        if(ctrl_doWrite)begin
          _zz_27 = 1'b1;
        end
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    _zz_28 = 1'b0;
    case(io_ctrl_cmd_payload_fragment_address)
      12'h010 : begin
        if(ctrl_doWrite)begin
          _zz_28 = 1'b1;
        end
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    _zz_29 = 1'b0;
    case(io_ctrl_cmd_payload_fragment_address)
      12'h014 : begin
        if(ctrl_doWrite)begin
          _zz_29 = 1'b1;
        end
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    _zz_30 = 1'b0;
    case(io_ctrl_cmd_payload_fragment_address)
      12'h00c : begin
        if(ctrl_doWrite)begin
          _zz_30 = 1'b1;
        end
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    _zz_31 = 1'b0;
    case(io_ctrl_cmd_payload_fragment_address)
      12'h010 : begin
        if(ctrl_doWrite)begin
          _zz_31 = 1'b1;
        end
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    _zz_32 = 1'b0;
    case(io_ctrl_cmd_payload_fragment_address)
      12'h014 : begin
        if(ctrl_doWrite)begin
          _zz_32 = 1'b1;
        end
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    _zz_33 = 1'b0;
    case(io_ctrl_cmd_payload_fragment_address)
      12'h00c : begin
        if(ctrl_doWrite)begin
          _zz_33 = 1'b1;
        end
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    _zz_34 = 1'b0;
    case(io_ctrl_cmd_payload_fragment_address)
      12'h010 : begin
        if(ctrl_doWrite)begin
          _zz_34 = 1'b1;
        end
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    _zz_35 = 1'b0;
    case(io_ctrl_cmd_payload_fragment_address)
      12'h014 : begin
        if(ctrl_doWrite)begin
          _zz_35 = 1'b1;
        end
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    _zz_36 = 1'b0;
    case(io_ctrl_cmd_payload_fragment_address)
      12'h00c : begin
        if(ctrl_doWrite)begin
          _zz_36 = 1'b1;
        end
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    _zz_37 = 1'b0;
    case(io_ctrl_cmd_payload_fragment_address)
      12'h010 : begin
        if(ctrl_doWrite)begin
          _zz_37 = 1'b1;
        end
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    _zz_38 = 1'b0;
    case(io_ctrl_cmd_payload_fragment_address)
      12'h014 : begin
        if(ctrl_doWrite)begin
          _zz_38 = 1'b1;
        end
      end
      default : begin
      end
    endcase
  end

  assign reg_hcInterrupt_doIrq = (reg_hcInterrupt_unmaskedPending && reg_hcInterrupt_MIE);
  assign io_interrupt = (reg_hcInterrupt_doIrq && (! reg_hcControl_IR));
  assign io_interruptBios = ((reg_hcInterrupt_doIrq && reg_hcControl_IR) || (reg_hcInterrupt_OC_status && reg_hcInterrupt_OC_enable));
  assign reg_hcHCCA_HCCA_address = {reg_hcHCCA_HCCA_reg,8'h0};
  assign reg_hcPeriodCurrentED_PCED_address = {reg_hcPeriodCurrentED_PCED_reg,4'b0000};
  assign reg_hcPeriodCurrentED_isZero = (reg_hcPeriodCurrentED_PCED_reg == 28'h0);
  assign reg_hcControlHeadED_CHED_address = {reg_hcControlHeadED_CHED_reg,4'b0000};
  assign reg_hcControlCurrentED_CCED_address = {reg_hcControlCurrentED_CCED_reg,4'b0000};
  assign reg_hcControlCurrentED_isZero = (reg_hcControlCurrentED_CCED_reg == 28'h0);
  assign reg_hcBulkHeadED_BHED_address = {reg_hcBulkHeadED_BHED_reg,4'b0000};
  assign reg_hcBulkCurrentED_BCED_address = {reg_hcBulkCurrentED_BCED_reg,4'b0000};
  assign reg_hcBulkCurrentED_isZero = (reg_hcBulkCurrentED_BCED_reg == 28'h0);
  assign reg_hcDoneHead_DH_address = {reg_hcDoneHead_DH_reg,4'b0000};
  assign reg_hcFmNumber_FNp1 = (reg_hcFmNumber_FN + 16'h0001);
  assign reg_hcLSThreshold_hit = (reg_hcFmRemaining_FR < _zz_155);
  assign reg_hcRhDescriptorA_NDP = 8'h01;
  always @ (*) begin
    _zz_39 = 1'b0;
    case(io_ctrl_cmd_payload_fragment_address)
      12'h050 : begin
        if(ctrl_doWrite)begin
          _zz_39 = 1'b1;
        end
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    reg_hcRhStatus_clearGlobalPower = 1'b0;
    if(_zz_40)begin
      if(io_ctrl_cmd_payload_fragment_data[0])begin
        reg_hcRhStatus_clearGlobalPower = _zz_157[0];
      end
    end
  end

  always @ (*) begin
    _zz_40 = 1'b0;
    case(io_ctrl_cmd_payload_fragment_address)
      12'h050 : begin
        if(ctrl_doWrite)begin
          _zz_40 = 1'b1;
        end
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    reg_hcRhStatus_setRemoteWakeupEnable = 1'b0;
    if(_zz_41)begin
      if(io_ctrl_cmd_payload_fragment_data[15])begin
        reg_hcRhStatus_setRemoteWakeupEnable = _zz_158[0];
      end
    end
  end

  always @ (*) begin
    _zz_41 = 1'b0;
    case(io_ctrl_cmd_payload_fragment_address)
      12'h050 : begin
        if(ctrl_doWrite)begin
          _zz_41 = 1'b1;
        end
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    reg_hcRhStatus_setGlobalPower = 1'b0;
    if(_zz_42)begin
      if(io_ctrl_cmd_payload_fragment_data[16])begin
        reg_hcRhStatus_setGlobalPower = _zz_159[0];
      end
    end
  end

  always @ (*) begin
    _zz_42 = 1'b0;
    case(io_ctrl_cmd_payload_fragment_address)
      12'h050 : begin
        if(ctrl_doWrite)begin
          _zz_42 = 1'b1;
        end
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    reg_hcRhStatus_clearRemoteWakeupEnable = 1'b0;
    if(_zz_43)begin
      if(io_ctrl_cmd_payload_fragment_data[31])begin
        reg_hcRhStatus_clearRemoteWakeupEnable = _zz_160[0];
      end
    end
  end

  always @ (*) begin
    _zz_43 = 1'b0;
    case(io_ctrl_cmd_payload_fragment_address)
      12'h050 : begin
        if(ctrl_doWrite)begin
          _zz_43 = 1'b1;
        end
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    reg_hcRhPortStatus_0_clearPortEnable = 1'b0;
    if(_zz_44)begin
      if(io_ctrl_cmd_payload_fragment_data[0])begin
        reg_hcRhPortStatus_0_clearPortEnable = _zz_161[0];
      end
    end
  end

  always @ (*) begin
    _zz_44 = 1'b0;
    case(io_ctrl_cmd_payload_fragment_address)
      12'h054 : begin
        if(ctrl_doWrite)begin
          _zz_44 = 1'b1;
        end
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    reg_hcRhPortStatus_0_setPortEnable = 1'b0;
    if(_zz_45)begin
      if(io_ctrl_cmd_payload_fragment_data[1])begin
        reg_hcRhPortStatus_0_setPortEnable = _zz_162[0];
      end
    end
  end

  always @ (*) begin
    _zz_45 = 1'b0;
    case(io_ctrl_cmd_payload_fragment_address)
      12'h054 : begin
        if(ctrl_doWrite)begin
          _zz_45 = 1'b1;
        end
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    reg_hcRhPortStatus_0_setPortSuspend = 1'b0;
    if(_zz_46)begin
      if(io_ctrl_cmd_payload_fragment_data[2])begin
        reg_hcRhPortStatus_0_setPortSuspend = _zz_163[0];
      end
    end
  end

  always @ (*) begin
    _zz_46 = 1'b0;
    case(io_ctrl_cmd_payload_fragment_address)
      12'h054 : begin
        if(ctrl_doWrite)begin
          _zz_46 = 1'b1;
        end
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    reg_hcRhPortStatus_0_clearSuspendStatus = 1'b0;
    if(_zz_47)begin
      if(io_ctrl_cmd_payload_fragment_data[3])begin
        reg_hcRhPortStatus_0_clearSuspendStatus = _zz_164[0];
      end
    end
  end

  always @ (*) begin
    _zz_47 = 1'b0;
    case(io_ctrl_cmd_payload_fragment_address)
      12'h054 : begin
        if(ctrl_doWrite)begin
          _zz_47 = 1'b1;
        end
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    reg_hcRhPortStatus_0_setPortReset = 1'b0;
    if(_zz_48)begin
      if(io_ctrl_cmd_payload_fragment_data[4])begin
        reg_hcRhPortStatus_0_setPortReset = _zz_165[0];
      end
    end
  end

  always @ (*) begin
    _zz_48 = 1'b0;
    case(io_ctrl_cmd_payload_fragment_address)
      12'h054 : begin
        if(ctrl_doWrite)begin
          _zz_48 = 1'b1;
        end
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    reg_hcRhPortStatus_0_setPortPower = 1'b0;
    if(_zz_49)begin
      if(io_ctrl_cmd_payload_fragment_data[8])begin
        reg_hcRhPortStatus_0_setPortPower = _zz_166[0];
      end
    end
  end

  always @ (*) begin
    _zz_49 = 1'b0;
    case(io_ctrl_cmd_payload_fragment_address)
      12'h054 : begin
        if(ctrl_doWrite)begin
          _zz_49 = 1'b1;
        end
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    reg_hcRhPortStatus_0_clearPortPower = 1'b0;
    if(_zz_50)begin
      if(io_ctrl_cmd_payload_fragment_data[9])begin
        reg_hcRhPortStatus_0_clearPortPower = _zz_167[0];
      end
    end
  end

  always @ (*) begin
    _zz_50 = 1'b0;
    case(io_ctrl_cmd_payload_fragment_address)
      12'h054 : begin
        if(ctrl_doWrite)begin
          _zz_50 = 1'b1;
        end
      end
      default : begin
      end
    endcase
  end

  assign reg_hcRhPortStatus_0_CCS = ((reg_hcRhPortStatus_0_connected || reg_hcRhDescriptorB_DR[0]) && reg_hcRhPortStatus_0_PPS);
  always @ (*) begin
    reg_hcRhPortStatus_0_CSC_clear = 1'b0;
    if(_zz_51)begin
      if(io_ctrl_cmd_payload_fragment_data[16])begin
        reg_hcRhPortStatus_0_CSC_clear = _zz_168[0];
      end
    end
  end

  always @ (*) begin
    _zz_51 = 1'b0;
    case(io_ctrl_cmd_payload_fragment_address)
      12'h054 : begin
        if(ctrl_doWrite)begin
          _zz_51 = 1'b1;
        end
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    reg_hcRhPortStatus_0_PESC_clear = 1'b0;
    if(_zz_52)begin
      if(io_ctrl_cmd_payload_fragment_data[17])begin
        reg_hcRhPortStatus_0_PESC_clear = _zz_169[0];
      end
    end
  end

  always @ (*) begin
    _zz_52 = 1'b0;
    case(io_ctrl_cmd_payload_fragment_address)
      12'h054 : begin
        if(ctrl_doWrite)begin
          _zz_52 = 1'b1;
        end
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    reg_hcRhPortStatus_0_PSSC_clear = 1'b0;
    if(_zz_53)begin
      if(io_ctrl_cmd_payload_fragment_data[18])begin
        reg_hcRhPortStatus_0_PSSC_clear = _zz_170[0];
      end
    end
    if(reg_hcRhPortStatus_0_PRSC_set)begin
      reg_hcRhPortStatus_0_PSSC_clear = 1'b1;
    end
  end

  always @ (*) begin
    _zz_53 = 1'b0;
    case(io_ctrl_cmd_payload_fragment_address)
      12'h054 : begin
        if(ctrl_doWrite)begin
          _zz_53 = 1'b1;
        end
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    reg_hcRhPortStatus_0_OCIC_clear = 1'b0;
    if(_zz_54)begin
      if(io_ctrl_cmd_payload_fragment_data[19])begin
        reg_hcRhPortStatus_0_OCIC_clear = _zz_171[0];
      end
    end
  end

  always @ (*) begin
    _zz_54 = 1'b0;
    case(io_ctrl_cmd_payload_fragment_address)
      12'h054 : begin
        if(ctrl_doWrite)begin
          _zz_54 = 1'b1;
        end
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    reg_hcRhPortStatus_0_PRSC_clear = 1'b0;
    if(_zz_55)begin
      if(io_ctrl_cmd_payload_fragment_data[20])begin
        reg_hcRhPortStatus_0_PRSC_clear = _zz_172[0];
      end
    end
  end

  always @ (*) begin
    _zz_55 = 1'b0;
    case(io_ctrl_cmd_payload_fragment_address)
      12'h054 : begin
        if(ctrl_doWrite)begin
          _zz_55 = 1'b1;
        end
      end
      default : begin
      end
    endcase
  end

  assign reg_hcRhPortStatus_0_CSC_set = ((((reg_hcRhPortStatus_0_CCS ^ reg_hcRhPortStatus_0_CCS_regNext) || (reg_hcRhPortStatus_0_setPortEnable && (! reg_hcRhPortStatus_0_CCS))) || (reg_hcRhPortStatus_0_setPortSuspend && (! reg_hcRhPortStatus_0_CCS))) || (reg_hcRhPortStatus_0_setPortReset && (! reg_hcRhPortStatus_0_CCS)));
  assign reg_hcRhPortStatus_0_PESC_set = io_phy_ports_0_overcurrent;
  assign reg_hcRhPortStatus_0_PSSC_set = ((io_phy_ports_0_suspend_valid && io_phy_ports_0_suspend_ready) || io_phy_ports_0_remoteResume);
  assign reg_hcRhPortStatus_0_OCIC_set = io_phy_ports_0_overcurrent;
  assign reg_hcRhPortStatus_0_PRSC_set = (io_phy_ports_0_reset_valid && io_phy_ports_0_reset_ready);
  assign io_phy_ports_0_disable_valid = reg_hcRhPortStatus_0_clearPortEnable;
  assign io_phy_ports_0_removable = reg_hcRhDescriptorB_DR[0];
  assign io_phy_ports_0_power = reg_hcRhPortStatus_0_PPS;
  assign io_phy_ports_0_resume_valid = reg_hcRhPortStatus_0_resume;
  assign io_phy_ports_0_reset_valid = reg_hcRhPortStatus_0_reset;
  assign io_phy_ports_0_suspend_valid = reg_hcRhPortStatus_0_suspend;
  always @ (*) begin
    frame_run = 1'b0;
    case(hc_stateReg)
      `hc_enumDefinition_binary_sequential_hc_RESET : begin
      end
      `hc_enumDefinition_binary_sequential_hc_RESUME : begin
      end
      `hc_enumDefinition_binary_sequential_hc_OPERATIONAL : begin
        frame_run = 1'b1;
      end
      `hc_enumDefinition_binary_sequential_hc_SUSPEND : begin
      end
      `hc_enumDefinition_binary_sequential_hc_ANY_TO_RESET : begin
      end
      `hc_enumDefinition_binary_sequential_hc_ANY_TO_SUSPEND : begin
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    frame_reload = 1'b0;
    if(_zz_102)begin
      if(frame_overflow)begin
        frame_reload = 1'b1;
      end
    end
    if(_zz_103)begin
      frame_reload = 1'b1;
    end
  end

  assign frame_overflow = (reg_hcFmRemaining_FR == 14'h0);
  always @ (*) begin
    frame_tick = 1'b0;
    if(_zz_102)begin
      if(frame_overflow)begin
        frame_tick = 1'b1;
      end
    end
  end

  assign frame_section1 = (reg_hcPeriodicStart_PS < reg_hcFmRemaining_FR);
  assign frame_limitHit = (frame_limitCounter == 15'h0);
  assign frame_decrementTimerOverflow = (frame_decrementTimer == 3'b110);
  always @ (*) begin
    token_wantExit = 1'b0;
    case(token_stateReg)
      `token_enumDefinition_binary_sequential_token_INIT : begin
      end
      `token_enumDefinition_binary_sequential_token_PID : begin
      end
      `token_enumDefinition_binary_sequential_token_B1 : begin
      end
      `token_enumDefinition_binary_sequential_token_B2 : begin
      end
      `token_enumDefinition_binary_sequential_token_EOP : begin
        if(io_phy_txEop)begin
          token_wantExit = 1'b1;
        end
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    token_wantStart = 1'b0;
    if(((sof_stateReg == `sof_enumDefinition_binary_sequential_sof_BOOT) && (! (sof_stateNext == `sof_enumDefinition_binary_sequential_sof_BOOT))))begin
      token_wantStart = 1'b1;
    end
    if(((! (endpoint_stateReg == `endpoint_enumDefinition_binary_sequential_endpoint_TOKEN)) && (endpoint_stateNext == `endpoint_enumDefinition_binary_sequential_endpoint_TOKEN)))begin
      token_wantStart = 1'b1;
    end
  end

  always @ (*) begin
    token_wantKill = 1'b0;
    if((unscheduleAll_valid && unscheduleAll_ready))begin
      token_wantKill = 1'b1;
    end
  end

  always @ (*) begin
    token_pid = 4'bxxxx;
    case(sof_stateReg)
      `sof_enumDefinition_binary_sequential_sof_FRAME_TX : begin
        token_pid = 4'b0101;
      end
      `sof_enumDefinition_binary_sequential_sof_FRAME_NUMBER_CMD : begin
      end
      `sof_enumDefinition_binary_sequential_sof_FRAME_NUMBER_RSP : begin
      end
      default : begin
      end
    endcase
    case(endpoint_stateReg)
      `endpoint_enumDefinition_binary_sequential_endpoint_ED_READ_CMD : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ED_READ_RSP : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ED_ANALYSE : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TD_READ_CMD : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TD_READ_RSP : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TD_ANALYSE : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TD_CHECK_TIME : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_BUFFER_READ : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TOKEN : begin
        case(endpoint_tockenType)
          2'b00 : begin
            token_pid = 4'b1101;
          end
          2'b01 : begin
            token_pid = 4'b0001;
          end
          2'b10 : begin
            token_pid = 4'b1001;
          end
          default : begin
          end
        endcase
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_DATA_TX : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_DATA_RX : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_DATA_RX_VALIDATE : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ACK_RX : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ACK_TX_0 : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ACK_TX_1 : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ACK_TX_EOP : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_DATA_RX_WAIT_DMA : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_UPDATE_TD_PROCESS : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_UPDATE_TD_CMD : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_UPDATE_ED_CMD : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_UPDATE_SYNC : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ABORD : begin
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    token_data = 11'bxxxxxxxxxxx;
    case(sof_stateReg)
      `sof_enumDefinition_binary_sequential_sof_FRAME_TX : begin
        token_data = _zz_204[10:0];
      end
      `sof_enumDefinition_binary_sequential_sof_FRAME_NUMBER_CMD : begin
      end
      `sof_enumDefinition_binary_sequential_sof_FRAME_NUMBER_RSP : begin
      end
      default : begin
      end
    endcase
    case(endpoint_stateReg)
      `endpoint_enumDefinition_binary_sequential_endpoint_ED_READ_CMD : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ED_READ_RSP : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ED_ANALYSE : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TD_READ_CMD : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TD_READ_RSP : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TD_ANALYSE : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TD_CHECK_TIME : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_BUFFER_READ : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TOKEN : begin
        token_data = {endpoint_ED_EN,endpoint_ED_FA};
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_DATA_TX : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_DATA_RX : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_DATA_RX_VALIDATE : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ACK_RX : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ACK_TX_0 : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ACK_TX_1 : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ACK_TX_EOP : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_DATA_RX_WAIT_DMA : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_UPDATE_TD_PROCESS : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_UPDATE_TD_CMD : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_UPDATE_ED_CMD : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_UPDATE_SYNC : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ABORD : begin
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    _zz_70 = 1'b0;
    if(((token_stateReg == `token_enumDefinition_binary_sequential_token_BOOT) && (! (token_stateNext == `token_enumDefinition_binary_sequential_token_BOOT))))begin
      _zz_70 = 1'b1;
    end
  end

  always @ (*) begin
    _zz_71 = 1'b0;
    case(token_stateReg)
      `token_enumDefinition_binary_sequential_token_INIT : begin
        _zz_71 = 1'b1;
      end
      `token_enumDefinition_binary_sequential_token_PID : begin
      end
      `token_enumDefinition_binary_sequential_token_B1 : begin
      end
      `token_enumDefinition_binary_sequential_token_B2 : begin
      end
      `token_enumDefinition_binary_sequential_token_EOP : begin
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    dataTx_wantExit = 1'b0;
    case(dataTx_stateReg)
      `dataTx_enumDefinition_binary_sequential_dataTx_PID : begin
      end
      `dataTx_enumDefinition_binary_sequential_dataTx_DATA : begin
      end
      `dataTx_enumDefinition_binary_sequential_dataTx_CRC_0 : begin
      end
      `dataTx_enumDefinition_binary_sequential_dataTx_CRC_1 : begin
      end
      `dataTx_enumDefinition_binary_sequential_dataTx_EOP : begin
        if(io_phy_txEop)begin
          dataTx_wantExit = 1'b1;
        end
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    dataTx_wantStart = 1'b0;
    if(((! (endpoint_stateReg == `endpoint_enumDefinition_binary_sequential_endpoint_DATA_TX)) && (endpoint_stateNext == `endpoint_enumDefinition_binary_sequential_endpoint_DATA_TX)))begin
      dataTx_wantStart = 1'b1;
    end
  end

  always @ (*) begin
    dataTx_wantKill = 1'b0;
    if((unscheduleAll_valid && unscheduleAll_ready))begin
      dataTx_wantKill = 1'b1;
    end
  end

  always @ (*) begin
    dataTx_pid = 4'bxxxx;
    case(endpoint_stateReg)
      `endpoint_enumDefinition_binary_sequential_endpoint_ED_READ_CMD : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ED_READ_RSP : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ED_ANALYSE : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TD_READ_CMD : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TD_READ_RSP : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TD_ANALYSE : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TD_CHECK_TIME : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_BUFFER_READ : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TOKEN : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_DATA_TX : begin
        dataTx_pid = {endpoint_dataPhase,3'b011};
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_DATA_RX : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_DATA_RX_VALIDATE : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ACK_RX : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ACK_TX_0 : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ACK_TX_1 : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ACK_TX_EOP : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_DATA_RX_WAIT_DMA : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_UPDATE_TD_PROCESS : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_UPDATE_TD_CMD : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_UPDATE_ED_CMD : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_UPDATE_SYNC : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ABORD : begin
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    dataTx_data_valid = 1'b0;
    case(endpoint_dmaLogic_stateReg)
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_INIT : begin
      end
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_TO_USB : begin
        dataTx_data_valid = 1'b1;
      end
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_FROM_USB : begin
      end
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_VALIDATION : begin
      end
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_CALC_CMD : begin
      end
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_READ_CMD : begin
      end
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_WRITE_CMD : begin
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    dataTx_data_payload_last = 1'bx;
    case(endpoint_dmaLogic_stateReg)
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_INIT : begin
      end
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_TO_USB : begin
        dataTx_data_payload_last = endpoint_dmaLogic_byteCtx_last;
      end
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_FROM_USB : begin
      end
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_VALIDATION : begin
      end
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_CALC_CMD : begin
      end
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_READ_CMD : begin
      end
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_WRITE_CMD : begin
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    dataTx_data_payload_fragment = 8'bxxxxxxxx;
    case(endpoint_dmaLogic_stateReg)
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_INIT : begin
      end
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_TO_USB : begin
        dataTx_data_payload_fragment = _zz_76;
      end
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_FROM_USB : begin
      end
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_VALIDATION : begin
      end
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_CALC_CMD : begin
      end
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_READ_CMD : begin
      end
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_WRITE_CMD : begin
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    dataTx_data_ready = 1'b0;
    case(dataTx_stateReg)
      `dataTx_enumDefinition_binary_sequential_dataTx_PID : begin
      end
      `dataTx_enumDefinition_binary_sequential_dataTx_DATA : begin
        if(io_phy_tx_ready)begin
          dataTx_data_ready = 1'b1;
        end
      end
      `dataTx_enumDefinition_binary_sequential_dataTx_CRC_0 : begin
      end
      `dataTx_enumDefinition_binary_sequential_dataTx_CRC_1 : begin
      end
      `dataTx_enumDefinition_binary_sequential_dataTx_EOP : begin
      end
      default : begin
      end
    endcase
  end

  assign _zz_73 = (dataTx_data_valid && dataTx_data_ready);
  always @ (*) begin
    _zz_72 = 1'b0;
    case(dataTx_stateReg)
      `dataTx_enumDefinition_binary_sequential_dataTx_PID : begin
        _zz_72 = 1'b1;
      end
      `dataTx_enumDefinition_binary_sequential_dataTx_DATA : begin
      end
      `dataTx_enumDefinition_binary_sequential_dataTx_CRC_0 : begin
      end
      `dataTx_enumDefinition_binary_sequential_dataTx_CRC_1 : begin
      end
      `dataTx_enumDefinition_binary_sequential_dataTx_EOP : begin
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    rxTimer_clear = 1'b0;
    if(io_phy_rx_active)begin
      rxTimer_clear = 1'b1;
    end
    if(_zz_104)begin
      rxTimer_clear = 1'b1;
    end
    if(_zz_105)begin
      rxTimer_clear = 1'b1;
    end
  end

  assign rxTimer_rxTimeout = (rxTimer_counter == (rxTimer_lowSpeed ? 8'hbf : 8'h17));
  assign rxTimer_ackTx = (rxTimer_counter == _zz_174);
  assign rxPidOk = (io_phy_rx_flow_payload_data[3 : 0] == (~ io_phy_rx_flow_payload_data[7 : 4]));
  always @ (*) begin
    dataRx_wantExit = 1'b0;
    case(dataRx_stateReg)
      `dataRx_enumDefinition_binary_sequential_dataRx_IDLE : begin
        if(! io_phy_rx_active) begin
          if(rxTimer_rxTimeout)begin
            dataRx_wantExit = 1'b1;
          end
        end
      end
      `dataRx_enumDefinition_binary_sequential_dataRx_PID : begin
        if(! io_phy_rx_flow_valid) begin
          if(_zz_106)begin
            dataRx_wantExit = 1'b1;
          end
        end
      end
      `dataRx_enumDefinition_binary_sequential_dataRx_DATA : begin
        if(_zz_107)begin
          dataRx_wantExit = 1'b1;
        end
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    dataRx_wantStart = 1'b0;
    if(_zz_108)begin
      dataRx_wantStart = 1'b1;
    end
  end

  always @ (*) begin
    dataRx_wantKill = 1'b0;
    if((unscheduleAll_valid && unscheduleAll_ready))begin
      dataRx_wantKill = 1'b1;
    end
  end

  assign dataRx_history_0 = _zz_56;
  assign dataRx_history_1 = _zz_57;
  always @ (*) begin
    dataRx_data_valid = 1'b0;
    case(dataRx_stateReg)
      `dataRx_enumDefinition_binary_sequential_dataRx_IDLE : begin
      end
      `dataRx_enumDefinition_binary_sequential_dataRx_PID : begin
      end
      `dataRx_enumDefinition_binary_sequential_dataRx_DATA : begin
        if(! _zz_107) begin
          if(io_phy_rx_flow_valid)begin
            if((dataRx_valids == 2'b11))begin
              dataRx_data_valid = 1'b1;
            end
          end
        end
      end
      default : begin
      end
    endcase
  end

  assign dataRx_data_payload = dataRx_history_1;
  always @ (*) begin
    _zz_75 = 1'b0;
    case(dataRx_stateReg)
      `dataRx_enumDefinition_binary_sequential_dataRx_IDLE : begin
      end
      `dataRx_enumDefinition_binary_sequential_dataRx_PID : begin
      end
      `dataRx_enumDefinition_binary_sequential_dataRx_DATA : begin
        if(! _zz_107) begin
          if(io_phy_rx_flow_valid)begin
            _zz_75 = 1'b1;
          end
        end
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    _zz_74 = 1'b0;
    case(dataRx_stateReg)
      `dataRx_enumDefinition_binary_sequential_dataRx_IDLE : begin
      end
      `dataRx_enumDefinition_binary_sequential_dataRx_PID : begin
        _zz_74 = 1'b1;
      end
      `dataRx_enumDefinition_binary_sequential_dataRx_DATA : begin
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    sof_wantExit = 1'b0;
    case(sof_stateReg)
      `sof_enumDefinition_binary_sequential_sof_FRAME_TX : begin
      end
      `sof_enumDefinition_binary_sequential_sof_FRAME_NUMBER_CMD : begin
      end
      `sof_enumDefinition_binary_sequential_sof_FRAME_NUMBER_RSP : begin
        if(ioDma_rsp_valid)begin
          sof_wantExit = 1'b1;
        end
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    sof_wantStart = 1'b0;
    if(((! (operational_stateReg == `operational_enumDefinition_binary_sequential_operational_SOF)) && (operational_stateNext == `operational_enumDefinition_binary_sequential_operational_SOF)))begin
      sof_wantStart = 1'b1;
    end
  end

  always @ (*) begin
    sof_wantKill = 1'b0;
    if((unscheduleAll_valid && unscheduleAll_ready))begin
      sof_wantKill = 1'b1;
    end
  end

  always @ (*) begin
    priority_tick = 1'b0;
    case(endpoint_stateReg)
      `endpoint_enumDefinition_binary_sequential_endpoint_ED_READ_CMD : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ED_READ_RSP : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ED_ANALYSE : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TD_READ_CMD : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TD_READ_RSP : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TD_ANALYSE : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TD_CHECK_TIME : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_BUFFER_READ : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TOKEN : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_DATA_TX : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_DATA_RX : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_DATA_RX_VALIDATE : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ACK_RX : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ACK_TX_0 : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ACK_TX_1 : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ACK_TX_EOP : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_DATA_RX_WAIT_DMA : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_UPDATE_TD_PROCESS : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_UPDATE_TD_CMD : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_UPDATE_ED_CMD : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_UPDATE_SYNC : begin
        if(dmaCtx_pendingEmpty)begin
          if((endpoint_flowType != `FlowType_binary_sequential_PERIODIC))begin
            priority_tick = 1'b1;
          end
        end
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ABORD : begin
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    priority_skip = 1'b0;
    if(priority_tick)begin
      if((priority_bulk || (priority_counter == reg_hcControl_CBSR)))begin
        priority_skip = 1'b1;
      end
    end
    case(operational_stateReg)
      `operational_enumDefinition_binary_sequential_operational_SOF : begin
      end
      `operational_enumDefinition_binary_sequential_operational_ARBITER : begin
        if(! operational_askExit) begin
          if(! frame_limitHit) begin
            if(! _zz_109) begin
              priority_skip = 1'b1;
              if(priority_bulk)begin
                if(operational_allowBulk)begin
                  if(reg_hcBulkCurrentED_isZero)begin
                    if(reg_hcCommandStatus_BLF)begin
                      priority_skip = 1'b0;
                    end
                  end else begin
                    priority_skip = 1'b0;
                  end
                end
              end else begin
                if(operational_allowControl)begin
                  if(reg_hcControlCurrentED_isZero)begin
                    if(reg_hcCommandStatus_CLF)begin
                      priority_skip = 1'b0;
                    end
                  end else begin
                    priority_skip = 1'b0;
                  end
                end
              end
            end
          end
        end
      end
      `operational_enumDefinition_binary_sequential_operational_END_POINT : begin
      end
      `operational_enumDefinition_binary_sequential_operational_PERIODIC_HEAD_CMD : begin
      end
      `operational_enumDefinition_binary_sequential_operational_PERIODIC_HEAD_RSP : begin
      end
      `operational_enumDefinition_binary_sequential_operational_WAIT_SOF : begin
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    interruptDelay_tick = 1'b0;
    case(sof_stateReg)
      `sof_enumDefinition_binary_sequential_sof_FRAME_TX : begin
      end
      `sof_enumDefinition_binary_sequential_sof_FRAME_NUMBER_CMD : begin
      end
      `sof_enumDefinition_binary_sequential_sof_FRAME_NUMBER_RSP : begin
        if(ioDma_rsp_valid)begin
          interruptDelay_tick = 1'b1;
        end
      end
      default : begin
      end
    endcase
  end

  assign interruptDelay_done = (interruptDelay_counter == 3'b000);
  assign interruptDelay_disabled = (interruptDelay_counter == 3'b111);
  always @ (*) begin
    interruptDelay_disable = 1'b0;
    case(sof_stateReg)
      `sof_enumDefinition_binary_sequential_sof_FRAME_TX : begin
      end
      `sof_enumDefinition_binary_sequential_sof_FRAME_NUMBER_CMD : begin
      end
      `sof_enumDefinition_binary_sequential_sof_FRAME_NUMBER_RSP : begin
        if(ioDma_rsp_valid)begin
          if(sof_doInterruptDelay)begin
            interruptDelay_disable = 1'b1;
          end
        end
      end
      default : begin
      end
    endcase
    if(_zz_110)begin
      interruptDelay_disable = 1'b1;
    end
  end

  always @ (*) begin
    interruptDelay_load_valid = 1'b0;
    case(endpoint_stateReg)
      `endpoint_enumDefinition_binary_sequential_endpoint_ED_READ_CMD : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ED_READ_RSP : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ED_ANALYSE : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TD_READ_CMD : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TD_READ_RSP : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TD_ANALYSE : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TD_CHECK_TIME : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_BUFFER_READ : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TOKEN : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_DATA_TX : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_DATA_RX : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_DATA_RX_VALIDATE : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ACK_RX : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ACK_TX_0 : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ACK_TX_1 : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ACK_TX_EOP : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_DATA_RX_WAIT_DMA : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_UPDATE_TD_PROCESS : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_UPDATE_TD_CMD : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_UPDATE_ED_CMD : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_UPDATE_SYNC : begin
        if(dmaCtx_pendingEmpty)begin
          if(endpoint_TD_retire)begin
            interruptDelay_load_valid = 1'b1;
          end
        end
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ABORD : begin
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    interruptDelay_load_payload = 3'bxxx;
    case(endpoint_stateReg)
      `endpoint_enumDefinition_binary_sequential_endpoint_ED_READ_CMD : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ED_READ_RSP : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ED_ANALYSE : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TD_READ_CMD : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TD_READ_RSP : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TD_ANALYSE : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TD_CHECK_TIME : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_BUFFER_READ : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TOKEN : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_DATA_TX : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_DATA_RX : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_DATA_RX_VALIDATE : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ACK_RX : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ACK_TX_0 : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ACK_TX_1 : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ACK_TX_EOP : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_DATA_RX_WAIT_DMA : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_UPDATE_TD_PROCESS : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_UPDATE_TD_CMD : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_UPDATE_ED_CMD : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_UPDATE_SYNC : begin
        if(dmaCtx_pendingEmpty)begin
          if(endpoint_TD_retire)begin
            interruptDelay_load_payload = endpoint_TD_DI;
          end
        end
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ABORD : begin
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    endpoint_wantExit = 1'b0;
    case(endpoint_stateReg)
      `endpoint_enumDefinition_binary_sequential_endpoint_ED_READ_CMD : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ED_READ_RSP : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ED_ANALYSE : begin
        if(_zz_111)begin
          endpoint_wantExit = 1'b1;
        end
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TD_READ_CMD : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TD_READ_RSP : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TD_ANALYSE : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TD_CHECK_TIME : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_BUFFER_READ : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TOKEN : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_DATA_TX : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_DATA_RX : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_DATA_RX_VALIDATE : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ACK_RX : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ACK_TX_0 : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ACK_TX_1 : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ACK_TX_EOP : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_DATA_RX_WAIT_DMA : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_UPDATE_TD_PROCESS : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_UPDATE_TD_CMD : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_UPDATE_ED_CMD : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_UPDATE_SYNC : begin
        if(dmaCtx_pendingEmpty)begin
          endpoint_wantExit = 1'b1;
        end
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ABORD : begin
        endpoint_wantExit = 1'b1;
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    endpoint_wantStart = 1'b0;
    case(operational_stateReg)
      `operational_enumDefinition_binary_sequential_operational_SOF : begin
      end
      `operational_enumDefinition_binary_sequential_operational_ARBITER : begin
        if(! operational_askExit) begin
          if(! frame_limitHit) begin
            if(_zz_109)begin
              if(! _zz_112) begin
                if(! reg_hcPeriodCurrentED_isZero) begin
                  endpoint_wantStart = 1'b1;
                end
              end
            end else begin
              if(priority_bulk)begin
                if(operational_allowBulk)begin
                  if(! reg_hcBulkCurrentED_isZero) begin
                    endpoint_wantStart = 1'b1;
                  end
                end
              end else begin
                if(operational_allowControl)begin
                  if(! reg_hcControlCurrentED_isZero) begin
                    endpoint_wantStart = 1'b1;
                  end
                end
              end
            end
          end
        end
      end
      `operational_enumDefinition_binary_sequential_operational_END_POINT : begin
      end
      `operational_enumDefinition_binary_sequential_operational_PERIODIC_HEAD_CMD : begin
      end
      `operational_enumDefinition_binary_sequential_operational_PERIODIC_HEAD_RSP : begin
      end
      `operational_enumDefinition_binary_sequential_operational_WAIT_SOF : begin
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    endpoint_wantKill = 1'b0;
    if((unscheduleAll_valid && unscheduleAll_ready))begin
      endpoint_wantKill = 1'b1;
    end
  end

  assign endpoint_ED_FA = endpoint_ED_words_0[6 : 0];
  assign endpoint_ED_EN = endpoint_ED_words_0[10 : 7];
  assign endpoint_ED_D = endpoint_ED_words_0[12 : 11];
  assign endpoint_ED_S = endpoint_ED_words_0[13];
  assign endpoint_ED_K = endpoint_ED_words_0[14];
  assign endpoint_ED_F = endpoint_ED_words_0[15];
  assign endpoint_ED_MPS = endpoint_ED_words_0[26 : 16];
  assign endpoint_ED_tailP = endpoint_ED_words_1[31 : 4];
  assign endpoint_ED_H = endpoint_ED_words_2[0];
  assign endpoint_ED_C = endpoint_ED_words_2[1];
  assign endpoint_ED_headP = endpoint_ED_words_2[31 : 4];
  assign endpoint_ED_nextED = endpoint_ED_words_3[31 : 4];
  assign endpoint_ED_tdEmpty = (endpoint_ED_tailP == endpoint_ED_headP);
  assign endpoint_ED_isFs = (! endpoint_ED_S);
  assign endpoint_ED_isoOut = endpoint_ED_D[0];
  assign rxTimer_lowSpeed = endpoint_ED_S;
  assign endpoint_TD_address = ({4'd0,endpoint_ED_headP} <<< 4);
  assign endpoint_TD_CC = endpoint_TD_words_0[31 : 28];
  assign endpoint_TD_EC = endpoint_TD_words_0[27 : 26];
  assign endpoint_TD_T = endpoint_TD_words_0[25 : 24];
  assign endpoint_TD_DI = endpoint_TD_words_0[23 : 21];
  assign endpoint_TD_DP = endpoint_TD_words_0[20 : 19];
  assign endpoint_TD_R = endpoint_TD_words_0[18];
  assign endpoint_TD_CBP = endpoint_TD_words_1[31 : 0];
  assign endpoint_TD_nextTD = endpoint_TD_words_2[31 : 4];
  assign endpoint_TD_BE = endpoint_TD_words_3[31 : 0];
  assign endpoint_TD_FC = endpoint_TD_words_0[26 : 24];
  assign endpoint_TD_SF = endpoint_TD_words_0[15 : 0];
  assign endpoint_TD_isoRelativeFrameNumber = (reg_hcFmNumber_FN - endpoint_TD_SF);
  assign endpoint_TD_tooEarly = endpoint_TD_isoRelativeFrameNumber[15];
  assign endpoint_TD_isoFrameNumber = endpoint_TD_isoRelativeFrameNumber[2 : 0];
  assign endpoint_TD_isoOverrun = ((! endpoint_TD_tooEarly) && (_zz_175 < endpoint_TD_isoRelativeFrameNumber));
  assign endpoint_TD_isoLast = (((! endpoint_TD_isoOverrun) && (! endpoint_TD_tooEarly)) && (endpoint_TD_isoFrameNumber == endpoint_TD_FC));
  assign endpoint_TD_isoZero = (endpoint_TD_isoLast ? (endpoint_TD_isoBaseNext < endpoint_TD_isoBase) : (endpoint_TD_isoBase == endpoint_TD_isoBaseNext));
  assign endpoint_TD_isSinglePage = (endpoint_TD_CBP[31 : 12] == endpoint_TD_BE[31 : 12]);
  assign endpoint_TD_firstOffset = (endpoint_ED_F ? endpoint_TD_isoBase : _zz_177);
  assign endpoint_TD_lastOffset = (endpoint_ED_F ? _zz_178 : {(! endpoint_TD_isSinglePage),endpoint_TD_BE[11 : 0]});
  assign endpoint_TD_allowRounding = ((! endpoint_ED_F) && endpoint_TD_R);
  assign endpoint_TD_TNext = (endpoint_TD_dataPhaseUpdate ? {1'b1,(! endpoint_dataPhase)} : endpoint_TD_T);
  assign endpoint_TD_dataPhaseNext = (endpoint_dataPhase ^ endpoint_TD_dataPhaseUpdate);
  assign endpoint_TD_dataPid = (endpoint_dataPhase ? 4'b1011 : 4'b0011);
  assign endpoint_TD_dataPidWrong = (endpoint_dataPhase ? 4'b0011 : 4'b1011);
  always @ (*) begin
    endpoint_TD_clear = 1'b0;
    case(endpoint_stateReg)
      `endpoint_enumDefinition_binary_sequential_endpoint_ED_READ_CMD : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ED_READ_RSP : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ED_ANALYSE : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TD_READ_CMD : begin
        endpoint_TD_clear = 1'b1;
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TD_READ_RSP : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TD_ANALYSE : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TD_CHECK_TIME : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_BUFFER_READ : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TOKEN : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_DATA_TX : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_DATA_RX : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_DATA_RX_VALIDATE : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ACK_RX : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ACK_TX_0 : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ACK_TX_1 : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ACK_TX_EOP : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_DATA_RX_WAIT_DMA : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_UPDATE_TD_PROCESS : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_UPDATE_TD_CMD : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_UPDATE_ED_CMD : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_UPDATE_SYNC : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ABORD : begin
      end
      default : begin
      end
    endcase
  end

  assign endpoint_tockenType = ((endpoint_ED_D[0] != endpoint_ED_D[1]) ? endpoint_ED_D : endpoint_TD_DP);
  assign endpoint_isIn = (endpoint_tockenType == 2'b10);
  always @ (*) begin
    endpoint_applyNextED = 1'b0;
    case(endpoint_stateReg)
      `endpoint_enumDefinition_binary_sequential_endpoint_ED_READ_CMD : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ED_READ_RSP : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ED_ANALYSE : begin
        if(_zz_111)begin
          endpoint_applyNextED = 1'b1;
        end
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TD_READ_CMD : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TD_READ_RSP : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TD_ANALYSE : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TD_CHECK_TIME : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_BUFFER_READ : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TOKEN : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_DATA_TX : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_DATA_RX : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_DATA_RX_VALIDATE : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ACK_RX : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ACK_TX_0 : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ACK_TX_1 : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ACK_TX_EOP : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_DATA_RX_WAIT_DMA : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_UPDATE_TD_PROCESS : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_UPDATE_TD_CMD : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_UPDATE_ED_CMD : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_UPDATE_SYNC : begin
        if(dmaCtx_pendingEmpty)begin
          if((! (endpoint_ED_F && endpoint_TD_isoOverrun)))begin
            endpoint_applyNextED = 1'b1;
          end
        end
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ABORD : begin
      end
      default : begin
      end
    endcase
  end

  assign endpoint_currentAddressFull = {(endpoint_currentAddress[12] ? endpoint_TD_BE[31 : 12] : endpoint_TD_CBP[31 : 12]),endpoint_currentAddress[11 : 0]};
  assign _zz_235 = zz__zz_58(1'b0);
  always @ (*) _zz_58 = _zz_235;
  assign endpoint_currentAddressBmb = (endpoint_currentAddressFull & _zz_58);
  assign endpoint_transactionSizeMinusOne = (_zz_181 - endpoint_currentAddress);
  assign endpoint_transactionSize = (endpoint_transactionSizeMinusOne + 14'h0001);
  assign endpoint_dataDone = (endpoint_zeroLength || (_zz_182 < endpoint_currentAddress));
  always @ (*) begin
    endpoint_dmaLogic_wantExit = 1'b0;
    case(endpoint_dmaLogic_stateReg)
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_INIT : begin
      end
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_TO_USB : begin
        if(dataTx_data_ready)begin
          if(endpoint_dmaLogic_byteCtx_last)begin
            endpoint_dmaLogic_wantExit = 1'b1;
          end
        end
      end
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_FROM_USB : begin
      end
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_VALIDATION : begin
        if(_zz_113)begin
          endpoint_dmaLogic_wantExit = 1'b1;
        end
      end
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_CALC_CMD : begin
        if(endpoint_dataDone)begin
          if(endpoint_isIn)begin
            endpoint_dmaLogic_wantExit = 1'b1;
          end
        end
      end
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_READ_CMD : begin
      end
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_WRITE_CMD : begin
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    endpoint_dmaLogic_wantStart = 1'b0;
    case(endpoint_stateReg)
      `endpoint_enumDefinition_binary_sequential_endpoint_ED_READ_CMD : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ED_READ_RSP : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ED_ANALYSE : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TD_READ_CMD : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TD_READ_RSP : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TD_ANALYSE : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TD_CHECK_TIME : begin
        if(! endpoint_timeCheck) begin
          if(! _zz_114) begin
            endpoint_dmaLogic_wantStart = 1'b1;
          end
        end
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_BUFFER_READ : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TOKEN : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_DATA_TX : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_DATA_RX : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_DATA_RX_VALIDATE : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ACK_RX : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ACK_TX_0 : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ACK_TX_1 : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ACK_TX_EOP : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_DATA_RX_WAIT_DMA : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_UPDATE_TD_PROCESS : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_UPDATE_TD_CMD : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_UPDATE_ED_CMD : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_UPDATE_SYNC : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ABORD : begin
      end
      default : begin
      end
    endcase
    if(_zz_108)begin
      endpoint_dmaLogic_wantStart = 1'b1;
    end
  end

  always @ (*) begin
    endpoint_dmaLogic_wantKill = 1'b0;
    case(endpoint_stateReg)
      `endpoint_enumDefinition_binary_sequential_endpoint_ED_READ_CMD : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ED_READ_RSP : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ED_ANALYSE : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TD_READ_CMD : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TD_READ_RSP : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TD_ANALYSE : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TD_CHECK_TIME : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_BUFFER_READ : begin
        if(_zz_115)begin
          if(endpoint_timeCheck)begin
            endpoint_dmaLogic_wantKill = 1'b1;
          end
        end
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TOKEN : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_DATA_TX : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_DATA_RX : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_DATA_RX_VALIDATE : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ACK_RX : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ACK_TX_0 : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ACK_TX_1 : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ACK_TX_EOP : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_DATA_RX_WAIT_DMA : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_UPDATE_TD_PROCESS : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_UPDATE_TD_CMD : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_UPDATE_ED_CMD : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_UPDATE_SYNC : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ABORD : begin
      end
      default : begin
      end
    endcase
    if((unscheduleAll_valid && unscheduleAll_ready))begin
      endpoint_dmaLogic_wantKill = 1'b1;
    end
  end

  always @ (*) begin
    endpoint_dmaLogic_validated = 1'b0;
    case(endpoint_stateReg)
      `endpoint_enumDefinition_binary_sequential_endpoint_ED_READ_CMD : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ED_READ_RSP : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ED_ANALYSE : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TD_READ_CMD : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TD_READ_RSP : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TD_ANALYSE : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TD_CHECK_TIME : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_BUFFER_READ : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TOKEN : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_DATA_TX : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_DATA_RX : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_DATA_RX_VALIDATE : begin
        endpoint_dmaLogic_validated = 1'b1;
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ACK_RX : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ACK_TX_0 : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ACK_TX_1 : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ACK_TX_EOP : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_DATA_RX_WAIT_DMA : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_UPDATE_TD_PROCESS : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_UPDATE_TD_CMD : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_UPDATE_ED_CMD : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_UPDATE_SYNC : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ABORD : begin
      end
      default : begin
      end
    endcase
  end

  assign endpoint_dmaLogic_lengthMax = (~ _zz_183);
  assign endpoint_dmaLogic_lengthCalc = _zz_184[5:0];
  assign endpoint_dmaLogic_beatCount = _zz_187[6 : 2];
  assign endpoint_dmaLogic_lengthBmb = _zz_190[5:0];
  assign endpoint_dmaLogic_underflowError = (endpoint_dmaLogic_underflow && (! endpoint_TD_allowRounding));
  assign endpoint_dmaLogic_byteCtx_last = (endpoint_dmaLogic_byteCtx_counter == endpoint_lastAddress);
  assign endpoint_dmaLogic_byteCtx_sel = endpoint_dmaLogic_byteCtx_counter[1:0];
  always @ (*) begin
    endpoint_dmaLogic_byteCtx_increment = 1'b0;
    case(endpoint_dmaLogic_stateReg)
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_INIT : begin
      end
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_TO_USB : begin
        if(dataTx_data_ready)begin
          endpoint_dmaLogic_byteCtx_increment = 1'b1;
        end
      end
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_FROM_USB : begin
        if(dataRx_data_valid)begin
          endpoint_dmaLogic_byteCtx_increment = 1'b1;
        end
      end
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_VALIDATION : begin
      end
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_CALC_CMD : begin
      end
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_READ_CMD : begin
      end
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_WRITE_CMD : begin
      end
      default : begin
      end
    endcase
  end

  assign endpoint_dmaLogic_headMask = {(endpoint_currentAddress[1 : 0] <= 2'b11),{(endpoint_currentAddress[1 : 0] <= 2'b10),{(endpoint_currentAddress[1 : 0] <= 2'b01),(endpoint_currentAddress[1 : 0] <= 2'b00)}}};
  assign endpoint_dmaLogic_lastMask = {(2'b11 <= _zz_191[1 : 0]),{(2'b10 <= _zz_193[1 : 0]),{(2'b01 <= _zz_195[1 : 0]),(2'b00 <= _zz_197[1 : 0])}}};
  assign endpoint_dmaLogic_fullMask = 4'b1111;
  assign endpoint_dmaLogic_beatLast = (dmaCtx_beatCounter == _zz_199);
  assign endpoint_byteCountCalc = (_zz_200 + 14'h0001);
  assign endpoint_fsTimeCheck = (endpoint_zeroLength ? (frame_limitCounter == 15'h0) : (_zz_202 <= _zz_203));
  assign endpoint_timeCheck = ((endpoint_ED_isFs && endpoint_fsTimeCheck) || (endpoint_ED_S && reg_hcLSThreshold_hit));
  assign endpoint_tdUpdateAddress = ((endpoint_TD_retire && (! ((endpoint_isIn && ((endpoint_TD_CC == 4'b0000) || (endpoint_TD_CC == 4'b1001))) && endpoint_dmaLogic_underflow))) ? 32'h0 : endpoint_currentAddressFull);
  always @ (*) begin
    operational_wantExit = 1'b0;
    case(operational_stateReg)
      `operational_enumDefinition_binary_sequential_operational_SOF : begin
      end
      `operational_enumDefinition_binary_sequential_operational_ARBITER : begin
        if(operational_askExit)begin
          operational_wantExit = 1'b1;
        end
      end
      `operational_enumDefinition_binary_sequential_operational_END_POINT : begin
      end
      `operational_enumDefinition_binary_sequential_operational_PERIODIC_HEAD_CMD : begin
      end
      `operational_enumDefinition_binary_sequential_operational_PERIODIC_HEAD_RSP : begin
      end
      `operational_enumDefinition_binary_sequential_operational_WAIT_SOF : begin
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    operational_wantStart = 1'b0;
    if(_zz_103)begin
      operational_wantStart = 1'b1;
    end
  end

  always @ (*) begin
    operational_wantKill = 1'b0;
    if((unscheduleAll_valid && unscheduleAll_ready))begin
      operational_wantKill = 1'b1;
    end
  end

  always @ (*) begin
    operational_askExit = 1'b0;
    case(hc_stateReg)
      `hc_enumDefinition_binary_sequential_hc_RESET : begin
      end
      `hc_enumDefinition_binary_sequential_hc_RESUME : begin
      end
      `hc_enumDefinition_binary_sequential_hc_OPERATIONAL : begin
      end
      `hc_enumDefinition_binary_sequential_hc_SUSPEND : begin
      end
      `hc_enumDefinition_binary_sequential_hc_ANY_TO_RESET : begin
      end
      `hc_enumDefinition_binary_sequential_hc_ANY_TO_SUSPEND : begin
        operational_askExit = 1'b1;
      end
      default : begin
      end
    endcase
  end

  assign hc_wantExit = 1'b0;
  always @ (*) begin
    hc_wantStart = 1'b0;
    case(hc_stateReg)
      `hc_enumDefinition_binary_sequential_hc_RESET : begin
      end
      `hc_enumDefinition_binary_sequential_hc_RESUME : begin
      end
      `hc_enumDefinition_binary_sequential_hc_OPERATIONAL : begin
      end
      `hc_enumDefinition_binary_sequential_hc_SUSPEND : begin
      end
      `hc_enumDefinition_binary_sequential_hc_ANY_TO_RESET : begin
      end
      `hc_enumDefinition_binary_sequential_hc_ANY_TO_SUSPEND : begin
      end
      default : begin
        hc_wantStart = 1'b1;
      end
    endcase
  end

  assign hc_wantKill = 1'b0;
  always @ (*) begin
    reg_hcControl_HCFS = `MainState_binary_sequential_RESET;
    case(hc_stateReg)
      `hc_enumDefinition_binary_sequential_hc_RESET : begin
      end
      `hc_enumDefinition_binary_sequential_hc_RESUME : begin
        reg_hcControl_HCFS = `MainState_binary_sequential_RESUME;
      end
      `hc_enumDefinition_binary_sequential_hc_OPERATIONAL : begin
        reg_hcControl_HCFS = `MainState_binary_sequential_OPERATIONAL;
      end
      `hc_enumDefinition_binary_sequential_hc_SUSPEND : begin
        reg_hcControl_HCFS = `MainState_binary_sequential_SUSPEND;
      end
      `hc_enumDefinition_binary_sequential_hc_ANY_TO_RESET : begin
        reg_hcControl_HCFS = `MainState_binary_sequential_RESET;
      end
      `hc_enumDefinition_binary_sequential_hc_ANY_TO_SUSPEND : begin
        reg_hcControl_HCFS = `MainState_binary_sequential_SUSPEND;
      end
      default : begin
      end
    endcase
  end

  assign io_phy_usbReset = (reg_hcControl_HCFS == `MainState_binary_sequential_RESET);
  assign io_phy_usbResume = (reg_hcControl_HCFS == `MainState_binary_sequential_RESUME);
  always @ (*) begin
    hc_error = 1'b0;
    case(hc_stateReg)
      `hc_enumDefinition_binary_sequential_hc_RESET : begin
        if(reg_hcControl_HCFSWrite_valid)begin
          case(reg_hcControl_HCFSWrite_payload)
            `MainState_binary_sequential_OPERATIONAL : begin
            end
            default : begin
              hc_error = 1'b1;
            end
          endcase
        end
      end
      `hc_enumDefinition_binary_sequential_hc_RESUME : begin
      end
      `hc_enumDefinition_binary_sequential_hc_OPERATIONAL : begin
      end
      `hc_enumDefinition_binary_sequential_hc_SUSPEND : begin
      end
      `hc_enumDefinition_binary_sequential_hc_ANY_TO_RESET : begin
      end
      `hc_enumDefinition_binary_sequential_hc_ANY_TO_SUSPEND : begin
      end
      default : begin
      end
    endcase
  end

  assign _zz_59 = io_ctrl_cmd_payload_fragment_data[7 : 6];
  assign reg_hcControl_HCFSWrite_payload = _zz_59;
  always @ (*) begin
    token_stateNext = token_stateReg;
    case(token_stateReg)
      `token_enumDefinition_binary_sequential_token_INIT : begin
        token_stateNext = `token_enumDefinition_binary_sequential_token_PID;
      end
      `token_enumDefinition_binary_sequential_token_PID : begin
        if(io_phy_tx_ready)begin
          token_stateNext = `token_enumDefinition_binary_sequential_token_B1;
        end
      end
      `token_enumDefinition_binary_sequential_token_B1 : begin
        if(io_phy_tx_ready)begin
          token_stateNext = `token_enumDefinition_binary_sequential_token_B2;
        end
      end
      `token_enumDefinition_binary_sequential_token_B2 : begin
        if(io_phy_tx_ready)begin
          token_stateNext = `token_enumDefinition_binary_sequential_token_EOP;
        end
      end
      `token_enumDefinition_binary_sequential_token_EOP : begin
        if(io_phy_txEop)begin
          token_stateNext = `token_enumDefinition_binary_sequential_token_BOOT;
        end
      end
      default : begin
      end
    endcase
    if(token_wantStart)begin
      token_stateNext = `token_enumDefinition_binary_sequential_token_INIT;
    end
    if(token_wantKill)begin
      token_stateNext = `token_enumDefinition_binary_sequential_token_BOOT;
    end
  end

  always @ (*) begin
    dataTx_stateNext = dataTx_stateReg;
    case(dataTx_stateReg)
      `dataTx_enumDefinition_binary_sequential_dataTx_PID : begin
        if(io_phy_tx_ready)begin
          if(dataTx_data_valid)begin
            dataTx_stateNext = `dataTx_enumDefinition_binary_sequential_dataTx_DATA;
          end else begin
            dataTx_stateNext = `dataTx_enumDefinition_binary_sequential_dataTx_CRC_0;
          end
        end
      end
      `dataTx_enumDefinition_binary_sequential_dataTx_DATA : begin
        if(io_phy_tx_ready)begin
          if(dataTx_data_payload_last)begin
            dataTx_stateNext = `dataTx_enumDefinition_binary_sequential_dataTx_CRC_0;
          end
        end
      end
      `dataTx_enumDefinition_binary_sequential_dataTx_CRC_0 : begin
        if(io_phy_tx_ready)begin
          dataTx_stateNext = `dataTx_enumDefinition_binary_sequential_dataTx_CRC_1;
        end
      end
      `dataTx_enumDefinition_binary_sequential_dataTx_CRC_1 : begin
        if(io_phy_tx_ready)begin
          dataTx_stateNext = `dataTx_enumDefinition_binary_sequential_dataTx_EOP;
        end
      end
      `dataTx_enumDefinition_binary_sequential_dataTx_EOP : begin
        if(io_phy_txEop)begin
          dataTx_stateNext = `dataTx_enumDefinition_binary_sequential_dataTx_BOOT;
        end
      end
      default : begin
      end
    endcase
    if(dataTx_wantStart)begin
      dataTx_stateNext = `dataTx_enumDefinition_binary_sequential_dataTx_PID;
    end
    if(dataTx_wantKill)begin
      dataTx_stateNext = `dataTx_enumDefinition_binary_sequential_dataTx_BOOT;
    end
  end

  always @ (*) begin
    dataRx_stateNext = dataRx_stateReg;
    case(dataRx_stateReg)
      `dataRx_enumDefinition_binary_sequential_dataRx_IDLE : begin
        if(io_phy_rx_active)begin
          dataRx_stateNext = `dataRx_enumDefinition_binary_sequential_dataRx_PID;
        end else begin
          if(rxTimer_rxTimeout)begin
            dataRx_stateNext = `dataRx_enumDefinition_binary_sequential_dataRx_BOOT;
          end
        end
      end
      `dataRx_enumDefinition_binary_sequential_dataRx_PID : begin
        if(io_phy_rx_flow_valid)begin
          dataRx_stateNext = `dataRx_enumDefinition_binary_sequential_dataRx_DATA;
        end else begin
          if(_zz_106)begin
            dataRx_stateNext = `dataRx_enumDefinition_binary_sequential_dataRx_BOOT;
          end
        end
      end
      `dataRx_enumDefinition_binary_sequential_dataRx_DATA : begin
        if(_zz_107)begin
          dataRx_stateNext = `dataRx_enumDefinition_binary_sequential_dataRx_BOOT;
        end
      end
      default : begin
      end
    endcase
    if(dataRx_wantStart)begin
      dataRx_stateNext = `dataRx_enumDefinition_binary_sequential_dataRx_IDLE;
    end
    if(dataRx_wantKill)begin
      dataRx_stateNext = `dataRx_enumDefinition_binary_sequential_dataRx_BOOT;
    end
  end

  always @ (*) begin
    sof_stateNext = sof_stateReg;
    case(sof_stateReg)
      `sof_enumDefinition_binary_sequential_sof_FRAME_TX : begin
        if(token_wantExit)begin
          sof_stateNext = `sof_enumDefinition_binary_sequential_sof_FRAME_NUMBER_CMD;
        end
      end
      `sof_enumDefinition_binary_sequential_sof_FRAME_NUMBER_CMD : begin
        if((ioDma_cmd_ready && ioDma_cmd_payload_last))begin
          sof_stateNext = `sof_enumDefinition_binary_sequential_sof_FRAME_NUMBER_RSP;
        end
      end
      `sof_enumDefinition_binary_sequential_sof_FRAME_NUMBER_RSP : begin
        if(ioDma_rsp_valid)begin
          sof_stateNext = `sof_enumDefinition_binary_sequential_sof_BOOT;
        end
      end
      default : begin
      end
    endcase
    if(sof_wantStart)begin
      sof_stateNext = `sof_enumDefinition_binary_sequential_sof_FRAME_TX;
    end
    if(sof_wantKill)begin
      sof_stateNext = `sof_enumDefinition_binary_sequential_sof_BOOT;
    end
  end

  always @ (*) begin
    endpoint_stateNext = endpoint_stateReg;
    case(endpoint_stateReg)
      `endpoint_enumDefinition_binary_sequential_endpoint_ED_READ_CMD : begin
        if(ioDma_cmd_ready)begin
          endpoint_stateNext = `endpoint_enumDefinition_binary_sequential_endpoint_ED_READ_RSP;
        end
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ED_READ_RSP : begin
        if((ioDma_rsp_valid && ioDma_rsp_payload_last))begin
          endpoint_stateNext = `endpoint_enumDefinition_binary_sequential_endpoint_ED_ANALYSE;
        end
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ED_ANALYSE : begin
        if(_zz_111)begin
          endpoint_stateNext = `endpoint_enumDefinition_binary_sequential_endpoint_BOOT;
        end else begin
          endpoint_stateNext = `endpoint_enumDefinition_binary_sequential_endpoint_TD_READ_CMD;
        end
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TD_READ_CMD : begin
        if(ioDma_cmd_ready)begin
          endpoint_stateNext = `endpoint_enumDefinition_binary_sequential_endpoint_TD_READ_RSP;
        end
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TD_READ_RSP : begin
        if(((ioDma_rsp_valid && ioDma_rsp_ready) && ioDma_rsp_payload_last))begin
          endpoint_stateNext = `endpoint_enumDefinition_binary_sequential_endpoint_TD_ANALYSE;
        end
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TD_ANALYSE : begin
        endpoint_stateNext = `endpoint_enumDefinition_binary_sequential_endpoint_TD_CHECK_TIME;
        if(endpoint_ED_F)begin
          if(endpoint_TD_tooEarly)begin
            endpoint_stateNext = `endpoint_enumDefinition_binary_sequential_endpoint_UPDATE_SYNC;
          end
          if(endpoint_TD_isoOverrun)begin
            endpoint_stateNext = `endpoint_enumDefinition_binary_sequential_endpoint_UPDATE_TD_CMD;
          end
        end
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TD_CHECK_TIME : begin
        if(endpoint_timeCheck)begin
          endpoint_stateNext = `endpoint_enumDefinition_binary_sequential_endpoint_ABORD;
        end else begin
          if(_zz_114)begin
            endpoint_stateNext = `endpoint_enumDefinition_binary_sequential_endpoint_TOKEN;
          end else begin
            endpoint_stateNext = `endpoint_enumDefinition_binary_sequential_endpoint_BUFFER_READ;
          end
        end
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_BUFFER_READ : begin
        if(_zz_115)begin
          endpoint_stateNext = `endpoint_enumDefinition_binary_sequential_endpoint_TOKEN;
          if(endpoint_timeCheck)begin
            endpoint_stateNext = `endpoint_enumDefinition_binary_sequential_endpoint_ABORD;
          end
        end
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TOKEN : begin
        if(token_wantExit)begin
          if(endpoint_isIn)begin
            endpoint_stateNext = `endpoint_enumDefinition_binary_sequential_endpoint_DATA_RX;
          end else begin
            endpoint_stateNext = `endpoint_enumDefinition_binary_sequential_endpoint_DATA_TX;
          end
        end
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_DATA_TX : begin
        if(dataTx_wantExit)begin
          if(endpoint_ED_F)begin
            endpoint_stateNext = `endpoint_enumDefinition_binary_sequential_endpoint_UPDATE_TD_PROCESS;
          end else begin
            endpoint_stateNext = `endpoint_enumDefinition_binary_sequential_endpoint_ACK_RX;
          end
        end
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_DATA_RX : begin
        if(dataRx_wantExit)begin
          endpoint_stateNext = `endpoint_enumDefinition_binary_sequential_endpoint_DATA_RX_VALIDATE;
        end
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_DATA_RX_VALIDATE : begin
        endpoint_stateNext = `endpoint_enumDefinition_binary_sequential_endpoint_DATA_RX_WAIT_DMA;
        if(! dataRx_notResponding) begin
          if(! dataRx_stuffingError) begin
            if(! dataRx_pidError) begin
              if(! endpoint_ED_F) begin
                case(dataRx_pid)
                  4'b1010 : begin
                  end
                  4'b1110 : begin
                  end
                  4'b0011, 4'b1011 : begin
                    if(_zz_116)begin
                      endpoint_stateNext = `endpoint_enumDefinition_binary_sequential_endpoint_ACK_TX_0;
                    end
                  end
                  default : begin
                  end
                endcase
              end
              if(_zz_62)begin
                if(! dataRx_crcError) begin
                  if((! endpoint_ED_F))begin
                    endpoint_stateNext = `endpoint_enumDefinition_binary_sequential_endpoint_ACK_TX_0;
                  end
                end
              end
            end
          end
        end
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ACK_RX : begin
        if(_zz_117)begin
          endpoint_stateNext = `endpoint_enumDefinition_binary_sequential_endpoint_UPDATE_TD_PROCESS;
          if(! _zz_118) begin
            if(! endpoint_ackRxStuffing) begin
              if(! endpoint_ackRxPidFailure) begin
                case(endpoint_ackRxPid)
                  4'b0010 : begin
                  end
                  4'b1010 : begin
                    endpoint_stateNext = `endpoint_enumDefinition_binary_sequential_endpoint_UPDATE_SYNC;
                  end
                  4'b1110 : begin
                  end
                  default : begin
                  end
                endcase
              end
            end
          end
        end
        if(rxTimer_rxTimeout)begin
          endpoint_stateNext = `endpoint_enumDefinition_binary_sequential_endpoint_UPDATE_TD_PROCESS;
        end
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ACK_TX_0 : begin
        if(rxTimer_ackTx)begin
          endpoint_stateNext = `endpoint_enumDefinition_binary_sequential_endpoint_ACK_TX_1;
        end
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ACK_TX_1 : begin
        if(io_phy_tx_ready)begin
          endpoint_stateNext = `endpoint_enumDefinition_binary_sequential_endpoint_ACK_TX_EOP;
        end
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ACK_TX_EOP : begin
        if(io_phy_txEop)begin
          endpoint_stateNext = `endpoint_enumDefinition_binary_sequential_endpoint_DATA_RX_WAIT_DMA;
        end
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_DATA_RX_WAIT_DMA : begin
        if((endpoint_dmaLogic_stateReg == `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_BOOT))begin
          endpoint_stateNext = `endpoint_enumDefinition_binary_sequential_endpoint_UPDATE_TD_PROCESS;
        end
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_UPDATE_TD_PROCESS : begin
        endpoint_stateNext = `endpoint_enumDefinition_binary_sequential_endpoint_UPDATE_TD_CMD;
        if(! endpoint_ED_F) begin
          if(endpoint_TD_noUpdate)begin
            endpoint_stateNext = `endpoint_enumDefinition_binary_sequential_endpoint_UPDATE_SYNC;
          end
        end
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_UPDATE_TD_CMD : begin
        if((ioDma_cmd_ready && ioDma_cmd_payload_last))begin
          endpoint_stateNext = `endpoint_enumDefinition_binary_sequential_endpoint_UPDATE_ED_CMD;
        end
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_UPDATE_ED_CMD : begin
        if((ioDma_cmd_ready && ioDma_cmd_payload_last))begin
          endpoint_stateNext = `endpoint_enumDefinition_binary_sequential_endpoint_UPDATE_SYNC;
        end
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_UPDATE_SYNC : begin
        if(dmaCtx_pendingEmpty)begin
          endpoint_stateNext = `endpoint_enumDefinition_binary_sequential_endpoint_BOOT;
        end
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ABORD : begin
        endpoint_stateNext = `endpoint_enumDefinition_binary_sequential_endpoint_BOOT;
      end
      default : begin
      end
    endcase
    if(endpoint_wantStart)begin
      endpoint_stateNext = `endpoint_enumDefinition_binary_sequential_endpoint_ED_READ_CMD;
    end
    if(endpoint_wantKill)begin
      endpoint_stateNext = `endpoint_enumDefinition_binary_sequential_endpoint_BOOT;
    end
  end

  assign _zz_61 = (_zz_206 - 14'h0001);
  always @ (*) begin
    _zz_62 = 1'b0;
    if(endpoint_ED_F)begin
      case(dataRx_pid)
        4'b1110, 4'b1010 : begin
        end
        4'b0011, 4'b1011 : begin
          _zz_62 = 1'b1;
        end
        default : begin
        end
      endcase
    end else begin
      case(dataRx_pid)
        4'b1010 : begin
        end
        4'b1110 : begin
        end
        4'b0011, 4'b1011 : begin
          if(! _zz_116) begin
            _zz_62 = 1'b1;
          end
        end
        default : begin
        end
      endcase
    end
  end

  assign _zz_63 = {endpoint_TD_CC,_zz_219};
  always @ (*) begin
    endpoint_dmaLogic_stateNext = endpoint_dmaLogic_stateReg;
    case(endpoint_dmaLogic_stateReg)
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_INIT : begin
        if(endpoint_isIn)begin
          endpoint_dmaLogic_stateNext = `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_FROM_USB;
        end else begin
          endpoint_dmaLogic_stateNext = `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_CALC_CMD;
        end
      end
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_TO_USB : begin
        if(dataTx_data_ready)begin
          if(endpoint_dmaLogic_byteCtx_last)begin
            endpoint_dmaLogic_stateNext = `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_BOOT;
          end
        end
      end
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_FROM_USB : begin
        if(dataRx_wantExit)begin
          endpoint_dmaLogic_stateNext = `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_VALIDATION;
        end
      end
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_VALIDATION : begin
        if(_zz_113)begin
          endpoint_dmaLogic_stateNext = `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_BOOT;
        end else begin
          if(endpoint_dmaLogic_validated)begin
            endpoint_dmaLogic_stateNext = `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_CALC_CMD;
          end
        end
      end
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_CALC_CMD : begin
        if(endpoint_dataDone)begin
          if(endpoint_isIn)begin
            endpoint_dmaLogic_stateNext = `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_BOOT;
          end else begin
            if(dmaCtx_pendingEmpty)begin
              endpoint_dmaLogic_stateNext = `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_TO_USB;
            end
          end
        end else begin
          if(endpoint_isIn)begin
            endpoint_dmaLogic_stateNext = `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_WRITE_CMD;
          end else begin
            endpoint_dmaLogic_stateNext = `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_READ_CMD;
          end
        end
      end
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_READ_CMD : begin
        if(ioDma_cmd_ready)begin
          endpoint_dmaLogic_stateNext = `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_CALC_CMD;
        end
      end
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_WRITE_CMD : begin
        if(ioDma_cmd_ready)begin
          if(endpoint_dmaLogic_beatLast)begin
            endpoint_dmaLogic_stateNext = `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_CALC_CMD;
          end
        end
      end
      default : begin
      end
    endcase
    if(endpoint_dmaLogic_wantStart)begin
      endpoint_dmaLogic_stateNext = `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_INIT;
    end
    if(endpoint_dmaLogic_wantKill)begin
      endpoint_dmaLogic_stateNext = `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_BOOT;
    end
  end

  assign _zz_64 = (_zz_222 < endpoint_transactionSize);
  assign _zz_65 = ({3'd0,1'b1} <<< endpoint_dmaLogic_byteCtx_sel);
  assign endpoint_dmaLogic_fsmStopped = (endpoint_dmaLogic_stateReg == `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_BOOT);
  always @ (*) begin
    operational_stateNext = operational_stateReg;
    case(operational_stateReg)
      `operational_enumDefinition_binary_sequential_operational_SOF : begin
        if(sof_wantExit)begin
          operational_stateNext = `operational_enumDefinition_binary_sequential_operational_ARBITER;
        end
      end
      `operational_enumDefinition_binary_sequential_operational_ARBITER : begin
        if(operational_askExit)begin
          operational_stateNext = `operational_enumDefinition_binary_sequential_operational_BOOT;
        end else begin
          if(frame_limitHit)begin
            operational_stateNext = `operational_enumDefinition_binary_sequential_operational_WAIT_SOF;
          end else begin
            if(_zz_109)begin
              if(_zz_112)begin
                operational_stateNext = `operational_enumDefinition_binary_sequential_operational_PERIODIC_HEAD_CMD;
              end else begin
                if(! reg_hcPeriodCurrentED_isZero) begin
                  operational_stateNext = `operational_enumDefinition_binary_sequential_operational_END_POINT;
                end
              end
            end else begin
              if(priority_bulk)begin
                if(operational_allowBulk)begin
                  if(! reg_hcBulkCurrentED_isZero) begin
                    operational_stateNext = `operational_enumDefinition_binary_sequential_operational_END_POINT;
                  end
                end
              end else begin
                if(operational_allowControl)begin
                  if(! reg_hcControlCurrentED_isZero) begin
                    operational_stateNext = `operational_enumDefinition_binary_sequential_operational_END_POINT;
                  end
                end
              end
            end
          end
        end
      end
      `operational_enumDefinition_binary_sequential_operational_END_POINT : begin
        if(endpoint_wantExit)begin
          case(endpoint_status_1)
            `endpoint_Status_binary_sequential_OK : begin
              operational_stateNext = `operational_enumDefinition_binary_sequential_operational_ARBITER;
            end
            default : begin
              operational_stateNext = `operational_enumDefinition_binary_sequential_operational_WAIT_SOF;
            end
          endcase
        end
      end
      `operational_enumDefinition_binary_sequential_operational_PERIODIC_HEAD_CMD : begin
        if(ioDma_cmd_ready)begin
          operational_stateNext = `operational_enumDefinition_binary_sequential_operational_PERIODIC_HEAD_RSP;
        end
      end
      `operational_enumDefinition_binary_sequential_operational_PERIODIC_HEAD_RSP : begin
        if(ioDma_rsp_valid)begin
          operational_stateNext = `operational_enumDefinition_binary_sequential_operational_ARBITER;
        end
      end
      `operational_enumDefinition_binary_sequential_operational_WAIT_SOF : begin
        if(frame_tick)begin
          operational_stateNext = `operational_enumDefinition_binary_sequential_operational_SOF;
        end
      end
      default : begin
      end
    endcase
    if(operational_wantStart)begin
      operational_stateNext = `operational_enumDefinition_binary_sequential_operational_WAIT_SOF;
    end
    if(operational_wantKill)begin
      operational_stateNext = `operational_enumDefinition_binary_sequential_operational_BOOT;
    end
  end

  assign hc_operationalIsDone = (operational_stateReg == `operational_enumDefinition_binary_sequential_operational_BOOT);
  always @ (*) begin
    hc_stateNext = hc_stateReg;
    case(hc_stateReg)
      `hc_enumDefinition_binary_sequential_hc_RESET : begin
        if(reg_hcControl_HCFSWrite_valid)begin
          case(reg_hcControl_HCFSWrite_payload)
            `MainState_binary_sequential_OPERATIONAL : begin
              hc_stateNext = `hc_enumDefinition_binary_sequential_hc_OPERATIONAL;
            end
            default : begin
            end
          endcase
        end
      end
      `hc_enumDefinition_binary_sequential_hc_RESUME : begin
        if((reg_hcControl_HCFSWrite_valid && (reg_hcControl_HCFSWrite_payload == `MainState_binary_sequential_OPERATIONAL)))begin
          hc_stateNext = `hc_enumDefinition_binary_sequential_hc_OPERATIONAL;
        end
      end
      `hc_enumDefinition_binary_sequential_hc_OPERATIONAL : begin
      end
      `hc_enumDefinition_binary_sequential_hc_SUSPEND : begin
        if(_zz_119)begin
          hc_stateNext = `hc_enumDefinition_binary_sequential_hc_RESUME;
        end else begin
          if((reg_hcControl_HCFSWrite_valid && (reg_hcControl_HCFSWrite_payload == `MainState_binary_sequential_OPERATIONAL)))begin
            hc_stateNext = `hc_enumDefinition_binary_sequential_hc_OPERATIONAL;
          end
        end
      end
      `hc_enumDefinition_binary_sequential_hc_ANY_TO_RESET : begin
        if((! doUnschedule))begin
          hc_stateNext = `hc_enumDefinition_binary_sequential_hc_RESET;
        end
      end
      `hc_enumDefinition_binary_sequential_hc_ANY_TO_SUSPEND : begin
        if((((! doUnschedule) && (! doSoftReset)) && hc_operationalIsDone))begin
          hc_stateNext = `hc_enumDefinition_binary_sequential_hc_SUSPEND;
        end
      end
      default : begin
      end
    endcase
    if((reg_hcControl_HCFSWrite_valid && (reg_hcControl_HCFSWrite_payload == `MainState_binary_sequential_RESET)))begin
      hc_stateNext = `hc_enumDefinition_binary_sequential_hc_ANY_TO_RESET;
    end
    if(reg_hcCommandStatus_startSoftReset)begin
      hc_stateNext = `hc_enumDefinition_binary_sequential_hc_ANY_TO_SUSPEND;
    end
    if(hc_wantStart)begin
      hc_stateNext = `hc_enumDefinition_binary_sequential_hc_RESET;
    end
    if(hc_wantKill)begin
      hc_stateNext = `hc_enumDefinition_binary_sequential_hc_BOOT;
    end
  end

  always @ (posedge ctrl_clk or posedge ctrl_reset) begin
    if (ctrl_reset) begin
      dmaCtx_pendingCounter <= 4'b0000;
      dmaCtx_beatCounter <= 6'h0;
      io_dma_cmd_payload_first <= 1'b1;
      dmaReadCtx_counter <= 4'b0000;
      dmaWriteCtx_counter <= 4'b0000;
      _zz_5 <= 1'b0;
      doUnschedule <= 1'b0;
      doSoftReset <= 1'b0;
      reg_hcControl_IR <= 1'b0;
      reg_hcControl_RWC <= 1'b0;
      reg_hcFmNumber_overflow <= 1'b0;
      reg_hcPeriodicStart_PS <= 14'h0;
      io_phy_overcurrent_regNext <= 1'b0;
      reg_hcRhPortStatus_0_connected <= 1'b0;
      reg_hcRhPortStatus_0_CCS_regNext <= 1'b0;
      interruptDelay_counter <= 3'b111;
      endpoint_dmaLogic_push <= 1'b0;
      _zz_60 <= 1'b1;
      token_stateReg <= `token_enumDefinition_binary_sequential_token_BOOT;
      dataTx_stateReg <= `dataTx_enumDefinition_binary_sequential_dataTx_BOOT;
      dataRx_stateReg <= `dataRx_enumDefinition_binary_sequential_dataRx_BOOT;
      sof_stateReg <= `sof_enumDefinition_binary_sequential_sof_BOOT;
      endpoint_stateReg <= `endpoint_enumDefinition_binary_sequential_endpoint_BOOT;
      endpoint_dmaLogic_stateReg <= `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_BOOT;
      operational_stateReg <= `operational_enumDefinition_binary_sequential_operational_BOOT;
      hc_stateReg <= `hc_enumDefinition_binary_sequential_hc_BOOT;
    end else begin
      dmaCtx_pendingCounter <= (_zz_120 - _zz_124);
      if((ioDma_cmd_valid && ioDma_cmd_ready))begin
        dmaCtx_beatCounter <= (dmaCtx_beatCounter + 6'h01);
        if(io_dma_cmd_payload_last)begin
          dmaCtx_beatCounter <= 6'h0;
        end
      end
      if((io_dma_cmd_valid && io_dma_cmd_ready))begin
        io_dma_cmd_payload_first <= io_dma_cmd_payload_last;
      end
      if((ioDma_rsp_valid && ioDma_rsp_ready))begin
        dmaReadCtx_counter <= (dmaReadCtx_counter + 4'b0001);
        if(ioDma_rsp_payload_last)begin
          dmaReadCtx_counter <= 4'b0000;
        end
      end
      if((ioDma_cmd_valid && ioDma_cmd_ready))begin
        dmaWriteCtx_counter <= (dmaWriteCtx_counter + 4'b0001);
        if(ioDma_cmd_payload_last)begin
          dmaWriteCtx_counter <= 4'b0000;
        end
      end
      if(_zz_3)begin
        _zz_5 <= (ctrl_rsp_valid && _zz_2);
      end
      if(unscheduleAll_ready)begin
        doUnschedule <= 1'b0;
      end
      if((! doUnschedule))begin
        doSoftReset <= 1'b0;
      end
      io_phy_overcurrent_regNext <= io_phy_overcurrent;
      if(io_phy_ports_0_connect)begin
        reg_hcRhPortStatus_0_connected <= 1'b1;
      end
      if(io_phy_ports_0_disconnect)begin
        reg_hcRhPortStatus_0_connected <= 1'b0;
      end
      reg_hcRhPortStatus_0_CCS_regNext <= reg_hcRhPortStatus_0_CCS;
      if(frame_reload)begin
        if((reg_hcFmNumber_FNp1[15] ^ reg_hcFmNumber_FN[15]))begin
          reg_hcFmNumber_overflow <= 1'b1;
        end
      end
      if(((interruptDelay_tick && (! interruptDelay_done)) && (! interruptDelay_disabled)))begin
        interruptDelay_counter <= (interruptDelay_counter - 3'b001);
      end
      if((interruptDelay_load_valid && (interruptDelay_load_payload < interruptDelay_counter)))begin
        interruptDelay_counter <= interruptDelay_load_payload;
      end
      if(interruptDelay_disable)begin
        interruptDelay_counter <= 3'b111;
      end
      endpoint_dmaLogic_push <= 1'b0;
      case(io_ctrl_cmd_payload_fragment_address)
        12'h004 : begin
          if(ctrl_doWrite)begin
            if(io_ctrl_cmd_payload_fragment_mask[1])begin
              reg_hcControl_IR <= io_ctrl_cmd_payload_fragment_data[8];
            end
            if(io_ctrl_cmd_payload_fragment_mask[1])begin
              reg_hcControl_RWC <= io_ctrl_cmd_payload_fragment_data[9];
            end
          end
        end
        12'h040 : begin
          if(ctrl_doWrite)begin
            if(io_ctrl_cmd_payload_fragment_mask[0])begin
              reg_hcPeriodicStart_PS[7 : 0] <= io_ctrl_cmd_payload_fragment_data[7 : 0];
            end
            if(io_ctrl_cmd_payload_fragment_mask[1])begin
              reg_hcPeriodicStart_PS[13 : 8] <= io_ctrl_cmd_payload_fragment_data[13 : 8];
            end
          end
        end
        default : begin
        end
      endcase
      _zz_60 <= 1'b0;
      token_stateReg <= token_stateNext;
      dataTx_stateReg <= dataTx_stateNext;
      dataRx_stateReg <= dataRx_stateNext;
      sof_stateReg <= sof_stateNext;
      case(sof_stateReg)
        `sof_enumDefinition_binary_sequential_sof_FRAME_TX : begin
        end
        `sof_enumDefinition_binary_sequential_sof_FRAME_NUMBER_CMD : begin
        end
        `sof_enumDefinition_binary_sequential_sof_FRAME_NUMBER_RSP : begin
          if(ioDma_rsp_valid)begin
            reg_hcFmNumber_overflow <= 1'b0;
          end
        end
        default : begin
        end
      endcase
      endpoint_stateReg <= endpoint_stateNext;
      endpoint_dmaLogic_stateReg <= endpoint_dmaLogic_stateNext;
      case(endpoint_dmaLogic_stateReg)
        `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_INIT : begin
        end
        `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_TO_USB : begin
        end
        `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_FROM_USB : begin
          if(dataRx_wantExit)begin
            endpoint_dmaLogic_push <= (endpoint_dmaLogic_byteCtx_sel != 2'b00);
          end
          if(dataRx_data_valid)begin
            if((endpoint_dmaLogic_byteCtx_sel == 2'b11))begin
              endpoint_dmaLogic_push <= 1'b1;
            end
          end
        end
        `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_VALIDATION : begin
        end
        `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_CALC_CMD : begin
        end
        `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_READ_CMD : begin
        end
        `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_WRITE_CMD : begin
        end
        default : begin
        end
      endcase
      operational_stateReg <= operational_stateNext;
      hc_stateReg <= hc_stateNext;
      if(((! (hc_stateReg == `hc_enumDefinition_binary_sequential_hc_ANY_TO_RESET)) && (hc_stateNext == `hc_enumDefinition_binary_sequential_hc_ANY_TO_RESET)))begin
        doUnschedule <= 1'b1;
      end
      if(((! (hc_stateReg == `hc_enumDefinition_binary_sequential_hc_ANY_TO_SUSPEND)) && (hc_stateNext == `hc_enumDefinition_binary_sequential_hc_ANY_TO_SUSPEND)))begin
        doUnschedule <= 1'b1;
      end
      if(reg_hcCommandStatus_startSoftReset)begin
        doSoftReset <= 1'b1;
      end
    end
  end

  always @ (posedge ctrl_clk) begin
    if(_zz_3)begin
      _zz_6 <= ctrl_rsp_payload_last;
      _zz_7 <= ctrl_rsp_payload_fragment_opcode;
      _zz_8 <= ctrl_rsp_payload_fragment_data;
    end
    if(_zz_10)begin
      if(io_ctrl_cmd_payload_fragment_data[1])begin
        reg_hcCommandStatus_CLF <= _zz_126[0];
      end
    end
    if(_zz_11)begin
      if(io_ctrl_cmd_payload_fragment_data[2])begin
        reg_hcCommandStatus_BLF <= _zz_127[0];
      end
    end
    if(_zz_12)begin
      if(io_ctrl_cmd_payload_fragment_data[3])begin
        reg_hcCommandStatus_OCR <= _zz_128[0];
      end
    end
    if(_zz_13)begin
      if(io_ctrl_cmd_payload_fragment_data[31])begin
        reg_hcInterrupt_MIE <= _zz_129[0];
      end
    end
    if(_zz_14)begin
      if(io_ctrl_cmd_payload_fragment_data[31])begin
        reg_hcInterrupt_MIE <= _zz_130[0];
      end
    end
    if(_zz_15)begin
      if(io_ctrl_cmd_payload_fragment_data[0])begin
        reg_hcInterrupt_SO_status <= _zz_131[0];
      end
    end
    if(_zz_16)begin
      if(io_ctrl_cmd_payload_fragment_data[0])begin
        reg_hcInterrupt_SO_enable <= _zz_132[0];
      end
    end
    if(_zz_17)begin
      if(io_ctrl_cmd_payload_fragment_data[0])begin
        reg_hcInterrupt_SO_enable <= _zz_133[0];
      end
    end
    if(_zz_18)begin
      if(io_ctrl_cmd_payload_fragment_data[1])begin
        reg_hcInterrupt_WDH_status <= _zz_134[0];
      end
    end
    if(_zz_19)begin
      if(io_ctrl_cmd_payload_fragment_data[1])begin
        reg_hcInterrupt_WDH_enable <= _zz_135[0];
      end
    end
    if(_zz_20)begin
      if(io_ctrl_cmd_payload_fragment_data[1])begin
        reg_hcInterrupt_WDH_enable <= _zz_136[0];
      end
    end
    if(_zz_21)begin
      if(io_ctrl_cmd_payload_fragment_data[2])begin
        reg_hcInterrupt_SF_status <= _zz_137[0];
      end
    end
    if(_zz_22)begin
      if(io_ctrl_cmd_payload_fragment_data[2])begin
        reg_hcInterrupt_SF_enable <= _zz_138[0];
      end
    end
    if(_zz_23)begin
      if(io_ctrl_cmd_payload_fragment_data[2])begin
        reg_hcInterrupt_SF_enable <= _zz_139[0];
      end
    end
    if(_zz_24)begin
      if(io_ctrl_cmd_payload_fragment_data[3])begin
        reg_hcInterrupt_RD_status <= _zz_140[0];
      end
    end
    if(_zz_25)begin
      if(io_ctrl_cmd_payload_fragment_data[3])begin
        reg_hcInterrupt_RD_enable <= _zz_141[0];
      end
    end
    if(_zz_26)begin
      if(io_ctrl_cmd_payload_fragment_data[3])begin
        reg_hcInterrupt_RD_enable <= _zz_142[0];
      end
    end
    if(_zz_27)begin
      if(io_ctrl_cmd_payload_fragment_data[4])begin
        reg_hcInterrupt_UE_status <= _zz_143[0];
      end
    end
    if(_zz_28)begin
      if(io_ctrl_cmd_payload_fragment_data[4])begin
        reg_hcInterrupt_UE_enable <= _zz_144[0];
      end
    end
    if(_zz_29)begin
      if(io_ctrl_cmd_payload_fragment_data[4])begin
        reg_hcInterrupt_UE_enable <= _zz_145[0];
      end
    end
    if(_zz_30)begin
      if(io_ctrl_cmd_payload_fragment_data[5])begin
        reg_hcInterrupt_FNO_status <= _zz_146[0];
      end
    end
    if(_zz_31)begin
      if(io_ctrl_cmd_payload_fragment_data[5])begin
        reg_hcInterrupt_FNO_enable <= _zz_147[0];
      end
    end
    if(_zz_32)begin
      if(io_ctrl_cmd_payload_fragment_data[5])begin
        reg_hcInterrupt_FNO_enable <= _zz_148[0];
      end
    end
    if(_zz_33)begin
      if(io_ctrl_cmd_payload_fragment_data[6])begin
        reg_hcInterrupt_RHSC_status <= _zz_149[0];
      end
    end
    if(_zz_34)begin
      if(io_ctrl_cmd_payload_fragment_data[6])begin
        reg_hcInterrupt_RHSC_enable <= _zz_150[0];
      end
    end
    if(_zz_35)begin
      if(io_ctrl_cmd_payload_fragment_data[6])begin
        reg_hcInterrupt_RHSC_enable <= _zz_151[0];
      end
    end
    if(_zz_36)begin
      if(io_ctrl_cmd_payload_fragment_data[30])begin
        reg_hcInterrupt_OC_status <= _zz_152[0];
      end
    end
    if(_zz_37)begin
      if(io_ctrl_cmd_payload_fragment_data[30])begin
        reg_hcInterrupt_OC_enable <= _zz_153[0];
      end
    end
    if(_zz_38)begin
      if(io_ctrl_cmd_payload_fragment_data[30])begin
        reg_hcInterrupt_OC_enable <= _zz_154[0];
      end
    end
    if(reg_hcCommandStatus_OCR)begin
      reg_hcInterrupt_OC_status <= 1'b1;
    end
    if(_zz_39)begin
      if(io_ctrl_cmd_payload_fragment_data[17])begin
        reg_hcRhStatus_CCIC <= _zz_156[0];
      end
    end
    if((io_phy_overcurrent ^ io_phy_overcurrent_regNext))begin
      reg_hcRhStatus_CCIC <= 1'b1;
    end
    if(reg_hcRhStatus_setRemoteWakeupEnable)begin
      reg_hcRhStatus_DRWE <= 1'b1;
    end
    if(reg_hcRhStatus_clearRemoteWakeupEnable)begin
      reg_hcRhStatus_DRWE <= 1'b0;
    end
    if(reg_hcRhPortStatus_0_CSC_clear)begin
      reg_hcRhPortStatus_0_CSC_reg <= 1'b0;
    end
    if(reg_hcRhPortStatus_0_CSC_set)begin
      reg_hcRhPortStatus_0_CSC_reg <= 1'b1;
    end
    if(reg_hcRhPortStatus_0_CSC_set)begin
      reg_hcInterrupt_RHSC_status <= 1'b1;
    end
    if(reg_hcRhPortStatus_0_PESC_clear)begin
      reg_hcRhPortStatus_0_PESC_reg <= 1'b0;
    end
    if(reg_hcRhPortStatus_0_PESC_set)begin
      reg_hcRhPortStatus_0_PESC_reg <= 1'b1;
    end
    if(reg_hcRhPortStatus_0_PESC_set)begin
      reg_hcInterrupt_RHSC_status <= 1'b1;
    end
    if(reg_hcRhPortStatus_0_PSSC_clear)begin
      reg_hcRhPortStatus_0_PSSC_reg <= 1'b0;
    end
    if(reg_hcRhPortStatus_0_PSSC_set)begin
      reg_hcRhPortStatus_0_PSSC_reg <= 1'b1;
    end
    if(reg_hcRhPortStatus_0_PSSC_set)begin
      reg_hcInterrupt_RHSC_status <= 1'b1;
    end
    if(reg_hcRhPortStatus_0_OCIC_clear)begin
      reg_hcRhPortStatus_0_OCIC_reg <= 1'b0;
    end
    if(reg_hcRhPortStatus_0_OCIC_set)begin
      reg_hcRhPortStatus_0_OCIC_reg <= 1'b1;
    end
    if(reg_hcRhPortStatus_0_OCIC_set)begin
      reg_hcInterrupt_RHSC_status <= 1'b1;
    end
    if(reg_hcRhPortStatus_0_PRSC_clear)begin
      reg_hcRhPortStatus_0_PRSC_reg <= 1'b0;
    end
    if(reg_hcRhPortStatus_0_PRSC_set)begin
      reg_hcRhPortStatus_0_PRSC_reg <= 1'b1;
    end
    if(reg_hcRhPortStatus_0_PRSC_set)begin
      reg_hcInterrupt_RHSC_status <= 1'b1;
    end
    if(((reg_hcRhPortStatus_0_clearPortEnable || reg_hcRhPortStatus_0_PESC_set) || (! reg_hcRhPortStatus_0_PPS)))begin
      reg_hcRhPortStatus_0_PES <= 1'b0;
    end
    if((reg_hcRhPortStatus_0_PRSC_set || reg_hcRhPortStatus_0_PSSC_set))begin
      reg_hcRhPortStatus_0_PES <= 1'b1;
    end
    if((reg_hcRhPortStatus_0_setPortEnable && reg_hcRhPortStatus_0_CCS))begin
      reg_hcRhPortStatus_0_PES <= 1'b1;
    end
    if((((reg_hcRhPortStatus_0_PSSC_set || reg_hcRhPortStatus_0_PRSC_set) || (! reg_hcRhPortStatus_0_PPS)) || (reg_hcControl_HCFS == `MainState_binary_sequential_RESUME)))begin
      reg_hcRhPortStatus_0_PSS <= 1'b0;
    end
    if((reg_hcRhPortStatus_0_setPortSuspend && reg_hcRhPortStatus_0_CCS))begin
      reg_hcRhPortStatus_0_PSS <= 1'b1;
    end
    if((reg_hcRhPortStatus_0_setPortSuspend && reg_hcRhPortStatus_0_CCS))begin
      reg_hcRhPortStatus_0_suspend <= 1'b1;
    end
    if((reg_hcRhPortStatus_0_clearSuspendStatus && reg_hcRhPortStatus_0_PSS))begin
      reg_hcRhPortStatus_0_resume <= 1'b1;
    end
    if((reg_hcRhPortStatus_0_setPortReset && reg_hcRhPortStatus_0_CCS))begin
      reg_hcRhPortStatus_0_reset <= 1'b1;
    end
    if(reg_hcRhDescriptorA_NPS)begin
      reg_hcRhPortStatus_0_PPS <= 1'b1;
    end else begin
      if(reg_hcRhDescriptorA_PSM)begin
        if(reg_hcRhDescriptorB_PPCM[0])begin
          if(reg_hcRhPortStatus_0_clearPortPower)begin
            reg_hcRhPortStatus_0_PPS <= 1'b0;
          end
          if(reg_hcRhPortStatus_0_setPortPower)begin
            reg_hcRhPortStatus_0_PPS <= 1'b1;
          end
        end else begin
          if(reg_hcRhStatus_clearGlobalPower)begin
            reg_hcRhPortStatus_0_PPS <= 1'b0;
          end
          if(reg_hcRhStatus_setGlobalPower)begin
            reg_hcRhPortStatus_0_PPS <= 1'b1;
          end
        end
      end else begin
        if(reg_hcRhStatus_clearGlobalPower)begin
          reg_hcRhPortStatus_0_PPS <= 1'b0;
        end
        if(reg_hcRhStatus_setGlobalPower)begin
          reg_hcRhPortStatus_0_PPS <= 1'b1;
        end
      end
    end
    if(io_phy_overcurrent)begin
      reg_hcRhPortStatus_0_PPS <= 1'b0;
    end
    if((io_phy_ports_0_resume_valid && io_phy_ports_0_resume_ready))begin
      reg_hcRhPortStatus_0_resume <= 1'b0;
    end
    if((io_phy_ports_0_reset_valid && io_phy_ports_0_reset_ready))begin
      reg_hcRhPortStatus_0_reset <= 1'b0;
    end
    if((io_phy_ports_0_suspend_valid && io_phy_ports_0_suspend_ready))begin
      reg_hcRhPortStatus_0_suspend <= 1'b0;
    end
    frame_decrementTimer <= (frame_decrementTimer + 3'b001);
    if(frame_decrementTimerOverflow)begin
      frame_decrementTimer <= 3'b000;
    end
    if(_zz_102)begin
      reg_hcFmRemaining_FR <= (reg_hcFmRemaining_FR - 14'h0001);
      if(((! frame_limitHit) && (! frame_decrementTimerOverflow)))begin
        frame_limitCounter <= (frame_limitCounter - 15'h0001);
      end
    end
    if(frame_reload)begin
      reg_hcFmRemaining_FR <= reg_hcFmInterval_FI;
      reg_hcFmRemaining_FRT <= reg_hcFmInterval_FIT;
      reg_hcFmNumber_FN <= reg_hcFmNumber_FNp1;
      frame_limitCounter <= reg_hcFmInterval_FSMPS;
      frame_decrementTimer <= 3'b000;
    end
    if(io_phy_tick)begin
      rxTimer_counter <= (rxTimer_counter + 8'h01);
    end
    if(rxTimer_clear)begin
      rxTimer_counter <= 8'h0;
    end
    if(io_phy_rx_flow_valid)begin
      _zz_56 <= io_phy_rx_flow_payload_data;
    end
    if(io_phy_rx_flow_valid)begin
      _zz_57 <= _zz_56;
    end
    if(priority_tick)begin
      priority_counter <= (priority_counter + 2'b01);
    end
    if(priority_skip)begin
      priority_bulk <= (! priority_bulk);
      priority_counter <= 2'b00;
    end
    endpoint_TD_isoOverrunReg <= endpoint_TD_isoOverrun;
    if(endpoint_TD_clear)begin
      endpoint_TD_retire <= 1'b0;
      endpoint_TD_dataPhaseUpdate <= 1'b0;
      endpoint_TD_upateCBP <= 1'b0;
      endpoint_TD_noUpdate <= 1'b0;
    end
    if(endpoint_applyNextED)begin
      case(endpoint_flowType)
        `FlowType_binary_sequential_BULK : begin
          reg_hcBulkCurrentED_BCED_reg <= endpoint_ED_nextED;
        end
        `FlowType_binary_sequential_CONTROL : begin
          reg_hcControlCurrentED_CCED_reg <= endpoint_ED_nextED;
        end
        default : begin
          reg_hcPeriodCurrentED_PCED_reg <= endpoint_ED_nextED;
        end
      endcase
    end
    if(endpoint_dmaLogic_byteCtx_increment)begin
      endpoint_dmaLogic_byteCtx_counter <= (endpoint_dmaLogic_byteCtx_counter + 13'h0001);
    end
    case(io_ctrl_cmd_payload_fragment_address)
      12'h004 : begin
        if(ctrl_doWrite)begin
          if(io_ctrl_cmd_payload_fragment_mask[0])begin
            reg_hcControl_CBSR[1 : 0] <= io_ctrl_cmd_payload_fragment_data[1 : 0];
          end
          if(io_ctrl_cmd_payload_fragment_mask[0])begin
            reg_hcControl_PLE <= io_ctrl_cmd_payload_fragment_data[2];
          end
          if(io_ctrl_cmd_payload_fragment_mask[0])begin
            reg_hcControl_IE <= io_ctrl_cmd_payload_fragment_data[3];
          end
          if(io_ctrl_cmd_payload_fragment_mask[0])begin
            reg_hcControl_CLE <= io_ctrl_cmd_payload_fragment_data[4];
          end
          if(io_ctrl_cmd_payload_fragment_mask[0])begin
            reg_hcControl_BLE <= io_ctrl_cmd_payload_fragment_data[5];
          end
          if(io_ctrl_cmd_payload_fragment_mask[1])begin
            reg_hcControl_RWE <= io_ctrl_cmd_payload_fragment_data[10];
          end
        end
      end
      12'h018 : begin
        if(ctrl_doWrite)begin
          if(io_ctrl_cmd_payload_fragment_mask[1])begin
            reg_hcHCCA_HCCA_reg[7 : 0] <= io_ctrl_cmd_payload_fragment_data[15 : 8];
          end
          if(io_ctrl_cmd_payload_fragment_mask[2])begin
            reg_hcHCCA_HCCA_reg[15 : 8] <= io_ctrl_cmd_payload_fragment_data[23 : 16];
          end
          if(io_ctrl_cmd_payload_fragment_mask[3])begin
            reg_hcHCCA_HCCA_reg[23 : 16] <= io_ctrl_cmd_payload_fragment_data[31 : 24];
          end
        end
      end
      12'h020 : begin
        if(ctrl_doWrite)begin
          if(io_ctrl_cmd_payload_fragment_mask[0])begin
            reg_hcControlHeadED_CHED_reg[3 : 0] <= io_ctrl_cmd_payload_fragment_data[7 : 4];
          end
          if(io_ctrl_cmd_payload_fragment_mask[1])begin
            reg_hcControlHeadED_CHED_reg[11 : 4] <= io_ctrl_cmd_payload_fragment_data[15 : 8];
          end
          if(io_ctrl_cmd_payload_fragment_mask[2])begin
            reg_hcControlHeadED_CHED_reg[19 : 12] <= io_ctrl_cmd_payload_fragment_data[23 : 16];
          end
          if(io_ctrl_cmd_payload_fragment_mask[3])begin
            reg_hcControlHeadED_CHED_reg[27 : 20] <= io_ctrl_cmd_payload_fragment_data[31 : 24];
          end
        end
      end
      12'h024 : begin
        if(ctrl_doWrite)begin
          if(io_ctrl_cmd_payload_fragment_mask[0])begin
            reg_hcControlCurrentED_CCED_reg[3 : 0] <= io_ctrl_cmd_payload_fragment_data[7 : 4];
          end
          if(io_ctrl_cmd_payload_fragment_mask[1])begin
            reg_hcControlCurrentED_CCED_reg[11 : 4] <= io_ctrl_cmd_payload_fragment_data[15 : 8];
          end
          if(io_ctrl_cmd_payload_fragment_mask[2])begin
            reg_hcControlCurrentED_CCED_reg[19 : 12] <= io_ctrl_cmd_payload_fragment_data[23 : 16];
          end
          if(io_ctrl_cmd_payload_fragment_mask[3])begin
            reg_hcControlCurrentED_CCED_reg[27 : 20] <= io_ctrl_cmd_payload_fragment_data[31 : 24];
          end
        end
      end
      12'h028 : begin
        if(ctrl_doWrite)begin
          if(io_ctrl_cmd_payload_fragment_mask[0])begin
            reg_hcBulkHeadED_BHED_reg[3 : 0] <= io_ctrl_cmd_payload_fragment_data[7 : 4];
          end
          if(io_ctrl_cmd_payload_fragment_mask[1])begin
            reg_hcBulkHeadED_BHED_reg[11 : 4] <= io_ctrl_cmd_payload_fragment_data[15 : 8];
          end
          if(io_ctrl_cmd_payload_fragment_mask[2])begin
            reg_hcBulkHeadED_BHED_reg[19 : 12] <= io_ctrl_cmd_payload_fragment_data[23 : 16];
          end
          if(io_ctrl_cmd_payload_fragment_mask[3])begin
            reg_hcBulkHeadED_BHED_reg[27 : 20] <= io_ctrl_cmd_payload_fragment_data[31 : 24];
          end
        end
      end
      12'h02c : begin
        if(ctrl_doWrite)begin
          if(io_ctrl_cmd_payload_fragment_mask[0])begin
            reg_hcBulkCurrentED_BCED_reg[3 : 0] <= io_ctrl_cmd_payload_fragment_data[7 : 4];
          end
          if(io_ctrl_cmd_payload_fragment_mask[1])begin
            reg_hcBulkCurrentED_BCED_reg[11 : 4] <= io_ctrl_cmd_payload_fragment_data[15 : 8];
          end
          if(io_ctrl_cmd_payload_fragment_mask[2])begin
            reg_hcBulkCurrentED_BCED_reg[19 : 12] <= io_ctrl_cmd_payload_fragment_data[23 : 16];
          end
          if(io_ctrl_cmd_payload_fragment_mask[3])begin
            reg_hcBulkCurrentED_BCED_reg[27 : 20] <= io_ctrl_cmd_payload_fragment_data[31 : 24];
          end
        end
      end
      12'h030 : begin
        if(ctrl_doWrite)begin
          if(io_ctrl_cmd_payload_fragment_mask[0])begin
            reg_hcDoneHead_DH_reg[3 : 0] <= io_ctrl_cmd_payload_fragment_data[7 : 4];
          end
          if(io_ctrl_cmd_payload_fragment_mask[1])begin
            reg_hcDoneHead_DH_reg[11 : 4] <= io_ctrl_cmd_payload_fragment_data[15 : 8];
          end
          if(io_ctrl_cmd_payload_fragment_mask[2])begin
            reg_hcDoneHead_DH_reg[19 : 12] <= io_ctrl_cmd_payload_fragment_data[23 : 16];
          end
          if(io_ctrl_cmd_payload_fragment_mask[3])begin
            reg_hcDoneHead_DH_reg[27 : 20] <= io_ctrl_cmd_payload_fragment_data[31 : 24];
          end
        end
      end
      12'h034 : begin
        if(ctrl_doWrite)begin
          if(io_ctrl_cmd_payload_fragment_mask[0])begin
            reg_hcFmInterval_FI[7 : 0] <= io_ctrl_cmd_payload_fragment_data[7 : 0];
          end
          if(io_ctrl_cmd_payload_fragment_mask[1])begin
            reg_hcFmInterval_FI[13 : 8] <= io_ctrl_cmd_payload_fragment_data[13 : 8];
          end
          if(io_ctrl_cmd_payload_fragment_mask[2])begin
            reg_hcFmInterval_FSMPS[7 : 0] <= io_ctrl_cmd_payload_fragment_data[23 : 16];
          end
          if(io_ctrl_cmd_payload_fragment_mask[3])begin
            reg_hcFmInterval_FSMPS[14 : 8] <= io_ctrl_cmd_payload_fragment_data[30 : 24];
          end
          if(io_ctrl_cmd_payload_fragment_mask[3])begin
            reg_hcFmInterval_FIT <= io_ctrl_cmd_payload_fragment_data[31];
          end
        end
      end
      12'h044 : begin
        if(ctrl_doWrite)begin
          if(io_ctrl_cmd_payload_fragment_mask[0])begin
            reg_hcLSThreshold_LST[7 : 0] <= io_ctrl_cmd_payload_fragment_data[7 : 0];
          end
          if(io_ctrl_cmd_payload_fragment_mask[1])begin
            reg_hcLSThreshold_LST[11 : 8] <= io_ctrl_cmd_payload_fragment_data[11 : 8];
          end
        end
      end
      12'h048 : begin
        if(ctrl_doWrite)begin
          if(io_ctrl_cmd_payload_fragment_mask[1])begin
            reg_hcRhDescriptorA_PSM <= io_ctrl_cmd_payload_fragment_data[8];
          end
          if(io_ctrl_cmd_payload_fragment_mask[1])begin
            reg_hcRhDescriptorA_NPS <= io_ctrl_cmd_payload_fragment_data[9];
          end
          if(io_ctrl_cmd_payload_fragment_mask[1])begin
            reg_hcRhDescriptorA_OCPM <= io_ctrl_cmd_payload_fragment_data[11];
          end
          if(io_ctrl_cmd_payload_fragment_mask[1])begin
            reg_hcRhDescriptorA_NOCP <= io_ctrl_cmd_payload_fragment_data[12];
          end
          if(io_ctrl_cmd_payload_fragment_mask[3])begin
            reg_hcRhDescriptorA_POTPGT[7 : 0] <= io_ctrl_cmd_payload_fragment_data[31 : 24];
          end
        end
      end
      12'h04c : begin
        if(ctrl_doWrite)begin
          if(io_ctrl_cmd_payload_fragment_mask[0])begin
            reg_hcRhDescriptorB_DR[0 : 0] <= io_ctrl_cmd_payload_fragment_data[1 : 1];
          end
          if(io_ctrl_cmd_payload_fragment_mask[2])begin
            reg_hcRhDescriptorB_PPCM[0 : 0] <= io_ctrl_cmd_payload_fragment_data[17 : 17];
          end
        end
      end
      default : begin
      end
    endcase
    if((doSoftReset || _zz_60))begin
      reg_hcControl_CBSR <= 2'b00;
      reg_hcControl_PLE <= 1'b0;
      reg_hcControl_IE <= 1'b0;
      reg_hcControl_CLE <= 1'b0;
      reg_hcControl_BLE <= 1'b0;
      reg_hcControl_RWE <= 1'b0;
      reg_hcCommandStatus_CLF <= 1'b0;
      reg_hcCommandStatus_BLF <= 1'b0;
      reg_hcCommandStatus_OCR <= 1'b0;
      reg_hcCommandStatus_SOC <= 2'b00;
      reg_hcInterrupt_MIE <= 1'b0;
      reg_hcInterrupt_SO_status <= 1'b0;
      reg_hcInterrupt_SO_enable <= 1'b0;
      reg_hcInterrupt_WDH_status <= 1'b0;
      reg_hcInterrupt_WDH_enable <= 1'b0;
      reg_hcInterrupt_SF_status <= 1'b0;
      reg_hcInterrupt_SF_enable <= 1'b0;
      reg_hcInterrupt_RD_status <= 1'b0;
      reg_hcInterrupt_RD_enable <= 1'b0;
      reg_hcInterrupt_UE_status <= 1'b0;
      reg_hcInterrupt_UE_enable <= 1'b0;
      reg_hcInterrupt_FNO_status <= 1'b0;
      reg_hcInterrupt_FNO_enable <= 1'b0;
      reg_hcInterrupt_RHSC_status <= 1'b0;
      reg_hcInterrupt_RHSC_enable <= 1'b0;
      reg_hcInterrupt_OC_status <= 1'b0;
      reg_hcInterrupt_OC_enable <= 1'b0;
      reg_hcHCCA_HCCA_reg <= 24'h0;
      reg_hcPeriodCurrentED_PCED_reg <= 28'h0;
      reg_hcControlHeadED_CHED_reg <= 28'h0;
      reg_hcControlCurrentED_CCED_reg <= 28'h0;
      reg_hcBulkHeadED_BHED_reg <= 28'h0;
      reg_hcBulkCurrentED_BCED_reg <= 28'h0;
      reg_hcDoneHead_DH_reg <= 28'h0;
      reg_hcFmInterval_FI <= 14'h2edf;
      reg_hcFmInterval_FIT <= 1'b0;
      reg_hcFmRemaining_FR <= 14'h0;
      reg_hcFmRemaining_FRT <= 1'b0;
      reg_hcFmNumber_FN <= 16'h0;
      reg_hcLSThreshold_LST <= 12'h628;
      reg_hcRhDescriptorA_PSM <= 1'b0;
      reg_hcRhDescriptorA_NPS <= 1'b0;
      reg_hcRhDescriptorA_OCPM <= 1'b0;
      reg_hcRhDescriptorA_NOCP <= 1'b0;
      reg_hcRhDescriptorA_POTPGT <= 8'h0a;
      reg_hcRhDescriptorB_DR <= 1'b0;
      reg_hcRhDescriptorB_PPCM <= 1'b1;
      reg_hcRhStatus_DRWE <= 1'b0;
      reg_hcRhStatus_CCIC <= 1'b0;
      reg_hcRhPortStatus_0_resume <= 1'b0;
      reg_hcRhPortStatus_0_reset <= 1'b0;
      reg_hcRhPortStatus_0_suspend <= 1'b0;
      reg_hcRhPortStatus_0_PSS <= 1'b0;
      reg_hcRhPortStatus_0_PPS <= 1'b0;
      reg_hcRhPortStatus_0_PES <= 1'b0;
      reg_hcRhPortStatus_0_CSC_reg <= 1'b0;
      reg_hcRhPortStatus_0_PESC_reg <= 1'b0;
      reg_hcRhPortStatus_0_PSSC_reg <= 1'b0;
      reg_hcRhPortStatus_0_OCIC_reg <= 1'b0;
      reg_hcRhPortStatus_0_PRSC_reg <= 1'b0;
    end
    case(dataRx_stateReg)
      `dataRx_enumDefinition_binary_sequential_dataRx_IDLE : begin
        if(! io_phy_rx_active) begin
          if(rxTimer_rxTimeout)begin
            dataRx_notResponding <= 1'b1;
          end
        end
      end
      `dataRx_enumDefinition_binary_sequential_dataRx_PID : begin
        dataRx_valids <= 2'b00;
        dataRx_pidError <= 1'b1;
        if(io_phy_rx_flow_valid)begin
          dataRx_pid <= io_phy_rx_flow_payload_data[3 : 0];
          dataRx_pidError <= (! rxPidOk);
        end
      end
      `dataRx_enumDefinition_binary_sequential_dataRx_DATA : begin
        if(_zz_107)begin
          if(((! (dataRx_valids == 2'b11)) || (dataRx_crc16_io_result != 16'h800d)))begin
            dataRx_crcError <= 1'b1;
          end
        end else begin
          if(io_phy_rx_flow_valid)begin
            dataRx_valids <= {dataRx_valids[0],1'b1};
          end
        end
      end
      default : begin
      end
    endcase
    if(_zz_104)begin
      dataRx_notResponding <= 1'b0;
      dataRx_stuffingError <= 1'b0;
      dataRx_pidError <= 1'b0;
      dataRx_crcError <= 1'b0;
    end
    if((! (dataRx_stateReg == `dataRx_enumDefinition_binary_sequential_dataRx_BOOT)))begin
      if(io_phy_rx_flow_valid)begin
        if(io_phy_rx_flow_payload_stuffingError)begin
          dataRx_stuffingError <= 1'b1;
        end
      end
    end
    case(sof_stateReg)
      `sof_enumDefinition_binary_sequential_sof_FRAME_TX : begin
        sof_doInterruptDelay <= (interruptDelay_done && (! reg_hcInterrupt_WDH_status));
      end
      `sof_enumDefinition_binary_sequential_sof_FRAME_NUMBER_CMD : begin
      end
      `sof_enumDefinition_binary_sequential_sof_FRAME_NUMBER_RSP : begin
        if(ioDma_rsp_valid)begin
          reg_hcInterrupt_SF_status <= 1'b1;
          if(reg_hcFmNumber_overflow)begin
            reg_hcInterrupt_FNO_status <= 1'b1;
          end
          if(sof_doInterruptDelay)begin
            reg_hcInterrupt_WDH_status <= 1'b1;
            reg_hcDoneHead_DH_reg <= 28'h0;
          end
        end
      end
      default : begin
      end
    endcase
    case(endpoint_stateReg)
      `endpoint_enumDefinition_binary_sequential_endpoint_ED_READ_CMD : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ED_READ_RSP : begin
        if((ioDma_rsp_valid && (dmaReadCtx_counter == 4'b0000)))begin
          endpoint_ED_words_0 <= dmaRspMux_vec_0[31 : 0];
        end
        if((ioDma_rsp_valid && (dmaReadCtx_counter == 4'b0001)))begin
          endpoint_ED_words_1 <= dmaRspMux_vec_0[31 : 0];
        end
        if((ioDma_rsp_valid && (dmaReadCtx_counter == 4'b0010)))begin
          endpoint_ED_words_2 <= dmaRspMux_vec_0[31 : 0];
        end
        if((ioDma_rsp_valid && (dmaReadCtx_counter == 4'b0011)))begin
          endpoint_ED_words_3 <= dmaRspMux_vec_0[31 : 0];
        end
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ED_ANALYSE : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TD_READ_CMD : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TD_READ_RSP : begin
        if((ioDma_rsp_valid && (dmaReadCtx_counter == 4'b0000)))begin
          endpoint_TD_words_0 <= dmaRspMux_vec_0[31 : 0];
        end
        if((ioDma_rsp_valid && (dmaReadCtx_counter == 4'b0001)))begin
          endpoint_TD_words_1 <= dmaRspMux_vec_0[31 : 0];
        end
        if((ioDma_rsp_valid && (dmaReadCtx_counter == 4'b0010)))begin
          endpoint_TD_words_2 <= dmaRspMux_vec_0[31 : 0];
        end
        if((ioDma_rsp_valid && (dmaReadCtx_counter == 4'b0011)))begin
          endpoint_TD_words_3 <= dmaRspMux_vec_0[31 : 0];
        end
        if((endpoint_TD_isoFrameNumber == 3'b000))begin
          if((ioDma_rsp_valid && (dmaReadCtx_counter == 4'b0100)))begin
            endpoint_TD_isoBase <= dmaRspMux_vec_0[12 : 0];
          end
          if((ioDma_rsp_valid && (dmaReadCtx_counter == 4'b0100)))begin
            endpoint_TD_isoBaseNext <= dmaRspMux_vec_0[28 : 16];
          end
        end
        if((endpoint_TD_isoFrameNumber == 3'b001))begin
          if((ioDma_rsp_valid && (dmaReadCtx_counter == 4'b0100)))begin
            endpoint_TD_isoBase <= dmaRspMux_vec_0[28 : 16];
          end
          if((ioDma_rsp_valid && (dmaReadCtx_counter == 4'b0101)))begin
            endpoint_TD_isoBaseNext <= dmaRspMux_vec_0[12 : 0];
          end
        end
        if((endpoint_TD_isoFrameNumber == 3'b010))begin
          if((ioDma_rsp_valid && (dmaReadCtx_counter == 4'b0101)))begin
            endpoint_TD_isoBase <= dmaRspMux_vec_0[12 : 0];
          end
          if((ioDma_rsp_valid && (dmaReadCtx_counter == 4'b0101)))begin
            endpoint_TD_isoBaseNext <= dmaRspMux_vec_0[28 : 16];
          end
        end
        if((endpoint_TD_isoFrameNumber == 3'b011))begin
          if((ioDma_rsp_valid && (dmaReadCtx_counter == 4'b0101)))begin
            endpoint_TD_isoBase <= dmaRspMux_vec_0[28 : 16];
          end
          if((ioDma_rsp_valid && (dmaReadCtx_counter == 4'b0110)))begin
            endpoint_TD_isoBaseNext <= dmaRspMux_vec_0[12 : 0];
          end
        end
        if((endpoint_TD_isoFrameNumber == 3'b100))begin
          if((ioDma_rsp_valid && (dmaReadCtx_counter == 4'b0110)))begin
            endpoint_TD_isoBase <= dmaRspMux_vec_0[12 : 0];
          end
          if((ioDma_rsp_valid && (dmaReadCtx_counter == 4'b0110)))begin
            endpoint_TD_isoBaseNext <= dmaRspMux_vec_0[28 : 16];
          end
        end
        if((endpoint_TD_isoFrameNumber == 3'b101))begin
          if((ioDma_rsp_valid && (dmaReadCtx_counter == 4'b0110)))begin
            endpoint_TD_isoBase <= dmaRspMux_vec_0[28 : 16];
          end
          if((ioDma_rsp_valid && (dmaReadCtx_counter == 4'b0111)))begin
            endpoint_TD_isoBaseNext <= dmaRspMux_vec_0[12 : 0];
          end
        end
        if((endpoint_TD_isoFrameNumber == 3'b110))begin
          if((ioDma_rsp_valid && (dmaReadCtx_counter == 4'b0111)))begin
            endpoint_TD_isoBase <= dmaRspMux_vec_0[12 : 0];
          end
          if((ioDma_rsp_valid && (dmaReadCtx_counter == 4'b0111)))begin
            endpoint_TD_isoBaseNext <= dmaRspMux_vec_0[28 : 16];
          end
        end
        if((endpoint_TD_isoFrameNumber == 3'b111))begin
          if((ioDma_rsp_valid && (dmaReadCtx_counter == 4'b0111)))begin
            endpoint_TD_isoBase <= dmaRspMux_vec_0[28 : 16];
          end
        end
        if(endpoint_TD_isoLast)begin
          endpoint_TD_isoBaseNext <= {(! endpoint_TD_isSinglePage),endpoint_TD_BE[11 : 0]};
        end
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TD_ANALYSE : begin
        case(endpoint_flowType)
          `FlowType_binary_sequential_CONTROL : begin
            reg_hcCommandStatus_CLF <= 1'b1;
          end
          `FlowType_binary_sequential_BULK : begin
            reg_hcCommandStatus_BLF <= 1'b1;
          end
          default : begin
          end
        endcase
        endpoint_dmaLogic_byteCtx_counter <= endpoint_TD_firstOffset;
        endpoint_currentAddress <= {1'd0, endpoint_TD_firstOffset};
        endpoint_lastAddress <= _zz_209[12:0];
        endpoint_zeroLength <= (endpoint_ED_F ? endpoint_TD_isoZero : (endpoint_TD_CBP == 32'h0));
        endpoint_dataPhase <= (endpoint_ED_F ? 1'b0 : (endpoint_TD_T[1] ? endpoint_TD_T[0] : endpoint_ED_C));
        if(endpoint_ED_F)begin
          if(endpoint_TD_isoOverrun)begin
            endpoint_TD_retire <= 1'b1;
          end
        end
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TD_CHECK_TIME : begin
        if(endpoint_timeCheck)begin
          endpoint_status_1 <= `endpoint_Status_binary_sequential_FRAME_TIME;
        end
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_BUFFER_READ : begin
        if(_zz_115)begin
          if(endpoint_timeCheck)begin
            endpoint_status_1 <= `endpoint_Status_binary_sequential_FRAME_TIME;
          end
        end
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_TOKEN : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_DATA_TX : begin
        if(dataTx_wantExit)begin
          if(endpoint_ED_F)begin
            endpoint_TD_words_0[31 : 28] <= 4'b0000;
          end
        end
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_DATA_RX : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_DATA_RX_VALIDATE : begin
        endpoint_TD_words_0[31 : 28] <= 4'b0000;
        if(dataRx_notResponding)begin
          endpoint_TD_words_0[31 : 28] <= 4'b0101;
        end else begin
          if(dataRx_stuffingError)begin
            endpoint_TD_words_0[31 : 28] <= 4'b0010;
          end else begin
            if(dataRx_pidError)begin
              endpoint_TD_words_0[31 : 28] <= 4'b0110;
            end else begin
              if(endpoint_ED_F)begin
                case(dataRx_pid)
                  4'b1110, 4'b1010 : begin
                    endpoint_TD_words_0[31 : 28] <= 4'b0100;
                  end
                  4'b0011, 4'b1011 : begin
                  end
                  default : begin
                    endpoint_TD_words_0[31 : 28] <= 4'b0111;
                  end
                endcase
              end else begin
                case(dataRx_pid)
                  4'b1010 : begin
                    endpoint_TD_noUpdate <= 1'b1;
                  end
                  4'b1110 : begin
                    endpoint_TD_words_0[31 : 28] <= 4'b0100;
                  end
                  4'b0011, 4'b1011 : begin
                    if(_zz_116)begin
                      endpoint_TD_words_0[31 : 28] <= 4'b0011;
                    end
                  end
                  default : begin
                    endpoint_TD_words_0[31 : 28] <= 4'b0111;
                  end
                endcase
              end
              if(_zz_62)begin
                if(dataRx_crcError)begin
                  endpoint_TD_words_0[31 : 28] <= 4'b0001;
                end else begin
                  if(endpoint_dmaLogic_underflowError)begin
                    endpoint_TD_words_0[31 : 28] <= 4'b1001;
                  end else begin
                    if(endpoint_dmaLogic_overflow)begin
                      endpoint_TD_words_0[31 : 28] <= 4'b1000;
                    end
                  end
                end
              end
            end
          end
        end
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ACK_RX : begin
        if(io_phy_rx_flow_valid)begin
          endpoint_ackRxFired <= 1'b1;
          endpoint_ackRxPid <= io_phy_rx_flow_payload_data[3 : 0];
          if(io_phy_rx_flow_payload_stuffingError)begin
            endpoint_ackRxStuffing <= 1'b1;
          end
          if(((! rxPidOk) || endpoint_ackRxFired))begin
            endpoint_ackRxPidFailure <= 1'b1;
          end
        end
        if(io_phy_rx_active)begin
          endpoint_ackRxActivated <= 1'b1;
        end
        if(_zz_117)begin
          if(_zz_118)begin
            endpoint_TD_words_0[31 : 28] <= 4'b0110;
          end else begin
            if(endpoint_ackRxStuffing)begin
              endpoint_TD_words_0[31 : 28] <= 4'b0010;
            end else begin
              if(endpoint_ackRxPidFailure)begin
                endpoint_TD_words_0[31 : 28] <= 4'b0110;
              end else begin
                case(endpoint_ackRxPid)
                  4'b0010 : begin
                    endpoint_TD_words_0[31 : 28] <= 4'b0000;
                  end
                  4'b1010 : begin
                  end
                  4'b1110 : begin
                    endpoint_TD_words_0[31 : 28] <= 4'b0100;
                  end
                  default : begin
                    endpoint_TD_words_0[31 : 28] <= 4'b0111;
                  end
                endcase
              end
            end
          end
        end
        if(rxTimer_rxTimeout)begin
          endpoint_TD_words_0[31 : 28] <= 4'b0101;
        end
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ACK_TX_0 : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ACK_TX_1 : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ACK_TX_EOP : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_DATA_RX_WAIT_DMA : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_UPDATE_TD_PROCESS : begin
        if(endpoint_ED_F)begin
          if(endpoint_TD_isoLast)begin
            endpoint_TD_retire <= 1'b1;
          end
        end else begin
          endpoint_TD_words_0[27 : 26] <= 2'b00;
          case(endpoint_TD_CC)
            4'b0000 : begin
              if(((endpoint_dmaLogic_underflow || (_zz_213 < endpoint_currentAddress)) || endpoint_zeroLength))begin
                endpoint_TD_retire <= 1'b1;
              end
              endpoint_TD_dataPhaseUpdate <= 1'b1;
              endpoint_TD_upateCBP <= 1'b1;
            end
            4'b1001 : begin
              endpoint_TD_retire <= 1'b1;
              endpoint_TD_dataPhaseUpdate <= 1'b1;
              endpoint_TD_upateCBP <= 1'b1;
            end
            4'b1000 : begin
              endpoint_TD_retire <= 1'b1;
              endpoint_TD_dataPhaseUpdate <= 1'b1;
            end
            4'b0010, 4'b0001, 4'b0110, 4'b0101, 4'b0111, 4'b0011 : begin
              endpoint_TD_words_0[27 : 26] <= _zz_214;
              if((endpoint_TD_EC != 2'b10))begin
                endpoint_TD_words_0[31 : 28] <= 4'b0000;
              end else begin
                endpoint_TD_retire <= 1'b1;
              end
            end
            default : begin
              endpoint_TD_retire <= 1'b1;
            end
          endcase
          if(endpoint_TD_noUpdate)begin
            endpoint_TD_retire <= 1'b0;
          end
        end
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_UPDATE_TD_CMD : begin
        endpoint_ED_words_2[0] <= ((! endpoint_ED_F) && (endpoint_TD_CC != 4'b0000));
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_UPDATE_ED_CMD : begin
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_UPDATE_SYNC : begin
        if(dmaCtx_pendingEmpty)begin
          if(endpoint_TD_retire)begin
            reg_hcDoneHead_DH_reg <= endpoint_ED_headP;
          end
        end
      end
      `endpoint_enumDefinition_binary_sequential_endpoint_ABORD : begin
      end
      default : begin
      end
    endcase
    if(((endpoint_stateReg == `endpoint_enumDefinition_binary_sequential_endpoint_BOOT) && (! (endpoint_stateNext == `endpoint_enumDefinition_binary_sequential_endpoint_BOOT))))begin
      endpoint_status_1 <= `endpoint_Status_binary_sequential_OK;
    end
    if(_zz_105)begin
      endpoint_ackRxFired <= 1'b0;
      endpoint_ackRxActivated <= 1'b0;
      endpoint_ackRxPidFailure <= 1'b0;
      endpoint_ackRxStuffing <= 1'b0;
    end
    case(endpoint_dmaLogic_stateReg)
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_INIT : begin
        endpoint_dmaLogic_underflow <= 1'b0;
        endpoint_dmaLogic_overflow <= 1'b0;
      end
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_TO_USB : begin
      end
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_FROM_USB : begin
        if(dataRx_wantExit)begin
          endpoint_dmaLogic_underflow <= _zz_64;
          endpoint_dmaLogic_overflow <= ((! _zz_64) && (_zz_223 != endpoint_transactionSize));
          if(endpoint_zeroLength)begin
            endpoint_dmaLogic_underflow <= 1'b0;
            endpoint_dmaLogic_overflow <= (endpoint_dmaLogic_fromUsbCounter != 11'h0);
          end
          if(_zz_64)begin
            endpoint_lastAddress <= _zz_224[12:0];
          end
        end
        if(dataRx_data_valid)begin
          endpoint_dmaLogic_fromUsbCounter <= (endpoint_dmaLogic_fromUsbCounter + _zz_228);
          if(_zz_65[0])begin
            endpoint_dmaLogic_buffer[7 : 0] <= dataRx_data_payload;
          end
          if(_zz_65[1])begin
            endpoint_dmaLogic_buffer[15 : 8] <= dataRx_data_payload;
          end
          if(_zz_65[2])begin
            endpoint_dmaLogic_buffer[23 : 16] <= dataRx_data_payload;
          end
          if(_zz_65[3])begin
            endpoint_dmaLogic_buffer[31 : 24] <= dataRx_data_payload;
          end
        end
      end
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_VALIDATION : begin
      end
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_CALC_CMD : begin
        endpoint_dmaLogic_length <= endpoint_dmaLogic_lengthCalc;
      end
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_READ_CMD : begin
        if(ioDma_cmd_ready)begin
          endpoint_currentAddress <= (_zz_229 + 14'h0001);
        end
      end
      `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_WRITE_CMD : begin
        if(ioDma_cmd_ready)begin
          if(endpoint_dmaLogic_beatLast)begin
            endpoint_currentAddress <= (_zz_231 + 14'h0001);
          end
        end
      end
      default : begin
      end
    endcase
    if(((! (endpoint_dmaLogic_stateReg == `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_FROM_USB)) && (endpoint_dmaLogic_stateNext == `endpoint_dmaLogic_enumDefinition_binary_sequential_endpoint_dmaLogic_FROM_USB)))begin
      endpoint_dmaLogic_fromUsbCounter <= 11'h0;
    end
    case(operational_stateReg)
      `operational_enumDefinition_binary_sequential_operational_SOF : begin
        if(sof_wantExit)begin
          if((operational_allowPeriodic && (! operational_periodicDone)))begin
            reg_hcInterrupt_SO_status <= 1'b1;
            reg_hcCommandStatus_SOC <= (reg_hcCommandStatus_SOC + 2'b01);
          end
          operational_allowBulk <= reg_hcControl_BLE;
          operational_allowControl <= reg_hcControl_CLE;
          operational_allowPeriodic <= reg_hcControl_PLE;
          operational_allowIsochronous <= reg_hcControl_IE;
          operational_periodicDone <= 1'b0;
          operational_periodicHeadFetched <= 1'b0;
          priority_bulk <= 1'b0;
          priority_counter <= 2'b00;
        end
      end
      `operational_enumDefinition_binary_sequential_operational_ARBITER : begin
        if(reg_hcControl_BLE)begin
          operational_allowBulk <= 1'b1;
        end
        if(reg_hcControl_CLE)begin
          operational_allowControl <= 1'b1;
        end
        if(! operational_askExit) begin
          if(! frame_limitHit) begin
            if(_zz_109)begin
              if(! _zz_112) begin
                if(reg_hcPeriodCurrentED_isZero)begin
                  operational_periodicDone <= 1'b1;
                end else begin
                  endpoint_flowType <= `FlowType_binary_sequential_PERIODIC;
                  endpoint_ED_address <= reg_hcPeriodCurrentED_PCED_address;
                end
              end
            end else begin
              if(priority_bulk)begin
                if(operational_allowBulk)begin
                  if(reg_hcBulkCurrentED_isZero)begin
                    if(reg_hcCommandStatus_BLF)begin
                      reg_hcBulkCurrentED_BCED_reg <= reg_hcBulkHeadED_BHED_reg;
                      reg_hcCommandStatus_BLF <= 1'b0;
                    end
                  end else begin
                    endpoint_flowType <= `FlowType_binary_sequential_BULK;
                    endpoint_ED_address <= reg_hcBulkCurrentED_BCED_address;
                  end
                end
              end else begin
                if(operational_allowControl)begin
                  if(reg_hcControlCurrentED_isZero)begin
                    if(reg_hcCommandStatus_CLF)begin
                      reg_hcControlCurrentED_CCED_reg <= reg_hcControlHeadED_CHED_reg;
                      reg_hcCommandStatus_CLF <= 1'b0;
                    end
                  end else begin
                    endpoint_flowType <= `FlowType_binary_sequential_CONTROL;
                    endpoint_ED_address <= reg_hcControlCurrentED_CCED_address;
                  end
                end
              end
            end
          end
        end
      end
      `operational_enumDefinition_binary_sequential_operational_END_POINT : begin
      end
      `operational_enumDefinition_binary_sequential_operational_PERIODIC_HEAD_CMD : begin
      end
      `operational_enumDefinition_binary_sequential_operational_PERIODIC_HEAD_RSP : begin
        if(ioDma_rsp_valid)begin
          operational_periodicHeadFetched <= 1'b1;
          reg_hcPeriodCurrentED_PCED_reg <= dmaRspMux_data[31 : 4];
        end
      end
      `operational_enumDefinition_binary_sequential_operational_WAIT_SOF : begin
      end
      default : begin
      end
    endcase
    if(_zz_110)begin
      operational_allowPeriodic <= 1'b0;
    end
    case(hc_stateReg)
      `hc_enumDefinition_binary_sequential_hc_RESET : begin
      end
      `hc_enumDefinition_binary_sequential_hc_RESUME : begin
      end
      `hc_enumDefinition_binary_sequential_hc_OPERATIONAL : begin
      end
      `hc_enumDefinition_binary_sequential_hc_SUSPEND : begin
        if(_zz_119)begin
          reg_hcInterrupt_RD_status <= 1'b1;
        end
      end
      `hc_enumDefinition_binary_sequential_hc_ANY_TO_RESET : begin
      end
      `hc_enumDefinition_binary_sequential_hc_ANY_TO_SUSPEND : begin
      end
      default : begin
      end
    endcase
  end

  always @ (posedge ctrl_clk or posedge ctrl_reset) begin
    if (ctrl_reset) begin
      ioDma_cmd_payload_first <= 1'b1;
    end else begin
      if((ioDma_cmd_valid && ioDma_cmd_ready))begin
        ioDma_cmd_payload_first <= ioDma_cmd_payload_last;
      end
    end
  end


endmodule

module UsbOhciWishbone_WishboneToBmb (
  input               io_input_CYC,
  input               io_input_STB,
  output              io_input_ACK,
  input               io_input_WE,
  input      [9:0]    io_input_ADR,
  output     [31:0]   io_input_DAT_MISO,
  input      [31:0]   io_input_DAT_MOSI,
  input      [3:0]    io_input_SEL,
  output              io_output_cmd_valid,
  input               io_output_cmd_ready,
  output              io_output_cmd_payload_last,
  output     [0:0]    io_output_cmd_payload_fragment_opcode,
  output     [11:0]   io_output_cmd_payload_fragment_address,
  output     [1:0]    io_output_cmd_payload_fragment_length,
  output     [31:0]   io_output_cmd_payload_fragment_data,
  output     [3:0]    io_output_cmd_payload_fragment_mask,
  input               io_output_rsp_valid,
  output              io_output_rsp_ready,
  input               io_output_rsp_payload_last,
  input      [0:0]    io_output_rsp_payload_fragment_opcode,
  input      [31:0]   io_output_rsp_payload_fragment_data,
  input               ctrl_clk,
  input               ctrl_reset
);
  reg                 _zz_1;

  assign io_output_cmd_payload_fragment_address = ({2'd0,io_input_ADR} <<< 2);
  assign io_output_cmd_payload_fragment_opcode = (io_input_WE ? 1'b1 : 1'b0);
  assign io_output_cmd_payload_fragment_data = io_input_DAT_MOSI;
  assign io_output_cmd_payload_fragment_mask = io_input_SEL;
  assign io_output_cmd_payload_fragment_length = 2'b11;
  assign io_output_cmd_payload_last = 1'b1;
  assign io_output_cmd_valid = ((io_input_CYC && io_input_STB) && (! _zz_1));
  assign io_input_ACK = (io_output_rsp_valid && io_output_rsp_ready);
  assign io_input_DAT_MISO = io_output_rsp_payload_fragment_data;
  assign io_output_rsp_ready = 1'b1;
  always @ (posedge ctrl_clk or posedge ctrl_reset) begin
    if (ctrl_reset) begin
      _zz_1 <= 1'b0;
    end else begin
      if((io_output_cmd_valid && io_output_cmd_ready))begin
        _zz_1 <= 1'b1;
      end
      if((io_output_rsp_valid && io_output_rsp_ready))begin
        _zz_1 <= 1'b0;
      end
    end
  end


endmodule

module UsbOhciWishbone_BmbToWishbone (
  input               io_input_cmd_valid,
  output              io_input_cmd_ready,
  input               io_input_cmd_payload_last,
  input      [0:0]    io_input_cmd_payload_fragment_opcode,
  input      [31:0]   io_input_cmd_payload_fragment_address,
  input      [5:0]    io_input_cmd_payload_fragment_length,
  input      [31:0]   io_input_cmd_payload_fragment_data,
  input      [3:0]    io_input_cmd_payload_fragment_mask,
  output              io_input_rsp_valid,
  input               io_input_rsp_ready,
  output              io_input_rsp_payload_last,
  output     [0:0]    io_input_rsp_payload_fragment_opcode,
  output     [31:0]   io_input_rsp_payload_fragment_data,
  output              io_output_CYC,
  output              io_output_STB,
  input               io_output_ACK,
  output              io_output_WE,
  output     [29:0]   io_output_ADR,
  input      [31:0]   io_output_DAT_MISO,
  output     [31:0]   io_output_DAT_MOSI,
  output     [3:0]    io_output_SEL,
  input               io_output_ERR,
  output     [2:0]    io_output_CTI,
  output     [1:0]    io_output_BTE,
  input               ctrl_clk,
  input               ctrl_reset
);
  wire                _zz_2;
  wire       [11:0]   _zz_3;
  wire       [11:0]   _zz_4;
  wire       [11:0]   _zz_5;
  wire       [5:0]    _zz_6;
  wire       [11:0]   _zz_7;
  wire                inputCmd_valid;
  wire                inputCmd_ready;
  wire                inputCmd_payload_last;
  wire       [0:0]    inputCmd_payload_fragment_opcode;
  wire       [31:0]   inputCmd_payload_fragment_address;
  wire       [5:0]    inputCmd_payload_fragment_length;
  wire       [31:0]   inputCmd_payload_fragment_data;
  wire       [3:0]    inputCmd_payload_fragment_mask;
  reg                 inputCmd_regs_valid;
  reg                 inputCmd_regs_ready;
  reg                 inputCmd_regs_payload_last;
  reg        [0:0]    inputCmd_regs_payload_fragment_opcode;
  reg        [31:0]   inputCmd_regs_payload_fragment_address;
  reg        [5:0]    inputCmd_regs_payload_fragment_length;
  reg        [31:0]   inputCmd_regs_payload_fragment_data;
  reg        [3:0]    inputCmd_regs_payload_fragment_mask;
  reg        [3:0]    beatCounter;
  wire                beatLast;
  reg                 inputCmd_payload_first;
  reg                 _zz_1;
  reg        [31:0]   io_output_DAT_MISO_regNext;
  reg                 beatLast_regNext;

  assign _zz_2 = (! inputCmd_regs_valid);
  assign _zz_3 = (_zz_5 + _zz_7);
  assign _zz_4 = inputCmd_payload_fragment_address[11 : 0];
  assign _zz_5 = _zz_4;
  assign _zz_6 = ({2'd0,beatCounter} <<< 2);
  assign _zz_7 = {6'd0, _zz_6};
  assign inputCmd_valid = inputCmd_regs_valid;
  assign inputCmd_payload_last = inputCmd_regs_payload_last;
  assign inputCmd_payload_fragment_opcode = inputCmd_regs_payload_fragment_opcode;
  assign inputCmd_payload_fragment_address = inputCmd_regs_payload_fragment_address;
  assign inputCmd_payload_fragment_length = inputCmd_regs_payload_fragment_length;
  assign inputCmd_payload_fragment_data = inputCmd_regs_payload_fragment_data;
  assign inputCmd_payload_fragment_mask = inputCmd_regs_payload_fragment_mask;
  assign io_input_cmd_ready = inputCmd_regs_ready;
  assign beatLast = (beatCounter == inputCmd_payload_fragment_length[5 : 2]);
  assign io_output_ADR = ({inputCmd_payload_fragment_address[31 : 12],_zz_3} >>> 2);
  assign io_output_CTI = (inputCmd_payload_last ? (inputCmd_payload_first ? 3'b000 : 3'b111) : 3'b010);
  assign io_output_BTE = 2'b00;
  assign io_output_SEL = ((inputCmd_payload_fragment_opcode == 1'b1) ? inputCmd_payload_fragment_mask : 4'b1111);
  assign io_output_WE = (inputCmd_payload_fragment_opcode == 1'b1);
  assign io_output_DAT_MOSI = inputCmd_payload_fragment_data;
  assign inputCmd_ready = (io_output_ACK && ((inputCmd_payload_fragment_opcode == 1'b1) || beatLast));
  assign io_output_CYC = inputCmd_valid;
  assign io_output_STB = inputCmd_valid;
  assign io_input_rsp_valid = _zz_1;
  assign io_input_rsp_payload_fragment_data = io_output_DAT_MISO_regNext;
  assign io_input_rsp_payload_last = beatLast_regNext;
  assign io_input_rsp_payload_fragment_opcode = 1'b0;
  always @ (posedge ctrl_clk or posedge ctrl_reset) begin
    if (ctrl_reset) begin
      inputCmd_regs_valid <= 1'b0;
      inputCmd_regs_ready <= 1'b1;
      beatCounter <= 4'b0000;
      inputCmd_payload_first <= 1'b1;
      _zz_1 <= 1'b0;
    end else begin
      if(_zz_2)begin
        inputCmd_regs_valid <= io_input_cmd_valid;
        inputCmd_regs_ready <= (! io_input_cmd_valid);
      end else begin
        inputCmd_regs_valid <= (! inputCmd_ready);
        inputCmd_regs_ready <= inputCmd_ready;
      end
      if((inputCmd_valid && io_output_ACK))begin
        beatCounter <= (beatCounter + 4'b0001);
        if((inputCmd_ready && inputCmd_payload_last))begin
          beatCounter <= 4'b0000;
        end
      end
      if((inputCmd_valid && inputCmd_ready))begin
        inputCmd_payload_first <= inputCmd_payload_last;
      end
      _zz_1 <= ((inputCmd_valid && io_output_ACK) && ((inputCmd_payload_fragment_opcode == 1'b0) || beatLast));
    end
  end

  always @ (posedge ctrl_clk) begin
    if(_zz_2)begin
      inputCmd_regs_payload_last <= io_input_cmd_payload_last;
      inputCmd_regs_payload_fragment_opcode <= io_input_cmd_payload_fragment_opcode;
      inputCmd_regs_payload_fragment_address <= io_input_cmd_payload_fragment_address;
      inputCmd_regs_payload_fragment_length <= io_input_cmd_payload_fragment_length;
      inputCmd_regs_payload_fragment_data <= io_input_cmd_payload_fragment_data;
      inputCmd_regs_payload_fragment_mask <= io_input_cmd_payload_fragment_mask;
    end
    io_output_DAT_MISO_regNext <= io_output_DAT_MISO;
    beatLast_regNext <= beatLast;
  end


endmodule

//UsbOhciWishbone_StreamCCByToggleWithoutBuffer replaced by UsbOhciWishbone_StreamCCByToggleWithoutBuffer

//UsbOhciWishbone_StreamCCByToggleWithoutBuffer replaced by UsbOhciWishbone_StreamCCByToggleWithoutBuffer

//UsbOhciWishbone_StreamCCByToggleWithoutBuffer replaced by UsbOhciWishbone_StreamCCByToggleWithoutBuffer

module UsbOhciWishbone_StreamCCByToggleWithoutBuffer (
  input               io_input_valid,
  output reg          io_input_ready,
  output              io_output_valid,
  input               io_output_ready,
  input               ctrl_clk,
  input               ctrl_reset,
  input               phy_clk,
  input               phy_reset
);
  wire                outHitSignal_buffercc_io_dataOut;
  wire                pushArea_target_buffercc_io_dataOut;
  wire                _zz_1;
  wire                _zz_2;
  wire                outHitSignal;
  wire                pushArea_hit;
  reg                 pushArea_target;
  reg                 pushArea_busy;
  wire                popArea_target;
  reg                 popArea_hit;
  wire                popArea_stream_valid;
  wire                popArea_stream_ready;

  assign _zz_1 = (! pushArea_busy);
  assign _zz_2 = (pushArea_hit == pushArea_target);
  UsbOhciWishbone_BufferCC outHitSignal_buffercc (
    .io_dataIn     (outHitSignal                      ), //i
    .io_dataOut    (outHitSignal_buffercc_io_dataOut  ), //o
    .ctrl_clk      (ctrl_clk                          ), //i
    .ctrl_reset    (ctrl_reset                        )  //i
  );
  UsbOhciWishbone_BufferCC_9 pushArea_target_buffercc (
    .io_dataIn     (pushArea_target                      ), //i
    .io_dataOut    (pushArea_target_buffercc_io_dataOut  ), //o
    .phy_clk       (phy_clk                              ), //i
    .phy_reset     (phy_reset                            )  //i
  );
  assign pushArea_hit = outHitSignal_buffercc_io_dataOut;
  always @ (*) begin
    io_input_ready = 1'b0;
    if(! _zz_1) begin
      if(_zz_2)begin
        io_input_ready = 1'b1;
      end
    end
  end

  assign popArea_target = pushArea_target_buffercc_io_dataOut;
  assign outHitSignal = popArea_hit;
  assign popArea_stream_valid = (popArea_target != popArea_hit);
  assign io_output_valid = popArea_stream_valid;
  assign popArea_stream_ready = io_output_ready;
  always @ (posedge ctrl_clk or posedge ctrl_reset) begin
    if (ctrl_reset) begin
      pushArea_target <= 1'b0;
      pushArea_busy <= 1'b0;
    end else begin
      if(_zz_1)begin
        if(io_input_valid)begin
          pushArea_target <= (! pushArea_target);
          pushArea_busy <= 1'b1;
        end
      end else begin
        if(_zz_2)begin
          pushArea_busy <= 1'b0;
        end
      end
    end
  end

  always @ (posedge phy_clk or posedge phy_reset) begin
    if (phy_reset) begin
      popArea_hit <= 1'b0;
    end else begin
      if((popArea_stream_valid && popArea_stream_ready))begin
        popArea_hit <= popArea_target;
      end
    end
  end


endmodule

//UsbOhciWishbone_PulseCCByToggle replaced by UsbOhciWishbone_PulseCCByToggle

//UsbOhciWishbone_PulseCCByToggle replaced by UsbOhciWishbone_PulseCCByToggle

//UsbOhciWishbone_PulseCCByToggle replaced by UsbOhciWishbone_PulseCCByToggle

//UsbOhciWishbone_BufferCC_19 replaced by UsbOhciWishbone_BufferCC_19

//UsbOhciWishbone_BufferCC_19 replaced by UsbOhciWishbone_BufferCC_19

//UsbOhciWishbone_BufferCC_16 replaced by UsbOhciWishbone_BufferCC_16

//UsbOhciWishbone_BufferCC_16 replaced by UsbOhciWishbone_BufferCC_16

//UsbOhciWishbone_PulseCCByToggle replaced by UsbOhciWishbone_PulseCCByToggle

//UsbOhciWishbone_BufferCC_19 replaced by UsbOhciWishbone_BufferCC_19

module UsbOhciWishbone_FlowCCByToggle (
  input               io_input_valid,
  input               io_input_payload_stuffingError,
  input      [7:0]    io_input_payload_data,
  output              io_output_valid,
  output              io_output_payload_stuffingError,
  output     [7:0]    io_output_payload_data,
  input               phy_clk,
  input               phy_reset,
  input               ctrl_clk,
  input               ctrl_reset
);
  wire                inputArea_target_buffercc_io_dataOut;
  wire                outHitSignal;
  reg                 inputArea_target;
  reg                 inputArea_data_stuffingError;
  reg        [7:0]    inputArea_data_data;
  wire                outputArea_target;
  reg                 outputArea_hit;
  wire                outputArea_flow_valid;
  wire                outputArea_flow_payload_stuffingError;
  wire       [7:0]    outputArea_flow_payload_data;
  reg                 outputArea_flow_regNext_valid;
  reg                 outputArea_flow_regNext_payload_stuffingError;
  reg        [7:0]    outputArea_flow_regNext_payload_data;

  UsbOhciWishbone_BufferCC inputArea_target_buffercc (
    .io_dataIn     (inputArea_target                      ), //i
    .io_dataOut    (inputArea_target_buffercc_io_dataOut  ), //o
    .ctrl_clk      (ctrl_clk                              ), //i
    .ctrl_reset    (ctrl_reset                            )  //i
  );
  assign outputArea_target = inputArea_target_buffercc_io_dataOut;
  assign outputArea_flow_valid = (outputArea_target != outputArea_hit);
  assign outputArea_flow_payload_stuffingError = inputArea_data_stuffingError;
  assign outputArea_flow_payload_data = inputArea_data_data;
  assign io_output_valid = outputArea_flow_regNext_valid;
  assign io_output_payload_stuffingError = outputArea_flow_regNext_payload_stuffingError;
  assign io_output_payload_data = outputArea_flow_regNext_payload_data;
  always @ (posedge phy_clk or posedge phy_reset) begin
    if (phy_reset) begin
      inputArea_target <= 1'b0;
    end else begin
      if(io_input_valid)begin
        inputArea_target <= (! inputArea_target);
      end
    end
  end

  always @ (posedge phy_clk) begin
    if(io_input_valid)begin
      inputArea_data_stuffingError <= io_input_payload_stuffingError;
      inputArea_data_data <= io_input_payload_data;
    end
  end

  always @ (posedge ctrl_clk or posedge ctrl_reset) begin
    if (ctrl_reset) begin
      outputArea_flow_regNext_valid <= 1'b0;
      outputArea_hit <= 1'b0;
    end else begin
      outputArea_hit <= outputArea_target;
      outputArea_flow_regNext_valid <= outputArea_flow_valid;
    end
  end

  always @ (posedge ctrl_clk) begin
    outputArea_flow_regNext_payload_stuffingError <= outputArea_flow_payload_stuffingError;
    outputArea_flow_regNext_payload_data <= outputArea_flow_payload_data;
  end


endmodule

module UsbOhciWishbone_PulseCCByToggle (
  input               io_pulseIn,
  output              io_pulseOut,
  input               phy_clk,
  input               phy_reset,
  input               ctrl_clk,
  input               ctrl_reset
);
  wire                inArea_target_buffercc_io_dataOut;
  reg                 inArea_target;
  wire                outArea_target;
  reg                 outArea_hit;

  UsbOhciWishbone_BufferCC inArea_target_buffercc (
    .io_dataIn     (inArea_target                      ), //i
    .io_dataOut    (inArea_target_buffercc_io_dataOut  ), //o
    .ctrl_clk      (ctrl_clk                           ), //i
    .ctrl_reset    (ctrl_reset                         )  //i
  );
  assign outArea_target = inArea_target_buffercc_io_dataOut;
  assign io_pulseOut = (outArea_target != outArea_hit);
  always @ (posedge phy_clk or posedge phy_reset) begin
    if (phy_reset) begin
      inArea_target <= 1'b0;
    end else begin
      if(io_pulseIn)begin
        inArea_target <= (! inArea_target);
      end
    end
  end

  always @ (posedge ctrl_clk or posedge ctrl_reset) begin
    if (ctrl_reset) begin
      outArea_hit <= 1'b0;
    end else begin
      if((outArea_target != outArea_hit))begin
        outArea_hit <= (! outArea_hit);
      end
    end
  end


endmodule

module UsbOhciWishbone_StreamCCByToggle (
  input               io_input_valid,
  output reg          io_input_ready,
  input               io_input_payload_last,
  input      [7:0]    io_input_payload_fragment,
  output              io_output_valid,
  input               io_output_ready,
  output              io_output_payload_last,
  output     [7:0]    io_output_payload_fragment,
  input               ctrl_clk,
  input               ctrl_reset,
  input               phy_clk,
  input               phy_reset
);
  wire                outHitSignal_buffercc_io_dataOut;
  wire                pushArea_target_buffercc_io_dataOut;
  wire                _zz_1;
  wire                outHitSignal;
  wire                pushArea_hit;
  reg                 pushArea_target;
  reg                 pushArea_data_last;
  reg        [7:0]    pushArea_data_fragment;
  wire                popArea_target;
  reg                 popArea_hit;
  wire                popArea_stream_valid;
  wire                popArea_stream_ready;
  wire                popArea_stream_payload_last;
  wire       [7:0]    popArea_stream_payload_fragment;
  wire                popArea_stream_m2sPipe_valid;
  wire                popArea_stream_m2sPipe_ready;
  wire                popArea_stream_m2sPipe_payload_last;
  wire       [7:0]    popArea_stream_m2sPipe_payload_fragment;
  reg                 popArea_stream_m2sPipe_rValid;
  reg                 popArea_stream_m2sPipe_rData_last;
  reg        [7:0]    popArea_stream_m2sPipe_rData_fragment;

  assign _zz_1 = (io_input_valid && (pushArea_hit == pushArea_target));
  UsbOhciWishbone_BufferCC outHitSignal_buffercc (
    .io_dataIn     (outHitSignal                      ), //i
    .io_dataOut    (outHitSignal_buffercc_io_dataOut  ), //o
    .ctrl_clk      (ctrl_clk                          ), //i
    .ctrl_reset    (ctrl_reset                        )  //i
  );
  UsbOhciWishbone_BufferCC_1 pushArea_target_buffercc (
    .io_dataIn     (pushArea_target                      ), //i
    .io_dataOut    (pushArea_target_buffercc_io_dataOut  ), //o
    .phy_clk       (phy_clk                              ), //i
    .phy_reset     (phy_reset                            )  //i
  );
  assign pushArea_hit = outHitSignal_buffercc_io_dataOut;
  always @ (*) begin
    io_input_ready = 1'b0;
    if(_zz_1)begin
      io_input_ready = 1'b1;
    end
  end

  assign popArea_target = pushArea_target_buffercc_io_dataOut;
  assign outHitSignal = popArea_hit;
  assign popArea_stream_valid = (popArea_target != popArea_hit);
  assign popArea_stream_payload_last = pushArea_data_last;
  assign popArea_stream_payload_fragment = pushArea_data_fragment;
  assign popArea_stream_ready = ((1'b1 && (! popArea_stream_m2sPipe_valid)) || popArea_stream_m2sPipe_ready);
  assign popArea_stream_m2sPipe_valid = popArea_stream_m2sPipe_rValid;
  assign popArea_stream_m2sPipe_payload_last = popArea_stream_m2sPipe_rData_last;
  assign popArea_stream_m2sPipe_payload_fragment = popArea_stream_m2sPipe_rData_fragment;
  assign io_output_valid = popArea_stream_m2sPipe_valid;
  assign popArea_stream_m2sPipe_ready = io_output_ready;
  assign io_output_payload_last = popArea_stream_m2sPipe_payload_last;
  assign io_output_payload_fragment = popArea_stream_m2sPipe_payload_fragment;
  always @ (posedge ctrl_clk or posedge ctrl_reset) begin
    if (ctrl_reset) begin
      pushArea_target <= 1'b0;
    end else begin
      if(_zz_1)begin
        pushArea_target <= (! pushArea_target);
      end
    end
  end

  always @ (posedge ctrl_clk) begin
    if(_zz_1)begin
      pushArea_data_last <= io_input_payload_last;
      pushArea_data_fragment <= io_input_payload_fragment;
    end
  end

  always @ (posedge phy_clk or posedge phy_reset) begin
    if (phy_reset) begin
      popArea_hit <= 1'b0;
      popArea_stream_m2sPipe_rValid <= 1'b0;
    end else begin
      if((popArea_stream_valid && popArea_stream_ready))begin
        popArea_hit <= (! popArea_hit);
      end
      if(popArea_stream_ready)begin
        popArea_stream_m2sPipe_rValid <= popArea_stream_valid;
      end
    end
  end

  always @ (posedge phy_clk) begin
    if(popArea_stream_ready)begin
      popArea_stream_m2sPipe_rData_last <= popArea_stream_payload_last;
      popArea_stream_m2sPipe_rData_fragment <= popArea_stream_payload_fragment;
    end
  end


endmodule

module UsbOhciWishbone_BufferCC_19 (
  input               io_dataIn,
  output              io_dataOut,
  input               ctrl_clk,
  input               ctrl_reset
);
  (* async_reg = "true" *) reg                 buffers_0;
  (* async_reg = "true" *) reg                 buffers_1;

  assign io_dataOut = buffers_1;
  always @ (posedge ctrl_clk) begin
    buffers_0 <= io_dataIn;
    buffers_1 <= buffers_0;
  end


endmodule

//UsbOhciWishbone_BufferCC_16 replaced by UsbOhciWishbone_BufferCC_16

//UsbOhciWishbone_BufferCC_16 replaced by UsbOhciWishbone_BufferCC_16

module UsbOhciWishbone_BufferCC_16 (
  input               io_dataIn,
  output              io_dataOut,
  input               phy_clk,
  input               phy_reset
);
  (* async_reg = "true" *) reg                 buffers_0;
  (* async_reg = "true" *) reg                 buffers_1;

  assign io_dataOut = buffers_1;
  always @ (posedge phy_clk) begin
    buffers_0 <= io_dataIn;
    buffers_1 <= buffers_0;
  end


endmodule

module UsbOhciWishbone_UsbLsFsPhyFilter (
  input               io_lowSpeed,
  input               io_usb_dp,
  input               io_usb_dm,
  output              io_filtred_dp,
  output              io_filtred_dm,
  output              io_filtred_d,
  output              io_filtred_se0,
  output              io_filtred_sample,
  input               phy_clk,
  input               phy_reset
);
  wire       [4:0]    _zz_1;
  reg                 timer_clear;
  reg        [4:0]    timer_counter;
  wire       [4:0]    timer_counterLimit;
  wire       [3:0]    timer_sampleAt;
  wire                timer_sampleDo;
  reg                 io_usb_dp_regNext;
  reg                 io_usb_dm_regNext;

  assign _zz_1 = {1'd0, timer_sampleAt};
  always @ (*) begin
    timer_clear = 1'b0;
    if(((io_usb_dp ^ io_usb_dp_regNext) || (io_usb_dm ^ io_usb_dm_regNext)))begin
      timer_clear = 1'b1;
    end
  end

  assign timer_counterLimit = (io_lowSpeed ? 5'h1f : 5'h03);
  assign timer_sampleAt = (io_lowSpeed ? 4'b1110 : 4'b0000);
  assign timer_sampleDo = ((timer_counter == _zz_1) && (! timer_clear));
  assign io_filtred_dp = io_usb_dp;
  assign io_filtred_dm = io_usb_dm;
  assign io_filtred_d = io_usb_dp;
  assign io_filtred_sample = timer_sampleDo;
  assign io_filtred_se0 = ((! io_usb_dp) && (! io_usb_dm));
  always @ (posedge phy_clk) begin
    timer_counter <= (timer_counter + 5'h01);
    if(((timer_counter == timer_counterLimit) || timer_clear))begin
      timer_counter <= 5'h0;
    end
    io_usb_dp_regNext <= io_usb_dp;
    io_usb_dm_regNext <= io_usb_dm;
  end


endmodule

module UsbOhciWishbone_Crc_2 (
  input               io_flush,
  input               io_input_valid,
  input      [7:0]    io_input_payload,
  output     [15:0]   io_result,
  output     [15:0]   io_resultNext,
  input               ctrl_clk,
  input               ctrl_reset
);
  wire       [15:0]   _zz_1;
  wire       [15:0]   _zz_2;
  wire       [15:0]   _zz_3;
  wire       [15:0]   _zz_4;
  wire       [15:0]   _zz_5;
  wire       [15:0]   _zz_6;
  wire       [15:0]   _zz_7;
  wire       [15:0]   _zz_8;
  reg        [15:0]   state_8;
  reg        [15:0]   state_7;
  reg        [15:0]   state_6;
  reg        [15:0]   state_5;
  reg        [15:0]   state_4;
  reg        [15:0]   state_3;
  reg        [15:0]   state_2;
  reg        [15:0]   state_1;
  reg        [15:0]   state;
  wire       [15:0]   stateXor;
  wire       [15:0]   accXor;

  assign _zz_1 = (state <<< 1);
  assign _zz_2 = (state_1 <<< 1);
  assign _zz_3 = (state_2 <<< 1);
  assign _zz_4 = (state_3 <<< 1);
  assign _zz_5 = (state_4 <<< 1);
  assign _zz_6 = (state_5 <<< 1);
  assign _zz_7 = (state_6 <<< 1);
  assign _zz_8 = (state_7 <<< 1);
  always @ (*) begin
    state_8 = state_7;
    state_8 = (_zz_8 ^ ((io_input_payload[7] ^ state_7[15]) ? 16'h8005 : 16'h0));
  end

  always @ (*) begin
    state_7 = state_6;
    state_7 = (_zz_7 ^ ((io_input_payload[6] ^ state_6[15]) ? 16'h8005 : 16'h0));
  end

  always @ (*) begin
    state_6 = state_5;
    state_6 = (_zz_6 ^ ((io_input_payload[5] ^ state_5[15]) ? 16'h8005 : 16'h0));
  end

  always @ (*) begin
    state_5 = state_4;
    state_5 = (_zz_5 ^ ((io_input_payload[4] ^ state_4[15]) ? 16'h8005 : 16'h0));
  end

  always @ (*) begin
    state_4 = state_3;
    state_4 = (_zz_4 ^ ((io_input_payload[3] ^ state_3[15]) ? 16'h8005 : 16'h0));
  end

  always @ (*) begin
    state_3 = state_2;
    state_3 = (_zz_3 ^ ((io_input_payload[2] ^ state_2[15]) ? 16'h8005 : 16'h0));
  end

  always @ (*) begin
    state_2 = state_1;
    state_2 = (_zz_2 ^ ((io_input_payload[1] ^ state_1[15]) ? 16'h8005 : 16'h0));
  end

  always @ (*) begin
    state_1 = state;
    state_1 = (_zz_1 ^ ((io_input_payload[0] ^ state[15]) ? 16'h8005 : 16'h0));
  end

  assign stateXor = (state ^ 16'h0);
  assign accXor = (state_8 ^ 16'h0);
  assign io_result = stateXor;
  assign io_resultNext = accXor;
  always @ (posedge ctrl_clk or posedge ctrl_reset) begin
    if (ctrl_reset) begin
      state <= 16'hffff;
    end else begin
      if(io_input_valid)begin
        state <= state_8;
      end
      if(io_flush)begin
        state <= 16'hffff;
      end
    end
  end


endmodule

module UsbOhciWishbone_Crc_1 (
  input               io_flush,
  input               io_input_valid,
  input      [7:0]    io_input_payload,
  output     [15:0]   io_result,
  output     [15:0]   io_resultNext,
  input               ctrl_clk,
  input               ctrl_reset
);
  wire       [15:0]   _zz_1;
  wire       [15:0]   _zz_2;
  wire       [15:0]   _zz_3;
  wire       [15:0]   _zz_4;
  wire       [15:0]   _zz_5;
  wire       [15:0]   _zz_6;
  wire       [15:0]   _zz_7;
  wire       [15:0]   _zz_8;
  wire                _zz_9;
  wire       [0:0]    _zz_10;
  wire       [4:0]    _zz_11;
  wire                _zz_12;
  wire       [0:0]    _zz_13;
  wire       [4:0]    _zz_14;
  reg        [15:0]   state_8;
  reg        [15:0]   state_7;
  reg        [15:0]   state_6;
  reg        [15:0]   state_5;
  reg        [15:0]   state_4;
  reg        [15:0]   state_3;
  reg        [15:0]   state_2;
  reg        [15:0]   state_1;
  reg        [15:0]   state;
  wire       [15:0]   stateXor;
  wire       [15:0]   accXor;

  assign _zz_1 = (state <<< 1);
  assign _zz_2 = (state_1 <<< 1);
  assign _zz_3 = (state_2 <<< 1);
  assign _zz_4 = (state_3 <<< 1);
  assign _zz_5 = (state_4 <<< 1);
  assign _zz_6 = (state_5 <<< 1);
  assign _zz_7 = (state_6 <<< 1);
  assign _zz_8 = (state_7 <<< 1);
  assign _zz_9 = stateXor[9];
  assign _zz_10 = stateXor[10];
  assign _zz_11 = {stateXor[11],{stateXor[12],{stateXor[13],{stateXor[14],stateXor[15]}}}};
  assign _zz_12 = accXor[9];
  assign _zz_13 = accXor[10];
  assign _zz_14 = {accXor[11],{accXor[12],{accXor[13],{accXor[14],accXor[15]}}}};
  always @ (*) begin
    state_8 = state_7;
    state_8 = (_zz_8 ^ ((io_input_payload[7] ^ state_7[15]) ? 16'h8005 : 16'h0));
  end

  always @ (*) begin
    state_7 = state_6;
    state_7 = (_zz_7 ^ ((io_input_payload[6] ^ state_6[15]) ? 16'h8005 : 16'h0));
  end

  always @ (*) begin
    state_6 = state_5;
    state_6 = (_zz_6 ^ ((io_input_payload[5] ^ state_5[15]) ? 16'h8005 : 16'h0));
  end

  always @ (*) begin
    state_5 = state_4;
    state_5 = (_zz_5 ^ ((io_input_payload[4] ^ state_4[15]) ? 16'h8005 : 16'h0));
  end

  always @ (*) begin
    state_4 = state_3;
    state_4 = (_zz_4 ^ ((io_input_payload[3] ^ state_3[15]) ? 16'h8005 : 16'h0));
  end

  always @ (*) begin
    state_3 = state_2;
    state_3 = (_zz_3 ^ ((io_input_payload[2] ^ state_2[15]) ? 16'h8005 : 16'h0));
  end

  always @ (*) begin
    state_2 = state_1;
    state_2 = (_zz_2 ^ ((io_input_payload[1] ^ state_1[15]) ? 16'h8005 : 16'h0));
  end

  always @ (*) begin
    state_1 = state;
    state_1 = (_zz_1 ^ ((io_input_payload[0] ^ state[15]) ? 16'h8005 : 16'h0));
  end

  assign stateXor = (state ^ 16'hffff);
  assign accXor = (state_8 ^ 16'hffff);
  assign io_result = {stateXor[0],{stateXor[1],{stateXor[2],{stateXor[3],{stateXor[4],{stateXor[5],{stateXor[6],{stateXor[7],{stateXor[8],{_zz_9,{_zz_10,_zz_11}}}}}}}}}}};
  assign io_resultNext = {accXor[0],{accXor[1],{accXor[2],{accXor[3],{accXor[4],{accXor[5],{accXor[6],{accXor[7],{accXor[8],{_zz_12,{_zz_13,_zz_14}}}}}}}}}}};
  always @ (posedge ctrl_clk or posedge ctrl_reset) begin
    if (ctrl_reset) begin
      state <= 16'hffff;
    end else begin
      if(io_input_valid)begin
        state <= state_8;
      end
      if(io_flush)begin
        state <= 16'hffff;
      end
    end
  end


endmodule

module UsbOhciWishbone_Crc (
  input               io_flush,
  input               io_input_valid,
  input      [10:0]   io_input_payload,
  output     [4:0]    io_result,
  output     [4:0]    io_resultNext,
  input               ctrl_clk,
  input               ctrl_reset
);
  wire       [4:0]    _zz_1;
  wire       [4:0]    _zz_2;
  wire       [4:0]    _zz_3;
  wire       [4:0]    _zz_4;
  wire       [4:0]    _zz_5;
  wire       [4:0]    _zz_6;
  wire       [4:0]    _zz_7;
  wire       [4:0]    _zz_8;
  wire       [4:0]    _zz_9;
  wire       [4:0]    _zz_10;
  wire       [4:0]    _zz_11;
  reg        [4:0]    state_11;
  reg        [4:0]    state_10;
  reg        [4:0]    state_9;
  reg        [4:0]    state_8;
  reg        [4:0]    state_7;
  reg        [4:0]    state_6;
  reg        [4:0]    state_5;
  reg        [4:0]    state_4;
  reg        [4:0]    state_3;
  reg        [4:0]    state_2;
  reg        [4:0]    state_1;
  reg        [4:0]    state;
  wire       [4:0]    stateXor;
  wire       [4:0]    accXor;

  assign _zz_1 = (state <<< 1);
  assign _zz_2 = (state_1 <<< 1);
  assign _zz_3 = (state_2 <<< 1);
  assign _zz_4 = (state_3 <<< 1);
  assign _zz_5 = (state_4 <<< 1);
  assign _zz_6 = (state_5 <<< 1);
  assign _zz_7 = (state_6 <<< 1);
  assign _zz_8 = (state_7 <<< 1);
  assign _zz_9 = (state_8 <<< 1);
  assign _zz_10 = (state_9 <<< 1);
  assign _zz_11 = (state_10 <<< 1);
  always @ (*) begin
    state_11 = state_10;
    state_11 = (_zz_11 ^ ((io_input_payload[10] ^ state_10[4]) ? 5'h05 : 5'h0));
  end

  always @ (*) begin
    state_10 = state_9;
    state_10 = (_zz_10 ^ ((io_input_payload[9] ^ state_9[4]) ? 5'h05 : 5'h0));
  end

  always @ (*) begin
    state_9 = state_8;
    state_9 = (_zz_9 ^ ((io_input_payload[8] ^ state_8[4]) ? 5'h05 : 5'h0));
  end

  always @ (*) begin
    state_8 = state_7;
    state_8 = (_zz_8 ^ ((io_input_payload[7] ^ state_7[4]) ? 5'h05 : 5'h0));
  end

  always @ (*) begin
    state_7 = state_6;
    state_7 = (_zz_7 ^ ((io_input_payload[6] ^ state_6[4]) ? 5'h05 : 5'h0));
  end

  always @ (*) begin
    state_6 = state_5;
    state_6 = (_zz_6 ^ ((io_input_payload[5] ^ state_5[4]) ? 5'h05 : 5'h0));
  end

  always @ (*) begin
    state_5 = state_4;
    state_5 = (_zz_5 ^ ((io_input_payload[4] ^ state_4[4]) ? 5'h05 : 5'h0));
  end

  always @ (*) begin
    state_4 = state_3;
    state_4 = (_zz_4 ^ ((io_input_payload[3] ^ state_3[4]) ? 5'h05 : 5'h0));
  end

  always @ (*) begin
    state_3 = state_2;
    state_3 = (_zz_3 ^ ((io_input_payload[2] ^ state_2[4]) ? 5'h05 : 5'h0));
  end

  always @ (*) begin
    state_2 = state_1;
    state_2 = (_zz_2 ^ ((io_input_payload[1] ^ state_1[4]) ? 5'h05 : 5'h0));
  end

  always @ (*) begin
    state_1 = state;
    state_1 = (_zz_1 ^ ((io_input_payload[0] ^ state[4]) ? 5'h05 : 5'h0));
  end

  assign stateXor = (state ^ 5'h1f);
  assign accXor = (state_11 ^ 5'h1f);
  assign io_result = {stateXor[0],{stateXor[1],{stateXor[2],{stateXor[3],stateXor[4]}}}};
  assign io_resultNext = {accXor[0],{accXor[1],{accXor[2],{accXor[3],accXor[4]}}}};
  always @ (posedge ctrl_clk or posedge ctrl_reset) begin
    if (ctrl_reset) begin
      state <= 5'h1f;
    end else begin
      if(io_input_valid)begin
        state <= state_11;
      end
      if(io_flush)begin
        state <= 5'h1f;
      end
    end
  end


endmodule

module UsbOhciWishbone_StreamFifo (
  input               io_push_valid,
  output              io_push_ready,
  input      [31:0]   io_push_payload,
  output              io_pop_valid,
  input               io_pop_ready,
  output     [31:0]   io_pop_payload,
  input               io_flush,
  output     [9:0]    io_occupancy,
  output     [9:0]    io_availability,
  input               ctrl_clk,
  input               ctrl_reset
);
  reg        [31:0]   _zz_3;
  wire       [0:0]    _zz_4;
  wire       [8:0]    _zz_5;
  wire       [0:0]    _zz_6;
  wire       [8:0]    _zz_7;
  wire       [8:0]    _zz_8;
  wire                _zz_9;
  reg                 _zz_1;
  reg                 logic_pushPtr_willIncrement;
  reg                 logic_pushPtr_willClear;
  reg        [8:0]    logic_pushPtr_valueNext;
  reg        [8:0]    logic_pushPtr_value;
  wire                logic_pushPtr_willOverflowIfInc;
  wire                logic_pushPtr_willOverflow;
  reg                 logic_popPtr_willIncrement;
  reg                 logic_popPtr_willClear;
  reg        [8:0]    logic_popPtr_valueNext;
  reg        [8:0]    logic_popPtr_value;
  wire                logic_popPtr_willOverflowIfInc;
  wire                logic_popPtr_willOverflow;
  wire                logic_ptrMatch;
  reg                 logic_risingOccupancy;
  wire                logic_pushing;
  wire                logic_popping;
  wire                logic_empty;
  wire                logic_full;
  reg                 _zz_2;
  wire       [8:0]    logic_ptrDif;
  reg [31:0] logic_ram [0:511];

  assign _zz_4 = logic_pushPtr_willIncrement;
  assign _zz_5 = {8'd0, _zz_4};
  assign _zz_6 = logic_popPtr_willIncrement;
  assign _zz_7 = {8'd0, _zz_6};
  assign _zz_8 = (logic_popPtr_value - logic_pushPtr_value);
  assign _zz_9 = 1'b1;
  always @ (posedge ctrl_clk) begin
    if(_zz_9) begin
      _zz_3 <= logic_ram[logic_popPtr_valueNext];
    end
  end

  always @ (posedge ctrl_clk) begin
    if(_zz_1) begin
      logic_ram[logic_pushPtr_value] <= io_push_payload;
    end
  end

  always @ (*) begin
    _zz_1 = 1'b0;
    if(logic_pushing)begin
      _zz_1 = 1'b1;
    end
  end

  always @ (*) begin
    logic_pushPtr_willIncrement = 1'b0;
    if(logic_pushing)begin
      logic_pushPtr_willIncrement = 1'b1;
    end
  end

  always @ (*) begin
    logic_pushPtr_willClear = 1'b0;
    if(io_flush)begin
      logic_pushPtr_willClear = 1'b1;
    end
  end

  assign logic_pushPtr_willOverflowIfInc = (logic_pushPtr_value == 9'h1ff);
  assign logic_pushPtr_willOverflow = (logic_pushPtr_willOverflowIfInc && logic_pushPtr_willIncrement);
  always @ (*) begin
    logic_pushPtr_valueNext = (logic_pushPtr_value + _zz_5);
    if(logic_pushPtr_willClear)begin
      logic_pushPtr_valueNext = 9'h0;
    end
  end

  always @ (*) begin
    logic_popPtr_willIncrement = 1'b0;
    if(logic_popping)begin
      logic_popPtr_willIncrement = 1'b1;
    end
  end

  always @ (*) begin
    logic_popPtr_willClear = 1'b0;
    if(io_flush)begin
      logic_popPtr_willClear = 1'b1;
    end
  end

  assign logic_popPtr_willOverflowIfInc = (logic_popPtr_value == 9'h1ff);
  assign logic_popPtr_willOverflow = (logic_popPtr_willOverflowIfInc && logic_popPtr_willIncrement);
  always @ (*) begin
    logic_popPtr_valueNext = (logic_popPtr_value + _zz_7);
    if(logic_popPtr_willClear)begin
      logic_popPtr_valueNext = 9'h0;
    end
  end

  assign logic_ptrMatch = (logic_pushPtr_value == logic_popPtr_value);
  assign logic_pushing = (io_push_valid && io_push_ready);
  assign logic_popping = (io_pop_valid && io_pop_ready);
  assign logic_empty = (logic_ptrMatch && (! logic_risingOccupancy));
  assign io_push_ready = (! logic_full);
  assign io_pop_valid = ((! logic_empty) && (! (_zz_2 && (! logic_full))));
  assign io_pop_payload = _zz_3;
  assign logic_ptrDif = (logic_pushPtr_value - logic_popPtr_value);
  assign io_occupancy = {(logic_risingOccupancy && logic_ptrMatch),logic_ptrDif};
  assign io_availability = {((! logic_risingOccupancy) && logic_ptrMatch),_zz_8};
  assign logic_full = 1'b0;
  always @ (posedge ctrl_clk or posedge ctrl_reset) begin
    if (ctrl_reset) begin
      logic_pushPtr_value <= 9'h0;
      logic_popPtr_value <= 9'h0;
      logic_risingOccupancy <= 1'b0;
      _zz_2 <= 1'b0;
    end else begin
      logic_pushPtr_value <= logic_pushPtr_valueNext;
      logic_popPtr_value <= logic_popPtr_valueNext;
      _zz_2 <= (logic_popPtr_valueNext == logic_pushPtr_value);
      if((logic_pushing != logic_popping))begin
        logic_risingOccupancy <= logic_pushing;
      end
      if(io_flush)begin
        logic_risingOccupancy <= 1'b0;
      end
    end
  end


endmodule

//UsbOhciWishbone_BufferCC_9 replaced by UsbOhciWishbone_BufferCC_9

//UsbOhciWishbone_BufferCC replaced by UsbOhciWishbone_BufferCC

//UsbOhciWishbone_BufferCC_9 replaced by UsbOhciWishbone_BufferCC_9

//UsbOhciWishbone_BufferCC replaced by UsbOhciWishbone_BufferCC

//UsbOhciWishbone_BufferCC_9 replaced by UsbOhciWishbone_BufferCC_9

//UsbOhciWishbone_BufferCC replaced by UsbOhciWishbone_BufferCC

module UsbOhciWishbone_BufferCC_9 (
  input               io_dataIn,
  output              io_dataOut,
  input               phy_clk,
  input               phy_reset
);
  (* async_reg = "true" *) reg                 buffers_0;
  (* async_reg = "true" *) reg                 buffers_1;
  (* async_reg = "true" *) reg                 buffers_2;

  assign io_dataOut = buffers_2;
  always @ (posedge phy_clk or posedge phy_reset) begin
    if (phy_reset) begin
      buffers_0 <= 1'b0;
      buffers_1 <= 1'b0;
      buffers_2 <= 1'b0;
    end else begin
      buffers_0 <= io_dataIn;
      buffers_1 <= buffers_0;
      buffers_2 <= buffers_1;
    end
  end


endmodule

//UsbOhciWishbone_BufferCC replaced by UsbOhciWishbone_BufferCC

//UsbOhciWishbone_BufferCC replaced by UsbOhciWishbone_BufferCC

//UsbOhciWishbone_BufferCC replaced by UsbOhciWishbone_BufferCC

//UsbOhciWishbone_BufferCC replaced by UsbOhciWishbone_BufferCC

//UsbOhciWishbone_BufferCC replaced by UsbOhciWishbone_BufferCC

//UsbOhciWishbone_BufferCC replaced by UsbOhciWishbone_BufferCC

//UsbOhciWishbone_BufferCC replaced by UsbOhciWishbone_BufferCC

module UsbOhciWishbone_BufferCC_1 (
  input               io_dataIn,
  output              io_dataOut,
  input               phy_clk,
  input               phy_reset
);
  (* async_reg = "true" *) reg                 buffers_0;
  (* async_reg = "true" *) reg                 buffers_1;

  assign io_dataOut = buffers_1;
  always @ (posedge phy_clk or posedge phy_reset) begin
    if (phy_reset) begin
      buffers_0 <= 1'b0;
      buffers_1 <= 1'b0;
    end else begin
      buffers_0 <= io_dataIn;
      buffers_1 <= buffers_0;
    end
  end


endmodule

module UsbOhciWishbone_BufferCC (
  input               io_dataIn,
  output              io_dataOut,
  input               ctrl_clk,
  input               ctrl_reset
);
  (* async_reg = "true" *) reg                 buffers_0;
  (* async_reg = "true" *) reg                 buffers_1;

  assign io_dataOut = buffers_1;
  always @ (posedge ctrl_clk or posedge ctrl_reset) begin
    if (ctrl_reset) begin
      buffers_0 <= 1'b0;
      buffers_1 <= 1'b0;
    end else begin
      buffers_0 <= io_dataIn;
      buffers_1 <= buffers_0;
    end
  end


endmodule
