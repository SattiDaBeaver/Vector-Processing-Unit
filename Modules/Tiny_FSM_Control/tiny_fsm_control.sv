module tiny_fsm_control #(
    
) (

);
    //======================================//
    //              Parameters
    //======================================//
    // Systolic Module Parameters
    parameter DATA_WIDTH    = 8;
    parameter MATRIX_SIZE   = 8;
    parameter ACC_WIDTH     = 32;
    parameter DATA_WIDTH    = 8;
    // DPRAM Parameters
    parameter DP_ADDR_WIDTH = 10;
    // UART Instruction Memory Loader Parameters
    parameter F_CLK         = 50_000_000;
    parameter BAUD          = 921_600;
    parameter CLK_PER_BIT   = F_CLK / BAUD;
    parameter INSTR_WIDTH   = 32;
    parameter INSTR_DEPTH   = 256;


    //======================================//
    //             Module Wires
    //======================================//
    // Systolic Module Wires
    logic                                clk,
    logic                                rst,
    logic                                acc_rst,
    logic                                acc_en,
    logic                                shift_en_right,
    logic                                shift_en_down,

    logic [ACC_ADDR_WIDTH-1:0]           addr_acc,

    logic                                buffer_rst_top,
    logic                                load_en_top,
    logic                                swap_buffers_top,
    logic [ADDR_WIDTH-1:0]               addr_top,
    logic [DATA_WIDTH-1:0]               data_in_top,

    logic                                buffer_rst_left,
    logic                                load_en_left,
    logic                                swap_buffers_left,
    logic [ADDR_WIDTH-1:0]               addr_left,
    logic [DATA_WIDTH-1:0]               data_in_left,

    logic [DATA_WIDTH*MATRIX_SIZE-1:0]   data_out_flat_bottom,
    logic [DATA_WIDTH*MATRIX_SIZE-1:0]   data_out_flat_right,
    logic [ACC_WIDTH-1:0]                acc_out

    // 

    //======================================//
    //        Module Instantiation
    //======================================//

    // Systolic Module Instantiation
    systolic_module #(
        .DATA_WIDTH(DATA_WIDTH),
        .MATRIX_SIZE(MATRIX_SIZE),
        .ACC_WIDTH(ACC_WIDTH)
        ) SYS_ARRAY (
        .clk(clk),
        .rst(rst),
        .acc_rst(acc_rst),
        .acc_en(acc_en),
        .shift_en_right(shift_en_right),
        .shift_en_down(shift_en_down),
        .addr_acc(addr_acc),

        .buffer_rst_top(buffer_rst_top),
        .load_en_top(load_en_top),
        .swap_buffers_top(swap_buffers_top),
        .addr_top(addr_top),
        .data_in_top(data_in_top),

        .buffer_rst_left(buffer_rst_left),
        .load_en_left(load_en_left),
        .swap_buffers_left(swap_buffers_left),
        .addr_left(addr_left),
        .data_in_left(data_in_left),

        .data_out_flat_bottom(data_out_flat_bottom),
        .data_out_flat_right(data_out_flat_right),
        .acc_out(acc_out)
    );

    // Dual Port RAM Instantiation
    dp_ram #(
        .DATA_WIDTH(DATA_WIDTH)
        .ADDR_WIDTH(DP_ADDR_WIDTH)
        ) DPRAM (
        .clk(),

        // Port A
        .we_a(),
        .addr_a(),
        .din_a(),
        .dout_a(),

        // Port B
        .we_b(),
        .addr_b(),
        .din_b(),
        .dout_b()
    );

    // UART Instruction Memory Loader Instantiation
    uart_instr_mem_loader #(
        .INSTR_WIDTH(INSTR_WIDTH),
        .DEPTH(INSTR_DEPTH),
        .CLK_PER_BIT(CLK_PER_BIT) 
        ) UART_MEM (
        .clk(clk),
        .rst(rst),

        // UART RX pin from FTDI / USB-UART
        .uart_rx(uart_rx),

        // Optional: UART TX pin back to PC for debugging
        .uart_tx(uart_tx),

        // FSM read interface
        .rd_addr(rd_addr),
        .rd_data(rd_data)
    );

endmodule : tiny_fsm_control