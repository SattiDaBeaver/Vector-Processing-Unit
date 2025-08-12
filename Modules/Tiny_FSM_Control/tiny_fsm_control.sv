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
    parameter ADDR_WIDTH = $clog2(MATRIX_SIZE);
    parameter ACC_ADDR_WIDTH = $clog2(MATRIX_SIZE*MATRIX_SIZE);

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
    logic                               clk;
    logic                               rst;
    logic                               acc_rst;
    logic                               acc_en;
    logic                               shift_en_right;
    logic                               shift_en_down;

    logic [ACC_ADDR_WIDTH-1:0]          addr_acc;

    logic                               buffer_rst_top;
    logic                               load_en_top;
    logic                               swap_buffers_top;
    logic [ADDR_WIDTH-1:0]              addr_top;
    logic [DATA_WIDTH-1:0]              data_in_top;

    logic                               buffer_rst_left;
    logic                               load_en_left;
    logic                               swap_buffers_left;
    logic [ADDR_WIDTH-1:0]              addr_left;
    logic [DATA_WIDTH-1:0]              data_in_left;

    logic [DATA_WIDTH*MATRIX_SIZE-1:0]  data_out_flat_bottom;
    logic [DATA_WIDTH*MATRIX_SIZE-1:0]  data_out_flat_right;
    logic [ACC_WIDTH-1:0]               acc_out;

    // Dual Port RAM Wires
    logic                               we_a;
    logic [DP_ADDR_WIDTH-1:0]           addr_a;
    logic [DATA_WIDTH-1:0]              din_a;
    logic [DATA_WIDTH-1:0]              dout_a;
    logic                               we_b;
    logic [DP_ADDR_WIDTH-1:0]           addr_b;
    logic [DATA_WIDTH-1:0]              din_b;
    logic [DATA_WIDTH-1:0]              dout_b;

    // UART Instruction Memory Loader Wires
    logic                               uart_rx;
    logic                               uart_tx;
    logic [$clog2(INSTR_DEPTH)-1:0]     rd_addr;
    logic [INSTR_WIDTH-1:0]             rd_data;

    //======================================//
    //          FSM Control Logic
    //======================================//

    typedef struct packed {
        logic           LOAD_LEFT;      // bit 31
        logic           LOAD_TOP;       // bit 30
        logic           SWAP_LEFT;      // bit 29
        logic           SWAP_TOP;       // bit 28
        logic           SHIFT_RIGHT;    // bit 27
        logic           SHIFT_DOWN;     // bit 26
        logic           LOAD_ACC;       // bit 25
        logic           WRITE_ACC_OUT;  // bit 24
        logic           WAIT_CYCLES;    // bit 23
        logic           JUMP;           // bit 22 -- Subject to change
        logic           CLR;            // bit 21
        logic           NOP;            // bit 20 -- Subject to change

        logic [1:0]     RESERVED;       // bits 19 down to 18 (2 bits)

        logic [4:0]     FLAGS;          // bits 17 down to 13 (5 bits)
        logic [12:0]    ADDR;           // bits 12 down to 0 (13 bits)
    } instruction_t;

    typedef enum logic [1:0] { 
        IDLE,
        FETCH,
        DECODE,
        WAIT
    } fsm_state_t;

    instruction_t   instruction;
    fsm_state_t     state;


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
        .DATA_WIDTH(DATA_WIDTH),
        .ADDR_WIDTH(DP_ADDR_WIDTH)
        ) DPRAM (
        .clk(clk),

        // Port A
        .we_a(we_a),
        .addr_a(addr_a),
        .din_a(din_a),
        .dout_a(dout_a),

        // Port B
        .we_b(we_b),
        .addr_b(addr_b),
        .din_b(din_b),
        .dout_b(dout_b)
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