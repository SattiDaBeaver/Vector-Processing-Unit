module top (
    input  logic     [9:0]   SW,
    input  logic     [1:0]   KEY,
    input  logic             CLOCK_50,

    output logic    [6:0]    HEX5,
    output logic    [6:0]    HEX4,
    output logic    [6:0]    HEX3,
    output logic    [6:0]    HEX2,
    output logic    [6:0]    HEX1,
    output logic    [6:0]    HEX0,
    output logic    [9:0]    LEDR, 

    inout  logic    [15:0]   ARDUINO_IO,
    inout  logic    [35:0]   GPIO
);

    // ====================================== //
    // Assign UART pins
    // ====================================== //
    logic uart_rx;
    logic uart_tx_en;
    logic uart_tx;

    assign uart_rx    = GPIO[0]; // RX from PC
    assign GPIO[1]    = uart_tx; // TX to PC
    assign uart_tx_en = SW[0];   // Enable UART transmission via switch

    // ====================================== //
    // Reset and mode control
    // ====================================== //
    logic rst;
    logic rst_mem_ptr;
    logic step;
    logic mem_internal_mode;

    assign rst               = ~KEY[0];    // Active-low reset
    assign rst_mem_ptr       = SW[1];      // Reset memory pointer
    assign step              = SW[2];      // Step FSM
    assign mem_internal_mode = SW[3];      // Select internal memory / instruction mem

    // ====================================== //
    // Instantiate the top_wrapper
    // ====================================== //
    top_wrapper #(
        .DATA_WIDTH(8),
        .MATRIX_SIZE(8),
        .ACC_WIDTH(32),
        .DP_ADDR_WIDTH(10),
        .INSTR_WIDTH(32),
        .INSTR_DEPTH(256),
        .F_CLK(50_000_000),
        .BAUD(921_600),
        .CLK_PER_BIT(50)
    ) u_top_wrapper (
        .clk(CLOCK_50),
        .rst(rst),
        .rst_mem_ptr(rst_mem_ptr),
        .uart_tx_en(uart_tx_en),
        .uart_rx(uart_rx),
        .uart_tx(uart_tx),
        .mem_internal_mode(mem_internal_mode),
        .step(step),

        .pc_out(),            // Optionally tie to HEX/LEDR
        .curr_instr_out(),
        .next_instr_out(),
        .state_out()
    );

    // ====================================== //
    // Optional: display PC or status on LEDs/HEX
    // ====================================== //
    assign LEDR[9:0] = {6'b0, step, mem_internal_mode, rst, uart_tx_en};

endmodule
