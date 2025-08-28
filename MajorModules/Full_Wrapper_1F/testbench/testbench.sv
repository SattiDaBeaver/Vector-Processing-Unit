`timescale 1ns/1ps
module tb_top_wrapper;

    parameter DATA_WIDTH     = 8;
    parameter MATRIX_SIZE    = 8;
    parameter ACC_WIDTH      = 32;
    parameter DP_ADDR_WIDTH  = 4; // small mem for simulation
    parameter INSTR_WIDTH    = 32;
    parameter INSTR_DEPTH    = 16;
    parameter F_CLK          = 50_000_000;
    parameter BAUD           = 921_600;
    parameter CLK_PER_BIT    = 50;

    // Signals
    logic clk;
    logic rst;
    logic rst_mem_ptr;

    logic uart_rx;
    logic uart_tx_en;
    logic uart_tx;

    logic mem_internal_mode;
    logic step;

    logic [$clog2(INSTR_DEPTH)-1:0] pc_out;
    logic [INSTR_WIDTH-1:0]         curr_instr_out;
    logic [INSTR_WIDTH-1:0]         next_instr_out;
    logic [2:0]                     state_out;

    // Clock
    initial clk = 0;
    always #10 clk = ~clk; // 50MHz

    // Instantiate DUT
    top_wrapper #(
        .DATA_WIDTH(DATA_WIDTH),
        .MATRIX_SIZE(MATRIX_SIZE),
        .ACC_WIDTH(ACC_WIDTH),
        .DP_ADDR_WIDTH(DP_ADDR_WIDTH),
        .INSTR_WIDTH(INSTR_WIDTH),
        .INSTR_DEPTH(INSTR_DEPTH),
        .F_CLK(F_CLK),
        .BAUD(BAUD),
        .CLK_PER_BIT(CLK_PER_BIT)
    ) DUT (
        .clk(clk),
        .rst(rst),
        .rst_mem_ptr(rst_mem_ptr),
        .uart_tx_en(uart_tx_en),
        .uart_rx(uart_rx),
        .uart_tx(uart_tx),
        .mem_internal_mode(mem_internal_mode),
        .step(step),
        .pc_out(pc_out),
        .curr_instr_out(curr_instr_out),
        .next_instr_out(next_instr_out),
        .state_out(state_out)
    );

    // Simple Test Sequence
    initial begin
        // Dump waveforms
        $dumpfile("top_wrapper_tb.vcd");
        $dumpvars(0, tb_top_wrapper);

        // Reset
        rst = 1;
        rst_mem_ptr = 1;
        step = 0;
        mem_internal_mode = 0;
        uart_rx = 0;
        uart_tx_en = 0;
        #100;
        rst = 0;
        rst_mem_ptr = 0;

        // Enable internal memory write mode
        mem_internal_mode = 1;

        // Simulate a few UART bytes being received
        // In reality UART_rx needs proper bit-level timing; here we just toggle mem_uart_wr_en
        for (int i = 0; i < 8; i++) begin
            uart_rx = i[0];         // dummy data
            uart_tx_en = 1;
            #20;
            uart_tx_en = 0;
            #20;
        end

        // Step the FSM
        step = 1;
        #40;
        step = 0;

        // Wait a few cycles to observe internal memory update
        #200;

        $finish;
    end

endmodule
