module tb_tiny_fsm_control;

    // =========================================
    // Parameters
    // =========================================
    parameter DATA_WIDTH     = 8;
    parameter MATRIX_SIZE    = 8;
    parameter ACC_WIDTH      = 32;
    parameter DP_ADDR_WIDTH  = 10;
    parameter INSTR_WIDTH    = 32;
    parameter INSTR_DEPTH    = 256;
    parameter CLK_PERIOD     = 20; // 50 MHz

    // =========================================
    // Signals
    // =========================================
    logic clk;
    logic fsm_rst;
    logic step, run, halt;
    logic uart_rx, uart_tx;

    logic [$clog2(INSTR_DEPTH)-1:0] rd_addr;
    logic [INSTR_WIDTH-1:0] rd_data;

    logic [7:0] pc_out;
    logic [INSTR_WIDTH-1:0] curr_instr_out;
    logic [INSTR_WIDTH-1:0] next_instr_out;
    logic [2:0] state_out;

    // =========================================
    // Clock Generation
    // =========================================
    initial clk = 0;
    always #(CLK_PERIOD/2) clk = ~clk;

    // =========================================
    // FSM Instantiation
    // =========================================
    tiny_fsm_control #(
        .DATA_WIDTH(DATA_WIDTH),
        .MATRIX_SIZE(MATRIX_SIZE),
        .ACC_WIDTH(ACC_WIDTH),
        .DP_ADDR_WIDTH(DP_ADDR_WIDTH),
        .INSTR_WIDTH(INSTR_WIDTH),
        .INSTR_DEPTH(INSTR_DEPTH)
    ) DUT (
        .clk(clk),
        .fsm_rst(fsm_rst),
        .step(step),
        .run(run),
        .halt(halt),
        .rd_addr(rd_addr),
        .rd_data(rd_data),
        .pc_out(pc_out),
        .curr_instr_out(curr_instr_out),
        .next_instr_out(next_instr_out),
        .state_out(state_out)
    );

    // =========================================
    // Instruction Memory Model (Simple ROM)
    // =========================================
    logic [INSTR_WIDTH-1:0] instr_mem [0:INSTR_DEPTH-1];

    initial begin
        // Example: Fill memory with instructions
        // Format: {LOAD_LEFT, LOAD_TOP, SWAP_LEFT, SWAP_TOP, SHIFT_RIGHT, ... FLAGS, ADDR}
        // You can replace these with your actual 32-bit instruction values
        // Load Left/Top
        instr_mem[0] = 32'h8000_0001; 
        instr_mem[1] = 32'h4000_0002;
        instr_mem[2] = 32'hC000_0003;
        instr_mem[3] = 32'h8000_FF30;

        // Swap, shift, and load acc
        // Swap
        instr_mem[4] = 32'h2000_0001; 
        instr_mem[5] = 32'h1000_0002;
        instr_mem[6] = 32'h3000_0003;
        instr_mem[7] = 32'h3000_0003;

        // Shift
        instr_mem[8] = 32'h0800_FF30;
        instr_mem[9] = 32'h0400_0001;
        instr_mem[10] = 32'h0C00_0001;
        
        // Acc Load
        instr_mem[11] = 32'h0200_0001;

        // Fill rest with NOPs
        for (int i=12; i<INSTR_DEPTH; i++)
            instr_mem[i] = 32'h0010_0000; // NOP
    end

    // Feed rd_data based on pc_out
    assign rd_data = instr_mem[rd_addr];

    // =========================================
    // Test Sequence
    // =========================================
    initial begin
        // --- Dump waveforms ---
        $dumpfile("tiny_fsm_control_tb.vcd");
        $dumpvars(0, tb_tiny_fsm_control);

        fsm_rst = 1;
        step = 0;
        run  = 0;
        halt = 0;
        uart_rx = 0;

        #100;
        fsm_rst = 0;

        // Single-step example
        for (int i=0; i<10; i++) begin
            step = 1;
            #CLK_PERIOD;
            $display("Cycle %0d: PC=%0d, CURR=%h, NEXT=%h, STATE=%0d", 
                      i, pc_out, curr_instr_out, next_instr_out, state_out);
        end

        // Run continuously
        run = 1;
        #500; // run for some cycles
        run = 0;

        $finish;
    end

endmodule
