module tb_mac_cell;

    parameter DATA_WIDTH = 8;
    parameter ACC_WIDTH  = 32;

    // Signals
    logic clk, rst, acc_rst, shift_en, acc_en;
    logic signed [DATA_WIDTH-1:0] in_left, in_top;
    logic signed [DATA_WIDTH-1:0] out_right, out_bottom;
    logic signed [ACC_WIDTH-1:0]  acc_out;

    // Instantiate the MAC Cell
    mac_cell #(
        .DATA_WIDTH(DATA_WIDTH),
        .ACC_WIDTH(ACC_WIDTH)
    ) uut (
        .clk(clk),
        .rst(rst),
        .acc_rst(acc_rst),

        .shift_en(shift_en),
        .acc_en(acc_en),

        .in_left(in_left),
        .in_top(in_top),
        .out_right(out_right),
        .out_bottom(out_bottom),
        .acc_out(acc_out)
    );

    // Clock generation
    initial clk = 0;
    always #5 clk = ~clk;

    // Dump waveform
    initial begin
        $dumpfile("tb_mac_cell.vcd");
        $dumpvars(0, tb_mac_cell);
    end

    // Test sequence
    initial begin
        $display("Starting MAC Cell Testbench...");

        // Initialize
        acc_rst = 0;
        rst = 1;
        shift_en = 0;
        acc_en = 0;
        in_left = 0;
        in_top  = 0;

        #10;
        rst = 0;

        // Apply inputs: (2×3) + (4×1) + (6×-1) = 6 + 4 - 6 = 4

        // Cycle 1
        @(posedge clk);
        shift_en = 1;
        acc_en = 1;
        in_left = 8'sd2;
        in_top  = 8'sd3;

        // Cycle 2
        @(posedge clk);
        in_left = 8'sd4;
        in_top  = 8'sd1;

        // Cycle 3
        @(posedge clk);
        in_left = 8'sd6;
        in_top  = -8'sd1;

        // Stop input
        @(posedge clk);
        shift_en = 1;
        acc_en = 1;
        in_left = 0;
        in_top = 0;

        // Wait a bit and print result
        repeat (2) @(posedge clk);
        $display("Final accumulated value: %0d", acc_out); // Expected: 4

        #10;
        $finish;
    end

endmodule
