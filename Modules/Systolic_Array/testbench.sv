module tb_systolic_array;

    parameter MATRIX_SIZE = 3;
    parameter DATA_WIDTH = 8;
    parameter ACC_WIDTH = 32;

    logic clk, rst, en;
    logic [DATA_WIDTH-1:0] in_left_0, in_left_1, in_left_2;
    logic [DATA_WIDTH-1:0] in_top_0, in_top_1, in_top_2;

    logic [DATA_WIDTH-1:0] out_right_0, out_right_1, out_right_2;
    logic [DATA_WIDTH-1:0] out_bottom_0, out_bottom_1, out_bottom_2;

    logic [ACC_WIDTH-1:0] acc_out_00, acc_out_01, acc_out_02;
    logic [ACC_WIDTH-1:0] acc_out_10, acc_out_11, acc_out_12;
    logic [ACC_WIDTH-1:0] acc_out_20, acc_out_21, acc_out_22;

    // Instantiate DUT
    systolic_array #(
        .MATRIX_SIZE(MATRIX_SIZE),
        .DATA_WIDTH(DATA_WIDTH),
        .ACC_WIDTH(ACC_WIDTH)
    ) dut (
        .clk(clk),
        .rst(rst),
        .en(en),
        .in_left_0(in_left_0),
        .in_left_1(in_left_1),
        .in_left_2(in_left_2),
        .in_top_0(in_top_0),
        .in_top_1(in_top_1),
        .in_top_2(in_top_2),
        .out_right_0(out_right_0),
        .out_right_1(out_right_1),
        .out_right_2(out_right_2),
        .out_bottom_0(out_bottom_0),
        .out_bottom_1(out_bottom_1),
        .out_bottom_2(out_bottom_2),
        .acc_out_00(acc_out_00),
        .acc_out_01(acc_out_01),
        .acc_out_02(acc_out_02),
        .acc_out_10(acc_out_10),
        .acc_out_11(acc_out_11),
        .acc_out_12(acc_out_12),
        .acc_out_20(acc_out_20),
        .acc_out_21(acc_out_21),
        .acc_out_22(acc_out_22)
    );

    // Clock generation
    always #5 clk = ~clk;

    initial begin
        $dumpfile("systolic_array.vcd");
        $dumpvars(0, tb_systolic_array);

        clk = 0;
        rst = 1;
        en = 0;
        {in_left_0, in_left_1, in_left_2} = 0;
        {in_top_0, in_top_1, in_top_2} = 0;

        #10;
        rst = 0;

        // Cycle 1
        en = 1;
        in_left_0 = 1; in_left_1 = 2; in_left_2 = 3;
        in_top_0  = 1; in_top_1  = 2; in_top_2  = 3;

        // Cycle 2
        #10;
        in_left_0 = 4; in_left_1 = 5; in_left_2 = 6;
        in_top_0  = 4; in_top_1  = 5; in_top_2  = 6;

        // Cycle 3
        #10;
        in_left_0 = 7; in_left_1 = 8; in_left_2 = 9;
        in_top_0  = 7; in_top_1  = 8; in_top_2  = 9;

        // Stop feeding inputs
        #10;
        en = 0;
        in_left_0 = 0; in_left_1 = 0; in_left_2 = 0;
        in_top_0  = 0; in_top_1  = 0; in_top_2  = 0;

        // Let the array propagate data
        #100;

        // Display accumulated outputs
        $display("ACC OUTPUTS:");
        $display("%0d %0d %0d", acc_out_00, acc_out_01, acc_out_02);
        $display("%0d %0d %0d", acc_out_10, acc_out_11, acc_out_12);
        $display("%0d %0d %0d", acc_out_20, acc_out_21, acc_out_22);

        $finish;
    end

endmodule
