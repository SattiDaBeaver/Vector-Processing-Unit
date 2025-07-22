module top_systolic_array_wrapper (
    input  logic clk,
    input  logic rst,
    input  logic en,

    input  logic [7:0] in_left_0,
    input  logic [7:0] in_left_1,
    input  logic [7:0] in_left_2,
    input  logic [7:0] in_top_0,
    input  logic [7:0] in_top_1,
    input  logic [7:0] in_top_2,

    output logic [31:0] acc_out_00,
    output logic [31:0] acc_out_01,
    output logic [31:0] acc_out_02,
    output logic [31:0] acc_out_10,
    output logic [31:0] acc_out_11,
    output logic [31:0] acc_out_12,
    output logic [31:0] acc_out_20,
    output logic [31:0] acc_out_21,
    output logic [31:0] acc_out_22
);

    // Internal-only signals — not connected to FPGA pins
    logic [7:0] out_right_0, out_right_1, out_right_2;
    logic [7:0] out_bottom_0, out_bottom_1, out_bottom_2;

    systolic_array #(
        .MATRIX_SIZE(3),
        .DATA_WIDTH(8),
        .ACC_WIDTH(32)
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

endmodule
