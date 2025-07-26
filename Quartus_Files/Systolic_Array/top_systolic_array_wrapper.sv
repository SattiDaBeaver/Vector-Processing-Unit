module top_systolic_array_wrapper #(
    parameter MATRIX_SIZE = 3,
    parameter DATA_WIDTH  = 8,
    parameter ACC_WIDTH   = 32
)(
    input  logic clk,
    input  logic rst,
    input  logic acc_rst,
    input  logic acc_en,
    input  logic shift_en,
    input  logic [DATA_WIDTH*MATRIX_SIZE-1:0] in_left_flat,
    input  logic [DATA_WIDTH*MATRIX_SIZE-1:0] in_top_flat,
    output logic [ACC_WIDTH*MATRIX_SIZE*MATRIX_SIZE-1:0] acc_out_flat
);

    // Optional: Flattened outputs if needed later
    // logic [DATA_WIDTH*MATRIX_SIZE-1:0] out_right_flat;
    // logic [DATA_WIDTH*MATRIX_SIZE-1:0] out_bottom_flat;

    systolic_array #(
        .MATRIX_SIZE(MATRIX_SIZE),
        .DATA_WIDTH(DATA_WIDTH),
        .ACC_WIDTH(ACC_WIDTH)
    ) dut (
        .clk(clk),
        .rst(rst),
        .acc_rst(acc_rst),
        .acc_en(acc_en),
        .shift_en(shift_en),
        .in_left_flat(in_left_flat),
        .in_top_flat(in_top_flat),
        .out_right_flat(), // Unused, not connected
        .out_bottom_flat(), // Unused, not connected
        .acc_out_flat(acc_out_flat)
    );

endmodule
