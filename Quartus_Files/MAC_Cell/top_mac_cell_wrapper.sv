module top_mac_cell_wrapper (
    input  logic clk,
    input  logic rst,
    input  logic en,
    input  logic [7:0] in_left,
    input  logic [7:0] in_top,
    output logic [7:0] out_right,
    output logic [7:0] out_bottom,
    output logic [31:0] acc_out
);

    mac_cell #(
        .DATA_WIDTH(8),
        .ACC_WIDTH(32)
    ) pe (
        .clk(clk),
        .rst(rst),
        .en(en),
        .in_left(in_left),
        .in_top(in_top),
        .out_right(out_right),
        .out_bottom(out_bottom),
        .acc_out(acc_out)
    );

endmodule
