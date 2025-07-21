module systolic_array # (
    parameter MATRIX_SIZE   = 3,
    parameter DATA_WIDTH    = 8,
    parameter ACC_WIDTH     = 32
) (
    input  logic                        clk,
    input  logic                        rst,
    input  logic                        en,

    input  logic    [DATA_WIDTH-1:0]    in_left_0,
    input  logic    [DATA_WIDTH-1:0]    in_left_1,
    input  logic    [DATA_WIDTH-1:0]    in_left_2,
    input  logic    [DATA_WIDTH-1:0]    in_top_0,
    input  logic    [DATA_WIDTH-1:0]    in_top_1,
    input  logic    [DATA_WIDTH-1:0]    in_top_2,

    output logic    [DATA_WIDTH-1:0]    out_right_0,
    output logic    [DATA_WIDTH-1:0]    out_right_1,    
    output logic    [DATA_WIDTH-1:0]    out_right_2,
    output logic    [DATA_WIDTH-1:0]    out_bottom_0,
    output logic    [DATA_WIDTH-1:0]    out_bottom_1,
    output logic    [DATA_WIDTH-1:0]    out_bottom_2,

    output logic    [ACC_WIDTH-1:0]     acc_out_00,
    output logic    [ACC_WIDTH-1:0]     acc_out_01,
    output logic    [ACC_WIDTH-1:0]     acc_out_02,
    output logic    [ACC_WIDTH-1:0]     acc_out_10,
    output logic    [ACC_WIDTH-1:0]     acc_out_11,
    output logic    [ACC_WIDTH-1:0]     acc_out_12,
    output logic    [ACC_WIDTH-1:0]     acc_out_20,
    output logic    [ACC_WIDTH-1:0]     acc_out_21,
    output logic    [ACC_WIDTH-1:0]     acc_out_22
);

    logic    [DATA_WIDTH-1:0]   out_right_00, out_right_01, out_right_02,
                                out_right_10, out_right_11, out_right_12,
                                out_right_20, out_right_21, out_right_22;

    logic    [DATA_WIDTH-1:0]   out_bottom_00, out_bottom_01, out_bottom_02,
                                out_bottom_10, out_bottom_11, out_bottom_12,
                                out_bottom_20, out_bottom_21, out_bottom_22;

    always_comb begin
        out_right_0 = out_right_02;
        out_right_1 = out_right_12;
        out_right_2 = out_right_22;
        out_bottom_0 = out_bottom_20;
        out_bottom_1 = out_bottom_21;
        out_bottom_2 = out_bottom_22;
    end

    // Cell row = 0, column = 0
    mac_cell # (
        .DATA_WIDTH(DATA_WIDTH),
        .ACC_WIDTH(ACC_WIDTH)
        ) mac_00 (
        .clk(clk),
        .rst(rst),
        .en(en),
        .in_left(in_left_0),
        .in_top(in_top_0),
        .out_right(out_right_00),
        .out_bottom(out_bottom_00),
        .acc_out(acc_out_00)
    );

    // Cell row = 0, column = 1 
    mac_cell # (
        .DATA_WIDTH(DATA_WIDTH),
        .ACC_WIDTH(ACC_WIDTH)
        ) mac_01 (
        .clk(clk),
        .rst(rst),
        .en(en),
        .in_left(out_right_00),
        .in_top(in_top_1),
        .out_right(out_right_01),
        .out_bottom(out_bottom_01),
        .acc_out(acc_out_01)
    );

    // Cell row = 0, column = 2
    mac_cell # (
        .DATA_WIDTH(DATA_WIDTH),
        .ACC_WIDTH(ACC_WIDTH)
        ) mac_02 (
        .clk(clk),
        .rst(rst),
        .en(en),
        .in_left(out_right_01),
        .in_top(in_top_2),
        .out_right(out_right_02),
        .out_bottom(out_bottom_02),
        .acc_out(acc_out_02)
    );

    // Cell row = 1, column = 0
    mac_cell # (
        .DATA_WIDTH(DATA_WIDTH),
        .ACC_WIDTH(ACC_WIDTH)
        ) mac_10 (
        .clk(clk),
        .rst(rst),
        .en(en),
        .in_left(in_left_1),
        .in_top(out_bottom_00),
        .out_right(out_right_10),
        .out_bottom(out_bottom_10),
        .acc_out(acc_out_10)
    );

    // Cell row = 1, column = 1
    mac_cell # (
        .DATA_WIDTH(DATA_WIDTH),
        .ACC_WIDTH(ACC_WIDTH)
        ) mac_11 (
        .clk(clk),
        .rst(rst),
        .en(en),
        .in_left(out_right_10),
        .in_top(out_bottom_01),
        .out_right(out_right_11),
        .out_bottom(out_bottom_11),
        .acc_out(acc_out_11)
    );

    // Cell row = 1, column = 2
    mac_cell # (
        .DATA_WIDTH(DATA_WIDTH),
        .ACC_WIDTH(ACC_WIDTH)
        ) mac_12 (
        .clk(clk),
        .rst(rst),
        .en(en),
        .in_left(out_right_11),
        .in_top(out_bottom_02),
        .out_right(out_right_12),
        .out_bottom(out_bottom_12),
        .acc_out(acc_out_12)
    );

    // Cell row = 2, column = 0
    mac_cell # (
        .DATA_WIDTH(DATA_WIDTH),
        .ACC_WIDTH(ACC_WIDTH)
        ) mac_20 (
        .clk(clk),
        .rst(rst),
        .en(en),
        .in_left(in_left_2),
        .in_top(out_bottom_10),
        .out_right(out_right_20),
        .out_bottom(out_bottom_20),
        .acc_out(acc_out_20)
    );

    // Cell row = 2, column = 1
    mac_cell # (
        .DATA_WIDTH(DATA_WIDTH),
        .ACC_WIDTH(ACC_WIDTH)
        ) mac_21 (
        .clk(clk),
        .rst(rst),
        .en(en),
        .in_left(out_right_20),
        .in_top(out_bottom_11),
        .out_right(out_right_21),
        .out_bottom(out_bottom_21),
        .acc_out(acc_out_21)
    );

    // Cell row = 2, column = 2
    mac_cell # (
        .DATA_WIDTH(DATA_WIDTH),
        .ACC_WIDTH(ACC_WIDTH)
        ) mac_22 (
        .clk(clk),
        .rst(rst),
        .en(en),
        .in_left(out_right_21),
        .in_top(out_bottom_12),
        .out_right(out_right_22),
        .out_bottom(out_bottom_22),
        .acc_out(acc_out_22)
    );

endmodule
