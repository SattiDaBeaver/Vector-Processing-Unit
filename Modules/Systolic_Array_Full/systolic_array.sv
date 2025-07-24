module systolic_array #(
    parameter MATRIX_SIZE = 3,
    parameter DATA_WIDTH = 8,
    parameter ACC_WIDTH  = 32
)(
    input  logic                    clk,
    input  logic                    rst,
    input  logic                    en,
    input  logic [DATA_WIDTH-1:0]   in_left     [MATRIX_SIZE],
    input  logic [DATA_WIDTH-1:0]   in_top      [MATRIX_SIZE],
    output logic [DATA_WIDTH-1:0]   out_right   [MATRIX_SIZE],
    output logic [DATA_WIDTH-1:0]   out_bottom  [MATRIX_SIZE],
    output logic [ACC_WIDTH-1:0]    acc_out     [MATRIX_SIZE][MATRIX_SIZE]
);

    logic [DATA_WIDTH-1:0] right_wires  [MATRIX_SIZE][MATRIX_SIZE];
    logic [DATA_WIDTH-1:0] bottom_wires [MATRIX_SIZE][MATRIX_SIZE];

    genvar i, j;
    generate
        for (i = 0; i < MATRIX_SIZE; i++) begin : row
            for (j = 0; j < MATRIX_SIZE; j++) begin : col
                mac_cell #(
                    .DATA_WIDTH(DATA_WIDTH),
                    .ACC_WIDTH(ACC_WIDTH)
                ) mac_inst (
                    .clk(clk),
                    .rst(rst),
                    .en(en),
                    .in_left( (j == 0) ? in_left[i] : right_wires[i][j-1] ),
                    .in_top(  (i == 0) ? in_top[j]  : bottom_wires[i-1][j] ),
                    .out_right(right_wires[i][j]),
                    .out_bottom(bottom_wires[i][j]),
                    .acc_out(acc_out[i][j])
                );
            end
        end
    endgenerate

    // Expose last column and last row
    generate
        for (i = 0; i < MATRIX_SIZE; i++) begin
            assign out_right[i]  = right_wires[i][MATRIX_SIZE-1];
            assign out_bottom[i] = bottom_wires[MATRIX_SIZE-1][i];
        end
    endgenerate

endmodule
