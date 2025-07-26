module systolic_array #(
    parameter MATRIX_SIZE = 3,
    parameter DATA_WIDTH = 8,
    parameter ACC_WIDTH  = 32
)(
    input  logic                         clk,
    input  logic                         rst,
    input  logic                         acc_rst,
    input  logic                         acc_en,
    input  logic                         shift_en,

    input  logic [DATA_WIDTH*MATRIX_SIZE-1:0] in_left_flat,
    input  logic [DATA_WIDTH*MATRIX_SIZE-1:0] in_top_flat,

    output logic [DATA_WIDTH*MATRIX_SIZE-1:0] out_right_flat,
    output logic [DATA_WIDTH*MATRIX_SIZE-1:0] out_bottom_flat,
    output logic [ACC_WIDTH*MATRIX_SIZE*MATRIX_SIZE-1:0] acc_out_flat
);

    logic [DATA_WIDTH-1:0] in_left  [MATRIX_SIZE];
    logic [DATA_WIDTH-1:0] in_top   [MATRIX_SIZE];
    logic [DATA_WIDTH-1:0] out_right[MATRIX_SIZE];
    logic [DATA_WIDTH-1:0] out_bottom[MATRIX_SIZE];
    logic [ACC_WIDTH-1:0]  acc_out  [MATRIX_SIZE][MATRIX_SIZE];

    // Unpack flattened inputs
    genvar x;
    generate
        for (x = 0; x < MATRIX_SIZE; x++) begin : assign_input
            assign in_left[x]     = in_left_flat[x*DATA_WIDTH +: DATA_WIDTH];
            assign in_top[x]      = in_top_flat[x*DATA_WIDTH +: DATA_WIDTH];
            assign out_right_flat[x*DATA_WIDTH +: DATA_WIDTH]  = out_right[x];
            assign out_bottom_flat[x*DATA_WIDTH +: DATA_WIDTH] = out_bottom[x];
        end
    endgenerate

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
                    .acc_rst(acc_rst),
                    .acc_en(acc_en),
                    .shift_en(shift_en),

                    .in_left( (j == 0) ? in_left[i] : right_wires[i][j-1] ),
                    .in_top(  (i == 0) ? in_top[j]  : bottom_wires[i-1][j] ),

                    .out_right(right_wires[i][j]),
                    .out_bottom(bottom_wires[i][j]),
                    .acc_out(acc_out[i][j])
                );
            end
        end
    endgenerate

    // Collect boundary wires
    generate
        for (i = 0; i < MATRIX_SIZE; i++) begin : assign_output
            assign out_right[i]  = right_wires[i][MATRIX_SIZE-1];
            assign out_bottom[i] = bottom_wires[MATRIX_SIZE-1][i];
        end
    endgenerate

    // Flatten acc_out
    generate
        for (i = 0; i < MATRIX_SIZE; i++) begin : row_out
            for (j = 0; j < MATRIX_SIZE; j++) begin : col_out
                assign acc_out_flat[(i*MATRIX_SIZE + j)*ACC_WIDTH +: ACC_WIDTH] = acc_out[i][j];
            end
        end
    endgenerate

endmodule
