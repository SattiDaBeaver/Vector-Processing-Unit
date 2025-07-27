module systolic_module # (
    parameter DATA_WIDTH = 8,
    parameter MATRIX_SIZE = 8,
    parameter ACC_WIDTH  = 32
) (
    input  logic                                clk,
    input  logic                                rst,
    input  logic                                acc_rst,
    input  logic                                acc_en,
    input  logic                                shift_en,

    input  logic                                load_en_top,            // Top - Write to load buffer
    input  logic                                swap_buffers_top,       // Top - Toggle active buffer

    input  logic                                load_en_left,           // Left - Write to load buffer
    input  logic                                swap_buffers_left,      // Left - Toggle active buffer

    input  logic [DATA_WIDTH*MATRIX_SIZE-1:0]   data_in_flat_top,       // Top - Flattened input
    input  logic [DATA_WIDTH*MATRIX_SIZE-1:0]   data_in_flat_left,      // Left - Flattened input
    
    output logic [DATA_WIDTH*MATRIX_SIZE-1:0]   data_out_flat_bottom,   // Bottom - Flattened output
    output logic [DATA_WIDTH*MATRIX_SIZE-1:0]   data_out_flat_right,    // Right - Flattened output
    
    output logic [ACC_WIDTH*MATRIX_SIZE*MATRIX_SIZE-1:0] acc_out_flat
);

    logic   [DATA_WIDTH*MATRIX_SIZE-1:0]        reg_out_top;
    logic   [DATA_WIDTH*MATRIX_SIZE-1:0]        reg_out_left;

    // Top Double Buffer
    double_buffer_array #(
        .DATA_WIDTH(DATA_WIDTH),
        .MATRIX_SIZE(MATRIX_SIZE)
    ) top_buf (
        .clk(clk),
        .rst(rst),
        .load_en(load_en_top),
        .swap_buffers(swap_buffers_top),
        .data_in_flat(data_in_flat_top),
        .data_out_flat(reg_out_top)
    );

    // Left Double Buffer
    double_buffer_array #(
        .DATA_WIDTH(DATA_WIDTH),
        .MATRIX_SIZE(MATRIX_SIZE)
    ) left_buf (
        .clk(clk),
        .rst(rst),
        .load_en(load_en_left),
        .swap_buffers(swap_buffers_left),
        .data_in_flat(data_in_flat_left),
        .data_out_flat(reg_out_left)
    );

    // Systolic Array
    systolic_array #(
        .MATRIX_SIZE(MATRIX_SIZE),
        .DATA_WIDTH(DATA_WIDTH),
        .ACC_WIDTH(ACC_WIDTH)
    ) sys_array (
        .clk(clk),
        .rst(rst),
        .acc_rst(acc_rst),
        .acc_en(acc_en),
        .shift_en(shift_en),

        .in_left_flat(reg_out_left),
        .in_top_flat(reg_out_top),

        .out_right_flat(data_out_flat_right),
        .out_bottom_flat(data_out_flat_bottom),

        .acc_out_flat(acc_out_flat)
    );

endmodule