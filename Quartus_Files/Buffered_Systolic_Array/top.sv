module top #(
    parameter DATA_WIDTH      = 8,
    parameter MATRIX_SIZE     = 8,
    parameter ACC_WIDTH       = 32,
    parameter ADDR_WIDTH      = $clog2(MATRIX_SIZE),
    parameter ACC_ADDR_WIDTH  = $clog2(MATRIX_SIZE * MATRIX_SIZE)
)(
    input  logic                                clk,
    input  logic                                rst,
    input  logic                                acc_rst,
    input  logic                                acc_en,
    input  logic                                shift_en,

    // Accumulator Address
    input  logic [ACC_ADDR_WIDTH-1:0]           addr_acc,

    // Top Buffer Interface
    input  logic                                buffer_rst_top,
    input  logic                                load_en_top,
    input  logic                                swap_buffers_top,
    input  logic [ADDR_WIDTH-1:0]               addr_top,
    input  logic [DATA_WIDTH-1:0]               data_in_top,

    // Left Buffer Interface
    input  logic                                buffer_rst_left,
    input  logic                                load_en_left,
    input  logic                                swap_buffers_left,
    input  logic [ADDR_WIDTH-1:0]               addr_left,
    input  logic [DATA_WIDTH-1:0]               data_in_left,

    // Outputs
    output logic [DATA_WIDTH*MATRIX_SIZE-1:0]   data_out_flat_bottom,
    output logic [DATA_WIDTH*MATRIX_SIZE-1:0]   data_out_flat_right,
    output logic [ACC_WIDTH-1:0]                acc_out
);

    systolic_module #(
        .DATA_WIDTH(DATA_WIDTH),
        .MATRIX_SIZE(MATRIX_SIZE),
        .ADDR_WIDTH(ADDR_WIDTH),
        .ACC_WIDTH(ACC_WIDTH),
        .ACC_ADDR_WIDTH(ACC_ADDR_WIDTH)
    ) dut (
        .clk(clk),
        .rst(rst),
        .acc_rst(acc_rst),
        .acc_en(acc_en),
        .shift_en(shift_en),
        .addr_acc(addr_acc),

        .buffer_rst_top(buffer_rst_top),
        .load_en_top(load_en_top),
        .swap_buffers_top(swap_buffers_top),
        .addr_top(addr_top),
        .data_in_top(data_in_top),

        .buffer_rst_left(buffer_rst_left),
        .load_en_left(load_en_left),
        .swap_buffers_left(swap_buffers_left),
        .addr_left(addr_left),
        .data_in_left(data_in_left),

        .data_out_flat_bottom(data_out_flat_bottom),
        .data_out_flat_right(data_out_flat_right),
        .acc_out(acc_out)
    );

endmodule
