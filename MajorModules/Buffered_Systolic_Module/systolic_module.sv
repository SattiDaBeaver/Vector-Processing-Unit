module systolic_module # (
    parameter DATA_WIDTH = 8,
    parameter MATRIX_SIZE = 8,
    parameter ADDR_WIDTH = $clog2(MATRIX_SIZE),
    parameter ACC_WIDTH  = 32,
    parameter ACC_ADDR_WIDTH = $clog2(MATRIX_SIZE*MATRIX_SIZE)
) (
    input  logic                                clk,
    input  logic                                rst,

    // Accumulator Control
    input  logic [ACC_ADDR_WIDTH-1:0]           addr_acc,
    input  logic                                acc_rst,
    input  logic                                acc_en,

    // Top Buffer Control
    input  logic                                buffer_rst_top,
    input  logic                                load_en_top,
    input  logic                                swap_buffers_top,
    input  logic [ADDR_WIDTH-1:0]               addr_top,
    input  logic [DATA_WIDTH-1:0]               data_in_top,

    // Left Buffer Control
    input  logic                                buffer_rst_left,
    input  logic                                load_en_left,
    input  logic                                swap_buffers_left,
    input  logic [ADDR_WIDTH-1:0]               addr_left,
    input  logic [DATA_WIDTH-1:0]               data_in_left,

    // Systolic Array Control
    input  logic                                shift_en,
    
    output logic [DATA_WIDTH*MATRIX_SIZE-1:0]   data_out_flat_bottom,
    output logic [DATA_WIDTH*MATRIX_SIZE-1:0]   data_out_flat_right,
    
    output logic [ACC_WIDTH-1:0]                acc_out
);

    logic   [DATA_WIDTH*MATRIX_SIZE-1:0]        reg_out_top;
    logic   [DATA_WIDTH*MATRIX_SIZE-1:0]        reg_out_left;

    // Accumulator Output Full
    logic [ACC_WIDTH*MATRIX_SIZE*MATRIX_SIZE-1:0] acc_out_flat;

    // Accumulator Output Logic
    always_comb begin
        acc_out = acc_out_flat[addr_acc * ACC_WIDTH +: ACC_WIDTH];
    end

    // Top Double Buffer (addressable)
    addressable_double_buffer #(
        .DATA_WIDTH(DATA_WIDTH),
        .MATRIX_SIZE(MATRIX_SIZE)
    ) top_buf (
        .clk(clk),
        .rst(rst),
        .buffer_rst(buffer_rst_top),
        .load_we(load_en_top),
        .swap_buffers(swap_buffers_top),
        .load_addr(addr_top),
        .load_data(data_in_top),
        .data_out_flat(reg_out_top)
    );

    // Left Double Buffer (addressable)
    addressable_double_buffer #(
        .DATA_WIDTH(DATA_WIDTH),
        .MATRIX_SIZE(MATRIX_SIZE)
    ) left_buf (
        .clk(clk),
        .rst(rst),
        .buffer_rst(buffer_rst_left),
        .load_we(load_en_left),
        .swap_buffers(swap_buffers_left),
        .load_addr(addr_left),
        .load_data(data_in_left),
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
