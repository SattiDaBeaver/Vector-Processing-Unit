module shift_register_array # (
    parameter DATA_WIDTH = 8,
    parameter MATRIX_SIZE = 8,
    parameter FIFO_DEPTH = 15
) (
    input  logic                    clk,
    input  logic                    rst,
    input  logic                    en,
    input  logic [DATA_WIDTH-1:0]   fifo_in [MATRIX_SIZE-1:0],
    

    output logic [DATA_WIDTH-1:0]   fifo_out [MATRIX_SIZE-1:0],
);



endmodule 