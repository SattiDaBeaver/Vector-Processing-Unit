module double_buffer_array #(
    parameter DATA_WIDTH  = 8,
    parameter MATRIX_SIZE = 3
)(
    input  logic clk,
    input  logic rst,

    input  logic load_en,           // Write to load buffer
    input  logic swap_buffers,      // Toggle active buffer

    input  logic [DATA_WIDTH*MATRIX_SIZE-1:0] data_in_flat,  // Flattened input
    output logic [DATA_WIDTH*MATRIX_SIZE-1:0] data_out_flat  // Flattened output
);

    // Buffers: 1D packed vectors internally
    logic [DATA_WIDTH-1:0] buffer0 [0:MATRIX_SIZE-1];
    logic [DATA_WIDTH-1:0] buffer1 [0:MATRIX_SIZE-1];
    logic [DATA_WIDTH-1:0] data_in  [0:MATRIX_SIZE-1];
    logic [DATA_WIDTH-1:0] data_out [0:MATRIX_SIZE-1];

    logic active_sel;

    // Unpack flattened input
    always_comb begin
        for (int i = 0; i < MATRIX_SIZE; i++) begin
            data_in[i] = data_in_flat[i*DATA_WIDTH +: DATA_WIDTH];
        end
    end

    // Active buffer toggle & loading
    always_ff @(posedge clk) begin
        if (rst) begin
            active_sel <= 0;
            for (int i = 0; i < MATRIX_SIZE; i++) begin
                buffer0[i] <= '0;
                buffer1[i] <= '0;
            end
        end else begin
            if (swap_buffers)
                active_sel <= ~active_sel;

            if (load_en) begin
                if (active_sel == 0) begin
                    for (int i = 0; i < MATRIX_SIZE; i++)
                        buffer1[i] <= data_in[i];
                end else begin
                    for (int i = 0; i < MATRIX_SIZE; i++)
                        buffer0[i] <= data_in[i];
                end
            end
        end
    end

    // Select output buffer
    always_comb begin
        for (int i = 0; i < MATRIX_SIZE; i++) begin
            data_out[i] = (active_sel == 0) ? buffer0[i] : buffer1[i];
            data_out_flat[i*DATA_WIDTH +: DATA_WIDTH] = data_out[i];
        end
    end

endmodule
