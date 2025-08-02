module addressable_double_buffer #(
    parameter DATA_WIDTH  = 8,
    parameter MATRIX_SIZE = 3
)(
    input  logic clk,
    input  logic rst,
    input  logic buffer_rst, // Reset Current Buffer only

    input  logic [$clog2(MATRIX_SIZE)-1:0] load_addr,
    input  logic [DATA_WIDTH-1:0]          load_data,
    input  logic                           load_we,       // Write enable
    input  logic                           swap_buffers,  // Toggle active buffer

    output logic [DATA_WIDTH*MATRIX_SIZE-1:0] data_out_flat
);

    logic [DATA_WIDTH-1:0] buffer0 [0:MATRIX_SIZE-1];
    logic [DATA_WIDTH-1:0] buffer1 [0:MATRIX_SIZE-1];
    logic                  active_sel;

    logic [DATA_WIDTH-1:0] data_out [0:MATRIX_SIZE-1];

    // Reset and write logic
    always_ff @(posedge clk) begin
        if (rst) begin
            active_sel <= 0;
            for (int i = 0; i < MATRIX_SIZE; i++) begin
                buffer0[i] <= '0;
                buffer1[i] <= '0;
            end
        end
        else if (buffer_rst) begin
            if (active_sel == 0) begin
                for (int i = 0; i < MATRIX_SIZE; i++) begin
                    buffer1[i] <= '0;
                end
            end
            else begin 
                for (int i = 0; i < MATRIX_SIZE; i++) begin
                    buffer0[i] <= '0;
                end
            end
        end
        else begin
            if (swap_buffers)
                active_sel <= ~active_sel;

            if (load_we) begin
                if (active_sel == 0)
                    buffer1[load_addr] <= load_data; // load into inactive buffer
                else
                    buffer0[load_addr] <= load_data;
            end
        end
    end

    // Output logic (always driving)
    always_comb begin
        for (int i = 0; i < MATRIX_SIZE; i++) begin
            data_out[i] = (active_sel == 0) ? buffer0[i] : buffer1[i];
            data_out_flat[i*DATA_WIDTH +: DATA_WIDTH] = data_out[i];
        end
    end

endmodule
