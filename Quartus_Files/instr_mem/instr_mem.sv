module instr_mem #(
    parameter INSTR_WIDTH = 32,
    parameter DEPTH       = 256
)(
    input  logic                      clk,

    // Write port
    input  logic                      wr_en,
    input  logic [$clog2(DEPTH)-1:0]  wr_addr,
    input  logic [INSTR_WIDTH-1:0]    wr_data,

    // Read port
    input  logic [$clog2(DEPTH)-1:0]  rd_addr,
    output logic [INSTR_WIDTH-1:0]    rd_data
);

    // Memory array
    logic [INSTR_WIDTH-1:0] mem [0:DEPTH-1];

    // Write logic
    always_ff @(posedge clk) begin
        if (wr_en) begin
            mem[wr_addr] <= wr_data;
        end
    end

    // Read logic (synchronous read)
    always_ff @(posedge clk) begin
        rd_data <= mem[rd_addr];
    end

endmodule