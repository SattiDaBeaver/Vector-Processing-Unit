module mac_cell #(
    parameter DATA_WIDTH = 8,
    parameter ACC_WIDTH = 32
)(
    input  logic clk,
    input  logic rst,
    input  logic acc_rst,
    input  logic shift_en_right,
    input  logic shift_en_down,
    input  logic acc_en,

    input  logic [DATA_WIDTH-1:0]   in_left,
    input  logic [DATA_WIDTH-1:0]   in_top,

    output logic [DATA_WIDTH-1:0]   out_right,
    output logic [DATA_WIDTH-1:0]   out_bottom,
    output logic [ACC_WIDTH-1:0]    acc_out
);

    logic [ACC_WIDTH-1:0] mult, acc;

    always_comb begin
        mult = $signed(in_left) * $signed(in_top);
    end

    always_ff @(posedge clk) begin
        if (rst) begin
            acc         <= 0;
            out_right   <= 0;
            out_bottom  <= 0;
        end
        else begin
            if (acc_rst) begin
                acc         <= 0;
            end 
            else if (acc_en) begin
                acc <= $signed(acc) + $signed(mult);
            end 
            if (shift_en_right) begin
                out_right   <= in_left;
            end 
            if (shift_en_down) begin
                out_bottom  <= in_top;
            end 
        end
    end

    assign acc_out = acc;
endmodule
