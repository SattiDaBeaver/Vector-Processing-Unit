`timescale 1ns / 1ps

module testbench;
    // Parameters
    localparam MATRIX_SIZE = 4;
    localparam DATA_WIDTH  = 8;
    localparam ACC_WIDTH   = 32;

    // DUT I/O
    logic clk = 0;
    logic rst;
    logic acc_rst;
    logic acc_en;
    logic shift_en;

    logic [DATA_WIDTH*MATRIX_SIZE-1:0] in_left_flat;
    logic [DATA_WIDTH*MATRIX_SIZE-1:0] in_top_flat;

    logic [DATA_WIDTH*MATRIX_SIZE-1:0] out_right_flat;
    logic [DATA_WIDTH*MATRIX_SIZE-1:0] out_bottom_flat;
    logic [ACC_WIDTH*MATRIX_SIZE*MATRIX_SIZE-1:0] acc_out_flat;

    // Clock generation
    always #5 clk = ~clk;

    // DUT instance
    systolic_array #(
        .MATRIX_SIZE(MATRIX_SIZE),
        .DATA_WIDTH(DATA_WIDTH),
        .ACC_WIDTH(ACC_WIDTH)
    ) dut (
        .clk(clk),
        .rst(rst),
        .acc_rst(acc_rst),
        .acc_en(acc_en),
        .shift_en(shift_en),
        .in_left_flat(in_left_flat),
        .in_top_flat(in_top_flat),
        .out_right_flat(out_right_flat),
        .out_bottom_flat(out_bottom_flat),
        .acc_out_flat(acc_out_flat)
    );

    // Task to pack an array into a flat bus
    task automatic pack_inputs(
        input  logic [DATA_WIDTH-1:0] left [MATRIX_SIZE],
        input  logic [DATA_WIDTH-1:0] top  [MATRIX_SIZE]
    );
        for (int i = 0; i < MATRIX_SIZE; i++) begin
            in_left_flat[i*DATA_WIDTH +: DATA_WIDTH] = left[i];
            in_top_flat[i*DATA_WIDTH +: DATA_WIDTH]  = top[i];
        end
    endtask

    // Stimulus
    initial begin
        logic [DATA_WIDTH-1:0] left_data [MATRIX_SIZE];
        logic [DATA_WIDTH-1:0] top_data  [MATRIX_SIZE];

        // Init
        rst = 1;
        acc_rst = 1;
        acc_en = 0;
        shift_en = 0;
        pack_inputs('{default:0}, '{default:0});
        #20;

        rst = 0;
        acc_rst = 0;
        acc_en = 1;
        shift_en = 1;

        // Feed a few diagonal inputs
        // for (int t = 0; t < 6; t++) begin
        //     for (int i = 0; i < MATRIX_SIZE; i++) begin
        //         left_data[i] = t + i + 1;
        //         top_data[i]  = (t + 1) * (i + 1);
        //     end
        //     pack_inputs(left_data, top_data);
        //     #10;
        // end

        for (int i = 0; i < MATRIX_SIZE; i++) begin
            left_data[i] = i + 1;
            top_data[i]  = i + 4;
        end
        pack_inputs(left_data, top_data);
        #10;
        pack_inputs('{default:0}, '{default:0});
        #10;
        for (int v = 0; v < 5; v++) begin
        $display("===== ACCUMULATOR OUTPUT =====");
            for (int i = 0; i < MATRIX_SIZE; i++) begin
                $write("[ ");
                for (int j = 0; j < MATRIX_SIZE; j++) begin
                    int unsigned acc_val;
                    acc_val = acc_out_flat[(i*MATRIX_SIZE + j)*ACC_WIDTH +: ACC_WIDTH];
                    $write("%0d ", acc_val);
                end
                $write("]\n");
            end
            #10;
        end

        // Stop shifting and accumulating
        acc_en = 0;
        shift_en = 0;

    end

endmodule
