module tb_systolic_array;

    // Parameters
    localparam MATRIX_SIZE = 3;
    localparam DATA_WIDTH  = 8;
    localparam ACC_WIDTH   = 32;

    // Clock and control
    logic clk;
    logic rst;
    logic en;

    // Inputs
    logic [DATA_WIDTH-1:0] in_left  [MATRIX_SIZE];
    logic [DATA_WIDTH-1:0] in_top   [MATRIX_SIZE];

    // Outputs
    logic [DATA_WIDTH-1:0] out_right  [MATRIX_SIZE];
    logic [DATA_WIDTH-1:0] out_bottom [MATRIX_SIZE];
    logic [ACC_WIDTH-1:0]  acc_out    [MATRIX_SIZE][MATRIX_SIZE];

    // Instantiate DUT
    systolic_array #(
        .MATRIX_SIZE(MATRIX_SIZE),
        .DATA_WIDTH(DATA_WIDTH),
        .ACC_WIDTH(ACC_WIDTH)
    ) dut (
        .clk(clk),
        .rst(rst),
        .en(en),
        .in_left(in_left),
        .in_top(in_top),
        .out_right(out_right),
        .out_bottom(out_bottom),
        .acc_out(acc_out)
    );

    // Clock generation
    initial clk = 0;
    always #5 clk = ~clk;  // 100MHz
  

    // Display task
    task print_matrix;
        $display("----- acc_out Matrix at time %0t -----", $time);
        for (int i = 0; i < MATRIX_SIZE; i++) begin
            for (int j = 0; j < MATRIX_SIZE; j++) begin
                $write("%4d ", acc_out[i][j]);
            end
            $write("\n");
        end
        $display("--------------------------------------\n");
    endtask

    // Stimulus
    initial begin
        // Wave Files
        $dumpfile("systolic_array.vcd");
        $dumpvars(0, tb_systolic_array);
        
        // Init
        rst = 1;
        en  = 0;
        for (int i = 0; i < MATRIX_SIZE; i++) begin
            in_left[i] = 0;
            in_top[i]  = 0;
        end

        // Release reset
        #20;
        rst = 0;
        en  = 1;

        // Apply inputs
        in_left[0] = 1;  in_top[0] = 4;
        in_left[1] = 2;  in_top[1] = 5;
        in_left[2] = 3;  in_top[2] = 6;

        #10;
        in_left[0] = 0; in_left[1] = 0; in_left[2] = 0;
        in_top[0] = 0;  in_top[1] = 0;  in_top[2] = 0;

        // Print every 20ns for a while
        repeat (10) begin
            #20;
            print_matrix();
        end

        // End simulation
        #100;
        $finish;
    end

endmodule
