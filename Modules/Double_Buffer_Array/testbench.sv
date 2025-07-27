module tb_double_buffer_array;

    localparam DATA_WIDTH  = 8;
    localparam MATRIX_SIZE = 3;

    logic clk = 0;
    logic rst;
    logic load_en;
    logic swap_buffers;
    logic [DATA_WIDTH*MATRIX_SIZE-1:0] data_in_flat;
    logic [DATA_WIDTH*MATRIX_SIZE-1:0] data_out_flat;

    // DUT
    double_buffer_array #(
        .DATA_WIDTH(DATA_WIDTH),
        .MATRIX_SIZE(MATRIX_SIZE)
    ) dut (
        .clk(clk),
        .rst(rst),
        .load_en(load_en),
        .swap_buffers(swap_buffers),
        .data_in_flat(data_in_flat),
        .data_out_flat(data_out_flat)
    );

    // Clock
    always #5 clk = ~clk;

    task set_data(input int a, b, c);
        data_in_flat = {c[DATA_WIDTH-1:0], b[DATA_WIDTH-1:0], a[DATA_WIDTH-1:0]};
    endtask

    task print_output(string label);
        $display("%s @ %0t ns → [%0d, %0d, %0d]",
                 label, $time,
                 data_out_flat[DATA_WIDTH*0 +: DATA_WIDTH],
                 data_out_flat[DATA_WIDTH*1 +: DATA_WIDTH],
                 data_out_flat[DATA_WIDTH*2 +: DATA_WIDTH]);
    endtask

    // Dump waveform
    initial begin
        $dumpfile("tb_double_buffer_array.vcd");
        $dumpvars(0, tb_double_buffer_array);
    end

    initial begin
        rst = 1;
        load_en = 0;
        swap_buffers = 0;
        set_data(0, 0, 0);
        #20;

        rst = 0;

        // Load 10, 20, 30 into buffer1 (inactive)
        set_data(10, 20, 30);
        load_en = 1; #10;
        load_en = 0;

        // Swap to buffer1
        swap_buffers = 1; #10;
        swap_buffers = 0;

        print_output("After Swap 1");

        // Load 40, 50, 60 into buffer0 (now inactive)
        set_data(40, 50, 60);
        load_en = 1; #10;
        load_en = 0;

        // Swap to buffer0
        swap_buffers = 1; #10;
        swap_buffers = 0;

        print_output("After Swap 2");

        // Reset
        rst = 1; #10;
        rst = 0;

        print_output("After Reset");

        $finish;
    end

endmodule
