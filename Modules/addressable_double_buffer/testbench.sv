module tb_addressable_double_buffer;

    localparam DATA_WIDTH  = 8;
    localparam MATRIX_SIZE = 3;

    logic clk = 0;
    logic rst;
    logic [$clog2(MATRIX_SIZE)-1:0] load_addr;
    logic [DATA_WIDTH-1:0]          load_data;
    logic load_we;
    logic swap_buffers;
    logic [DATA_WIDTH*MATRIX_SIZE-1:0] data_out_flat;

    // DUT
    addressable_double_buffer #(
        .DATA_WIDTH(DATA_WIDTH),
        .MATRIX_SIZE(MATRIX_SIZE)
    ) dut (
        .clk(clk),
        .rst(rst),
        .load_addr(load_addr),
        .load_data(load_data),
        .load_we(load_we),
        .swap_buffers(swap_buffers),
        .data_out_flat(data_out_flat)
    );

    // Clock
    always #5 clk = ~clk;

    task write_data(input int index, input int value);
        load_addr = index;
        load_data = value;
        load_we = 1; #10;
        load_we = 0;
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
        $dumpfile("tb_addressable_double_buffer.vcd");
        $dumpvars(0, tb_addressable_double_buffer);
    end

    initial begin
        rst = 1;
        load_we = 0;
        swap_buffers = 0;
        #20;

        rst = 0;

        // Load into buffer1 (inactive)
        write_data(0, 10);
        write_data(1, 20);
        write_data(2, 30);

        // Swap to buffer1
        swap_buffers = 1; #10;
        swap_buffers = 0;

        print_output("After Swap 1");

        // Load into buffer0 (now inactive)
        write_data(0, 40);
        write_data(1, 50);
        write_data(2, 60);

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
