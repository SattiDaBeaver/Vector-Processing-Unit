module tb_systolic_module;

    localparam DATA_WIDTH  = 8;
    localparam MATRIX_SIZE = 3;
    localparam ACC_WIDTH   = 32;
    localparam ADDR_WIDTH  = $clog2(MATRIX_SIZE);
    localparam ACC_ADDR_WIDTH = $clog2(MATRIX_SIZE * MATRIX_SIZE);

    logic clk = 0;
    logic rst;
    logic acc_rst;
    logic acc_en;
    logic shift_en;
    logic [ACC_ADDR_WIDTH-1:0]  addr_acc;
    logic [ACC_WIDTH-1:0]       acc_out;

    logic load_en_top;
    logic swap_buffers_top;
    logic [ADDR_WIDTH-1:0] addr_top;
    logic [DATA_WIDTH-1:0] data_in_top;

    logic load_en_left;
    logic swap_buffers_left;
    logic [ADDR_WIDTH-1:0] addr_left;
    logic [DATA_WIDTH-1:0] data_in_left;

    logic [DATA_WIDTH*MATRIX_SIZE-1:0] data_out_flat_bottom;
    logic [DATA_WIDTH*MATRIX_SIZE-1:0] data_out_flat_right;

    systolic_module #(
        .DATA_WIDTH(DATA_WIDTH),
        .MATRIX_SIZE(MATRIX_SIZE),
        .ACC_WIDTH(ACC_WIDTH)
    ) dut (
        .clk(clk),
        .rst(rst),
        .acc_rst(acc_rst),
        .acc_en(acc_en),
        .shift_en(shift_en),
        .addr_acc(addr_acc),

        .load_en_top(load_en_top),
        .buffer_rst_top(0),
        .swap_buffers_top(swap_buffers_top),
        .addr_top(addr_top),
        .data_in_top(data_in_top),

        .load_en_left(load_en_left),
        .buffer_rst_left(0),
        .swap_buffers_left(swap_buffers_left),
        .addr_left(addr_left),
        .data_in_left(data_in_left),

        .data_out_flat_bottom(data_out_flat_bottom),
        .data_out_flat_right(data_out_flat_right),
        .acc_out(acc_out)
    );

    // Clock
    always #5 clk = ~clk;

    task write_top_row(input int row_id, input int value);
        addr_top     = row_id;
        data_in_top  = value[DATA_WIDTH-1:0];
        load_en_top  = 1;
        #10;
        load_en_top  = 0;
    endtask

    task write_left_col(input int col_id, input int value);
        addr_left     = col_id;
        data_in_left  = value[DATA_WIDTH-1:0];
        load_en_left  = 1;
        #10;
        load_en_left  = 0;
    endtask

    initial begin
        $dumpfile("tb_systolic_module.vcd");
        $dumpvars(0, tb_systolic_module);

        clk = 0;
        rst = 1;
        acc_rst = 0;
        acc_en = 0;
        shift_en = 0;
        load_en_top = 0;
        swap_buffers_top = 0;
        load_en_left = 0;
        swap_buffers_left = 0;
        addr_top = 0;
        data_in_top = 0;
        addr_left = 0;
        data_in_left = 0;
        addr_acc = 0;

        #20;
        rst = 0;

        // Load top row (3 elements)
        write_top_row(0, 1);
        write_top_row(1, 2);
        write_top_row(2, 3);

        // Load left column (3 elements)
        write_left_col(0, 1);
        write_left_col(1, 2);
        write_left_col(2, 3);

        // Swap in both buffers
        swap_buffers_top  = 1;
        swap_buffers_left = 1;
        #10;
        swap_buffers_top  = 0;
        swap_buffers_left = 0;

        // Simulate systolic array run
        acc_rst = 1; #10; acc_rst = 0;
        acc_en = 1;
        shift_en = 1;

        repeat (3) #10;

        acc_en = 0;
        shift_en = 0;

      	repeat (9) begin
          	$display("acc_addr = %d", addr_acc);
        	$display("acc_out = %d", acc_out);
            addr_acc = (addr_acc + 1) % (MATRIX_SIZE * MATRIX_SIZE);
        	#10;  // Enough cycles for 3x3 to finish
      	end

        acc_en = 1;
        shift_en = 1;

        repeat (3) #10;

        acc_en = 0;
        shift_en = 0;

      	repeat (9) begin
          	$display("acc_addr = %d", addr_acc);
        	$display("acc_out = %d", acc_out);
            addr_acc = (addr_acc + 1) % (MATRIX_SIZE * MATRIX_SIZE);
        	#10;  // Enough cycles for 3x3 to finish
      	end

        acc_en = 0;
        shift_en = 0; 

        $finish;
    end

endmodule
