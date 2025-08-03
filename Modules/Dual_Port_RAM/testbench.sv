module tb_dp_ram;

    parameter DATA_WIDTH = 8;
    parameter ADDR_WIDTH = 6;
    parameter DEPTH = 1 << ADDR_WIDTH;

    logic clk = 0;

    // Port A
    logic we_a;
    logic [ADDR_WIDTH-1:0] addr_a;
    logic [DATA_WIDTH-1:0] din_a;
    logic [DATA_WIDTH-1:0] dout_a;

    // Port B
    logic we_b;
    logic [ADDR_WIDTH-1:0] addr_b;
    logic [DATA_WIDTH-1:0] din_b;
    logic [DATA_WIDTH-1:0] dout_b;

    // Clock generation
    always #5 clk = ~clk;

    // Instantiate DUT
    dp_ram #(
        .DATA_WIDTH(DATA_WIDTH),
        .ADDR_WIDTH(ADDR_WIDTH)
    ) dut (
        .clk(clk),
        .we_a(we_a),
        .addr_a(addr_a),
        .din_a(din_a),
        .dout_a(dout_a),
        .we_b(we_b),
        .addr_b(addr_b),
        .din_b(din_b),
        .dout_b(dout_b)
    );

    initial begin
        $dumpfile("tb_dp_ram.vcd");
        $dumpvars(0, tb_dp_ram);

        $display("Starting DP RAM test...");

        // Initialize inputs
        we_a = 0; addr_a = 0; din_a = 0;
        we_b = 0; addr_b = 0; din_b = 0;

        @(posedge clk);

        // Write to Port A
        we_a = 1;
        addr_a = 10'h01;
        din_a = 8'hAA;
        @(posedge clk);
        we_a = 0;

        // Write to Port B
        we_b = 1;
        addr_b = 10'h02;
        din_b = 8'hBB;
        @(posedge clk);
        we_b = 0;

        // Read from both ports
        addr_a = 10'h01;
        addr_b = 10'h02;
        @(posedge clk);
        $display("Read A: %h (Expected AA), Read B: %h (Expected BB)", dout_a, dout_b);

        // Simultaneous write
        addr_a = 10'h03; din_a = 8'h11; we_a = 1;
        addr_b = 10'h04; din_b = 8'h22; we_b = 1;
        @(posedge clk);
        we_a = 0; we_b = 0;

        // Simultaneous read
        addr_a = 10'h03;
        addr_b = 10'h04;
        @(posedge clk);
        $display("Read A: %h (Expected 11), Read B: %h (Expected 22)", dout_a, dout_b);

        $display("Test finished.");
        $finish;
    end

endmodule
