module top (
	input logic     [9:0]   SW,
	input logic     [1:0]   KEY,
	input logic             CLOCK_50,

	output logic    [6:0]   HEX5,
	output logic    [6:0]   HEX4,
	output logic    [6:0]   HEX3,
	output logic    [6:0]   HEX2,
	output logic    [6:0]   HEX1,
	output logic    [6:0]   HEX0,
	output logic    [9:0]   LEDR, 

    inout  logic    [15:0]  ARDUINO_IO,
    inout  logic    [35:0]  GPIO
);
    parameter F_CLK         = 50_000_000;
    parameter BAUD          = 921_600;
    parameter CLK_PER_BIT   = F_CLK / BAUD;

    parameter INSTR_WIDTH   = 32;
    parameter DEPTH         = 256;

    // Instructrion Mem Loader Wires
    logic                       clk, rst, uart_rx, uart_tx;
    logic   [$clog2(DEPTH)-1:0] rd_addr;
    logic   [INSTR_WIDTH-1:0]   rd_data;

    // Wire Assignments
    always_comb begin
        // Clock and Reset Wires
        clk = CLOCK_50;
        rst = ~KEY[0];

        // UART Wires
        uart_rx = GPIO[0];
        GPIO[1] = uart_rx;
        GPIO[2] = uart_tx;

        // Read Write Wires
        rd_addr = SW[7:0];
        LEDR[7:0] = rd_data;

        HEX2 = 7'h7F;
        HEX3 = 7'h7F;
    end

    // 7-Seg Hex 
    hex7seg H0 (rd_addr[3:0], HEX0);
    hex7seg H1 (rd_addr[7:4], HEX1);
    hex7seg H4 (rd_data[3:0], HEX4);
    hex7seg H5 (rd_data[7:4], HEX5);

    uart_instr_mem_loader #(
        .INSTR_WIDTH(INSTR_WIDTH),
        .DEPTH(DEPTH),
        .CLK_PER_BIT(CLK_PER_BIT) 
    ) MEM_UART_1 (
        .clk(clk),
        .rst(rst),

        // UART RX pin from FTDI / USB-UART
        .uart_rx(uart_rx),

        // Optional: UART TX pin back to PC for debugging
        .uart_tx(uart_tx),

        // FSM read interface
        .rd_addr(rd_addr),
        .rd_data(rd_data)
    );

endmodule