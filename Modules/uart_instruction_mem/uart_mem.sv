module uart_instr_mem_loader #(
    parameter INSTR_WIDTH = 32,
    parameter DEPTH = 256,
    parameter CLK_PER_BIT = 50 // match your baud
)(
    input  logic clk,
    input  logic rst,

    // UART RX pin from FTDI / USB-UART
    input  logic uart_rx,

    // Optional: UART TX pin back to PC for debugging
    output logic uart_tx,

    // FSM read interface
    input  logic [$clog2(DEPTH)-1:0] rd_addr,
    output logic [INSTR_WIDTH-1:0]   rd_data
);

    // ==========================================
    // Wires between UART and loader
    // ==========================================
    logic [7:0] rx_byte;
    logic       rx_done;
    logic       rx_parity_err;

    // TX not really used here, but wired for debug
    logic [7:0] tx_byte;
    logic       tx_en;
    logic       tx_done;
    logic       tx_busy;

    // ==========================================
    // Instantiate your UART wrapper
    // ==========================================
    UART_wrapper #(
        .CLK_PER_BIT(CLK_PER_BIT),
        .dataWidth(8),
        .parityBits(0),
        .stopBits(1)
    ) uart_inst (
        .clk(clk),
        .rst(rst),

        .TX_dataIn(tx_byte),
        .TX_en(tx_en),

        .RX_dataIn(uart_rx),

        .TX_out(uart_tx),
        .TX_done(tx_done),
        .TX_busy(tx_busy),

        .RX_dataOut(rx_byte),
        .RX_done(rx_done),
        .RX_parityError(rx_parity_err)
    );

    // ==========================================
    // Instruction RAM
    // ==========================================
    logic                      wr_en;
    logic [$clog2(DEPTH)-1:0]  wr_addr;
    logic [INSTR_WIDTH-1:0]    wr_data;

    instr_mem #(
        .INSTR_WIDTH(INSTR_WIDTH),
        .DEPTH(DEPTH)
    ) instr_mem_inst (
        .clk(clk),
        .wr_en(wr_en),
        .wr_addr(wr_addr),
        .wr_data(wr_data),
        .rd_addr(rd_addr),
        .rd_data(rd_data)
    );

    // ==========================================
    // Loader FSM: collect 4 bytes into 32-bit word
    // ==========================================
    logic [1:0] byte_count;
    logic [INSTR_WIDTH-1:0] shift_reg;

    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            byte_count <= 0;
            wr_addr    <= 0;
            wr_en      <= 0;
        end else begin
            wr_en <= 0; // default no write

            if (rx_done) begin
                // Shift left and append byte
                shift_reg <= {shift_reg[INSTR_WIDTH-9:0], rx_byte};
                byte_count <= byte_count + 1;

                if (byte_count == 2'd3) begin
                    // 4th byte received → write to memory
                    wr_data <= {shift_reg[INSTR_WIDTH-9:0], rx_byte};
                    wr_en   <= 1;
                    wr_addr <= wr_addr + 1;
                    byte_count <= 0;
                end
            end
        end
    end

endmodule
