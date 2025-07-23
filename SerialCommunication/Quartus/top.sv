module ftdi_loopback (
    input  logic clk, rst,

    // FTDI FIFO interface
    input  logic rxf_n,          // FTDI: data available
    input  logic txe_n,          // FTDI: ready to accept data
    inout  tri   [7:0] ftdi_data,// FTDI: bidirectional data bus
    output logic rd_n,           // FTDI: read enable (active low)
    output logic wr_n            // FTDI: write enable (active low)
);

    typedef enum logic [1:0] {IDLE, READ, WRITE} state_t;
    state_t state;

    logic [7:0] data_buf;
    logic       drive_data;

    assign ftdi_data = (drive_data) ? data_buf : 8'bZ;

    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            rd_n       <= 1;
            wr_n       <= 1;
            data_buf   <= 8'h00;
            drive_data <= 0;
            state      <= IDLE;
        end else begin
            case (state)
                IDLE: begin
                    rd_n       <= 1;
                    wr_n       <= 1;
                    drive_data <= 0;
                    if (~rxf_n) begin  // Data available from PC
                        rd_n   <= 0;
                        state  <= READ;
                    end
                end

                READ: begin
                    rd_n     <= 1;
                    data_buf <= ftdi_data;
                    state    <= WRITE;
                end

                WRITE: begin
                    if (~txe_n) begin  // PC ready to receive
                        drive_data <= 1;
                        wr_n       <= 0;
                        state      <= IDLE;
                    end
                end
            endcase
        end
    end
endmodule
