module UART_RX #(
    parameter CLK_PER_BIT = 50,    // system clock / baud rate = 50M / 1M = 50
    parameter dataWidth = 8,
    parameter stopBits = 2,  // either 1 or 2 stop bits
    parameter parityBits = 1,
    parameter packetSize = dataWidth + stopBits + parityBits + 1
    // Total Packet Size = dataWidth + stopBits + 1 Start Bit + 1 Parity Bit
) ( 
    input  logic                                clk,
    input  logic                                rst,
    input  logic                                dataIn,

    output logic    [dataWidth - 1 : 0]         RXout,
    output logic                                RXdone,
    output logic                                parityError
);
    
    localparam clkBits = $clog2(CLK_PER_BIT);
    localparam indexBits = $clog2(packetSize);

    logic   [indexBits - 1 : 0]     index;
    logic   [clkBits - 1 : 0]       clkCount;

    logic                           regInMeta;
    logic                           regIn;
    logic                           parity;

    logic    [dataWidth - 1 : 0]    dataOut;
    logic                           dataDone;

    // Remove Problems due to Metastability
    always_ff @(posedge clk) begin
        regInMeta <= dataIn;
        regIn <= regInMeta;
    end



    typedef enum logic [1:0] {
        IDLE,
        START,
        RECEIVE,
        DONE
    } 
    state_t;

    state_t state;

    always_ff @(posedge clk) begin
        if (rst) begin
            dataOut <= 0;
            state <= IDLE;
            index <= 1'b0;
            clkCount <= 0;
            dataDone <= 0;
        end
        else begin
            case (state)
                IDLE: begin
                    clkCount <= 0;
                    index <= 0;
                    dataOut <= 0;
                    dataDone <= 0;

                    if (regIn == 1'b0) begin    // Start Condition
                        state <= START;
                    end
                    else begin
                        state <= IDLE;
                    end
                end

                START: begin
                    if (clkCount == ((CLK_PER_BIT - 1) / 2)) begin
                        clkCount <= 0;
                        state <= RECEIVE;
                    end
                    else begin
                        clkCount <= clkCount + 1;
                        state <= START;
                    end

                end

                RECEIVE: begin

                    if (clkCount < CLK_PER_BIT - 1) begin
                        clkCount <= clkCount + 1;
                        state <= RECEIVE;
                    end

                    else begin
                        clkCount <= 0;
                        if (index < dataWidth) begin
                            dataOut[index] <= regIn;
                            index <= index + 1;
                            state <= RECEIVE;
                        end
                        else if (index == dataWidth && parityBits > 0) begin
                            parity <= regIn;
                            state <= DONE;
                        end
                        else begin
                            state <= DONE;
                        end
                    end
                end

                DONE: begin
                    if (clkCount < CLK_PER_BIT - 1) begin
                        clkCount <= clkCount + 1;
                        state <= DONE;
                    end
                    else begin
                        clkCount <= 0;
                        state <= IDLE;
                        dataDone <= 1'b1;
                        index <= 0;
                        RXout <= dataOut;
                    end
                end

                default: begin
                    state <= IDLE;
                end
                
            endcase
        end 
    end

    always_comb begin
        RXdone = dataDone;
        if (parityBits > 0) begin
            parityError = (^RXout) ^ parity;
        end
        else begin
            parityError = 0;
        end
    end
    
endmodule