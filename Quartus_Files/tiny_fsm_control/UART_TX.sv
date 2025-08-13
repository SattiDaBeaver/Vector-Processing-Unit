module UART_TX #(
    parameter CLK_PER_BIT = 50,    // system clock / baud rate = 50M / 1M = 50
    parameter dataWidth = 8,
    parameter stopBits = 1,        // either 1 or 2 stop bits
    parameter parityBits = 1,
    parameter packetSize = dataWidth + stopBits + parityBits + 1 
    // Total Packet Size = dataWidth + stopBits + 1 Start Bit + 1 Parity Bit
) ( 
    input  logic                                clk,
    input  logic                                rst,
    input  logic      [dataWidth - 1 : 0]       dataIn,
    input  logic                                TXen,

    output logic                                TXout,
    output logic                                TXdone,
    output logic                                busy
);
    
    localparam clkBits = $clog2(CLK_PER_BIT);
    localparam indexBits = $clog2(packetSize);

    logic   [packetSize - 1 : 0]        packet;
    logic                               parityBit;
    logic   [indexBits - 1 : 0]         index;
    logic   [clkBits - 1 : 0]           clkCount;



    typedef enum logic [1:0] {
        IDLE,
        TRANSMIT,
        DONE
    } 
    state_t;

    state_t state;

    always_comb begin
        parityBit = ^dataIn;    // 0 for even number of 1's, 1 for odd number of 1's
    end

    always_ff @(posedge clk) begin
        if (rst) begin
            TXout <= 1'b1;
            state <= IDLE;
            busy <= 1'b0;
            index <= 1'b0;
            clkCount <= 0;
            TXdone <= 0;
        end
        else begin
            case (state)
                IDLE: begin
                    TXout <= 1'b1;
                    index <= 1'b0;
                    clkCount <= 0;
                    TXdone <= 0;

                    if (TXen) begin
                        if (parityBits > 0) begin
                            packet <= {{stopBits{1'b1}}, parityBit, dataIn, 1'b0};
                        end 
                        else begin
                            packet <= {{stopBits{1'b1}}, dataIn, 1'b0};
                        end
                        //                ^                               ^
                        //                |                               |
                        //              Stop                            Start
                        busy <= 1'b1;
                        state <= TRANSMIT;
                    end
                    else begin
                        state <= IDLE;
                    end
                end

                TRANSMIT: begin
                    TXout <= packet[index];

                    if (clkCount < CLK_PER_BIT - 1) begin
                        clkCount <= clkCount + 1;
                        state <= TRANSMIT;
                    end

                    else begin
                        clkCount <= 0;
                        if (index == packetSize - 1) begin
                            state <= DONE;
                        end
                        else begin
                            index <= index + 1;
                            state <= TRANSMIT;
                        end
                    end
                end

                DONE: begin
                    state <= IDLE;
                    busy <= 1'b0;
                    TXdone <= 1'b1;
                end

                default: begin
                    state <= IDLE;
                end
            endcase
        end 
    end
    
endmodule