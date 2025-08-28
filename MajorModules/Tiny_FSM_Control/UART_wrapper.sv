module UART_wrapper #(
    parameter CLK_PER_BIT = 50,
    parameter dataWidth   = 8,
    parameter parityBits  = 0,
    parameter stopBits    = 1
) (
    input  logic                    clk,
    input  logic                    rst,

    input  logic   [dataWidth-1:0]  TX_dataIn,
    input  logic                    TX_en,

    input  logic                    RX_dataIn,

    output logic                    TX_out,
    output logic                    TX_done,
    output logic                    TX_busy,

    output logic   [dataWidth-1:0]  RX_dataOut,
    output logic                    RX_done,
    output logic                    RX_parityError
);
    // UART Transmitter Module
    UART_TX #(
        .CLK_PER_BIT(CLK_PER_BIT),
        .dataWidth(dataWidth),
        .parityBits(parityBits),
        .stopBits(stopBits)
        ) 
        UART_TX1 ( 
        .clk(clk),
        .rst(rst),
        .dataIn(TX_dataIn),
        .TXen(TX_en),

        .TXout(TX_out),
        .TXdone(TX_done),
        .busy(TX_busy)
    );

    // UART Receiver Module
    UART_RX #(
        .CLK_PER_BIT(CLK_PER_BIT),
        .dataWidth(dataWidth),
        .parityBits(parityBits),
        .stopBits(stopBits)
        )
        UART_RX1 (
        .clk(clk),
        .rst(rst),
        .dataIn(RX_dataIn),

        .RXout(RX_dataOut),
        .RXdone(RX_done),
        .parityError(RX_parityError)
    );
endmodule