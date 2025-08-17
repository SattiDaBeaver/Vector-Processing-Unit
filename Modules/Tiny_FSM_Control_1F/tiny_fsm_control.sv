module tiny_fsm_control #(
    //======================================//
    //              Parameters              //
    //======================================//

    // Systolic Module Parameters
    parameter DATA_WIDTH        = 8,
    parameter MATRIX_SIZE       = 8,
    parameter ACC_WIDTH         = 32,
    parameter ADDR_WIDTH        = $clog2(MATRIX_SIZE),
    parameter ACC_ADDR_WIDTH    = $clog2(MATRIX_SIZE*MATRIX_SIZE),

    // DPRAM Parameters
    parameter DP_ADDR_WIDTH     = 10,

    // UART Instruction Memory Loader Parameters
    parameter F_CLK             = 50_000_000,
    parameter BAUD              = 921_600,
    parameter CLK_PER_BIT       = F_CLK / BAUD,
    parameter INSTR_WIDTH       = 32,
    parameter INSTR_DEPTH       = 256,

    // FSM Parameters
    parameter PC_WIDTH          = $clog2(INSTR_DEPTH)
) (
    //======================================//
    //          Module I/O Wires            //
    //======================================//

    // Clock and Reset
    input  logic                        clk,
    input  logic                        fsm_rst,

    // FSM External Control Wires
    input  logic                        step,
    input  logic                        run,
    input  logic                        halt,

    // Instruction Memory Logic Wires
    input  logic [INSTR_WIDTH-1:0]      rd_data,
    output logic [PC_WIDTH-1:0]         rd_addr,

    // Debug Outputs
    output logic [PC_WIDTH-1:0]         pc_out,
    output logic [INSTR_WIDTH-1:0]      curr_instr_out,
    output logic [INSTR_WIDTH-1:0]      next_instr_out,
    output logic [2:0]                  state_out
);
    

    //======================================//
    //             Module Wires             //
    //======================================//

    // Systolic Module Wires
    logic                               systolic_master_rst;
    logic                               acc_rst;
    logic                               acc_en;
    logic                               shift_en_right;
    logic                               shift_en_down;

    logic                               buffer_rst_top;
    logic                               load_en_top;
    logic                               swap_buffers_top;
    logic [ADDR_WIDTH-1:0]              addr_top;
    logic [DATA_WIDTH-1:0]              data_in_top;

    logic                               buffer_rst_left;
    logic                               load_en_left;
    logic                               swap_buffers_left;
    logic [ADDR_WIDTH-1:0]              addr_left;
    logic [DATA_WIDTH-1:0]              data_in_left;

    logic [DATA_WIDTH*MATRIX_SIZE-1:0]  data_out_flat_bottom;
    logic [DATA_WIDTH*MATRIX_SIZE-1:0]  data_out_flat_right;

    logic [ACC_ADDR_WIDTH-1:0]          addr_acc;
    logic [ACC_WIDTH-1:0]               acc_out;

    // Dual Port RAM Wires
    logic                               we_a;
    logic [DP_ADDR_WIDTH-1:0]           addr_a;
    logic [DATA_WIDTH-1:0]              din_a;
    logic [DATA_WIDTH-1:0]              dout_a;
    logic                               we_b;
    logic [DP_ADDR_WIDTH-1:0]           addr_b;
    logic [DATA_WIDTH-1:0]              din_b;
    logic [DATA_WIDTH-1:0]              dout_b;

    //======================================//
    //          FSM Control Logic           //
    //======================================//

    typedef struct packed {
        logic           LOAD_LEFT;      // bit 31
        logic           LOAD_TOP;       // bit 30
        logic           SWAP_LEFT;      // bit 29
        logic           SWAP_TOP;       // bit 28
        logic           SHIFT_RIGHT;    // bit 27
        logic           SHIFT_DOWN;     // bit 26
        logic           LOAD_ACC;       // bit 25
        logic           WRITE_ACC_OUT;  // bit 24
        logic           WAIT_CYCLES;    // bit 23
        logic           JUMP;           // bit 22 -- Subject to change
        logic           CLR;            // bit 21
        logic           NOP;            // bit 20 -- Subject to change

        logic [1:0]     RESERVED;       // bits 19 down to 18 (2 bits)

        logic [4:0]     FLAGS;          // bits 17 down to 13 (5 bits)
        logic [12:0]    ADDR;           // bits 12 down to 0 (13 bits)
    } instruction_t;

    typedef enum logic [2:0] { 
        IDLE,
        PRE_FETCH,
        FETCH_EXECUTE,
        LOAD_COMMIT,
        WAIT
    } fsm_state_t;

    logic [PC_WIDTH-1:0]    pc;
    logic [12:0]            wait_counter;
    logic                   load_left_delay, load_top_delay;

    instruction_t           curr_instr, next_instr;
    fsm_state_t             state;

    // Assign Wires 
    always_comb begin
        // Output Wires
        pc_out          = pc;
        next_instr_out  = next_instr;
        curr_instr_out  = curr_instr;
        state_out       = state;

        // Internal Wiring
        rd_addr         = pc;   // Intruction Read Address = Program Counter
    end

    // FSM Control Logic
    always_ff @(posedge clk) begin
        if (fsm_rst) begin
            state       <= IDLE;  // Reset State
            pc          <= 0;     // Reset Program Counter
            curr_instr  <= 0;
            next_instr  <= 0;
        end
        else begin
            // Default Wire Values
            // Multi-Cycle Load
            load_left_delay     <= 0;
            load_top_delay      <= 0;

            // Systolic Array
            // Load
            load_en_left        <= 0;
            load_en_top         <= 0;
            // Swap
            swap_buffers_left   <= 0;
            swap_buffers_top    <= 0;
            // Shift
            shift_en_right      <= 0;
            shift_en_down       <= 0;
            // Accumulator Load
            acc_en              <= 0;

            // Dual Port RAM
            addr_a              <= 0;
            addr_b              <= 0;

            

            // Case Statement for FSM
            case (state) 
                IDLE: begin
                    if (step) begin
                        // State Wires
                        state       <= PRE_FETCH;
                        next_instr  <= instruction_t'(rd_data);
                        pc          <= pc + 1;  // Increment PC
                    end
                end
                
                PRE_FETCH: begin
                    if (step) begin
                        // Fetch Next Instruction
                        state       <= FETCH_EXECUTE;
                        curr_instr  <= next_instr;
                        next_instr  <= instruction_t'(rd_data);
                        pc          <= pc + 1;
                    end
                end

                FETCH_EXECUTE: begin
                    // Default State
                    state       <= FETCH_EXECUTE;
                    if (step) begin
                        // Default Values
                        curr_instr  <= next_instr;
                        next_instr  <= instruction_t'(rd_data);
                        pc          <= pc + 1;

                        // Load Instructions
                        // If double instruction LOAD_LEFT and LOAD_TOP
                        if (curr_instr.LOAD_LEFT && curr_instr.LOAD_TOP) begin
                            addr_a          <= curr_instr.ADDR[9:0];
                            addr_b          <= curr_instr.ADDR[9:0] + 'h40;
                            addr_left       <= curr_instr.ADDR[12:10];
                            addr_top        <= curr_instr.ADDR[12:10];
                            state           <= LOAD_COMMIT;
                            load_left_delay <= 1;
                            load_top_delay  <= 1;
                        end
                        else begin
                            if (curr_instr.LOAD_LEFT) begin
                                if (curr_instr.FLAGS[0]) begin // Immediate Flag
                                    data_in_left    <= curr_instr.ADDR[7:0];
                                    addr_left       <= curr_instr.ADDR[12:10];
                                    load_en_left    <= 1;
                                end else begin
                                    addr_a          <= curr_instr.ADDR[9:0];
                                    addr_left       <= curr_instr.ADDR[12:10];
                                    state           <= LOAD_COMMIT;
                                    load_left_delay <= 1;
                                end
                            end
                            if (curr_instr.LOAD_TOP) begin
                                if (curr_instr.FLAGS[0]) begin // Immediate Flag
                                    data_in_top     <= curr_instr.ADDR[7:0];
                                    addr_top        <= curr_instr.ADDR[12:10];
                                    load_en_top     <= 1;
                                end else begin
                                    addr_b          <= curr_instr.ADDR[9:0];
                                    addr_top        <= curr_instr.ADDR[12:10];
                                    state           <= LOAD_COMMIT;
                                    load_top_delay  <= 1;
                                end
                            end
                        end

                        // Swap Instructions
                        if (curr_instr.SWAP_LEFT) begin
                            swap_buffers_left   <= 1;
                        end
                        if (curr_instr.SWAP_TOP) begin
                            swap_buffers_top    <= 1;
                        end

                        // Shift Instructions
                        if (curr_instr.SHIFT_RIGHT) begin
                            shift_en_right  <= 1;
                        end
                        if (curr_instr.SHIFT_DOWN) begin
                            shift_en_down   <= 1;
                        end

                        // Accumulator Load Instruction
                        if (curr_instr.LOAD_ACC) begin
                            acc_en  <= 1;
                        end
                    end
                end

                LOAD_COMMIT: begin
                    state       <= FETCH_EXECUTE;
                    if (load_left_delay) begin
                        data_in_left    <= dout_a;
                        addr_left       <= curr_instr.ADDR[12:10];
                        load_en_left    <= 1;
                    end
                    if (load_top_delay) begin
                        data_in_top     <= dout_b;
                        addr_top        <= curr_instr.ADDR[12:10];
                        load_en_top     <= 1;
                    end
                end

                WAIT: begin

                end

            endcase

        end
    end

    //======================================//
    //        Module Instantiation          //
    //======================================//

    // Systolic Module Instantiation
    systolic_module #(
        .DATA_WIDTH(DATA_WIDTH),
        .MATRIX_SIZE(MATRIX_SIZE),
        .ACC_WIDTH(ACC_WIDTH)
        ) SYS_ARRAY (
        .clk(clk),
        .rst(systolic_master_rst),
        .acc_rst(acc_rst),
        .acc_en(acc_en),
        .shift_en_right(shift_en_right),
        .shift_en_down(shift_en_down),
        .addr_acc(addr_acc),

        .buffer_rst_top(buffer_rst_top),
        .load_en_top(load_en_top),
        .swap_buffers_top(swap_buffers_top),
        .addr_top(addr_top),
        .data_in_top(data_in_top),

        .buffer_rst_left(buffer_rst_left),
        .load_en_left(load_en_left),
        .swap_buffers_left(swap_buffers_left),
        .addr_left(addr_left),
        .data_in_left(data_in_left),

        .data_out_flat_bottom(data_out_flat_bottom),
        .data_out_flat_right(data_out_flat_right),
        .acc_out(acc_out)
    );

    // Dual Port RAM Instantiation
    dp_ram #(
        .DATA_WIDTH(DATA_WIDTH),
        .ADDR_WIDTH(DP_ADDR_WIDTH)
        ) DPRAM (
        .clk(clk),

        // Port A
        .we_a(we_a),
        .addr_a(addr_a),
        .din_a(din_a),
        .dout_a(dout_a),

        // Port B
        .we_b(we_b),
        .addr_b(addr_b),
        .din_b(din_b),
        .dout_b(dout_b)
    );

endmodule : tiny_fsm_control

module addressable_double_buffer #(
    parameter DATA_WIDTH  = 8,
    parameter MATRIX_SIZE = 3
)(
    input  logic clk,
    input  logic rst,
    input  logic buffer_rst, // Reset Current Buffer only

    input  logic [$clog2(MATRIX_SIZE)-1:0] load_addr,
    input  logic [DATA_WIDTH-1:0]          load_data,
    input  logic                           load_we,       // Write enable
    input  logic                           swap_buffers,  // Toggle active buffer

    output logic [DATA_WIDTH*MATRIX_SIZE-1:0] data_out_flat
);

    logic [DATA_WIDTH-1:0] buffer0 [0:MATRIX_SIZE-1];
    logic [DATA_WIDTH-1:0] buffer1 [0:MATRIX_SIZE-1];
    logic                  active_sel;

    logic [DATA_WIDTH-1:0] data_out [0:MATRIX_SIZE-1];

    // Reset and write logic
    always_ff @(posedge clk) begin
        if (rst) begin
            active_sel <= 0;
            for (int i = 0; i < MATRIX_SIZE; i++) begin
                buffer0[i] <= '0;
                buffer1[i] <= '0;
            end
        end
        else if (buffer_rst) begin
            if (active_sel == 0) begin
                for (int i = 0; i < MATRIX_SIZE; i++) begin
                    buffer1[i] <= '0;
                end
            end
            else begin 
                for (int i = 0; i < MATRIX_SIZE; i++) begin
                    buffer0[i] <= '0;
                end
            end
        end
        else begin
            if (swap_buffers)
                active_sel <= ~active_sel;

            if (load_we) begin
                if (active_sel == 0)
                    buffer1[load_addr] <= load_data; // load into inactive buffer
                else
                    buffer0[load_addr] <= load_data;
            end
        end
    end

    // Output logic (always driving)
    always_comb begin
        for (int i = 0; i < MATRIX_SIZE; i++) begin
            data_out[i] = (active_sel == 0) ? buffer0[i] : buffer1[i];
            data_out_flat[i*DATA_WIDTH +: DATA_WIDTH] = data_out[i];
        end
    end

endmodule

module dp_ram #(
    parameter DATA_WIDTH = 8,
    parameter ADDR_WIDTH = 10  // 2^10 = 1024 entries
) (
    input  logic                    clk,

    // Port A
    input  logic                    we_a,
    input  logic [ADDR_WIDTH-1:0]   addr_a,
    input  logic [DATA_WIDTH-1:0]   din_a,
    output logic [DATA_WIDTH-1:0]   dout_a,

    // Port B
    input  logic                    we_b,
    input  logic [ADDR_WIDTH-1:0]   addr_b,
    input  logic [DATA_WIDTH-1:0]   din_b,
    output logic [DATA_WIDTH-1:0]   dout_b
);

    logic [DATA_WIDTH-1:0] mem [0:(1<<ADDR_WIDTH)-1];

    always_ff @(posedge clk) begin
        if (we_a) begin
            mem[addr_a] <= din_a;
        end
        dout_a <= mem[addr_a];
    end

    always_ff @(posedge clk) begin
        if (we_b) begin
            mem[addr_b] <= din_b;
        end
        dout_b <= mem[addr_b];
    end

endmodule

module hex7seg (hex, display);
    input [3:0] hex;
    output [6:0] display;

    reg [6:0] display;

    /*
     *       0  
     *      ---  
     *     |   |
     *    5|   |1
     *     | 6 |
     *      ---  
     *     |   |
     *    4|   |2
     *     |   |
     *      ---  
     *       3  
     */
    always @ (hex)
        case (hex)
            4'h0: display = 7'b1000000;
            4'h1: display = 7'b1111001;
            4'h2: display = 7'b0100100;
            4'h3: display = 7'b0110000;
            4'h4: display = 7'b0011001;
            4'h5: display = 7'b0010010;
            4'h6: display = 7'b0000010;
            4'h7: display = 7'b1111000;
            4'h8: display = 7'b0000000;
            4'h9: display = 7'b0011000;
            4'hA: display = 7'b0001000;
            4'hB: display = 7'b0000011;
            4'hC: display = 7'b1000110;
            4'hD: display = 7'b0100001;
            4'hE: display = 7'b0000110;
            4'hF: display = 7'b0001110;
        endcase
endmodule: hex7seg

module instr_mem #(
    parameter INSTR_WIDTH = 32,
    parameter DEPTH       = 256
)(
    input  logic                      clk,

    // Write port
    input  logic                      wr_en,
    input  logic [$clog2(DEPTH)-1:0]  wr_addr,
    input  logic [INSTR_WIDTH-1:0]    wr_data,

    // Read port
    input  logic [$clog2(DEPTH)-1:0]  rd_addr,
    output logic [INSTR_WIDTH-1:0]    rd_data
);

    // Memory array
    (* ramstyle = "M9K" *) logic [INSTR_WIDTH-1:0] mem [0:DEPTH-1];

    // Write logic
    always_ff @(posedge clk) begin
        if (wr_en) begin
            mem[wr_addr] <= wr_data;
        end
    end

    // Read logic (synchronous read)
    always_ff @(posedge clk) begin
        rd_data <= mem[rd_addr];
    end

endmodule

module mac_cell #(
    parameter DATA_WIDTH = 8,
    parameter ACC_WIDTH = 32
)(
    input  logic clk,
    input  logic rst,
    input  logic acc_rst,
    input  logic shift_en_right,
    input  logic shift_en_down,
    input  logic acc_en,

    input  logic [DATA_WIDTH-1:0]   in_left,
    input  logic [DATA_WIDTH-1:0]   in_top,

    output logic [DATA_WIDTH-1:0]   out_right,
    output logic [DATA_WIDTH-1:0]   out_bottom,
    output logic [ACC_WIDTH-1:0]    acc_out
);

    logic [ACC_WIDTH-1:0] mult, acc;

    always_comb begin
        mult = $signed(in_left) * $signed(in_top);
    end

    always_ff @(posedge clk) begin
        if (rst) begin
            acc         <= 0;
            out_right   <= 0;
            out_bottom  <= 0;
        end
        else begin
            if (acc_rst) begin
                acc         <= 0;
            end 
            else if (acc_en) begin
                acc <= $signed(acc) + $signed(mult);
            end 
            if (shift_en_right) begin
                out_right   <= in_left;
            end 
            if (shift_en_down) begin
                out_bottom  <= in_top;
            end 
        end
    end

    assign acc_out = acc;
endmodule

module systolic_array #(
    parameter MATRIX_SIZE = 3,
    parameter DATA_WIDTH = 8,
    parameter ACC_WIDTH  = 32
)(
    input  logic                         clk,
    input  logic                         rst,
    input  logic                         acc_rst,
    input  logic                         acc_en,
    input  logic                         shift_en_right,
    input  logic                         shift_en_down,

    input  logic [DATA_WIDTH*MATRIX_SIZE-1:0] in_left_flat,
    input  logic [DATA_WIDTH*MATRIX_SIZE-1:0] in_top_flat,

    output logic [DATA_WIDTH*MATRIX_SIZE-1:0] out_right_flat,
    output logic [DATA_WIDTH*MATRIX_SIZE-1:0] out_bottom_flat,
    output logic [ACC_WIDTH*MATRIX_SIZE*MATRIX_SIZE-1:0] acc_out_flat
);

    logic [DATA_WIDTH-1:0] in_left  [MATRIX_SIZE];
    logic [DATA_WIDTH-1:0] in_top   [MATRIX_SIZE];
    logic [DATA_WIDTH-1:0] out_right[MATRIX_SIZE];
    logic [DATA_WIDTH-1:0] out_bottom[MATRIX_SIZE];
    logic [ACC_WIDTH-1:0]  acc_out  [MATRIX_SIZE][MATRIX_SIZE];

    // Unpack flattened inputs
    genvar x;
    generate
        for (x = 0; x < MATRIX_SIZE; x++) begin : assign_input
            assign in_left[x]     = in_left_flat[x*DATA_WIDTH +: DATA_WIDTH];
            assign in_top[x]      = in_top_flat[x*DATA_WIDTH +: DATA_WIDTH];
            assign out_right_flat[x*DATA_WIDTH +: DATA_WIDTH]  = out_right[x];
            assign out_bottom_flat[x*DATA_WIDTH +: DATA_WIDTH] = out_bottom[x];
        end
    endgenerate

    logic [DATA_WIDTH-1:0] right_wires  [MATRIX_SIZE][MATRIX_SIZE];
    logic [DATA_WIDTH-1:0] bottom_wires [MATRIX_SIZE][MATRIX_SIZE];

    genvar i, j;
    generate
        for (i = 0; i < MATRIX_SIZE; i++) begin : row
            for (j = 0; j < MATRIX_SIZE; j++) begin : col
                mac_cell #(
                    .DATA_WIDTH(DATA_WIDTH),
                    .ACC_WIDTH(ACC_WIDTH)
                ) mac_inst (
                    .clk(clk),
                    .rst(rst),
                    .acc_rst(acc_rst),
                    .acc_en(acc_en),
                    .shift_en_right(shift_en_right),
                    .shift_en_down(shift_en_down),

                    .in_left( (j == 0) ? in_left[i] : right_wires[i][j-1] ),
                    .in_top(  (i == 0) ? in_top[j]  : bottom_wires[i-1][j] ),

                    .out_right(right_wires[i][j]),
                    .out_bottom(bottom_wires[i][j]),
                    .acc_out(acc_out[i][j])
                );
            end
        end
    endgenerate

    // Collect boundary wires
    generate
        for (i = 0; i < MATRIX_SIZE; i++) begin : assign_output
            assign out_right[i]  = right_wires[i][MATRIX_SIZE-1];
            assign out_bottom[i] = bottom_wires[MATRIX_SIZE-1][i];
        end
    endgenerate

    // Flatten acc_out
    generate
        for (i = 0; i < MATRIX_SIZE; i++) begin : row_out
            for (j = 0; j < MATRIX_SIZE; j++) begin : col_out
                assign acc_out_flat[(i*MATRIX_SIZE + j)*ACC_WIDTH +: ACC_WIDTH] = acc_out[i][j];
            end
        end
    endgenerate

endmodule

module systolic_module # (
    parameter DATA_WIDTH = 8,
    parameter MATRIX_SIZE = 8,
    parameter ADDR_WIDTH = $clog2(MATRIX_SIZE),
    parameter ACC_WIDTH  = 32,
    parameter ACC_ADDR_WIDTH = $clog2(MATRIX_SIZE*MATRIX_SIZE)
) (
    input  logic                                clk,
    input  logic                                rst,
    input  logic                                acc_rst,
    input  logic                                acc_en,
    input  logic                                shift_en_right,
    input  logic                                shift_en_down,

    // Accumulator Address
    input  logic [ACC_ADDR_WIDTH-1:0]           addr_acc,

    // Top Buffer Control
    input  logic                                buffer_rst_top,
    input  logic                                load_en_top,
    input  logic                                swap_buffers_top,
    input  logic [ADDR_WIDTH-1:0]               addr_top,
    input  logic [DATA_WIDTH-1:0]               data_in_top,

    // Left Buffer Control
    input  logic                                buffer_rst_left,
    input  logic                                load_en_left,
    input  logic                                swap_buffers_left,
    input  logic [ADDR_WIDTH-1:0]               addr_left,
    input  logic [DATA_WIDTH-1:0]               data_in_left,
    
    output logic [DATA_WIDTH*MATRIX_SIZE-1:0]   data_out_flat_bottom,
    output logic [DATA_WIDTH*MATRIX_SIZE-1:0]   data_out_flat_right,
    
    output logic [ACC_WIDTH-1:0]                acc_out
);

    logic   [DATA_WIDTH*MATRIX_SIZE-1:0]        reg_out_top;
    logic   [DATA_WIDTH*MATRIX_SIZE-1:0]        reg_out_left;

    // Accumulator Output Full
    logic [ACC_WIDTH*MATRIX_SIZE*MATRIX_SIZE-1:0] acc_out_flat;

    // Accumulator Output Logic
    always_comb begin
        acc_out = acc_out_flat[addr_acc * ACC_WIDTH +: ACC_WIDTH];
    end

    // Top Double Buffer (addressable)
    addressable_double_buffer #(
        .DATA_WIDTH(DATA_WIDTH),
        .MATRIX_SIZE(MATRIX_SIZE)
    ) top_buf (
        .clk(clk),
        .rst(rst),
        .buffer_rst(buffer_rst_top),
        .load_we(load_en_top),
        .swap_buffers(swap_buffers_top),
        .load_addr(addr_top),
        .load_data(data_in_top),
        .data_out_flat(reg_out_top)
    );

    // Left Double Buffer (addressable)
    addressable_double_buffer #(
        .DATA_WIDTH(DATA_WIDTH),
        .MATRIX_SIZE(MATRIX_SIZE)
    ) left_buf (
        .clk(clk),
        .rst(rst),
        .buffer_rst(buffer_rst_left),
        .load_we(load_en_left),
        .swap_buffers(swap_buffers_left),
        .load_addr(addr_left),
        .load_data(data_in_left),
        .data_out_flat(reg_out_left)
    );

    // Systolic Array
    systolic_array #(
        .MATRIX_SIZE(MATRIX_SIZE),
        .DATA_WIDTH(DATA_WIDTH),
        .ACC_WIDTH(ACC_WIDTH)
    ) sys_array (
        .clk(clk),
        .rst(rst),
        .acc_rst(acc_rst),
        .acc_en(acc_en),
        .shift_en_right(shift_en_right),
        .shift_en_down(shift_en_down),

        .in_left_flat(reg_out_left),
        .in_top_flat(reg_out_top),

        .out_right_flat(data_out_flat_right),
        .out_bottom_flat(data_out_flat_bottom),

        .acc_out_flat(acc_out_flat)
    );

endmodule

module uart_instr_mem_loader #(
    parameter INSTR_WIDTH = 32,
    parameter DEPTH       = 256,
    parameter CLK_PER_BIT = 50 // match your baud
) (
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
    // Loader FSM with start/stop mode
    // ==========================================
    logic [1:0] byte_count;
    logic [INSTR_WIDTH-1:0] shift_reg;
    logic                   write_mode;

    `ifdef UART_CMD_MODE
    // ====================================================
    // Command mode: enter/exit write mode with special bytes
    // ====================================================
    always_ff @(posedge clk) begin
        if (rst) begin
            byte_count <= 0;
            wr_addr    <= 0;
            wr_en      <= 0;
            write_mode <= 0;
        end else begin
            wr_en <= 0;

            if (rx_done) begin
                if (rx_byte == 8'hFF) begin
                    write_mode <= 1;
                    wr_addr    <= 0;
                    byte_count <= 0;
                end else if (rx_byte == 8'hFE) begin
                    write_mode <= 0;
                end else if (write_mode) begin
                    shift_reg  <= {shift_reg[INSTR_WIDTH-9:0], rx_byte};
                    byte_count <= byte_count + 1;

                    if (byte_count == (INSTR_WIDTH/8 - 1)) begin
                        wr_data    <= {shift_reg[INSTR_WIDTH-9:0], rx_byte};
                        wr_en      <= 1;
                        wr_addr    <= wr_addr + 1;
                        byte_count <= 0;
                    end
                end
            end
        end
    end

    `else
    // ====================================================
    // Simple continuous write mode
    // ====================================================
    always_ff @(posedge clk) begin
        if (rst) begin
            byte_count <= 0;
            wr_addr    <= 0;
            wr_en      <= 0;
        end else begin
            wr_en <= 0;

            // Increment address when write was enabled in previous cycle
            if (wr_en) begin
                wr_addr <= wr_addr + 1;
            end

            if (rx_done) begin
                shift_reg  <= {shift_reg[INSTR_WIDTH-9:0], rx_byte};
                byte_count <= byte_count + 1;

                if (byte_count == (INSTR_WIDTH/8 - 1)) begin
                    wr_data    <= {shift_reg[INSTR_WIDTH-9:0], rx_byte};
                    wr_en      <= 1;
                    byte_count <= 0;
                end
            end
        end
    end
    `endif
endmodule

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

