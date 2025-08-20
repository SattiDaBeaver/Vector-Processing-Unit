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

    // Internal Memory Write Wires
    input  logic                        mem_ext_en,
    input  logic                        mem_wr_en,
    input  logic [DP_ADDR_WIDTH-1:0]    mem_ext_addr,
    input  logic [DATA_WIDTH-1:0]       mem_ext_din,
    output logic [DATA_WIDTH-1:0]       mem_ext_dout,


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
    // Wires fed to the DPRAM Port A
    logic                               we_a_ram;
    logic [DP_ADDR_WIDTH-1:0]           addr_a_ram;
    logic [DATA_WIDTH-1:0]              din_a_ram;
    logic [DATA_WIDTH-1:0]              dout_a_ram;

    // Wires in control of FSM
    logic                               we_a;
    logic [DP_ADDR_WIDTH-1:0]           addr_a;
    logic [DATA_WIDTH-1:0]              din_a;
    logic [DATA_WIDTH-1:0]              dout_a;
    
    logic                               we_b;
    logic [DP_ADDR_WIDTH-1:0]           addr_b;
    logic [DATA_WIDTH-1:0]              din_b;
    logic [DATA_WIDTH-1:0]              dout_b;

    //======================================//
    //    External Memory Control Logic     //
    //======================================//

    always_comb begin
        if (mem_ext_en) begin // External source has control
            we_a_ram    = mem_wr_en;
            addr_a_ram  = mem_ext_addr;
            din_a_ram   = mem_ext_din;
        end
        else begin  // Internal FSM has control
            we_a_ram    = we_a;
            addr_a_ram  = addr_a;
            din_a_ram   = din_a;
        end
        mem_ext_dout = dout_a_ram;
        dout_a = dout_a_ram;
    end



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

        logic           CLR_ACC;        // bit 17
        logic           CLR_SYSTOLIC;   // bit 16
        logic           CLR_LEFT_BUF;   // bit 15
        logic           CLR_TOP_BUF;    // bit 14
        logic           IMM_FLAG;       // bit 13

        logic [12:0]    ADDR;           // bits 12 down to 0 (13 bits)
    } instruction_t;

    typedef enum logic [2:0] { 
        IDLE,
        PRE_FETCH,
        FETCH_EXECUTE,
        // Extra States for Special Instructions
        WRITE_ACC,
        LOAD_COMMIT,
        WAIT
    } fsm_state_t;

    logic [PC_WIDTH-1:0]        pc;
    logic [12:0]                wait_counter;
    logic                       load_left_delay, load_top_delay;
    logic [ACC_WIDTH-1:0]       acc_out_reg;
    // logic [ACC_ADDR_WIDTH-1:0]  acc_base_addr;
    logic [DP_ADDR_WIDTH-1:0]   acc_mem_addr;
    logic [2:0]                 acc_byte_index;


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

        // Accumulator Address
        addr_acc = curr_instr[17:12];
    end

    // FSM Control Logic
    always_ff @(posedge clk) begin
        if (fsm_rst) begin
            state               <= IDLE;  // Reset State
            systolic_master_rst <= 1;
            pc                  <= 0;     // Reset Program Counter
            curr_instr          <= 0;
            next_instr          <= 0;
        end
        else begin
            // Default Wire Values
            // Multi-Cycle Load
            load_left_delay     <= 0;
            load_top_delay      <= 0;

            // Systolic Array
            // Reset
            systolic_master_rst <= 0;
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
            // Reset Signals
            acc_rst             <= 0;
            buffer_rst_left     <= 0;
            buffer_rst_top      <= 0;
            systolic_master_rst <= 0;
            // Accumulator Signals

            // Dual Port RAM
            addr_a              <= 0;
            addr_b              <= 0;
            we_b                <= 0;
            din_b               <= 0;

            

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

                        // Write Accumulator Out Instructions
                        if (curr_instr.WRITE_ACC_OUT) begin
                            acc_out_reg     <= acc_out;
                            acc_mem_addr    <= curr_instr[9:0];
                            acc_byte_index  <= 0;
                            state           <= WRITE_ACC;
                        end

                        else begin
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
                                    if (curr_instr.IMM_FLAG) begin // Immediate Flag
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
                                    if (curr_instr.IMM_FLAG) begin // Immediate Flag
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
                                acc_en      <= 1;
                            end

                            // Clear Flags
                            if (curr_instr.CLR_ACC) begin
                                acc_rst             <= 1;
                            end
                            if (curr_instr.CLR_SYSTOLIC) begin
                                systolic_master_rst <= 1;
                            end
                            if (curr_instr.CLR_LEFT_BUF) begin
                                buffer_rst_left     <= 1;
                            end
                            if (curr_instr.CLR_TOP_BUF) begin
                                buffer_rst_top      <= 1;
                            end
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

                WRITE_ACC: begin
                    addr_b      <= acc_mem_addr + acc_byte_index;
                    din_b       <= acc_out_reg[8*acc_byte_index +: 8];
                    we_b        <= 1;
                    if (acc_byte_index == 3) begin
                        state           <= FETCH_EXECUTE;
                    end
                    else begin
                        acc_byte_index  <= acc_byte_index + 1;
                        state           <= WRITE_ACC;
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
        .we_a(we_a_ram),
        .addr_a(addr_a_ram),
        .din_a(din_a_ram),
        .dout_a(dout_a_ram),

        // Port B
        .we_b(we_b),
        .addr_b(addr_b),
        .din_b(din_b),
        .dout_b(dout_b)
    );

endmodule : tiny_fsm_control