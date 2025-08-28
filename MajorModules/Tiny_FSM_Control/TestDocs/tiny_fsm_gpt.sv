module tiny_fsm_control #(
    //======================================//
    //              Parameters              //
    //======================================//
    // Systolic Module Parameters
    parameter DATA_WIDTH     = 8,
    parameter MATRIX_SIZE    = 8,
    parameter ACC_WIDTH      = 32,
    parameter ADDR_WIDTH     = $clog2(MATRIX_SIZE),
    parameter ACC_ADDR_WIDTH = $clog2(MATRIX_SIZE*MATRIX_SIZE),

    // DPRAM Parameters
    parameter DP_ADDR_WIDTH  = 10,

    // UART Instruction Memory Loader Parameters
    parameter F_CLK          = 50_000_000,
    parameter BAUD           = 921_600,
    parameter CLK_PER_BIT    = F_CLK / BAUD,
    parameter INSTR_WIDTH    = 32,
    parameter INSTR_DEPTH    = 256
) (
    // Clocks / resets
    input  logic                               clk,
    input  logic                               rst,

    // UART pins out to the board
    input  logic                               uart_rx,
    output logic                               uart_tx,

    // Optional debug outs
    output logic [$clog2(INSTR_DEPTH)-1:0]     pc_out,
    output logic [1:0]                         fsm_state_out
);
    //======================================//
    //             Module Wires             //
    //======================================//

    // ========== Systolic Module Wires ==========
    logic                               systolic_master_rst;
    logic                               acc_rst;
    logic                               acc_en;
    logic                               shift_en_right;
    logic                               shift_en_down;

    logic [ACC_ADDR_WIDTH-1:0]          addr_acc;

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
    logic [ACC_WIDTH-1:0]               acc_out;

    // ========== Dual Port RAM Wires ==========
    logic                               we_a;
    logic [DP_ADDR_WIDTH-1:0]           addr_a;
    logic [DATA_WIDTH-1:0]              din_a;
    logic [DATA_WIDTH-1:0]              dout_a;

    logic                               we_b;
    logic [DP_ADDR_WIDTH-1:0]           addr_b;
    logic [DATA_WIDTH-1:0]              din_b;
    logic [DATA_WIDTH-1:0]              dout_b;

    // ========== UART Instruction Mem Loader Wires ==========
    logic                               uart_master_rst;
    logic [$clog2(INSTR_DEPTH)-1:0]     rd_addr;  // PC
    logic [INSTR_WIDTH-1:0]             rd_data;  // fetched 32-bit instr

    //======================================//
    //          Instruction format          //
    //======================================//
    typedef struct packed {
        logic           LOAD_LEFT;      // bit 31
        logic           LOAD_TOP;       // bit 30
        logic           SWAP_LEFT;      // bit 29
        logic           SWAP_TOP;       // bit 28
        logic           SHIFT_RIGHT;    // bit 27
        logic           SHIFT_DOWN;     // bit 26
        logic           LOAD_ACC;       // bit 25
        logic           WRITE_ACC_OUT;  // bit 24  (TODO: serialize to DP RAM)
        logic           WAIT_CYCLES;    // bit 23   (use ADDR as count)
        logic           JUMP;           // bit 22   (use ADDR as target)
        logic           CLR;            // bit 21
        logic           NOP;            // bit 20

        logic [1:0]     RESERVED;       // bits 19..18

        // FLAGS mapping (example):
        // [17] CLR_ACC, [16] CLR_SYST, [15] CLR_A_BUF, [14] CLR_B_BUF, [13] reserved
        logic [4:0]     FLAGS;          // bits 17..13
        logic [12:0]    ADDR;           // bits 12..0 (address / immediate)
    } instruction_t;

    //======================================//
    //              Prefetch FSM            //
    //======================================//
    typedef enum logic [1:0] {
        IDLE           = 2'd0,
        FETCH_EXECUTE  = 2'd1,
        LOAD_ISSUE     = 2'd2,  // phase-1: place DP RAM addr (read)
        LOAD_COMMIT    = 2'd3   // phase-2: RAM data available -> write to buffer
    } fsm_state_t;

    instruction_t                     curr_instr, next_instr;
    fsm_state_t                       state;

    logic [$clog2(INSTR_DEPTH)-1:0]   pc;
    logic [12:0]                      wait_counter;

    // Simple 1-byte latch for LOAD_* (from DP RAM)
    logic [DATA_WIDTH-1:0]            load_byte_latch;
    logic                             pending_load_left, pending_load_top;

    // Debug outs
    assign pc_out        = pc;
    assign fsm_state_out = state;

    //======================================//
    //        Default datapath controls     //
    //======================================//
    always_comb begin
        // Defaults every cycle
        systolic_master_rst = rst;

        // Acc / shift defaults
        acc_rst        = 1'b0;
        acc_en         = 1'b0;
        shift_en_right = 1'b0;
        shift_en_down  = 1'b0;

        // Buffer controls default
        buffer_rst_top   = 1'b0;
        buffer_rst_left  = 1'b0;
        load_en_top      = 1'b0;
        load_en_left     = 1'b0;
        swap_buffers_top = 1'b0;
        swap_buffers_left= 1'b0;
        addr_top         = '0;
        addr_left        = '0;
        data_in_top      = '0;
        data_in_left     = '0;

        // Acc address default (can be driven by microcode later)
        addr_acc = '0;

        // DP RAM defaults
        we_a  = 1'b0; din_a = '0; addr_a = '0; // Port A read by default
        we_b  = 1'b0; din_b = '0; addr_b = '0; // (free for future)

        // ========== Map clears from FLAGS when CLR is set ==========
        if (curr_instr.CLR) begin
            if (curr_instr.FLAGS[17-13]) begin end // keep synthesis calm
            // FLAGS[17] = CLR_ACC
            if (curr_instr.FLAGS[4]) acc_rst = 1'b1;
            // FLAGS[16] = CLR_SYSTOLIC
            if (curr_instr.FLAGS[3]) begin
                // You may map this to a dedicated systolic reset if you have one.
                // Here we can pulse acc_rst plus you might add internal systolic clear.
            end
            // FLAGS[15] = CLR_A_BUF (current inactive buffer clear handled inside buffer)
            if (curr_instr.FLAGS[2]) buffer_rst_top  = 1'b1;
            // FLAGS[14] = CLR_B_BUF
            if (curr_instr.FLAGS[1]) buffer_rst_left = 1'b1;
        end

        // ========== Map simple single-cycle controls ==========
        if (curr_instr.SWAP_TOP)  swap_buffers_top  = 1'b1;
        if (curr_instr.SWAP_LEFT) swap_buffers_left = 1'b1;

        if (curr_instr.SHIFT_RIGHT) shift_en_right = 1'b1;
        if (curr_instr.SHIFT_DOWN)  shift_en_down  = 1'b1;

        if (curr_instr.LOAD_ACC) acc_en = 1'b1;
 
        // Note: WRITE_ACC_OUT is TODO (needs serialization to DP RAM)
    end

    //======================================//
    //         Prefetch + Execute loop      //
    //======================================//
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            state            <= IDLE;
            pc               <= '0;
            wait_counter     <= '0;
            curr_instr       <= '0;
            next_instr       <= '0;
            rd_addr          <= '0;

            load_byte_latch  <= '0;
            pending_load_left<= 1'b0;
            pending_load_top <= 1'b0;
        end else begin
            unique case (state)
                IDLE: begin
                    // Kick the very first fetch
                    rd_addr    <= '0;
                    // next_instr will be available combinationally from rd_data
                    next_instr <= instruction_t'(rd_data);
                    // Advance PC to point at the next
                    pc         <= 'd1;
                    state      <= FETCH_EXECUTE;
                end

                FETCH_EXECUTE: begin
                    // Move prefetched into current
                    curr_instr <= next_instr;

                    // Pre-fetch next (pc)
                    rd_addr    <= pc;
                    next_instr <= instruction_t'(rd_data);
                    pc         <= pc + 1'b1;

                    // Timing/flow controls
                    if (curr_instr.NOP) begin
                        // do nothing this cycle
                    end

                    // Wait stall: load counter from ADDR and park
                    if (curr_instr.WAIT_CYCLES) begin
                        wait_counter <= curr_instr.ADDR[12:0];
                        state        <= WAIT;
                    end
                    // Jump
                    else if (curr_instr.JUMP) begin
                        pc      <= curr_instr.ADDR[$bits(pc)-1:0];
                        rd_addr <= curr_instr.ADDR[$bits(pc)-1:0];
                        // next_instr refreshed next cycle at new PC
                    end
                    // LOAD_* issue phase (synchronous DP RAM read → data next cycle)
                    else if (curr_instr.LOAD_LEFT || curr_instr.LOAD_TOP) begin
                        // Use ADDR as DP RAM address source
                        addr_a <= curr_instr.ADDR[DP_ADDR_WIDTH-1:0];
                        // Flag which path will commit next cycle
                        pending_load_left <= curr_instr.LOAD_LEFT;
                        pending_load_top  <= curr_instr.LOAD_TOP;
                        state             <= LOAD_ISSUE;
                    end
                end

                // Phase-1: we already drove addr_a; give RAM one cycle
                LOAD_ISSUE: begin
                    // Capture RAM data into a latch this cycle
                    load_byte_latch <= dout_a;
                    state           <= LOAD_COMMIT;
                end

                // Phase-2: write into the inactive buffer selected by LOAD_*
                LOAD_COMMIT: begin
                    if (pending_load_left) begin
                        load_en_left <= 1'b1;
                        // Using lower bits of ADDR as the buffer index (0..MATRIX_SIZE-1)
                        addr_left    <= curr_instr.ADDR[ADDR_WIDTH-1:0];
                        data_in_left <= load_byte_latch;
                    end
                    if (pending_load_top) begin
                        load_en_top  <= 1'b1;
                        addr_top     <= curr_instr.ADDR[ADDR_WIDTH-1:0];
                        data_in_top  <= load_byte_latch;
                    end

                    // Clear pending flags
                    pending_load_left <= 1'b0;
                    pending_load_top  <= 1'b0;

                    // Return to main loop
                    state <= FETCH_EXECUTE;
                end

                WAIT: begin
                    if (wait_counter != 0) begin
                        wait_counter <= wait_counter - 1'b1;
                    end else begin
                        state <= FETCH_EXECUTE;
                    end
                end

                default: state <= IDLE;
            endcase
        end
    end

    //======================================//
    //        Module Instantiation          //
    //======================================//

    // ---------- Systolic Module ----------
    systolic_module #(
        .DATA_WIDTH (DATA_WIDTH),
        .MATRIX_SIZE(MATRIX_SIZE),
        .ACC_WIDTH  (ACC_WIDTH)
    ) SYS_ARRAY (
        .clk               (clk),
        .rst               (systolic_master_rst),
        .acc_rst           (acc_rst),
        .acc_en            (acc_en),
        .shift_en_right    (shift_en_right),
        .shift_en_down     (shift_en_down),
        .addr_acc          (addr_acc),

        .buffer_rst_top    (buffer_rst_top),
        .load_en_top       (load_en_top),
        .swap_buffers_top  (swap_buffers_top),
        .addr_top          (addr_top),
        .data_in_top       (data_in_top),

        .buffer_rst_left   (buffer_rst_left),
        .load_en_left      (load_en_left),
        .swap_buffers_left (swap_buffers_left),
        .addr_left         (addr_left),
        .data_in_left      (data_in_left),

        .data_out_flat_bottom (data_out_flat_bottom),
        .data_out_flat_right  (data_out_flat_right),
        .acc_out              (acc_out)
    );

    // ---------- Dual Port RAM ----------
    // Port A used by micro-FSM for reads (and future writes)
    dp_ram #(
        .DATA_WIDTH(DATA_WIDTH),
        .ADDR_WIDTH(DP_ADDR_WIDTH)
    ) DPRAM (
        .clk   (clk),

        .we_a  (we_a),
        .addr_a(addr_a),
        .din_a (din_a),
        .dout_a(dout_a),

        .we_b  (we_b),
        .addr_b(addr_b),
        .din_b (din_b),
        .dout_b(dout_b)
    );

    // ---------- UART Instruction Memory Loader ----------
    // Provides instruction stream at rd_addr -> rd_data
    uart_instr_mem_loader #(
        .INSTR_WIDTH(INSTR_WIDTH),
        .DEPTH      (INSTR_DEPTH),
        .CLK_PER_BIT(CLK_PER_BIT)
    ) UART_MEM (
        .clk     (clk),
        .rst     (uart_master_rst),

        .uart_rx (uart_rx),
        .uart_tx (uart_tx),

        .rd_addr (rd_addr),
        .rd_data (rd_data)
    );

    // Tie loader reset to system reset (you can decouple later)
    always_comb begin
        uart_master_rst = rst;
    end

endmodule
