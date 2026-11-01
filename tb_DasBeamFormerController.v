// =============================================================================
// Testbench : tb_DasBeamFormerController.v
// DUT       : DasBeamFormerController (beamformer.v)
// Simulator : Icarus Verilog
//   iverilog -g2005-sv -o tb_ctrl.vvp tb_DasBeamFormerController.v beamformer.v
//   vvp tb_ctrl.vvp
//
// Design-under-test: 3-state FSM (IDLE→REQUEST→WAIT) that iterates over 25
// transducer lines (0..24), reads one sample per line from an external memory,
// sign-extends each 16-bit sample to 21 bits, accumulates them, and outputs
// the pixel_value together with a one-cycle pixel_value_valid pulse.
//
// Test-case inventory
// -------------------
//  TC01  Reset: all outputs deasserted / zeroed
//  TC02  start ignored while rst_n held low
//  TC03  25 x (+1)  → pixel_value = +25
//  TC04  25 x (-1)  → pixel_value = -25
//  TC05  Mixed ramp i-12 (i=0..24) → sum = 0
//  TC06  All +32767 (max positive) → 25*32767 = 819175
//  TC07  All -32768 (max negative) → 25*(-32768) = -819200
//  TC08  Alternating +32767/-32768 → ref-model check
//  TC09  busy asserted same cycle start is sampled; clears with pixel_value_valid
//  TC10  pixel_value_valid is exactly a one-cycle pulse
//  TC11  read_enable asserted exactly 25 times per computation
//  TC12  read_line increments 0..24; read_address mirrors delay_address
//  TC13  start while busy ignored (computation not restarted)
//  TC14  Back-to-back computations with different delay_address values
//  TC15  WAIT-state stall: 5-cycle delay per sample
//  TC16  WAIT-state stall: 10-cycle delay per sample
//  TC17  Spurious read_data_valid in IDLE ignored
//  TC18  pixel_value holds last value during idle
//  TC19  Illegal state 2'b11 forced: DUT recovers to IDLE
//  TC20  Re-start immediately the cycle busy falls
//  TC21  rst_n asserted mid-computation: clean abort
//  TC22  Hold-high start: FSM auto re-triggers (documents Bug-3)
//  TC23  All-zero samples → pixel_value = 0
//  TC24  Single-cycle read_data_valid pulse
// =============================================================================

`timescale 1ns/1ps

module tb_DasBeamFormerController;

// ── Parameters ────────────────────────────────────────────────────────────────
localparam AW      = 5;
localparam N_LINES = 25;

// ── DUT port regs/wires ───────────────────────────────────────────────────────
reg                clk             = 1'b0;
reg                rst_n           = 1'b1;
reg                start           = 1'b0;
reg  [AW-1:0]      delay_address   = {AW{1'b0}};
reg  signed [15:0] read_data       = 16'h0000;
reg                read_data_valid = 1'b0;

wire [AW-1:0]      read_address;
wire               read_enable;
wire [4:0]         read_line;
wire signed [20:0] pixel_value;
wire               pixel_value_valid;
wire               busy;

// ── DUT ───────────────────────────────────────────────────────────────────────
DasBeamFormerController #(.ADDRESS_WIDTH(AW)) dut (
    .clk             (clk),
    .rst_n           (rst_n),
    .start           (start),
    .delay_address   (delay_address),
    .read_address    (read_address),
    .read_enable     (read_enable),
    .read_line       (read_line),
    .read_data       (read_data),
    .read_data_valid (read_data_valid),
    .pixel_value     (pixel_value),
    .pixel_value_valid(pixel_value_valid),
    .busy            (busy)
);

// ── Clock ─────────────────────────────────────────────────────────────────────
always #5 clk = ~clk;   // 100 MHz

// ── Scoreboard ────────────────────────────────────────────────────────────────
integer pass_cnt = 0;
integer fail_cnt = 0;

// ── Shared sample buffer ─────────────────────────────────────────────────────
reg signed [15:0] smp [0:N_LINES-1];

// ── check_cond ───────────────────────────────────────────────────────────────
task check_cond;
    input [255:0] label;
    input         cond;
    begin
        if (cond) begin
            $display("  PASS  %0s", label);
            pass_cnt = pass_cnt + 1;
        end else begin
            $display("  FAIL  %0s", label);
            fail_cnt = fail_cnt + 1;
        end
    end
endtask

// ── do_reset ─────────────────────────────────────────────────────────────────
// Asserts reset for 4 negedges then releases.  Caller is left at a negedge.
task do_reset;
    begin
        @(negedge clk);
        rst_n           = 1'b0;
        start           = 1'b0;
        read_data       = 16'h0;
        read_data_valid = 1'b0;
        delay_address   = {AW{1'b0}};
        repeat(4) @(negedge clk);
        rst_n = 1'b1;
        @(negedge clk);
    end
endtask

// ── run_comp ─────────────────────────────────────────────────────────────────
// Drives a complete 25-line beamforming computation from smp[0..24].
//   extra_wait: number of additional idle negedges to insert before asserting
//               read_data_valid for each sample (simulates memory latency).
//   start_input: if 1 the task asserts start; if 0 assumes start was already
//               sent by the caller (used for TC22).
// Returns captured pixel_value in 'result' and 1 in 'ok' when pvv went high.
// Exits at the posedge where pixel_value_valid was observed.
task run_comp;
    input  integer        extra_wait;
    input                 assert_start;
    output signed [20:0]  result;
    output reg            ok;

    integer line_idx, w, timeout;
    begin
        result  = 21'hx;
        ok      = 1'b0;

        // ── Optionally assert start ──────────────────────────────────────────
        if (assert_start) begin
            @(negedge clk); start = 1'b1;
            @(negedge clk); start = 1'b0;
        end

        // ── Service 25 REQUEST → WAIT cycles ─────────────────────────────────
        line_idx = 0;
        timeout  = 0;
        while (line_idx < N_LINES) begin
            @(posedge clk);
            timeout = timeout + 1;
            if (timeout > 5000) begin
                $display("  ERROR run_comp: timeout waiting for read_enable (line %0d)", line_idx);
                fail_cnt = fail_cnt + 1;
                disable run_comp;
            end

            if (read_enable) begin
                // Move to negedge so we drive cleanly between clock edges
                @(negedge clk);

                // Optional extra stall
                for (w = 0; w < extra_wait; w = w + 1) begin
                    read_data_valid = 1'b0;
                    @(negedge clk);
                end

                // Present sample for one clock cycle
                read_data       = smp[line_idx];
                read_data_valid = 1'b1;
                @(negedge clk);
                read_data_valid = 1'b0;
                read_data       = 16'h0;

                line_idx = line_idx + 1;
                // We are now at a negedge; the while loop will do @posedge next.
            end
        end

        // ── Wait for pixel_value_valid ────────────────────────────────────────
        timeout = 0;
        @(posedge clk);
        while (!pixel_value_valid) begin
            @(posedge clk);
            timeout = timeout + 1;
            if (timeout > 30) begin
                $display("  ERROR run_comp: pixel_value_valid never asserted");
                fail_cnt = fail_cnt + 1;
                disable run_comp;
            end
        end
        result = pixel_value;
        ok     = 1'b1;
        // Caller sits at the posedge where pvv is high
    end
endtask

// =============================================================================
//  Main test sequence
// =============================================================================
integer        i;
reg signed [20:0] result, expected, ref_acc;
reg               ok;

initial begin
    $dumpfile("tb_ctrl.vcd");
    $dumpvars(0, tb_DasBeamFormerController);

    // =========================================================================
    // TC01 – Reset: all outputs deasserted / zeroed
    // =========================================================================
    $display("\n--- TC01: Reset outputs ---");
    rst_n = 1'b0; start = 1'b0;
    read_data = 16'h0; read_data_valid = 1'b0; delay_address = 0;
    repeat(3) @(negedge clk);

    check_cond("TC01 pixel_value_valid==0", pixel_value_valid === 1'b0);
    check_cond("TC01 busy==0",              busy              === 1'b0);
    check_cond("TC01 read_enable==0",       read_enable       === 1'b0);
    check_cond("TC01 pixel_value==0",       pixel_value       === 21'sh00000);
    check_cond("TC01 read_line==0",         read_line         === 5'h0);

    // =========================================================================
    // TC02 – start ignored while rst_n held low
    // =========================================================================
    $display("\n--- TC02: start ignored during reset ---");
    // rst_n is still 0
    @(negedge clk); start = 1'b1;
    repeat(2) @(negedge clk); start = 1'b0;
    check_cond("TC02 busy==0",        busy        === 1'b0);
    check_cond("TC02 read_enable==0", read_enable === 1'b0);

    rst_n = 1'b1; @(negedge clk);

    // =========================================================================
    // TC03 – 25 x (+1) → +25
    // =========================================================================
    $display("\n--- TC03: 25 x (+1) => +25 ---");
    do_reset;
    for (i = 0; i < N_LINES; i = i + 1) smp[i] = 16'sh0001;
    run_comp(0, 1, result, ok);
    expected = $signed(21'sd25);
    check_cond("TC03 valid",       ok      === 1'b1);
    if (result !== expected) $display("  FAIL TC03: got %0d expected 25", $signed(result));
    check_cond("TC03 pixel_value", result  === expected);

    // =========================================================================
    // TC04 – 25 x (-1) → -25
    // =========================================================================
    $display("\n--- TC04: 25 x (-1) => -25 ---");
    do_reset;
    for (i = 0; i < N_LINES; i = i + 1) smp[i] = 16'shFFFF;
    run_comp(0, 1, result, ok);
    expected = $signed(-21'sd25);
    check_cond("TC04 valid",       ok      === 1'b1);
    if (result !== expected) $display("  FAIL TC04: got %0d expected -25", $signed(result));
    check_cond("TC04 pixel_value", result  === expected);

    // =========================================================================
    // TC05 – Ramp (i-12): sum = 0
    // =========================================================================
    $display("\n--- TC05: Ramp (i-12), i=0..24 => 0 ---");
    do_reset;
    ref_acc = 21'sh00000;
    for (i = 0; i < N_LINES; i = i + 1) begin
        smp[i] = $signed(16'(i)) - 16'sh000C;
        ref_acc = ref_acc + {{5{smp[i][15]}}, smp[i]};
    end
    run_comp(0, 1, result, ok);
    expected = ref_acc;
    check_cond("TC05 valid",       ok      === 1'b1);
    if (result !== expected)
        $display("  FAIL TC05: got %0d expected %0d", $signed(result), $signed(expected));
    check_cond("TC05 pixel_value", result  === expected);

    // =========================================================================
    // TC06 – All +32767 → 819175
    // =========================================================================
    $display("\n--- TC06: 25 x +32767 => 819175 ---");
    do_reset;
    for (i = 0; i < N_LINES; i = i + 1) smp[i] = 16'sh7FFF;
    run_comp(0, 1, result, ok);
    // 25 * 32767 = 819175
    expected = $signed(21'sd819175);
    check_cond("TC06 valid",       ok      === 1'b1);
    if (result !== expected) $display("  FAIL TC06: got %0d expected 819175", $signed(result));
    check_cond("TC06 pixel_value", result  === expected);

    // =========================================================================
    // TC07 – All -32768 → -819200
    // =========================================================================
    $display("\n--- TC07: 25 x -32768 => -819200 ---");
    do_reset;
    for (i = 0; i < N_LINES; i = i + 1) smp[i] = 16'sh8000;
    run_comp(0, 1, result, ok);
    // 25 * -32768 = -819200
    expected = $signed(-21'sd819200);
    check_cond("TC07 valid",       ok      === 1'b1);
    if (result !== expected) $display("  FAIL TC07: got %0d expected -819200", $signed(result));
    check_cond("TC07 pixel_value", result  === expected);

    // =========================================================================
    // TC08 – Alternating +32767 / -32768
    // =========================================================================
    $display("\n--- TC08: Alternating max pos/neg ---");
    do_reset;
    ref_acc = 21'sh00000;
    for (i = 0; i < N_LINES; i = i + 1) begin
        smp[i]  = i[0] ? 16'sh8000 : 16'sh7FFF;
        ref_acc = ref_acc + {{5{smp[i][15]}}, smp[i]};
    end
    run_comp(0, 1, result, ok);
    expected = ref_acc;
    check_cond("TC08 valid",       ok      === 1'b1);
    if (result !== expected)
        $display("  FAIL TC08: got %0d expected %0d", $signed(result), $signed(expected));
    check_cond("TC08 pixel_value", result  === expected);

    // =========================================================================
    // TC09 – busy timing
    // =========================================================================
    $display("\n--- TC09: busy timing ---");
    do_reset;
    for (i = 0; i < N_LINES; i = i + 1) smp[i] = 16'sh0001;

    @(negedge clk); start = 1'b1;
    @(posedge clk); #1;
    check_cond("TC09 busy after start", busy === 1'b1);
    @(negedge clk); start = 1'b0;

    // run_comp with assert_start=0 (start already asserted above)
    run_comp(0, 0, result, ok);
    #1;
    check_cond("TC09 busy cleared with pvv", busy === 1'b0);

    // =========================================================================
    // TC10 – pixel_value_valid single-cycle pulse
    // =========================================================================
    $display("\n--- TC10: pixel_value_valid is single-cycle ---");
    @(posedge clk); #1;
    check_cond("TC10 valid cleared next cycle", pixel_value_valid === 1'b0);

    // =========================================================================
    // TC11 – read_enable asserted exactly 25 times
    // =========================================================================
    $display("\n--- TC11: read_enable count == 25 ---");
    do_reset;
    for (i = 0; i < N_LINES; i = i + 1) smp[i] = 16'sh0064;

    begin : tc11_blk
        integer re_cnt, to11;
        re_cnt = 0; to11 = 0;

        @(negedge clk); start = 1'b1;
        @(negedge clk); start = 1'b0;

        while (!pixel_value_valid && to11 < 3000) begin
            @(posedge clk);
            to11 = to11 + 1;
            if (read_enable) begin
                re_cnt = re_cnt + 1;
                @(negedge clk);
                read_data       = smp[re_cnt-1];
                read_data_valid = 1'b1;
                @(negedge clk);
                read_data_valid = 1'b0;
                read_data       = 16'h0;
            end
        end

        if (re_cnt !== N_LINES)
            $display("  FAIL TC11: read_enable count=%0d expected 25", re_cnt);
        check_cond("TC11 re_cnt==25", re_cnt === N_LINES);
    end

    // =========================================================================
    // TC12 – read_line = 0..24 and read_address == delay_address
    // =========================================================================
    $display("\n--- TC12: read_line sequence and read_address ---");
    do_reset;
    delay_address = 5'd7;

    begin : tc12_blk
        integer ok_line, ok_addr, idx12, to12;
        ok_line = 1; ok_addr = 1; idx12 = 0; to12 = 0;

        @(negedge clk); start = 1'b1;
        @(negedge clk); start = 1'b0;

        while (!pixel_value_valid && to12 < 3000) begin
            @(posedge clk);
            to12 = to12 + 1;
            if (read_enable) begin
                if (read_line    !== idx12[4:0])    ok_line = 0;
                if (read_address !== delay_address) ok_addr = 0;
                @(negedge clk);
                read_data       = 16'sh0001;
                read_data_valid = 1'b1;
                @(negedge clk);
                read_data_valid = 1'b0;
                read_data       = 16'h0;
                idx12 = idx12 + 1;
            end
        end

        check_cond("TC12 read_line seq", ok_line === 1);
        check_cond("TC12 read_address",  ok_addr === 1);
        if (idx12 !== N_LINES)
            $display("  FAIL TC12: only %0d reads observed", idx12);
        check_cond("TC12 idx==25",       idx12 === N_LINES);
    end

    // =========================================================================
    // TC13 – start while busy is ignored
    // =========================================================================
    $display("\n--- TC13: start ignored while busy ---");
    do_reset;
    for (i = 0; i < N_LINES; i = i + 1) smp[i] = 16'sh0001;

    begin : tc13_blk
        integer to13;
        to13 = 0;

        @(negedge clk); start = 1'b1;
        @(negedge clk); start = 1'b0;

        while (!pixel_value_valid && to13 < 3000) begin
            @(posedge clk);
            to13 = to13 + 1;
            if (read_enable) begin
                @(negedge clk);
                start           = 1'b1;   // spurious start while busy
                read_data       = 16'sh0001;
                read_data_valid = 1'b1;
                @(negedge clk);
                start           = 1'b0;
                read_data_valid = 1'b0;
                read_data       = 16'h0;
            end
        end
        #1;
        if (pixel_value !== $signed(21'sd25))
            $display("  FAIL TC13: expected 25, got %0d", $signed(pixel_value));
        check_cond("TC13 pixel_value==25", pixel_value === $signed(21'sd25));
    end

    // =========================================================================
    // TC14 – Back-to-back computations with different delay_address
    // =========================================================================
    $display("\n--- TC14: Back-to-back computations ---");
    do_reset;

    begin : tc14_blk
        reg signed [20:0] ra, rb;
        reg oka, okb;

        // Comp A: all +10, addr=3
        for (i = 0; i < N_LINES; i = i + 1) smp[i] = 16'sh000A;
        delay_address = 5'd3;
        run_comp(0, 1, ra, oka);
        check_cond("TC14 A valid", oka === 1'b1);
        if (ra !== $signed(21'sd250))
            $display("  FAIL TC14 A: got %0d expected 250", $signed(ra));
        check_cond("TC14 A=250", ra === $signed(21'sd250));

        // Comp B: all -10, addr=15
        for (i = 0; i < N_LINES; i = i + 1) smp[i] = 16'shFFF6; // -10
        delay_address = 5'd15;
        run_comp(0, 1, rb, okb);
        check_cond("TC14 B valid", okb === 1'b1);
        if (rb !== $signed(-21'sd250))
            $display("  FAIL TC14 B: got %0d expected -250", $signed(rb));
        check_cond("TC14 B=-250", rb === $signed(-21'sd250));
    end

    // =========================================================================
    // TC15 – WAIT stall: 5 cycles per sample
    // =========================================================================
    $display("\n--- TC15: WAIT stall 5 cycles ---");
    do_reset;
    for (i = 0; i < N_LINES; i = i + 1) smp[i] = 16'sh0002; // +2
    run_comp(5, 1, result, ok);
    expected = $signed(21'sd50);
    check_cond("TC15 valid",       ok      === 1'b1);
    if (result !== expected) $display("  FAIL TC15: got %0d expected 50", $signed(result));
    check_cond("TC15 pixel_value", result  === expected);

    // =========================================================================
    // TC16 – WAIT stall: 10 cycles per sample
    // =========================================================================
    $display("\n--- TC16: WAIT stall 10 cycles ---");
    do_reset;
    for (i = 0; i < N_LINES; i = i + 1) smp[i] = 16'sh0003; // +3
    run_comp(10, 1, result, ok);
    expected = $signed(21'sd75);
    check_cond("TC16 valid",       ok      === 1'b1);
    if (result !== expected) $display("  FAIL TC16: got %0d expected 75", $signed(result));
    check_cond("TC16 pixel_value", result  === expected);

    // =========================================================================
    // TC17 – Spurious read_data_valid in IDLE ignored
    // =========================================================================
    $display("\n--- TC17: Spurious valid in IDLE ---");
    do_reset;

    @(negedge clk);
    read_data       = 16'sh7FFF;
    read_data_valid = 1'b1;
    @(negedge clk);
    read_data_valid = 1'b0;
    @(posedge clk); #1;

    check_cond("TC17 not busy", busy              === 1'b0);
    check_cond("TC17 no pvv",   pixel_value_valid === 1'b0);

    // =========================================================================
    // TC18 – pixel_value holds last value during idle
    // =========================================================================
    $display("\n--- TC18: pixel_value retention ---");
    do_reset;
    for (i = 0; i < N_LINES; i = i + 1) smp[i] = 16'sh000F; // +15
    run_comp(0, 1, result, ok);
    expected = $signed(21'sd375); // 25*15
    check_cond("TC18 first result 375", result === expected);
    if (result !== expected) $display("  FAIL TC18: got %0d expected 375", $signed(result));

    repeat(10) @(posedge clk); #1;
    if (pixel_value !== expected)
        $display("  FAIL TC18 hold: pixel_value changed to %0d", $signed(pixel_value));
    check_cond("TC18 held value",   pixel_value       === expected);
    check_cond("TC18 pvv==0 idle",  pixel_value_valid === 1'b0);

    // =========================================================================
    // TC19 – Illegal state 2'b11 forced → DUT recovers to IDLE
    // =========================================================================
    $display("\n--- TC19: Illegal state recovery ---");
    do_reset;

    force dut.state = 2'b11;
    @(posedge clk); #1;
    release dut.state;

    @(posedge clk); #1;
    check_cond("TC19 state==IDLE", dut.state === 2'b00);
    check_cond("TC19 busy==0",     busy      === 1'b0);
    // Known Bug-5: accumulator and current_line NOT reset in default case
    $display("  NOTE TC19(Bug-5): acc=%0d line=%0d (not zeroed by default case)",
             $signed(dut.pixel_accumulator), dut.current_line);

    // =========================================================================
    // TC20 – Re-start immediately the cycle busy falls
    // =========================================================================
    $display("\n--- TC20: Immediate re-start after busy falls ---");
    do_reset;

    begin : tc20_blk
        reg signed [20:0] r1, r2;
        reg ok1;
        integer idx20, to20;

        // First: all +4 → 100
        for (i = 0; i < N_LINES; i = i + 1) smp[i] = 16'sh0004;
        run_comp(0, 1, r1, ok1);
        // We are now sitting at the posedge where pvv was high (busy just fell).

        // Assert start on the very next negedge
        @(negedge clk); start = 1'b1;
        @(negedge clk); start = 1'b0;

        // Load second sample set
        for (i = 0; i < N_LINES; i = i + 1) smp[i] = 16'sh0008; // +8 → 200

        // Service the second computation manually
        idx20 = 0; to20 = 0;
        while (!pixel_value_valid && to20 < 3000) begin
            @(posedge clk);
            to20 = to20 + 1;
            if (read_enable && idx20 < N_LINES) begin
                @(negedge clk);
                read_data       = smp[idx20];
                read_data_valid = 1'b1;
                @(negedge clk);
                read_data_valid = 1'b0;
                read_data       = 16'h0;
                idx20 = idx20 + 1;
            end
        end
        #1; r2 = pixel_value;

        if (r1 !== $signed(21'sd100))
            $display("  FAIL TC20 1st: got %0d expected 100", $signed(r1));
        check_cond("TC20 first=100",  r1 === $signed(21'sd100));
        if (r2 !== $signed(21'sd200))
            $display("  FAIL TC20 2nd: got %0d expected 200", $signed(r2));
        check_cond("TC20 second=200", r2 === $signed(21'sd200));
    end

    // =========================================================================
    // TC21 – rst_n mid-computation: clean abort
    // =========================================================================
    $display("\n--- TC21: Mid-computation reset ---");
    do_reset;
    for (i = 0; i < N_LINES; i = i + 1) smp[i] = 16'sh7FFF;

    begin : tc21_blk
        integer k21, to21;
        k21 = 0; to21 = 0;

        @(negedge clk); start = 1'b1;
        @(negedge clk); start = 1'b0;

        // Feed 10 samples then assert reset
        while (k21 < 10 && to21 < 500) begin
            @(posedge clk);
            to21 = to21 + 1;
            if (read_enable && k21 < 10) begin
                @(negedge clk);
                read_data       = smp[k21];
                read_data_valid = 1'b1;
                @(negedge clk);
                read_data_valid = 1'b0;
                read_data       = 16'h0;
                k21 = k21 + 1;
            end
        end

        @(negedge clk); rst_n = 1'b0;
        repeat(3) @(negedge clk);

        check_cond("TC21 busy==0",  busy              === 1'b0);
        check_cond("TC21 pvv==0",   pixel_value_valid === 1'b0);
        check_cond("TC21 pv==0",    pixel_value       === 21'sh00000);
        check_cond("TC21 re==0",    read_enable       === 1'b0);

        rst_n = 1'b1; @(negedge clk);
    end

    // =========================================================================
    // TC22 – Hold-high start: auto re-trigger (documents Bug-3)
    // =========================================================================
    $display("\n--- TC22: Hold-high start => auto re-trigger (Bug-3) ---");
    do_reset;
    for (i = 0; i < N_LINES; i = i + 1) smp[i] = 16'sh0001;

    begin : tc22_blk
        integer to22, pvv_cnt, re_idx;
        to22    = 0;
        pvv_cnt = 0;
        re_idx  = 0;

        @(negedge clk); start = 1'b1; // keep permanently high

        while (pvv_cnt < 2 && to22 < 5000) begin
            @(posedge clk);
            to22 = to22 + 1;
            if (pixel_value_valid) pvv_cnt = pvv_cnt + 1;
            if (read_enable) begin
                @(negedge clk);
                read_data       = 16'sh0001;
                read_data_valid = 1'b1;
                @(negedge clk);
                read_data_valid = 1'b0;
                read_data       = 16'h0;
                re_idx = re_idx + 1;
            end
        end

        start = 1'b0;
        $display("  INFO TC22: completions=%0d (FSM re-triggers while start held high)", pvv_cnt);
        check_cond("TC22 re-trigger observed", pvv_cnt >= 2);
    end

    // =========================================================================
    // TC23 – All-zero samples → 0
    // =========================================================================
    $display("\n--- TC23: All-zero samples => 0 ---");
    do_reset;
    for (i = 0; i < N_LINES; i = i + 1) smp[i] = 16'sh0000;
    run_comp(0, 1, result, ok);
    check_cond("TC23 valid",       ok     === 1'b1);
    if (result !== 21'sh00000)
        $display("  FAIL TC23: got %0d expected 0", $signed(result));
    check_cond("TC23 pixel_value", result === 21'sh00000);

    // =========================================================================
    // TC24 – Single-cycle read_data_valid pulse (tight timing)
    // =========================================================================
    $display("\n--- TC24: Single-cycle pulse (tight timing) ---");
    do_reset;
    for (i = 0; i < N_LINES; i = i + 1) smp[i] = 16'sh0005; // +5
    run_comp(0, 1, result, ok);
    expected = $signed(21'sd125); // 25*5
    check_cond("TC24 valid",       ok      === 1'b1);
    if (result !== expected) $display("  FAIL TC24: got %0d expected 125", $signed(result));
    check_cond("TC24 pixel_value", result  === expected);

    // =========================================================================
    // Final summary
    // =========================================================================
    $display("\n========================================");
    $display("  TEST SUMMARY");
    $display("  PASSED : %0d", pass_cnt);
    $display("  FAILED : %0d", fail_cnt);
    $display("========================================\n");

    if (fail_cnt == 0)
        $display("ALL TESTS PASSED");
    else
        $display("*** FAILURES DETECTED: %0d ***", fail_cnt);

    $finish;
end

// ── Timeout guard ────────────────────────────────────────────────────────────
initial begin
    #10_000_000;
    $display("FATAL: simulation timeout");
    $finish;
end

endmodule
