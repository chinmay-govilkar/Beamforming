
`timescale 1ns / 1ps

module tb_DasMemoryMatrix;

    localparam AW = 5;  // 32 samples per line

    // Inputs to the memory module
    reg clk = 0;
    reg rst_n = 0;
    reg beamform_mode = 0;

    reg we = 0;
    reg [4:0] current_line = 0;
    reg [AW-1:0] write_addr = 0;
    reg signed [15:0] data_in = 0;

    reg read_en = 0;
    reg [4:0] read_line = 0;
    reg [AW-1:0] read_addr = 0;

    // Outputs from the memory module
    wire signed [15:0] read_data;
    wire read_valid;

    integer errors = 0;

    // Generate a 100 MHz simulation clock
    always #5 clk = ~clk;

    // Instantiate the module under test
    DasMemoryMatrix #(
        .ADDR_WIDTH(AW)
    ) dut (
        .clk(clk),
        .rst_n(rst_n),
        .beamform_mode(beamform_mode),

        .we(we),
        .current_line(current_line),
        .write_addr(write_addr),
        .data_in(data_in),

        .read_en(read_en),
        .read_line(read_line),
        .read_addr(read_addr),

        .read_data(read_data),
        .read_valid(read_valid)
    );

    // Task: write one sample into memory
    task write_sample;
        input [4:0] line;
        input [AW-1:0] addr;
        input signed [15:0] value;
        begin
            @(negedge clk);

            beamform_mode = 0;
            current_line = line;
            write_addr = addr;
            data_in = value;
            we = 1;

            @(negedge clk);
            we = 0;
        end
    endtask

    // Task: read and verify one sample
    task check_sample;
        input [4:0] line;
        input [AW-1:0] addr;
        input signed [15:0] expected;
        begin
            @(negedge clk);

            beamform_mode = 1;
            read_line = line;
            read_addr = addr;
            read_en = 1;

            @(posedge clk);
            #1;  // Wait for nonblocking assignments

            if (read_valid !== 1'b1 ||
                read_data !== expected) begin

                $display(
                    "FAIL: line=%0d addr=%0d expected=%0d got=%0d valid=%b",
                    line, addr, expected, read_data, read_valid
                );

                errors = errors + 1;
            end
            else begin
                $display(
                    "PASS: line=%0d addr=%0d data=%0d",
                    line, addr, read_data
                );
            end

            @(negedge clk);
            read_en = 0;

            @(posedge clk);
            #1;

            if (read_valid !== 1'b0) begin
                $display("FAIL: read_valid did not clear");
                errors = errors + 1;
            end
        end
    endtask

    // Main test sequence
    initial begin

        // Reset the module
        rst_n = 0;
        #12;
        @(negedge clk);
        rst_n = 1;

        // TEST 1: Store three samples
        write_sample(5, 3, 1500);
        write_sample(5, 4, 1234);
        write_sample(12, 3, -2000);

        // TEST 2: Read them back
        check_sample(5, 3, 1500);
        check_sample(5, 4, 1234);
        check_sample(12, 3, -2000);

        // TEST 3: Writes must be disabled
        // during beamforming
        @(negedge clk);
        beamform_mode = 1;
        we = 1;
        current_line = 5;
        write_addr = 3;
        data_in = 9999;

        @(negedge clk);
        we = 0;

        // Original value must still be 1500
        check_sample(5, 3, 1500);

        // TEST 4: Reads must be disabled
        // during acquisition
        @(negedge clk);
        beamform_mode = 0;
        read_en = 1;
        read_line = 12;
        read_addr = 3;

        @(posedge clk);
        #1;

        if (read_valid !== 0) begin
            $display("FAIL: read occurred during acquisition");
            errors = errors + 1;
        end

        @(negedge clk);
        read_en = 0;

        // TEST 5: Invalid line should not
        // produce a valid response
        @(negedge clk);
        beamform_mode = 1;
        read_en = 1;
        read_line = 25;
        read_addr = 0;

        @(posedge clk);
        #1;

        if (read_valid !== 0) begin
            $display("FAIL: invalid line accepted");
            errors = errors + 1;
        end

        @(negedge clk);
        read_en = 0;

        // Final result
        if (errors == 0)
            $display("ALL TESTS PASSED");
        else
            $display("FAILED: %0d errors", errors);

        $finish;
    end

    // Prevent simulation from hanging forever
    initial begin
        #5000;
        $display("FAIL: simulation timeout");
        $finish;
    end

endmodule
