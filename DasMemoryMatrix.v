
module DasMemoryMatrix#(parameter ADDR_WIDTH = 5) 
(
    input wire clk,
    input wire rst_n,

    // 0 = acquisition, 1 = beamforming
    input wire beamform_mode,

    // Acquisition inputs
    input wire we,
    input wire [4:0] current_line,
    input wire [ADDR_WIDTH-1:0] write_addr,
    input wire signed [15:0] data_in,

    // Beamformer read inputs
    input wire read_en,
    input wire [4:0] read_line,
    input wire [ADDR_WIDTH-1:0] read_addr,

    // Memory output
    output reg signed [15:0] read_data,
    output reg read_valid
);

    localparam DEPTH = 1<<ADDR_WIDTH; // 16384, 2^14
  

    reg signed [15:0] ram [0:25*DEPTH-1];

    wire [ADDR_WIDTH+4:0] write_index;
    wire [ADDR_WIDTH+4:0] read_index;

    assign write_index = {current_line, write_addr};
    assign read_index  = {read_line, read_addr};

    always @(posedge clk) begin

        if (!rst_n) begin
            read_valid <= 1'b0;
        end
        else begin
            read_valid <= 1'b0;

            if (!beamform_mode) begin
                // Writing from ADC to memory
                if (we && current_line < 25)
                    ram[write_index] <= data_in;
            end
            else begin
                // Beamforming mode: reading from memory
                if (read_en && read_line < 25) begin
                    read_data <= ram[read_index];
                    read_valid <= 1'b1;
                end
            end
        end
    end

endmodule
