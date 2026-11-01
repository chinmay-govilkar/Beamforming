module DasSingleChannel(   
    input wire clk,
    input wire rst_n,
    input wire signed [15:0] data_in,
    input wire data_valid,
    output reg signed [15:0] data_out,
    output reg data_out_valid
);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            data_out <= 16'b0;
            data_out_valid <= 1'b0;
        end else if (data_valid) begin
            data_out <= data_in;
            data_out_valid <= 1'b1;
        end else begin
            data_out_valid <= 1'b0;
        end
    end
    //currently this module just passes the input data to output when data_valid is high. You can add your processing logic here as needed.  