module DasBeamFormerController#(parameter ADDRESS_WIDTH = 5)(
    //Standard   
    input clk,   
    input rst_n,
    input  start,

    //From delay address calculator
    input wire [ADDRESS_WIDTH-1:0] delay_address,

    //Control signals to memory matrix
    output wire [ADDRESS_WIDTH-1:0] read_address,
    output wire read_enable,
    output wire [4:0] read_line,

    //Data from memory matrix
    input wire signed [15:0] read_data,
    input wire read_data_valid,
    
    //Ouputs of this module
    output reg signed [20:0] pixel_value,
    output reg pixel_value_valid,
    output reg busy
);
//FSM states
localparam IDLE = 2'b00;
localparam REQUEST = 2'b01;
localparam WAIT = 2'b10;

reg [1:0] state;

//For storing current line
reg [4:0] current_line;

//Accumulator for pixel value
reg signed [20:0] pixel_accumulator;

//Sign extened each read sample,why??
wire signed [20:0] sample_extended;
assign sample_extended = {{5{read_data[15]}}, read_data};

//Assign memory controls
assign read_enable = (state == REQUEST);
assign read_line = current_line;
assign read_address =delay_address;
reg start_prev;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        start_prev <= 1'b1;  // Important: reset to 1, not 0
    else
        start_prev <= start;
end

wire start_pulse;
assign start_pulse = rst_n && start && !start_prev;

always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        state <= IDLE;
        current_line <= 5'b0;
        pixel_accumulator <= 21'b0;
        pixel_value <= 21'b0;
        pixel_value_valid <= 1'b0;
        busy <= 1'b0;
    end 
    else begin
        pixel_value_valid <= 1'b0; // Reset valid signal each cycle unless set in WAIT state

        case(state)
            IDLE: begin
                busy <= 1'b0;
                if(start) begin
                    busy <= 1'b1;
                    current_line <= 5'b0;
                    pixel_accumulator <= 21'b0;
                    state <= REQUEST;
                end
            end

            REQUEST: begin
            //Issue only one read at a time
                state <= WAIT;
            end

            WAIT: begin
            if(read_data_valid) begin
                //Processing for the final line read
                if(current_line == 5'd24)begin
                    pixel_accumulator <= pixel_accumulator + sample_extended;
                    pixel_value <= pixel_accumulator + sample_extended;
                    pixel_value_valid <= 1'b1;
                    busy<= 1'b0;
                    state <= IDLE;
                end
                else begin
                    pixel_accumulator <= pixel_accumulator + sample_extended;
                    current_line<=current_line+1'b1;
                    state<=REQUEST;

                end
            end
        end

            default: begin
                // Default case to handle unexpected states
                state <= IDLE;
                busy <= 1'b0;
            end
        endcase
    end
end
endmodule