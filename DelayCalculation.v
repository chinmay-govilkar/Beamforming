
module DelayCalculation #(
    parameter ADDRESS_WIDTH = 13
)(
    // Standard signals
    input wire clk,
    input wire rst_n,
    input wire start,

    // Acquisition line: 0 to 24
    input wire [4:0] lineIndex,

    // Target pixel coordinates
    input wire [7:0] pixel_x_index,   // 0 to 240
    input wire [8:0] pixel_z_index,   // 0 to 400

    // Outputs
    output reg [ADDRESS_WIDTH-1:0] delay_address,
    output reg done,
    output reg busy,
    output reg coordinateError
);


localparam IDLE = 1'b0;
localparam SQRT = 1'b1;

reg state;

// 2. START EDGE DETECTION
reg start_prev;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        start_prev <= 1'b1;
    else
        start_prev <= start;
end

wire start_pulse;
assign start_pulse = rst_n && start && !start_prev;

// 3. COORDINATE CONVERSION
// All physical coordinates are in micrometres.

// Convert acquisition line into signed coordinate.
// Line 0  = -12000 um
// Line 12 =      0 um
// Line 24 = +12000 um

wire signed [15:0] line_index_signed;
wire signed [15:0] line_offset;
wire signed [31:0] x_probe_um;

assign line_index_signed = $signed({11'b0, lineIndex});
assign line_offset = line_index_signed - 16'sd12;
assign x_probe_um = line_offset * 32'sd1000;


// Convert horizontal pixel index into coordinate.
// Pixel 0   = -12000 um
// Pixel 120 =      0 um
// Pixel 240 = +12000 um

wire signed [15:0] pixel_x_index_signed;
wire signed [15:0] pixel_x_offset;
wire signed [31:0] x_pixel_um;

assign pixel_x_index_signed = $signed({8'b0, pixel_x_index});
assign pixel_x_offset = pixel_x_index_signed - 16'sd120;
assign x_pixel_um = pixel_x_offset * 32'sd100;


// Calculate signed lateral separation.

wire signed [31:0] dx_um_signed;

assign dx_um_signed = x_probe_um - x_pixel_um;


// Take absolute value of lateral separation.

wire [31:0] dx_um_abs;

assign dx_um_abs =dx_um_signed[31]? (~dx_um_signed + 32'd1): dx_um_signed; //Finding the absoulete by 2's complement method


// Calculate target pixel depth.
// Pixel Z index 0   = 10000 um
// Pixel Z index 200 = 30000 um
// Pixel Z index 400 = 50000 um

wire [31:0] z_pixel_um;

assign z_pixel_um = 32'd10000 + ({23'b0, pixel_z_index} * 32'd100);

// 4. SQUARED DISTANCE
// D^2 = dx^2 + z^2

wire [63:0] dx_um_64;
wire [63:0] z_pixel_um_64;

assign dx_um_64 = {32'b0, dx_um_abs};
assign z_pixel_um_64 = {32'b0, z_pixel_um};

wire [63:0] dx_squared;
wire [63:0] z_squared;
wire [63:0] distance_squared_64;

assign dx_squared = dx_um_64 * dx_um_64;
assign z_squared = z_pixel_um_64 * z_pixel_um_64;

assign distance_squared_64 = dx_squared + z_squared;

// For valid coordinates, this fits in 32 bits.
// The square-root unit processes a 32-bit value.

wire [31:0] distance_squared;

assign distance_squared = distance_squared_64[31:0];

wire valid_coordinates;

assign valid_coordinates =(lineIndex <= 5'd24)&&(pixel_x_index <= 8'd240) &&(pixel_z_index <= 9'd400) &&(distance_squared_64[63:32] == 32'd0);

// 5. ITERATIVE INTEGER SQUARE ROOT
// 32-bit input, 16 iterations.

reg [31:0] radicand_shift;
reg [15:0] sqrt_root;
reg [17:0] sqrt_remainder;
reg [4:0] iteration_count;

// Take the next two radicand bits.
wire [17:0] remainder_shifted;

assign remainder_shifted = {sqrt_remainder[15:0], radicand_shift[31:30]};


// Shift partial root left by one bit.

wire [15:0] root_shifted;

assign root_shifted =
    {sqrt_root[14:0], 1'b0};


// Trial divisor = (partial root * 4) + 1.

wire [16:0] trial_divisor;
wire [17:0] trial_divisor_extended;

assign trial_divisor = {root_shifted, 1'b1};
assign trial_divisor_extended = {1'b0, trial_divisor};


// Decide whether the next root bit is 1 or 0.

wire take_root_bit;

assign take_root_bit =
    (remainder_shifted >= trial_divisor_extended);


wire [17:0] remainder_next;
wire [15:0] root_next;

assign remainder_next =
    take_root_bit
    ? (remainder_shifted - trial_divisor_extended)
    : remainder_shifted;

assign root_next =
    take_root_bit
    ? (root_shifted + 16'd1)
    : root_shifted;


// Round square root to nearest micrometre.
// If remainder > root, round upward.

wire [15:0] rounded_distance_um;

assign rounded_distance_um =
    (remainder_next > {2'b00, root_next})
    ? (root_next + 16'd1)
    : root_next;

// 6. DISTANCE TO ADC SAMPLE ADDRESS
//
// For Fs = 105 MHz and c = 1540 m/s:
//
// address = round(3 * distance_um / 22)
//
// Integer rounding:
// address = (3 * distance_um + 11) / 22


wire [31:0] rounded_distance_32;
wire [31:0] sample_address_calculated;

assign rounded_distance_32 =
    {16'd0, rounded_distance_um};

assign sample_address_calculated =
    ((rounded_distance_32 * 32'd3) + 32'd11) / 32'd22;


// Check that address fits in configured memory.

localparam [31:0] MEMORY_DEPTH =
    (32'd1 << ADDRESS_WIDTH);


// ==================================================
// 7. MAIN CONTROL FSM
// ==================================================

always @(posedge clk or negedge rst_n) begin

    if (!rst_n) begin

        state             <= IDLE;

        radicand_shift    <= 32'd0;
        sqrt_root         <= 16'd0;
        sqrt_remainder    <= 18'd0;
        iteration_count   <= 5'd0;

        delay_address     <= {ADDRESS_WIDTH{1'b0}};

        done              <= 1'b0;
        busy              <= 1'b0;
        coordinateError   <= 1'b0;

    end

    else begin

        // Default: done is a one-cycle pulse.
        done <= 1'b0;

        case (state)

            // --------------------------------------
            // IDLE
            // --------------------------------------

            IDLE: begin

                busy <= 1'b0;

                if (start_pulse) begin

                    if (!valid_coordinates) begin

                        // Invalid input coordinates.
                        coordinateError <= 1'b1;
                        delay_address   <= {ADDRESS_WIDTH{1'b0}};
                        done            <= 1'b1;

                    end

                    else begin

                        // Capture squared distance.
                        radicand_shift <= distance_squared;

                        // Initialize square-root calculation.
                        sqrt_root       <= 16'd0;
                        sqrt_remainder  <= 18'd0;
                        iteration_count <= 5'd0;

                        coordinateError <= 1'b0;
                        busy             <= 1'b1;

                        state <= SQRT;

                    end
                end
            end


            // --------------------------------------
            // SQRT
            // --------------------------------------

            SQRT: begin

                // Shift radicand by two bits.
                radicand_shift <=
                    {radicand_shift[29:0], 2'b00};

                // Update square-root registers.
                sqrt_root      <= root_next;
                sqrt_remainder <= remainder_next;


                // Sixteen iterations are complete.
                if (iteration_count == 5'd15) begin

                    // Check calculated address against memory depth.
                    if (sample_address_calculated >= MEMORY_DEPTH) begin

                        delay_address   <= {ADDRESS_WIDTH{1'b0}};
                        coordinateError <= 1'b1;

                    end

                    else begin

                        delay_address <=
                            sample_address_calculated[ADDRESS_WIDTH-1:0];

                        coordinateError <= 1'b0;

                    end

                    done  <= 1'b1;
                    busy  <= 1'b0;
                    state <= IDLE;

                end

                else begin

                    iteration_count <= iteration_count + 1'b1;

                end

            end


            // --------------------------------------
            // DEFAULT
            // --------------------------------------

            default: begin

                state             <= IDLE;
                busy              <= 1'b0;
                done              <= 1'b0;

                radicand_shift    <= 32'd0;
                sqrt_root         <= 16'd0;
                sqrt_remainder    <= 18'd0;
                iteration_count   <= 5'd0;

                delay_address     <= {ADDRESS_WIDTH{1'b0}};
                coordinateError   <= 1'b1;

            end

        endcase

    end

end

endmodule
