module top (
    input wire CLOCK_50,         // 50 MHz clock on DE10-Lite
    input wire [1:0] SW,         // SW[0] = sw_rst_start, SW[1] = sw_sin_cos
    input wire [9:0] KEY,        // Angle input (e.g., from switches or keys)
    output wire [6:0] HEX0,      // Ones digit
    output wire [6:0] HEX1,      // Tens digit
    output wire [6:0] HEX2,      // Hundreds digit
    output wire [6:0] HEX3,      // Thousands digit
    output wire [6:0] HEX4,      // Sign display
    output wire [6:0] HEX5,       // Mode display (S or C)
    output wire HEX3_dp      
);

    assign HEX3_dp = 1'b0;

    // CORDIC instance
    wire [9:0] sin_out, cos_out;
    wire busy;
    cordic_core cordic_inst (
        .clk(CLOCK_50),
        .sw_rst_start(SW[0]),
        .angle(KEY[9:0]),
        .sin_out(sin_out),
        .cos_out(cos_out),
        .busy(busy)
    );

    // Select sine or cosine based on SW[1]
    wire [9:0] selected_out = SW[1] ? cos_out : sin_out;

    // Extract sign and absolute value
    wire sign = selected_out[9];
    wire [9:0] abs_value = sign ? -selected_out : selected_out;

    // Scale for display: (abs_value * 1000) / 512 ≈ abs_value * 1.953125
    // Displays value * 1000 for 3 decimal places (e.g., 0.500 as 500)
    wire [19:0] product = abs_value * 10'd1000; // 10-bit * 10-bit = 20-bit
    wire [10:0] display_value = product >> 9;   // /512, result <= 998

    // Convert to BCD (simplified using division for clarity)
    wire [3:0] digit0 = display_value % 10;
    wire [3:0] digit1 = (display_value / 10) % 10;
    wire [3:0] digit2 = (display_value / 100) % 10;
    wire [3:0] digit3 = (display_value / 1000) % 10;

    // Seven-segment display drivers
    reg [6:0] hex5_seg, hex4_seg, hex3_seg, hex2_seg, hex1_seg, hex0_seg;
    always @(*) begin
        // HEX5: Mode ('S' for sine, 'C' for cosine)
        hex5_seg = SW[1] ? 7'b1000110 : 7'b0110001; // 'C' : 'S'

        // HEX4: Sign ('-' if negative, blank if positive)
        hex4_seg = sign ? 7'b0111111 : 7'b1111111;  // '-' : blank

        // HEX3 to HEX0: BCD digits
        case (digit3)
    4'd0: hex3_seg = 7'b1000000;
    4'd1: hex3_seg = 7'b1111001;
    4'd2: hex3_seg = 7'b0100100;
    4'd3: hex3_seg = 7'b0110000;
    4'd4: hex3_seg = 7'b0011001;
    4'd5: hex3_seg = 7'b0010010;
    4'd6: hex3_seg = 7'b0000010;
    4'd7: hex3_seg = 7'b1111000;
    4'd8: hex3_seg = 7'b0000000;
    4'd9: hex3_seg = 7'b0010000;
    default: hex3_seg = 7'b1111111; // All segments OFF
        endcase
        case (digit2)
    4'd0: hex2_seg = 7'b1000000;
    4'd1: hex2_seg = 7'b1111001;
    4'd2: hex2_seg = 7'b0100100;
    4'd3: hex2_seg = 7'b0110000;
    4'd4: hex2_seg = 7'b0011001;
    4'd5: hex2_seg = 7'b0010010;
    4'd6: hex2_seg = 7'b0000010;
    4'd7: hex2_seg = 7'b1111000;
    4'd8: hex2_seg = 7'b0000000;
    4'd9: hex2_seg = 7'b0010000;
    default: hex2_seg = 7'b1111111; // All segments OFF
        endcase
        case (digit1)
    4'd0: hex1_seg = 7'b1000000;
    4'd1: hex1_seg = 7'b1111001;
    4'd2: hex1_seg = 7'b0100100;
    4'd3: hex1_seg = 7'b0110000;
    4'd4: hex1_seg = 7'b0011001;
    4'd5: hex1_seg = 7'b0010010;
    4'd6: hex1_seg = 7'b0000010;
    4'd7: hex1_seg = 7'b1111000;
    4'd8: hex1_seg = 7'b0000000;
    4'd9: hex1_seg = 7'b0010000;
    default: hex1_seg = 7'b1111111; // All segments OFF
        endcase
        case (digit0)
    4'd0: hex0_seg = 7'b1000000;
    4'd1: hex0_seg = 7'b1111001;
    4'd2: hex0_seg = 7'b0100100;
    4'd3: hex0_seg = 7'b0110000;
    4'd4: hex0_seg = 7'b0011001;
    4'd5: hex0_seg = 7'b0010010;
    4'd6: hex0_seg = 7'b0000010;
    4'd7: hex0_seg = 7'b1111000;
    4'd8: hex0_seg = 7'b0000000;
    4'd9: hex0_seg = 7'b0010000;
    default: hex0_seg = 7'b1111111; // All segments OFF
        endcase
    end

    // Assign outputs
    assign HEX5 = hex5_seg;
    assign HEX4 = hex4_seg;
    assign HEX3 = hex3_seg;
    assign HEX2 = hex2_seg;
    assign HEX1 = hex1_seg;
    assign HEX0 = hex0_seg;
endmodule


module cordic_core (
    input wire clk,              // Clock input
    input wire sw_rst_start,     // Combined reset/start switch (0 = reset, 1 = start)
    input wire [9:0] angle,      // Input angle (0 to 1023 represents 0 to ~2π radians)
    output reg [9:0] sin_out,    // Sine output (signed, s0.9 format)
    output reg [9:0] cos_out,    // Cosine output (signed, s0.9 format)
    output reg busy              // Busy flag (high during computation)
);

    // Fixed-point constants for angle boundaries (10-bit, scaled such that 1024 = 2π)
    localparam PI_OVER_2      = 10'd256;   // π/2 ≈ 0.25 * 1024
    localparam PI             = 10'd512;   // π ≈ 0.5 * 1024
    localparam THREE_PI_OVER_2 = 10'd768;  // 3π/2 ≈ 0.75 * 1024
    localparam X_INIT         = 10'sd311;  // 1/K ≈ 0.60725 * 2^9 (K ≈ 1.64676)

    // Internal registers
    reg signed [9:0] x, y, z;    // CORDIC registers (s0.9 format)
    reg signed [9:0] angle_accum;// Angle accumulator
    reg [3:0] iter;              // Iteration counter (0 to 9)
    reg [1:0] Q_reg;             // Stored quadrant number
    reg sw_rst_start_prev;       // Previous value of sw_rst_start for edge detection

    // Define reset and start signals
    wire rst = ~sw_rst_start;    // Reset when sw_rst_start = 0
    wire start_internal = sw_rst_start & ~sw_rst_start_prev; // Start on rising edge

    // Quadrant detection and angle reduction
    reg [1:0] Q;
    reg [9:0] theta_prime;
    always @(*) begin
        if (angle < PI_OVER_2) begin
            Q = 2'd0;
            theta_prime = angle;
        end else if (angle < PI) begin
            Q = 2'd1;
            theta_prime = angle - PI_OVER_2;
        end else if (angle < THREE_PI_OVER_2) begin
            Q = 2'd2;
            theta_prime = angle - PI;
        end else begin
            Q = 2'd3;
            theta_prime = angle - THREE_PI_OVER_2;
        end
    end

    // Arctangent table (scaled by 1024 / (2π), 10-bit values)
    reg [9:0] atan_val;
    always @(*) begin
        case (iter)
            4'd0: atan_val = 10'd128;  // atan(2^0)   ≈ 0.7854 rad
            4'd1: atan_val = 10'd76;   // atan(2^-1)  ≈ 0.4636 rad
            4'd2: atan_val = 10'd40;   // atan(2^-2)  ≈ 0.2450 rad
            4'd3: atan_val = 10'd20;   // atan(2^-3)  ≈ 0.1244 rad
            4'd4: atan_val = 10'd10;   // atan(2^-4)  ≈ 0.0624 rad
            4'd5: atan_val = 10'd5;    // atan(2^-5)  ≈ 0.0312 rad
            4'd6: atan_val = 10'd3;    // atan(2^-6)  ≈ 0.0156 rad
            4'd7: atan_val = 10'd1;    // atan(2^-7)  ≈ 0.0078 rad
            4'd8: atan_val = 10'd1;    // atan(2^-8)  ≈ 0.0039 rad
            4'd9: atan_val = 10'd0;    // atan(2^-9)  ≈ 0.0020 rad
            default: atan_val = 10'd0;
        endcase
    end

    // Main sequential logic
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            busy <= 0;
            sin_out <= 0;
            cos_out <= 0;
            x <= 0;
            y <= 0;
            z <= 0;
            iter <= 0;
            Q_reg <= 0;
            angle_accum <= 0;
            sw_rst_start_prev <= 0;
        end else begin
            sw_rst_start_prev <= sw_rst_start;
            if (start_internal && !busy) begin
                busy <= 1;
                iter <= 0;
                Q_reg <= Q;
                x <= X_INIT;
                y <= 10'sd0;
                z <= theta_prime;
                angle_accum <= 0;
            end else if (busy) begin
                if (iter < 10) begin
                    if (z >= 0) begin
                        x <= x - (y >>> iter);
                        y <= y + (x >>> iter);
                        z <= z - atan_val;
                    end else begin
                        x <= x + (y >>> iter);
                        y <= y - (x >>> iter);
                        z <= z + atan_val;
                    end
                    angle_accum <= angle_accum + (z >= 0 ? atan_val : -atan_val);
                    iter <= iter + 1;
                end
                if (iter == 9) begin
                    busy <= 0;
                    case (Q_reg)
                        2'd0: begin
                            sin_out <= y;
                            cos_out <= x;
                        end
                        2'd1: begin
                            sin_out <= x;
                            cos_out <= -y;
                        end
                        2'd2: begin
                            sin_out <= -y;
                            cos_out <= -x;
                        end
                        2'd3: begin
                            sin_out <= -x;
                            cos_out <= y;
                        end
                    endcase
                end
            end
        end
    end
endmodule
