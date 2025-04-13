module cordic_core (
    input wire clk,              // Clock input
    input wire rst,              // Synchronous reset (active high)
    input wire start,            // Start signal to initiate computation
    input wire [15:0] angle,     // Input angle (0 to 65535 represents 0 to ~2π radians)
    output reg [15:0] sin_out,   // Sine output (signed fixed-point)
    output reg [15:0] cos_out,   // Cosine output (signed fixed-point)
    output reg busy              // Busy flag (high during computation)
);

    // Fixed-point constants for angle boundaries (16-bit, scaled such that 65536 = 2π)
    localparam PI_OVER_2      = 16'd16384;  // π/2 ≈ 0.25 * 65536
    localparam PI             = 16'd32768;  // π ≈ 0.5 * 65536
    localparam THREE_PI_OVER_2 = 16'd49152; // 3π/2 ≈ 0.75 * 65536
    localparam X_INIT         = 16'sd9951;  // 1/K ≈ 0.60725 in s1.14 format (K ≈ 1.64676)

    // Internal registers
    reg signed [15:0] x, y, z;   // CORDIC registers (s1.14 fixed-point format)
    reg signed [15:0] angle_accum; // Register to track the angle of the rotating vector
    reg [3:0] iter;              // Iteration counter (0 to 15)
    reg [1:0] Q_reg;             // Stored quadrant number

    // Combinational logic for quadrant detection and angle reduction
    reg [1:0] Q;                 // Quadrant (0 to 3)
    reg [15:0] theta_prime;      // Reduced angle in [0, π/2)
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

    // Arctangent table (scaled by 65536 / (2π), 16-bit values)
    reg [15:0] atan_val;
    always @(*) begin
        case (iter)
            4'd0:  atan_val = 16'd8192;  // atan(2^0)   ≈ 0.7854 rad
            4'd1:  atan_val = 16'd4836;  // atan(2^-1)  ≈ 0.4636 rad
            4'd2:  atan_val = 16'd2555;  // atan(2^-2)  ≈ 0.2450 rad
            4'd3:  atan_val = 16'd1297;  // atan(2^-3)  ≈ 0.1244 rad
            4'd4:  atan_val = 16'd651;   // atan(2^-4)  ≈ 0.0624 rad
            4'd5:  atan_val = 16'd326;   // atan(2^-5)  ≈ 0.0312 rad
            4'd6:  atan_val = 16'd163;   // atan(2^-6)  ≈ 0.0156 rad
            4'd7:  atan_val = 16'd81;    // atan(2^-7)  ≈ 0.0078 rad
            4'd8:  atan_val = 16'd41;    // atan(2^-8)  ≈ 0.0039 rad
            4'd9:  atan_val = 16'd20;    // atan(2^-9)  ≈ 0.0020 rad
            4'd10: atan_val = 16'd10;    // atan(2^-10) ≈ 0.0010 rad
            4'd11: atan_val = 16'd5;     // atan(2^-11) ≈ 0.0005 rad
            4'd12: atan_val = 16'd3;     // atan(2^-12) ≈ 0.0002 rad
            4'd13: atan_val = 16'd1;     // atan(2^-13) ≈ 0.0001 rad
            4'd14: atan_val = 16'd1;     // atan(2^-14) ≈ 0.00005 rad
            4'd15: atan_val = 16'd0;     // atan(2^-15) ≈ 0 rad
            default: atan_val = 16'd0;
        endcase
    end

    // Main sequential logic
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            // Reset all registers
            busy <= 0;
            sin_out <= 0;
            cos_out <= 0;
            x <= 0;
            y <= 0;
            z <= 0;
            iter <= 0;
            Q_reg <= 0;
            angle_accum <= 0;
        end else begin
            if (start && !busy) begin
                // Start computation
                busy <= 1;
                iter <= 0;
                Q_reg <= Q;
                x <= X_INIT;      // Initialize x with inverse CORDIC gain
                y <= 16'sd0;
                z <= theta_prime; // Reduced angle
                angle_accum <= 0; // Initialize angle tracker
            end else if (busy) begin
                if (iter < 16) begin
                    // CORDIC iteration
                    if (z >= 0) begin
                        x <= x - (y >>> iter);
                        y <= y + (x >>> iter);
                        z <= z - atan_val;
                    end else begin
                        x <= x + (y >>> iter);
                        y <= y - (x >>> iter);
                        z <= z + atan_val;
                    end
                    // Update the angle accumulator
                    angle_accum <= angle_accum + (z >= 0 ? atan_val : -atan_val);
                    iter <= iter + 1;
                end
                if (iter == 15) begin
                    // Computation complete, assign outputs based on quadrant
                    busy <= 0;
                    case (Q_reg)
                        2'd0: begin  // 0 to 90°
                            sin_out <= y;
                            cos_out <= x;
                        end
                        2'd1: begin  // 90° to 180°
                            sin_out <= x;
                            cos_out <= -y;
                        end
                        2'd2: begin  // 180° to 270°
                            sin_out <= -y;
                            cos_out <= -x;
                        end
                        2'd3: begin  // 270° to 360°
                            sin_out <= -x;
                            cos_out <= y;
                        end
                    endcase
                end
            end
        end
    end

endmodule



module cordic_core_tb;

    // Testbench signals
    reg clk;              // Clock signal
    reg rst;              // Reset signal (active high)
    reg start;            // Start signal to initiate computation
    reg [15:0] angle;     // 16-bit angle input (0 to 65536)
    wire [15:0] sin_out;  // 16-bit signed fixed-point sine output
    wire [15:0] cos_out;  // 16-bit signed fixed-point cosine output
    wire busy;            // Busy flag indicating computation in progress

    // Instantiate the CORDIC core module
    cordic_core uut (
        .clk(clk),
        .rst(rst),
        .start(start),
        .angle(angle),
        .sin_out(sin_out),
        .cos_out(cos_out),
        .busy(busy)
    );

    // Clock generation (10ns period)
    always begin
        #5 clk = ~clk;
    end

    // Test procedure
    initial begin
        // Initialize signals
        clk = 0;
        rst = 1;
        start = 0;
        angle = 0;

        // Apply reset for 10ns
        #10;
        rst = 0;
        #10;

        // Repeat the test 5 times
        repeat (5) begin
            // Iterate through angles from 0 to 65536 in steps of 256
            for (angle = 0; angle <= 65536; angle = angle + 256) begin
                // Start computation by setting start high for one clock cycle
                start = 1;
                #10;
                start = 0;

                // Wait until computation is complete (busy goes low)
                while (busy) begin
                    #10;
                end

                // Display the angle, sine, and cosine results
                $display("Angle: %d, Sin: %d, Cos: %d", angle, sin_out, cos_out);
            end
        end

        // End the simulation
        $finish;
    end

endmodule


`timescale 1ns / 1ps

module cordic_core_tb;

    // Testbench signals
    reg clk;
    reg rst;
    reg start;
    reg [15:0] angle;
    wire [15:0] sin_out;
    wire [15:0] cos_out;
    wire busy;

    // Instantiate the DUT (Device Under Test)
    cordic_core uut (
        .clk(clk),
        .rst(rst),
        .start(start),
        .angle(angle),
        .sin_out(sin_out),
        .cos_out(cos_out),
        .busy(busy)
    );

    // Clock generation
    always #5 clk = ~clk;  // 100 MHz clock (10 ns period)

    initial begin
        // Initialize inputs
        clk = 0;
        rst = 1;
        start = 0;
        angle = 16'd10014;  // Test angle ≈ 55.1°

        // Apply reset
        #10;
        rst = 0;
        
        // Start the CORDIC computation
        #10;
        start = 1;
        
        #10;
        start = 0;  // De-assert start

        // Wait for the computation to finish
        wait (!busy);

        // Display results
        $display("Angle: %d", angle);
        $display("Sine Output: %d", sin_out);
        $display("Cosine Output: %d", cos_out);

        #20;
        $finish;
    end

endmodule
