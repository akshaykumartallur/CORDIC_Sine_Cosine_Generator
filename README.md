# CORDIC_Sine_Cosine_Generator
## Implementation of Sine and Cosine Generators using CORDIC Algorithm on FPGA

<!DOCTYPE html>
<html lang = "en">
<body>
<h2>Team Members</h2>
<p>
  <span><b>Abhay Surya Shankar &nbsp;&nbsp;</b><a href="mailto:abhaysurya2912@gmail.com"><b>abhaysurya2912@gmail.com</b></a> </span><br>
  <span><b>Akshaykumar N T &nbsp;&nbsp;</b><a href="mailto:tallurakshaykumar@gmail.com"><b>tallurakshaykumar@gmail.com</b></a> </span><br>
  <span><b>Bhavya Y &nbsp;&nbsp;</b><a href="mailto:bhavyayerangali@gmail.com"><b>bhavyayerangali@gmail.com</b></a> </span><br>
  <span><b>Deepthi S R &nbsp;&nbsp;</b><a href="mailto:deepthisr@gmail.com"><b>deepthisr@gmail.com</b></a> </span><br><br>
  <em>Electronics and Communication students at Dayananda Sagar College of Engineering, Bengaluru</em>
</p>
<h2>Project Overview</h2>
  <p>This project demonstrates the <em><b>hardware and software implementation</b></em> of sine and cosine wave generation using the <b><em>CORDIC (Coordinate Rotation Digital Computer) algorithm</em></b>  on two platforms:
<ol type="1">
    <li><b>Intel FPGA DE10-Lite (MAX10M50DAF484C7G)</b> – HDL implementation with output on 7-segment displays.</li>
    <li><b>ESP32 Microcontroller</b> – C implementation with analog output on an oscilloscope via DAC.</li>
</ol>
The CORDIC algorithm is chosen for its <b>hardware efficiency</b>, eliminating the need for multipliers and making it ideal for <b>FPGA and embedded systems.</b></p>
<h2>Introduction</h2>
  <p>The CORDIC algorithm is an iterative method that uses only shift, add, and subtract operations to compute functions. This makes it ideal for hardware implementations, especially where hardware multipliers are expensive or unavailable.

The algorithm works by rotating a vector in a plane by a given angle using a series of predefined micro-rotations.</p>
  <h3>CORDIC Algorithm (COordinate Rotation DIgital Computer)</h3>

  <p>The CORDIC algorithm is an iterative, hardware-friendly technique for calculating various mathematical functions such as:</p>
  <ul>
    <li>Trigonometric functions (sine, cosine)</li>
    <li>Hyperbolic functions</li>
    <li>Multiplication and division</li>
    <li>Exponential and logarithmic functions</li>
    <li>Square roots</li>
  </ul>

  <h3>Core Idea Behind CORDIC</h3>
  <p>CORDIC is based on vector rotation. The goal is to rotate a vector <code>(x, y)</code> by an angle <code>θ</code> using only shift and add operations.</p>

  <h4>Vector Rotation Formula:</h4>
  <pre>
x' = x * cos(θ) - y * sin(θ)
y' = x * sin(θ) + y * cos(θ)
  </pre>

  <p>CORDIC breaks this into a series of micro-rotations with fixed angles such that <code>tan(θ) = 2<sup>-i</sup></code>.</p>

  <h3>CORDIC Iterative Equations</h3>
  <pre>
x_{i+1} = x_i - d_i * y_i * 2^-i
y_{i+1} = y_i + d_i * x_i * 2^-i
z_{i+1} = z_i - d_i * atan(2^-i)
  </pre>

  <p>Where:</p>
  <ul>
    <li><code>d_i</code> is the rotation direction: +1 or -1 depending on <code>z_i</code></li>
    <li><code>atan(2<sup>-i</sup>)</code> are precomputed values</li>
    <li>Shifts (<code>2^-i</code>) are implemented using bit-shifts</li>
  </ul>

  <h3>Convergence Range</h3>
  <p>CORDIC in rotation mode converges for <code>θ</code> in the range ±99.88° (±1.743 radians). Angle range is extended using quadrant corrections.</p>

  <h3>CORDIC Scaling Factor</h3>
  <p>After n iterations, the result is scaled by a constant factor <code>K_n</code>:</p>
  <pre>
K_n = ∏ (1 / √(1 + 2^(-2i)))  for i = 0 to n-1
≈ 0.607253 for large n
  </pre>

  <p>You must compensate for this scaling factor either before or after computation.</p>

  <h2>Computing Sine and Cosine</h2>
  <ul>
    <li>Initialize: <code>x₀ = K</code>, <code>y₀ = 0</code>, <code>z₀ = θ</code></li>
    <li>After n iterations:
      <ul>
        <li><code>x_n ≈ K * cos(θ)</code></li>
        <li><code>y_n ≈ K * sin(θ)</code></li>
      </ul>
    </li>
  </ul>

  <h2>Advantages of CORDIC</h2>
  <ul>
    <li>Only shift and add operations (no multiplication/division)</li>
    <li>Efficient in hardware (FPGA, ASIC, microcontrollers)</li>
    <li>Great for fixed-point arithmetic</li>
  </ul>

  <h2>Limitations</h2>
  <ul>
    <li>Slower convergence compared to lookup tables or series expansion</li>
    <li>Needs pre-scaling or post-scaling</li>
    <li>Limited direct angle range</li>
  </ul>

  <h2>Applications</h2>
  <ul>
    <li>Scientific calculators</li>
    <li>FPGA-based signal processing</li>
    <li>Real-time embedded systems</li>
    <li>Digital communication systems</li>
    <li>Robotics and control systems</li>
  </ul>
  <h3>How Does CORDIC Rotation Work?</h3>

https://github.com/user-attachments/assets/3234d375-1aa5-4ce9-aec5-aba40471c3c3

  
<h2>Simulation using Xilinx Vivado</h2>
<h3>Verilog Code</h3>
  
  ```verilog
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

module cordic_core_tb_1;

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

module cordic_core_tb_2;

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
```

<h3>Simulation Results</h3>
<img src="https://github.com/akshaykumartallur/CORDIC_Sine_Cosine_Generator/blob/main/Cordic_Simulation_Xilinx.png" alt="Simulation Results xilinx">
<br>
<h2>FPGA Implementation</h2>
<p><b>FGPA Used:</b> De-10 lite 10M50DAF484C7G.</p>
<h3>HDL Code</h3>

```verilog
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

```

<h3>Results</h3>

<span><img src="https://github.com/akshaykumartallur/CORDIC_Sine_Cosine_Generator/blob/main/FPGA%20Results/sin(35).jpg" alt="sine35" width=45%></span>
<span><img src="https://github.com/akshaykumartallur/CORDIC_Sine_Cosine_Generator/blob/main/FPGA%20Results/cos(35).jpg" alt="cos35" width=45%></span>
<span><img src="https://github.com/akshaykumartallur/CORDIC_Sine_Cosine_Generator/blob/main/FPGA%20Results/cos(50).jpg" alt="cos50" width=45%></span>
<span><img src="https://github.com/akshaykumartallur/CORDIC_Sine_Cosine_Generator/blob/main/FPGA%20Results/sin(-50).jpg" alt="sineMinus50" width=45%></span>
<span><img src="https://github.com/akshaykumartallur/CORDIC_Sine_Cosine_Generator/blob/main/FPGA%20Results/sin(-45).jpg" alt="sineMinus45" width=45%></span>
<span><img src="https://github.com/akshaykumartallur/CORDIC_Sine_Cosine_Generator/blob/main/FPGA%20Results/cos(135).jpg" alt="cos135" width=45%></span>

<h2>Oscilloscope Implementation</h2>
<h3>ESP-32 Code</h3>

```cpp
#include <Arduino.h>

class CordicCore {
private:
    const uint8_t CORDIC_PI_OVER_2 = 64;
    const uint8_t CORDIC_PI = 128;
    const uint8_t THREE_PI_OVER_2 = 192;
    const int8_t X_INIT = 77;

    int8_t x, y, z;
    int8_t angle_accum;
    uint8_t iter;
    uint8_t Q_reg;
    bool busy;

    const uint8_t atan_table[8] = {32, 19, 10, 5, 3, 1, 1, 0};

public:
    void reset() {
        busy = false;
        x = y = z = 0;
        iter = 0;
        Q_reg = 0;
        angle_accum = 0;
    }

    void compute(uint8_t angle) {
        if (busy) return;

        uint8_t theta_prime;
        if (angle < CORDIC_PI_OVER_2) {
            Q_reg = 0;
            theta_prime = angle;
        } else if (angle < CORDIC_PI) {
            Q_reg = 1;
            theta_prime = angle - CORDIC_PI_OVER_2;
        } else if (angle < THREE_PI_OVER_2) {
            Q_reg = 2;
            theta_prime = angle - CORDIC_PI;
        } else {
            Q_reg = 3;
            theta_prime = angle - THREE_PI_OVER_2;
        }

        busy = true;
        iter = 0;
        x = X_INIT;
        y = 0;
        z = theta_prime;
        angle_accum = 0;
    }

    void update() {
        if (!busy) return;

        if (iter < 8) {
            uint8_t atan_val = atan_table[iter];
            
            if (z >= 0) {
                int8_t x_new = x - (y >> iter);
                int8_t y_new = y + (x >> iter);
                x = x_new;
                y = y_new;
                z -= atan_val;
            } else {
                int8_t x_new = x + (y >> iter);
                int8_t y_new = y - (x >> iter);
                x = x_new;
                y = y_new;
                z += atan_val;
            }
            
            angle_accum += (z >= 0) ? atan_val : -atan_val;
            iter++;
        }

        if (iter == 8) {
            busy = false;
        }
    }

    int8_t getSin() {
        switch (Q_reg) {
            case 0: return y;
            case 1: return x;
            case 2: return -y;
            case 3: return -x;
            default: return 0;
        }
    }

    int8_t getCos() {
        switch (Q_reg) {
            case 0: return x;
            case 1: return -y;
            case 2: return -x;
            case 3: return y;
            default: return 0;
        }
    }

    bool isBusy() const { return busy; }
};
CordicCore cordic;

// ESP32 DAC Pins (Arduino framework uses GPIO numbers)
#define SIN_PIN 25  // DAC1 (GPIO25)
#define COS_PIN 26  // DAC2 (GPIO26)

void setup() {
    Serial.begin(115200);
    
    // No need for explicit DAC enable in Arduino framework
    cordic.reset();
}

void loop() {
    static uint8_t angle = 0;
    
    if (!cordic.isBusy()) {
        cordic.compute(angle);
        angle += 4;  // Adjust this value to change frequency
    }
    
    cordic.update();
    
    if (!cordic.isBusy()) {
        // Convert 8-bit signed (-128 to 127) to 8-bit unsigned (0-255)
        uint8_t sin_value = cordic.getSin() + 128;
        uint8_t cos_value = cordic.getCos() + 128;
        
        // Write directly to DAC pins using Arduino's dacWrite
        dacWrite(SIN_PIN, sin_value);
        dacWrite(COS_PIN, cos_value);
        
        // Optional: print values to serial for debugging
        Serial.printf("Angle: %3u, Sin: %4d, Cos: %4d\n", angle, cordic.getSin(), cordic.getCos());
    }
    
    // Adjust delay to control frequency
    delayMicroseconds(1000);  // Fine-tune this for desired frequency
}

```
<h3>Results</h3>
<img src="https://github.com/akshaykumartallur/CORDIC_Sine_Cosine_Generator/blob/main/OscilloscopeResults.jpg" alt="OscilloscopeOutput">
</body>
</html>
