module forklift_controller (
    input  wire clk,        // 27 MHz Tang Nano 20K onboard clock
    input  wire rst_n,      // Active-low reset

    // SPI interface
    input  wire sck,        // SPI clock from Jetson
    input  wire mosi,       // Master Out Slave In
    output wire miso,       // Master In Slave Out
    input  wire cs_n,       // Chip Select, active low

    // Motor driver outputs
    output wire pwm_out,    // PWM signal to motor driver
    output reg  dir_out,    // Direction signal
    output reg  brake_out,  // Brake / enable / complementary signal

    // Encoder inputs
    input wire enc_a,
    input wire enc_b,

    // IR limit sensors
    input wire lower_ir,    // Bottom sensor
    input wire upper_ir,    // Top sensor

    // Debug LEDs
    output wire [5:0] leds
);

    // ============================================================
    // Command definitions
    // Commands received from Jetson through SPI
    // ============================================================

    localparam CMD_STOP          = 8'h00;
    localparam CMD_LOWER         = 8'h01;
    localparam CMD_LIFT          = 8'h02;
    localparam CMD_HOLD          = 8'h03;
    localparam CMD_RESET_ENCODER = 8'h04;
    localparam CMD_STATUS        = 8'h10;

    // ============================================================
    // Status definitions
    // Status byte returned to Jetson through SPI
    // ============================================================

    localparam STATUS_IDLE        = 8'h00;
    localparam STATUS_MOVING_DOWN = 8'h01;
    localparam STATUS_MOVING_UP   = 8'h02;
    localparam STATUS_HOLDING     = 8'h03;
    localparam STATUS_LOW_LIMIT   = 8'h04;
    localparam STATUS_UP_LIMIT    = 8'h05;
    localparam STATUS_ERROR       = 8'hE0;

    // ============================================================
    // FSM states
    // ============================================================

    localparam ST_IDLE      = 3'd0;
    localparam ST_LIFTING   = 3'd1;
    localparam ST_LOWERING  = 3'd2;
    localparam ST_HOLDING   = 3'd3;
    localparam ST_UP_LIMIT  = 3'd4;
    localparam ST_LOW_LIMIT = 3'd5;
    localparam ST_ERROR     = 3'd6;

    reg [2:0] state = ST_IDLE;
    reg [5:0] leds_state;

    assign leds = ~leds_state;
    reg [7:0] current_cmd = CMD_STOP;
    reg [7:0] status_byte = STATUS_IDLE;

    // ============================================================
    // IR limit sensor logic
    // ============================================================
    // Most IR sensors are active-low:
    // No object detected = 1
    // Object detected    = 0
    //
    // If your IR sensors are active-high, change these two assigns:
    // assign lower_limit_reached = lower_ir;
    // assign upper_limit_reached = upper_ir;
    // ============================================================

    wire lower_limit_reached;
    wire upper_limit_reached;

    assign lower_limit_reached = ~lower_ir;
    assign upper_limit_reached = ~upper_ir;

    // ============================================================
    // SPI slave logic
    // Mode 0:
    // - Data sampled on rising edge of SCK
    // - Data updated on falling edge of SCK
    //
    // This follows the same idea as the previous SPI practice:
    // Jetson sends one byte, FPGA receives command and returns status.
    // ============================================================

    reg [1:0] sck_sync  = 2'b00;
    reg [2:0] cs_sync   = 3'b111;
    reg [1:0] mosi_sync = 2'b00;

    always @(posedge clk) begin
        sck_sync  <= {sck_sync[0], sck};
        cs_sync   <= {cs_sync[1:0], cs_n};
        mosi_sync <= {mosi_sync[0], mosi};
    end

    wire sck_rising  = (sck_sync == 2'b01);
    wire sck_falling = (sck_sync == 2'b10);
    wire cs_falling  = (cs_sync[2:1] == 2'b10);
    wire cs_active   = ~cs_sync[1];
    wire mosi_bit    = mosi_sync[1];

    reg [2:0] bit_cnt = 3'd0;
    reg [7:0] rx_shift = 8'd0;
    reg [7:0] tx_shift = 8'd0;
    reg miso_reg = 1'b0;
    reg cmd_received = 1'b0;

    assign miso = cs_active ? miso_reg : 1'bz;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            bit_cnt      <= 3'd0;
            rx_shift     <= 8'd0;
            tx_shift     <= 8'd0;
            miso_reg     <= 1'b0;
            cmd_received <= 1'b0;
            current_cmd  <= CMD_STOP;
        end else begin
            cmd_received <= 1'b0;

            // Start of SPI transaction
            if (cs_falling) begin
                bit_cnt  <= 3'd0;
                tx_shift <= status_byte;
                miso_reg <= status_byte[7];
            end

            if (cs_active) begin

                // Read MOSI on rising edge
                if (sck_rising) begin
                    rx_shift <= {rx_shift[6:0], mosi_bit};

                    if (bit_cnt == 3'd7) begin
                        current_cmd  <= {rx_shift[6:0], mosi_bit};
                        cmd_received <= 1'b1;
                    end

                    bit_cnt <= bit_cnt + 3'd1;
                end

                // Update MISO on falling edge
                if (sck_falling) begin
                    miso_reg <= tx_shift[6];
                    tx_shift <= {tx_shift[6:0], 1'b0};
                end
            end
        end
    end

    // ============================================================
    // PWM generator for DC motor
    // ============================================================
    // Tang Nano clock = 27 MHz
    //
    // PWM frequency target approximately 20 kHz:
    // 27,000,000 / 20,000 = 1350 counts
    //
    // duty_cycle controls motor speed.
    // DUTY_RUN can be adjusted after real motor testing.
    // ============================================================

    localparam integer PWM_PERIOD = 1350;
    localparam integer DUTY_STOP  = 0;
    localparam integer DUTY_RUN   = 1050;   // 850 = Around 63% duty cycle

    reg [10:0] pwm_counter = 11'd0;
    reg [10:0] duty_cycle = DUTY_STOP;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pwm_counter <= 11'd0;
        end else begin
            if (pwm_counter < PWM_PERIOD - 1)
                pwm_counter <= pwm_counter + 1'b1;
            else
                pwm_counter <= 11'd0;
        end
    end

    assign pwm_out = (pwm_counter < duty_cycle) ? 1'b1 : 1'b0;

    // ============================================================
    // Encoder counter
    // ============================================================
    // Basic quadrature reading:
    // Counts on rising edge of encoder A.
    // Direction is determined by encoder B.
    //
    // For now, encoder_count is internal.
    // Later we can send it back to ROS2 using multiple SPI bytes.
    // ============================================================

    reg enc_a_d = 1'b0;
    reg enc_b_d = 1'b0;
    reg signed [31:0] encoder_count = 32'sd0;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            enc_a_d <= 1'b0;
            enc_b_d <= 1'b0;
            encoder_count <= 32'sd0;
        end else begin
            enc_a_d <= enc_a;
            enc_b_d <= enc_b;

            if (cmd_received && current_cmd == CMD_RESET_ENCODER) begin
                encoder_count <= 32'sd0;
            end else begin
                if (!enc_a_d && enc_a) begin
                    if (enc_b)
                        encoder_count <= encoder_count - 1;
                    else
                        encoder_count <= encoder_count + 1;
                end
            end
        end
    end

    // ============================================================
    // Main forklift FSM
    // ============================================================
    // Motor behavior:
    //
    // ST_IDLE:
    //   Motor stopped, waiting for command.
    //
    // ST_LIFTING:
    //   Motor moves upward until upper IR detects limit.
    //
    // ST_LOWERING:
    //   Motor moves downward until lower IR detects limit.
    //
    // ST_HOLDING:
    //   Motor stopped. Useful if the mechanism mechanically holds position.
    //
    // ST_UP_LIMIT:
    //   Top reached. It will not continue lifting.
    //
    // ST_LOW_LIMIT:
    //   Bottom reached. It will not continue lowering.
    // ============================================================

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state       <= ST_IDLE;
            duty_cycle  <= DUTY_STOP;
            dir_out     <= 1'b0;
            brake_out   <= 1'b1;
            status_byte <= STATUS_IDLE;
            leds_state  <= 6'b000001;
        end else begin

            case (state)

                // ------------------------------------------------
                // IDLE
                // ------------------------------------------------
                ST_IDLE: begin
                    duty_cycle  <= DUTY_STOP;
                    dir_out     <= 1'b0;
                    brake_out   <= 1'b0;
                    status_byte <= STATUS_IDLE;
                    leds_state  <= 6'b000001;

                    if (cmd_received) begin
                        if (current_cmd == CMD_LIFT && !upper_limit_reached) begin
                            state <= ST_LIFTING;
                        end else if (current_cmd == CMD_LOWER && !lower_limit_reached) begin
                            state <= ST_LOWERING;
                        end else if (current_cmd == CMD_HOLD) begin
                            state <= ST_HOLDING;
                        end else if (current_cmd == CMD_STOP) begin
                            state <= ST_IDLE;
                        end
                    end
                end

                // ------------------------------------------------
                // LIFTING
                // ------------------------------------------------
                ST_LIFTING: begin
                    dir_out     <= 1'b1;       // L298N IN1
                    brake_out   <= 1'b0;       // L298N IN2
                    duty_cycle  <= DUTY_RUN;   // L298N ENA receives PWM
                    status_byte <= STATUS_MOVING_UP;
                    leds_state  <= 6'b000010;

                    if (upper_limit_reached) begin
                        state <= ST_UP_LIMIT;
                    end else if (cmd_received && current_cmd == CMD_STOP) begin
                        state <= ST_IDLE;
                    end else if (cmd_received && current_cmd == CMD_HOLD) begin
                        state <= ST_HOLDING;
                    end else if (cmd_received && current_cmd == CMD_LOWER) begin
                        state <= ST_LOWERING;
                    end
                end

                // ------------------------------------------------
                // LOWERING
                // ------------------------------------------------
                ST_LOWERING: begin
                    dir_out     <= 1'b0;       // L298N IN1
                    brake_out   <= 1'b1;       // L298N IN2
                    duty_cycle  <= DUTY_RUN;   // L298N ENA receives PWM
                    status_byte <= STATUS_MOVING_DOWN;
                    leds_state  <= 6'b000100;

                    if (lower_limit_reached) begin
                        state <= ST_LOW_LIMIT;
                    end else if (cmd_received && current_cmd == CMD_STOP) begin
                        state <= ST_IDLE;
                    end else if (cmd_received && current_cmd == CMD_HOLD) begin
                        state <= ST_HOLDING;
                    end else if (cmd_received && current_cmd == CMD_LIFT) begin
                        state <= ST_LIFTING;
                    end
                end

                // ------------------------------------------------
                // HOLDING
                // ------------------------------------------------
                ST_HOLDING: begin
                    duty_cycle  <= DUTY_STOP;
                    dir_out     <= 1'b0;
                    brake_out   <= 1'b0;
                    status_byte <= STATUS_HOLDING;
                    leds_state  <= 6'b001000;

                    if (cmd_received) begin
                        if (current_cmd == CMD_LIFT && !upper_limit_reached) begin
                            state <= ST_LIFTING;
                        end else if (current_cmd == CMD_LOWER && !lower_limit_reached) begin
                            state <= ST_LOWERING;
                        end else if (current_cmd == CMD_STOP) begin
                            state <= ST_IDLE;
                        end
                    end
                end

                // ------------------------------------------------
                // UPPER LIMIT REACHED
                // ------------------------------------------------
                ST_UP_LIMIT: begin
                    duty_cycle  <= DUTY_STOP;
                    dir_out     <= 1'b0;
                    brake_out   <= 1'b0;
                    status_byte <= STATUS_UP_LIMIT;
                    leds_state  <= 6'b010000;

                    if (cmd_received) begin
                        if (current_cmd == CMD_LOWER && !lower_limit_reached) begin
                            state <= ST_LOWERING;
                        end else if (current_cmd == CMD_STOP) begin
                            state <= ST_IDLE;
                        end else if (current_cmd == CMD_HOLD) begin
                            state <= ST_HOLDING;
                        end
                    end
                end

                // ------------------------------------------------
                // LOWER LIMIT REACHED
                // ------------------------------------------------
                ST_LOW_LIMIT: begin
                    duty_cycle  <= DUTY_STOP;
                    dir_out     <= 1'b0;
                    brake_out   <= 1'b0;
                    status_byte <= STATUS_LOW_LIMIT;
                    leds_state  <= 6'b100000;

                    if (cmd_received) begin
                        if (current_cmd == CMD_LIFT && !upper_limit_reached) begin
                            state <= ST_LIFTING;
                        end else if (current_cmd == CMD_STOP) begin
                            state <= ST_IDLE;
                        end else if (current_cmd == CMD_HOLD) begin
                            state <= ST_HOLDING;
                        end
                    end
                end

                // ------------------------------------------------
                // ERROR
                // ------------------------------------------------
                ST_ERROR: begin
                    duty_cycle  <= DUTY_STOP;
                    dir_out     <= 1'b0;
                    brake_out   <= 1'b0;
                    status_byte <= STATUS_ERROR;
                    leds_state  <= 6'b111111;

                    if (cmd_received && current_cmd == CMD_STOP) begin
                        state <= ST_IDLE;
                    end
                end

                // ------------------------------------------------
                // DEFAULT SAFETY
                // ------------------------------------------------
                default: begin
                    state       <= ST_ERROR;
                    duty_cycle  <= DUTY_STOP;
                    brake_out   <= 1'b1;
                    status_byte <= STATUS_ERROR;
                    leds_state  <= 6'b111111;
                end

            endcase
        end
    end

endmodule