module forklift_controller (
    input  wire clk,        // 27 MHz Tang Nano 20K onboard clock
    input  wire rst_n,      // Active-low reset

    // SPI interface
    input  wire sck,        // SPI clock from Jetson
    input  wire mosi,       // Master Out Slave In
    output wire miso,       // Master In Slave Out
    input  wire cs_n,       // Chip Select, active low

    // L298N motor driver outputs
    output wire pwm_out,    // L298N ENA
    output reg  dir_out,    // L298N IN1
    output reg  brake_out,  // L298N IN2

    // Encoder inputs
    input wire enc_a,
    input wire enc_b,

    // IR limit sensors
    input wire lower_ir,    // Bottom sensor: 1 free, 0 detected
    input wire upper_ir,    // Top sensor: 1 free, 0 detected

    // Debug LEDs
    output wire [5:0] leds
);

    // ============================================================
    // Command definitions
    // ============================================================

    localparam CMD_STOP          = 8'h00;
    localparam CMD_LOWER         = 8'h01;
    localparam CMD_LIFT          = 8'h02;  // Manual lift until upper limit
    localparam CMD_HOLD          = 8'h03;
    localparam CMD_RESET_ENCODER = 8'h04;
    localparam CMD_CONVEYOR      = 8'h05;  // Lift 2.5 cm
    localparam CMD_RACK          = 8'h06;  // Lift 1.0 cm
    localparam CMD_STATUS        = 8'h10;

    // ============================================================
    // Status definitions
    // ============================================================

    localparam STATUS_IDLE        = 8'h00;
    localparam STATUS_MOVING_DOWN = 8'h01;
    localparam STATUS_MOVING_UP   = 8'h02;
    localparam STATUS_HOLDING     = 8'h03;
    localparam STATUS_LOW_LIMIT   = 8'h04;
    localparam STATUS_UP_LIMIT    = 8'h05;
    localparam STATUS_RACK_UP     = 8'h06;
    localparam STATUS_CONVEYOR_UP = 8'h07;
    localparam STATUS_ERROR       = 8'hE0;

    // ============================================================
    // FSM states
    // ============================================================

    localparam ST_IDLE          = 4'd0;
    localparam ST_LIFTING       = 4'd1;
    localparam ST_LOWERING      = 4'd2;
    localparam ST_HOLDING       = 4'd3;
    localparam ST_UP_LIMIT      = 4'd4;
    localparam ST_LOW_LIMIT     = 4'd5;
    localparam ST_ERROR         = 4'd6;
    localparam ST_LIFT_RACK     = 4'd7;  // Relative lift: 1 cm
    localparam ST_LIFT_CONVEYOR = 4'd8;  // Relative lift: 2.5 cm

    reg [3:0] state = ST_IDLE;

    // ============================================================
    // Position parameters
    // ============================================================
    // Your measured data:
    // - lifting makes encoder_count decrease
    // - 1 cm  = 100 ticks
    // - 2.5cm = 250 ticks
    //
    // Therefore, target for upward motion is:
    // target_count = encoder_count - target_ticks
    // ============================================================

    localparam signed [31:0] COUNTS_RACK_1CM       = 32'sd65;
    localparam signed [31:0] COUNTS_CONVEYOR_2_5CM = 32'sd180;
    localparam signed [31:0] POSITION_TOLERANCE    = 32'sd3;

    reg signed [31:0] target_count = 32'sd0;

    // ============================================================
    // LEDs
    // ============================================================
    // Tang Nano onboard LEDs are active-low:
    // leds_state bit = 1 means "turn this LED on" logically.
    // Physical LED output must be inverted.
    // ============================================================

    reg [5:0] leds_state = 6'b000001;
    assign leds = ~leds_state;

    // ============================================================
    // IR limit sensor logic
    // ============================================================
    // Confirmed sensors:
    // free = 1, object/limit detected = 0
    // ============================================================

    wire lower_limit_reached;
    wire upper_limit_reached;

    assign lower_limit_reached = ~lower_ir;
    assign upper_limit_reached = ~upper_ir;

    // ============================================================
    // SPI slave logic
    // Mode 0:
    // - MOSI sampled on rising edge of SCK
    // - MISO updated on falling edge of SCK
    // ============================================================

    reg [7:0] current_cmd = CMD_STOP;
    reg [7:0] last_cmd_seen = CMD_STOP;

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
    reg cmd_new = 1'b0;

    assign miso = cs_active ? miso_reg : 1'bz;

    // Status is generated from state so MISO is not stuck at old status_byte.
    reg [7:0] status_now;

    always @(*) begin
        case (state)
            ST_IDLE:          status_now = STATUS_IDLE;
            ST_LIFTING:       status_now = STATUS_MOVING_UP;
            ST_LOWERING:      status_now = STATUS_MOVING_DOWN;
            ST_HOLDING:       status_now = STATUS_HOLDING;
            ST_UP_LIMIT:      status_now = STATUS_UP_LIMIT;
            ST_LOW_LIMIT:     status_now = STATUS_LOW_LIMIT;
            ST_LIFT_RACK:     status_now = STATUS_RACK_UP;
            ST_LIFT_CONVEYOR: status_now = STATUS_CONVEYOR_UP;
            ST_ERROR:         status_now = STATUS_ERROR;
            default:          status_now = STATUS_ERROR;
        endcase
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            bit_cnt       <= 3'd0;
            rx_shift      <= 8'd0;
            tx_shift      <= 8'd0;
            miso_reg      <= 1'b0;
            cmd_received  <= 1'b0;
            cmd_new       <= 1'b0;
            current_cmd   <= CMD_STOP;
            last_cmd_seen <= CMD_STOP;
        end else begin
            cmd_received <= 1'b0;
            cmd_new      <= 1'b0;

            if (cs_falling) begin
                bit_cnt  <= 3'd0;
                tx_shift <= status_now;
                miso_reg <= status_now[7];
            end

            if (cs_active) begin
                if (sck_rising) begin
                    rx_shift <= {rx_shift[6:0], mosi_bit};

                    if (bit_cnt == 3'd7) begin
                        current_cmd  <= {rx_shift[6:0], mosi_bit};
                        cmd_received <= 1'b1;

                        if ({rx_shift[6:0], mosi_bit} != last_cmd_seen) begin
                            cmd_new <= 1'b1;
                        end

                        last_cmd_seen <= {rx_shift[6:0], mosi_bit};
                    end

                    bit_cnt <= bit_cnt + 3'd1;
                end

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

    localparam integer PWM_PERIOD = 1350;  // 20 kHz @ 27 MHz
    localparam integer DUTY_STOP  = 0;
    localparam integer DUTY_RUN   = 1050;

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
            end else if (state == ST_LOW_LIMIT) begin
                // Bottom reference. Keeps encoder zeroed while lower limit is detected.
                encoder_count <= 32'sd0;
            end else begin
                // Count on rising edge of encoder A.
                // Keep this exactly as tested unless count direction changes.
                if (!enc_a_d && enc_a) begin
                    if (enc_b)
                        encoder_count <= encoder_count + 1;
                    else
                        encoder_count <= encoder_count - 1;
                end
            end
        end
    end

    // ============================================================
    // Helper condition for upward relative moves
    // Since upward motion gives negative ticks:
    // Done when encoder_count <= target_count + tolerance
    // ============================================================

    wire relative_lift_target_reached;
    assign relative_lift_target_reached = (encoder_count <= (target_count + POSITION_TOLERANCE));

    // ============================================================
    // Main forklift FSM
    // ============================================================
    //
    // IMPORTANT motor direction correction for your wiring:
    // You observed:
    //   w/lift  made forklift go DOWN
    //   s/lower made forklift go UP
    //
    // Therefore L298N outputs were swapped from the previous version:
    //
    // LIFTING / RACK / CONVEYOR:
    //   IN1 = 0, IN2 = 1
    //
    // LOWERING:
    //   IN1 = 1, IN2 = 0
    // ============================================================

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= ST_IDLE;
            duty_cycle   <= DUTY_STOP;
            dir_out      <= 1'b0;
            brake_out    <= 1'b0;
            leds_state   <= 6'b000001;
            target_count <= 32'sd0;
        end else begin

            case (state)

                ST_IDLE: begin
                    duty_cycle <= DUTY_STOP;
                    dir_out    <= 1'b0;
                    brake_out  <= 1'b0;
                    leds_state <= 6'b000001;

                    if (cmd_received) begin
                        if (current_cmd == CMD_LIFT) begin
                            if (!upper_limit_reached)
                                state <= ST_LIFTING;
                            else
                                state <= ST_UP_LIMIT;

                        end else if (current_cmd == CMD_LOWER) begin
                            if (!lower_limit_reached)
                                state <= ST_LOWERING;
                            else
                                state <= ST_LOW_LIMIT;

                        end else if (cmd_new && current_cmd == CMD_RACK) begin
                            if (!upper_limit_reached) begin
                                target_count <= encoder_count - COUNTS_RACK_1CM;
                                state <= ST_LIFT_RACK;
                            end else begin
                                state <= ST_UP_LIMIT;
                            end

                        end else if (cmd_new && current_cmd == CMD_CONVEYOR) begin
                            if (!upper_limit_reached) begin
                                target_count <= encoder_count - COUNTS_CONVEYOR_2_5CM;
                                state <= ST_LIFT_CONVEYOR;
                            end else begin
                                state <= ST_UP_LIMIT;
                            end

                        end else if (current_cmd == CMD_HOLD) begin
                            state <= ST_HOLDING;

                        end else if (current_cmd == CMD_STOP) begin
                            state <= ST_IDLE;
                        end
                    end
                end

                ST_LIFTING: begin
                    // Manual lift until upper limit
                    dir_out    <= 1'b0;       // L298N IN1
                    brake_out  <= 1'b1;       // L298N IN2
                    duty_cycle <= DUTY_RUN;
                    leds_state <= 6'b000010;

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

                ST_LIFT_RACK: begin
                    // Relative lift of 1 cm
                    dir_out    <= 1'b0;
                    brake_out  <= 1'b1;
                    duty_cycle <= DUTY_RUN;
                    leds_state <= 6'b000010;  // Same LED as lift while moving up

                    if (upper_limit_reached) begin
                        state <= ST_UP_LIMIT;
                    end else if (relative_lift_target_reached) begin
                        state <= ST_HOLDING;
                    end else if (cmd_received && current_cmd == CMD_STOP) begin
                        state <= ST_IDLE;
                    end else if (cmd_received && current_cmd == CMD_HOLD) begin
                        state <= ST_HOLDING;
                    end else if (cmd_received && current_cmd == CMD_LOWER) begin
                        state <= ST_LOWERING;
                    end
                end

                ST_LIFT_CONVEYOR: begin
                    // Relative lift of 2.5 cm
                    dir_out    <= 1'b0;
                    brake_out  <= 1'b1;
                    duty_cycle <= DUTY_RUN;
                    leds_state <= 6'b000010;  // Same LED as lift while moving up

                    if (upper_limit_reached) begin
                        state <= ST_UP_LIMIT;
                    end else if (relative_lift_target_reached) begin
                        state <= ST_HOLDING;
                    end else if (cmd_received && current_cmd == CMD_STOP) begin
                        state <= ST_IDLE;
                    end else if (cmd_received && current_cmd == CMD_HOLD) begin
                        state <= ST_HOLDING;
                    end else if (cmd_received && current_cmd == CMD_LOWER) begin
                        state <= ST_LOWERING;
                    end
                end

                ST_LOWERING: begin
                    dir_out    <= 1'b1;       // L298N IN1
                    brake_out  <= 1'b0;       // L298N IN2
                    duty_cycle <= DUTY_RUN;
                    leds_state <= 6'b000100;

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

                ST_HOLDING: begin
                    duty_cycle <= DUTY_STOP;
                    dir_out    <= 1'b0;
                    brake_out  <= 1'b0;
                    leds_state <= 6'b001000;

                    if (cmd_received) begin
                        if (current_cmd == CMD_LIFT) begin
                            if (!upper_limit_reached)
                                state <= ST_LIFTING;
                            else
                                state <= ST_UP_LIMIT;

                        end else if (current_cmd == CMD_LOWER) begin
                            if (!lower_limit_reached)
                                state <= ST_LOWERING;
                            else
                                state <= ST_LOW_LIMIT;

                        end else if (cmd_new && current_cmd == CMD_RACK) begin
                            if (!upper_limit_reached) begin
                                target_count <= encoder_count - COUNTS_RACK_1CM;
                                state <= ST_LIFT_RACK;
                            end else begin
                                state <= ST_UP_LIMIT;
                            end

                        end else if (cmd_new && current_cmd == CMD_CONVEYOR) begin
                            if (!upper_limit_reached) begin
                                target_count <= encoder_count - COUNTS_CONVEYOR_2_5CM;
                                state <= ST_LIFT_CONVEYOR;
                            end else begin
                                state <= ST_UP_LIMIT;
                            end

                        end else if (current_cmd == CMD_STOP) begin
                            state <= ST_IDLE;
                        end
                    end
                end

                ST_UP_LIMIT: begin
                    duty_cycle <= DUTY_STOP;
                    dir_out    <= 1'b0;
                    brake_out  <= 1'b0;
                    leds_state <= 6'b010000;

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

                ST_LOW_LIMIT: begin
                    duty_cycle <= DUTY_STOP;
                    dir_out    <= 1'b0;
                    brake_out  <= 1'b0;
                    leds_state <= 6'b100000;

                    if (cmd_received) begin
                        if (current_cmd == CMD_LIFT && !upper_limit_reached) begin
                            state <= ST_LIFTING;

                        end else if (cmd_new && current_cmd == CMD_RACK && !upper_limit_reached) begin
                            target_count <= encoder_count - COUNTS_RACK_1CM;
                            state <= ST_LIFT_RACK;

                        end else if (cmd_new && current_cmd == CMD_CONVEYOR && !upper_limit_reached) begin
                            target_count <= encoder_count - COUNTS_CONVEYOR_2_5CM;
                            state <= ST_LIFT_CONVEYOR;

                        end else if (current_cmd == CMD_STOP) begin
                            state <= ST_IDLE;
                        end else if (current_cmd == CMD_HOLD) begin
                            state <= ST_HOLDING;
                        end
                    end
                end

                ST_ERROR: begin
                    duty_cycle <= DUTY_STOP;
                    dir_out    <= 1'b0;
                    brake_out  <= 1'b0;
                    leds_state <= 6'b111111;

                    if (cmd_received && current_cmd == CMD_STOP) begin
                        state <= ST_IDLE;
                    end
                end

                default: begin
                    state      <= ST_ERROR;
                    duty_cycle <= DUTY_STOP;
                    dir_out    <= 1'b0;
                    brake_out  <= 1'b0;
                    leds_state <= 6'b111111;
                end

            endcase
        end
    end

endmodule