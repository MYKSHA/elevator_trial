`timescale 1ns / 1ps

// Multi-car elevator bank (default: 6 lifts x 16 floors).
//
// Hall calls enter a FIFO queue and are assigned to one car by elevator_dispatch.
// In-cabin buttons route directly to that car (no dispatch).
//
// Dispatch priority (lower score wins):
//   Band 0 - idle car already parked at the call floor
//   Band 1 - idle car elsewhere (score grows with distance + pending load)
//   Band 2 - moving car with favorable direction toward the call
//   Band 3 - unfavorable direction / heavily loaded car
//
// Same-floor tie (all idle at floor 0): round-robin rotates priority across cars.
// Different floors: nearest idle car wins; moving cars compete by direction + distance.

module elevator_dispatch #(
    parameter int unsigned NUM_LIFTS   = 6,
    parameter int unsigned NUM_FLOORS  = 16,
    parameter int unsigned FLOOR_W     = (NUM_FLOORS <= 1) ? 1 : $clog2(NUM_FLOORS),
    parameter int unsigned LIFT_W      = (NUM_LIFTS <= 1) ? 1 : $clog2(NUM_LIFTS),
    parameter int unsigned SCORE_W     = 12
) (
    input  logic                         clk,
    input  logic                         reset,

    input  logic [FLOOR_W-1:0]           call_floor,
    input  logic                         call_up,
    input  logic                         call_valid,

    input  logic [FLOOR_W-1:0]           lift_floor   [NUM_LIFTS],
    input  logic                         lift_idle    [NUM_LIFTS],
    input  logic                         lift_moving_up [NUM_LIFTS],
    input  logic                         lift_moving_down [NUM_LIFTS],
    input  logic                         lift_dir_up  [NUM_LIFTS],
    input  logic                         lift_dir_down [NUM_LIFTS],
    input  logic                         lift_estop   [NUM_LIFTS],
    input  logic                         lift_hold_beep [NUM_LIFTS],
    input  logic [NUM_FLOORS-1:0]        lift_pending [NUM_LIFTS],

    output logic [LIFT_W-1:0]            selected_lift,
    output logic                         assign_valid,
    output logic [SCORE_W-1:0]           selected_score,
    output logic [SCORE_W-1:0]           lift_score   [NUM_LIFTS]
);

    localparam logic [SCORE_W-1:0] SCORE_UNAVAILABLE = {SCORE_W{1'b1}};

    logic [LIFT_W-1:0] rr_ptr_q;

    function automatic logic [FLOOR_W-1:0] abs_diff(
        input logic [FLOOR_W-1:0] a,
        input logic [FLOOR_W-1:0] b
    );
        if (a >= b) begin
            return a - b;
        end
        return b - a;
    endfunction

    function automatic logic [FLOOR_W-1:0] count_pending(
        input logic [NUM_FLOORS-1:0] pending
    );
        logic [FLOOR_W-1:0] total;
        total = '0;
        for (int i = 0; i < int'(NUM_FLOORS); i++) begin
            if (pending[i]) begin
                total = total + FLOOR_W'(1);
            end
        end
        return total;
    endfunction

    function automatic logic moving_favorably(
        input logic [FLOOR_W-1:0] lift_f,
        input logic [FLOOR_W-1:0] call_f,
        input logic               call_is_up,
        input logic               moving_up,
        input logic               moving_down,
        input logic               dir_up,
        input logic               dir_down
    );
        if (call_f > lift_f) begin
            return call_is_up && (moving_up || dir_up) && !moving_down;
        end
        if (call_f < lift_f) begin
            return !call_is_up && (moving_down || dir_down) && !moving_up;
        end
        return 1'b1;
    endfunction

    function automatic logic [SCORE_W-1:0] compute_lift_score(
        input int unsigned         lift_idx,
        input logic [FLOOR_W-1:0]  lift_f,
        input logic [FLOOR_W-1:0]  call_f,
        input logic                call_is_up,
        input logic                idle,
        input logic                moving_up,
        input logic                moving_down,
        input logic                dir_up,
        input logic                dir_down,
        input logic                estop,
        input logic                hold_beep,
        input logic [NUM_FLOORS-1:0] pending,
        input logic [LIFT_W-1:0]   rr_ptr
    );
        logic [FLOOR_W-1:0] distance;
        logic [FLOOR_W-1:0] load;
        logic [SCORE_W-1:0] band;
        logic [SCORE_W-1:0] metric;
        logic [LIFT_W-1:0]  rotated_id;
        logic [LIFT_W-1:0]  lift_id;

        lift_id = LIFT_W'(lift_idx);

        if (estop || hold_beep) begin
            return SCORE_UNAVAILABLE;
        end

        distance = abs_diff(lift_f, call_f);
        load     = count_pending(pending);

        if (idle && (lift_f == call_f)) begin
            band   = SCORE_W'(0);
            metric = SCORE_W'(lift_id);
        end else if (idle) begin
            band   = SCORE_W'(1);
            metric = (SCORE_W'(distance) << 3) + (SCORE_W'(load) << 2);
        end else if (moving_favorably(lift_f, call_f, call_is_up, moving_up, moving_down, dir_up, dir_down)) begin
            band   = SCORE_W'(2);
            metric = (SCORE_W'(distance) << 1) + SCORE_W'(load);
        end else begin
            band   = SCORE_W'(3);
            metric = (SCORE_W'(distance) << 3) + (SCORE_W'(load) << 2) + SCORE_W'(10);
        end

        if (lift_id >= rr_ptr) begin
            rotated_id = lift_id - rr_ptr;
        end else begin
            rotated_id = LIFT_W'(NUM_LIFTS) + lift_id - rr_ptr;
        end

        return (band << 8) + (metric << LIFT_W) + SCORE_W'(rotated_id);
    endfunction

    always_comb begin
        logic [SCORE_W-1:0] best_score;
        logic [LIFT_W-1:0]  best_lift;

        best_score = SCORE_UNAVAILABLE;
        best_lift  = '0;

        for (int i = 0; i < int'(NUM_LIFTS); i++) begin
            lift_score[i] = compute_lift_score(
                i,
                lift_floor[i],
                call_floor,
                call_up,
                lift_idle[i],
                lift_moving_up[i],
                lift_moving_down[i],
                lift_dir_up[i],
                lift_dir_down[i],
                lift_estop[i],
                lift_hold_beep[i],
                lift_pending[i],
                rr_ptr_q
            );

            if (lift_score[i] < best_score) begin
                best_score = lift_score[i];
                best_lift  = LIFT_W'(i);
            end
        end

        selected_lift  = best_lift;
        selected_score = best_score;
        assign_valid    = call_valid && (best_score != SCORE_UNAVAILABLE);
    end

    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            rr_ptr_q <= '0;
        end else if (call_valid && assign_valid) begin
            if (rr_ptr_q == LIFT_W'(NUM_LIFTS - 1)) begin
                rr_ptr_q <= '0;
            end else begin
                rr_ptr_q <= rr_ptr_q + LIFT_W'(1);
            end
        end
    end

endmodule


// Top-level: six parallel elevator cars with centralized hall-call dispatch.
module elevator_group #(
    parameter int unsigned NUM_LIFTS          = 6,
    parameter int unsigned NUM_FLOORS         = 16,
    parameter int unsigned DOOR_OPEN_CYCLES   = 10,
    parameter int unsigned HOLD_BEEP_CYCLES     = 16,
    parameter int unsigned ESTOP_BEEP_CYCLES    = 12,
    parameter int unsigned FLOOR_W          = (NUM_FLOORS <= 1) ? 1 : $clog2(NUM_FLOORS),
    parameter int unsigned LIFT_W           = (NUM_LIFTS <= 1) ? 1 : $clog2(NUM_LIFTS),
    parameter int unsigned HALL_QUEUE_DEPTH = 8,
    parameter int unsigned QUEUE_PTR_W      = (HALL_QUEUE_DEPTH <= 1) ? 1 : $clog2(HALL_QUEUE_DEPTH)
) (
    input  logic                          clk,
    input  logic                          reset,

    input  logic                          hall_req_valid,
    input  logic [FLOOR_W-1:0]            hall_req_floor,
    input  logic                          hall_req_up,

    input  logic [NUM_FLOORS-1:0]         hall_up_buttons,
    input  logic [NUM_FLOORS-1:0]         hall_down_buttons,

    input  logic [NUM_FLOORS-1:0]         cabin_buttons [NUM_LIFTS],

    input  logic [NUM_LIFTS-1:0]          emergency_stop,
    input  logic [NUM_LIFTS-1:0]          door_obstructed,

    output logic                          door_open     [NUM_LIFTS],
    output logic                          idle          [NUM_LIFTS],
    output logic                          moving_up     [NUM_LIFTS],
    output logic                          moving_down   [NUM_LIFTS],
    output logic                          service_dir_up [NUM_LIFTS],
    output logic                          service_dir_down [NUM_LIFTS],
    output logic [FLOOR_W-1:0]            current_floor [NUM_LIFTS],
    output logic [NUM_FLOORS-1:0]         pending_requests [NUM_LIFTS],
    output logic [FLOOR_W-1:0]            max_request   [NUM_LIFTS],
    output logic [FLOOR_W-1:0]            min_request   [NUM_LIFTS],
    output logic                          estop_latched [NUM_LIFTS],
    output logic                          hold_beep     [NUM_LIFTS],
    output logic                          estop_beep    [NUM_LIFTS],
    output logic                          door_hold_active [NUM_LIFTS],
    output logic [2:0]                    fsm_state     [NUM_LIFTS],

    output logic                          group_hold_alarm,
    output logic                          group_estop_alarm,

    output logic [LIFT_W-1:0]             last_assigned_lift,
    output logic [FLOOR_W-1:0]            last_assigned_floor,
    output logic                          last_assigned_up,
    output logic                          hall_queue_full,
    output logic [QUEUE_PTR_W:0]          hall_queue_count
);

    localparam int unsigned TOP_FLOOR = NUM_FLOORS - 1;

    typedef enum logic [1:0] {
        DS_IDLE   = 2'd0,
        DS_ASSIGN = 2'd1
    } dispatch_state_e;

    logic [FLOOR_W-1:0] hall_q_floor [HALL_QUEUE_DEPTH];
    logic               hall_q_up    [HALL_QUEUE_DEPTH];

    logic [QUEUE_PTR_W:0] hall_count_q;

    logic [FLOOR_W-1:0] hall_up_latched;
    logic [FLOOR_W-1:0] hall_down_latched;

    dispatch_state_e ds_q;

    logic [LIFT_W-1:0] selected_lift;
    logic              dispatch_valid;
    logic [11:0]       selected_score;

    logic [FLOOR_W-1:0] head_floor;
    logic               head_up;
    logic               head_valid;

    logic [NUM_LIFTS-1:0] lift_req_valid;
    logic [FLOOR_W-1:0]   lift_req_floor [NUM_LIFTS];
    logic [NUM_LIFTS-1:0] lift_req_cabin;
    logic [NUM_LIFTS-1:0] lift_req_hall_up;

    logic [11:0] lift_score [NUM_LIFTS];

    logic top_limit [NUM_LIFTS];
    logic bottom_limit [NUM_LIFTS];
    logic [NUM_FLOORS-1:0] transit_unused [NUM_LIFTS];

    logic [LIFT_W-1:0] last_assigned_lift_q;
    logic [FLOOR_W-1:0] last_assigned_floor_q;
    logic last_assigned_up_q;

    logic hall_up_rise_valid;
    logic hall_down_rise_valid;
    logic [FLOOR_W-1:0] hall_up_rise_floor;
    logic [FLOOR_W-1:0] hall_down_rise_floor;

    logic [FLOOR_W-1:0] hall_up_edge;
    logic [FLOOR_W-1:0] hall_down_edge;

    assign hall_up_edge   = hall_up_buttons & ~hall_up_latched;
    assign hall_down_edge = hall_down_buttons & ~hall_down_latched;

    assign head_valid = (hall_count_q != '0);
    assign head_floor = head_valid ? hall_q_floor[0] : '0;
    assign head_up    = head_valid ? hall_q_up[0] : 1'b0;

    assign last_assigned_lift  = last_assigned_lift_q;
    assign last_assigned_floor = last_assigned_floor_q;
    assign last_assigned_up    = last_assigned_up_q;
    assign hall_queue_full     = (hall_count_q >= HALL_QUEUE_DEPTH);
    assign hall_queue_count    = hall_count_q;

    always_comb begin
        group_hold_alarm  = 1'b0;
        group_estop_alarm = 1'b0;
        for (int i = 0; i < int'(NUM_LIFTS); i++) begin
            group_hold_alarm  = group_hold_alarm  | hold_beep[i];
            group_estop_alarm = group_estop_alarm | estop_beep[i];
        end
    end

    always_comb begin
        hall_up_rise_valid   = 1'b0;
        hall_up_rise_floor   = '0;
        hall_down_rise_valid = 1'b0;
        hall_down_rise_floor = '0;

        for (int i = int'(TOP_FLOOR); i >= 0; i--) begin
            if (!hall_up_rise_valid && hall_up_edge[i]) begin
                hall_up_rise_valid = 1'b1;
                hall_up_rise_floor = FLOOR_W'(i);
            end
        end

        for (int i = 0; i < int'(NUM_FLOORS); i++) begin
            if (!hall_down_rise_valid && hall_down_edge[i]) begin
                hall_down_rise_valid = 1'b1;
                hall_down_rise_floor = FLOOR_W'(i);
            end
        end
    end

    elevator_dispatch #(
        .NUM_LIFTS(NUM_LIFTS),
        .NUM_FLOORS(NUM_FLOORS),
        .FLOOR_W(FLOOR_W),
        .LIFT_W(LIFT_W)
    ) u_dispatch (
        .clk(clk),
        .reset(reset),
        .call_floor(head_floor),
        .call_up(head_up),
        .call_valid(ds_q == DS_ASSIGN),
        .lift_floor(current_floor),
        .lift_idle(idle),
        .lift_moving_up(moving_up),
        .lift_moving_down(moving_down),
        .lift_dir_up(service_dir_up),
        .lift_dir_down(service_dir_down),
        .lift_estop(estop_latched),
        .lift_hold_beep(hold_beep),
        .lift_pending(pending_requests),
        .selected_lift(selected_lift),
        .assign_valid(dispatch_valid),
        .selected_score(selected_score),
        .lift_score(lift_score)
    );

    always_comb begin
        lift_req_valid   = {NUM_LIFTS{1'b0}};
        lift_req_cabin   = {NUM_LIFTS{1'b0}};
        lift_req_hall_up = {NUM_LIFTS{1'b0}};

        for (int i = 0; i < int'(NUM_LIFTS); i++) begin
            lift_req_floor[i] = '0;
        end

        if ((ds_q == DS_ASSIGN) && dispatch_valid) begin
            lift_req_valid[selected_lift]   = 1'b1;
            lift_req_floor[selected_lift]   = head_floor;
            lift_req_cabin[selected_lift]   = 1'b0;
            lift_req_hall_up[selected_lift] = head_up;
        end
    end

    genvar gi;
    generate
        for (gi = 0; gi < NUM_LIFTS; gi++) begin : gen_lifts
            assign top_limit[gi]    = (current_floor[gi] == FLOOR_W'(TOP_FLOOR));
            assign bottom_limit[gi] = (current_floor[gi] == FLOOR_W'(0));

            elevator_controller #(
                .NUM_FLOORS(NUM_FLOORS),
                .DOOR_OPEN_CYCLES(DOOR_OPEN_CYCLES),
                .HOLD_BEEP_CYCLES(HOLD_BEEP_CYCLES),
                .ESTOP_BEEP_CYCLES(ESTOP_BEEP_CYCLES),
                .FLOOR_W(FLOOR_W)
            ) u_lift (
                .clk(clk),
                .reset(reset),
                .req_valid(lift_req_valid[gi]),
                .req_floor(lift_req_floor[gi]),
                .req_cabin(lift_req_cabin[gi]),
                .req_hall_up(lift_req_hall_up[gi]),
                .cabin_buttons(cabin_buttons[gi]),
                .hall_up_buttons('0),
                .hall_down_buttons('0),
                .emergency_stop(emergency_stop[gi]),
                .top_limit(top_limit[gi]),
                .bottom_limit(bottom_limit[gi]),
                .door_obstructed(door_obstructed[gi]),
                .door_open(door_open[gi]),
                .idle(idle[gi]),
                .moving_up(moving_up[gi]),
                .moving_down(moving_down[gi]),
                .service_dir_up(service_dir_up[gi]),
                .service_dir_down(service_dir_down[gi]),
                .current_floor(current_floor[gi]),
                .pending_requests(pending_requests[gi]),
                .transit_hall_pending(transit_unused[gi]),
                .max_request(max_request[gi]),
                .min_request(min_request[gi]),
                .estop_latched(estop_latched[gi]),
                .hold_beep(hold_beep[gi]),
                .estop_beep(estop_beep[gi]),
                .door_hold_active(door_hold_active[gi]),
                .fsm_state(fsm_state[gi])
            );
        end
    endgenerate

    always_ff @(posedge clk or posedge reset) begin
        integer i;
        integer j;
        logic [QUEUE_PTR_W:0] count_next;
        logic [FLOOR_W-1:0]   floor_next [HALL_QUEUE_DEPTH];
        logic                 up_next    [HALL_QUEUE_DEPTH];
        logic duplicate;

        if (reset) begin
            hall_count_q          <= '0;
            hall_up_latched       <= '0;
            hall_down_latched     <= '0;
            ds_q                  <= DS_IDLE;
            last_assigned_lift_q  <= '0;
            last_assigned_floor_q <= '0;
            last_assigned_up_q    <= 1'b0;

            for (i = 0; i < HALL_QUEUE_DEPTH; i++) begin
                hall_q_floor[i] <= '0;
                hall_q_up[i]    <= 1'b0;
            end
        end else begin
            hall_up_latched   <= hall_up_buttons | hall_up_latched;
            hall_down_latched <= hall_down_buttons | hall_down_latched;

            count_next = hall_count_q;
            for (i = 0; i < HALL_QUEUE_DEPTH; i++) begin
                floor_next[i] = hall_q_floor[i];
                up_next[i]    = hall_q_up[i];
            end

            duplicate = 1'b0;
            for (j = 0; j < HALL_QUEUE_DEPTH; j++) begin
                if (j < count_next) begin
                    if ((floor_next[j] == hall_req_floor) && (up_next[j] == hall_req_up)) begin
                        duplicate = 1'b1;
                    end
                end
            end
            if (hall_req_valid && (count_next < HALL_QUEUE_DEPTH) && !duplicate) begin
                floor_next[count_next] = hall_req_floor;
                up_next[count_next]    = hall_req_up;
                count_next             = count_next + 1'b1;
            end

            duplicate = 1'b0;
            for (j = 0; j < HALL_QUEUE_DEPTH; j++) begin
                if (j < count_next) begin
                    if ((floor_next[j] == hall_up_rise_floor) && up_next[j]) begin
                        duplicate = 1'b1;
                    end
                end
            end
            if (hall_up_rise_valid && (count_next < HALL_QUEUE_DEPTH) && !duplicate) begin
                floor_next[count_next] = hall_up_rise_floor;
                up_next[count_next]    = 1'b1;
                count_next             = count_next + 1'b1;
            end

            duplicate = 1'b0;
            for (j = 0; j < HALL_QUEUE_DEPTH; j++) begin
                if (j < count_next) begin
                    if ((floor_next[j] == hall_down_rise_floor) && !up_next[j]) begin
                        duplicate = 1'b1;
                    end
                end
            end
            if (hall_down_rise_valid && (count_next < HALL_QUEUE_DEPTH) && !duplicate) begin
                floor_next[count_next] = hall_down_rise_floor;
                up_next[count_next]    = 1'b0;
                count_next             = count_next + 1'b1;
            end

            case (ds_q)
                DS_IDLE: begin
                    if (count_next != '0) begin
                        ds_q <= DS_ASSIGN;
                    end
                end

                DS_ASSIGN: begin
                    if (dispatch_valid) begin
                        last_assigned_lift_q  <= selected_lift;
                        last_assigned_floor_q <= head_floor;
                        last_assigned_up_q    <= head_up;

                        for (i = 0; i < HALL_QUEUE_DEPTH - 1; i++) begin
                            floor_next[i] = floor_next[i + 1];
                            up_next[i]    = up_next[i + 1];
                        end
                        floor_next[HALL_QUEUE_DEPTH - 1] = '0;
                        up_next[HALL_QUEUE_DEPTH - 1]    = 1'b0;
                        count_next                             = count_next - 1'b1;

                        if (count_next != '0) begin
                            ds_q <= DS_ASSIGN;
                        end else begin
                            ds_q <= DS_IDLE;
                        end
                    end
                end

                default: ds_q <= DS_IDLE;
            endcase

            hall_count_q <= count_next;
            for (i = 0; i < HALL_QUEUE_DEPTH; i++) begin
                hall_q_floor[i] <= floor_next[i];
                hall_q_up[i]    <= up_next[i];
            end
        end
    end

endmodule
