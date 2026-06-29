`timescale 1ns / 1ps

// Single-car elevator controller with:
//   - Direction-persistent sweep scheduling (max/min bounds)
//   - Transit hall-call buffer (hall calls registered while moving)
//   - Door timer, obstruction extension, intentional-hold beep, estop beep
//   - Emergency stop with state/direction save-restore

module elevator_controller #(
    parameter int unsigned NUM_FLOORS         = 16,
    parameter int unsigned DOOR_OPEN_CYCLES     = 10,
    parameter int unsigned HOLD_BEEP_CYCLES     = 16,
    parameter int unsigned ESTOP_BEEP_CYCLES    = 12,
    parameter int unsigned MAX_TRANSIT_STOPS    = 4,
    parameter int unsigned FLOOR_W              = (NUM_FLOORS <= 1) ? 1 : $clog2(NUM_FLOORS),
    parameter int unsigned DOOR_TIMER_W         = (DOOR_OPEN_CYCLES <= 1) ? 1 : $clog2(DOOR_OPEN_CYCLES + 1),
    parameter int unsigned HOLD_TIMER_W         = (HOLD_BEEP_CYCLES <= 1) ? 1 : $clog2(HOLD_BEEP_CYCLES + 1),
    parameter int unsigned ESTOP_BEEP_TIMER_W   = (ESTOP_BEEP_CYCLES <= 1) ? 1 : $clog2(ESTOP_BEEP_CYCLES + 1)
) (
    input  logic                      clk,
    input  logic                      reset,

    input  logic                      req_valid,
    input  logic [FLOOR_W-1:0]        req_floor,
    input  logic                      req_cabin,
    input  logic                      req_hall_up,

    input  logic [NUM_FLOORS-1:0]     cabin_buttons,
    input  logic [NUM_FLOORS-1:0]     hall_up_buttons,
    input  logic [NUM_FLOORS-1:0]     hall_down_buttons,

    input  logic                      emergency_stop,
    input  logic                      top_limit,
    input  logic                      bottom_limit,
    input  logic                      door_obstructed,

    output logic                      door_open,
    output logic                      idle,
    output logic                      moving_up,
    output logic                      moving_down,
    output logic                      service_dir_up,
    output logic                      service_dir_down,
    output logic [FLOOR_W-1:0]        current_floor,

    output logic [NUM_FLOORS-1:0]     pending_requests,
    output logic [NUM_FLOORS-1:0]     transit_hall_pending,
    output logic [FLOOR_W-1:0]        max_request,
    output logic [FLOOR_W-1:0]        min_request,
    output logic                      estop_latched,
    output logic                      hold_beep,
    output logic                      estop_beep,
    output logic                      door_hold_active,
    output logic                      transit_full,
    output logic [2:0]                fsm_state
);

    localparam int unsigned TOP_FLOOR = NUM_FLOORS - 1;
    localparam logic [FLOOR_W-1:0] MIN_INIT = FLOOR_W'(TOP_FLOOR);

    typedef enum logic [2:0] {
        ST_RESET            = 3'd0,
        ST_DOOR_CLOSED_IDLE = 3'd1,
        ST_DOOR_OPEN_IDLE   = 3'd2,
        ST_MOVING_UP        = 3'd3,
        ST_MOVING_DOWN      = 3'd4,
        ST_UP_DIR_SETTER    = 3'd5,
        ST_DOWN_DIR_SETTER  = 3'd6
    } lift_state_e;

    lift_state_e state_q, state_d;

    logic [NUM_FLOORS-1:0] requests_q, requests_d;
    logic [NUM_FLOORS-1:0] transit_q, transit_d;
    logic [FLOOR_W-1:0]    max_req_q, max_req_d;
    logic [FLOOR_W-1:0]    min_req_q, min_req_d;
    logic [FLOOR_W-1:0]    floor_q, floor_d;

    logic dir_up_q, dir_up_d;
    logic dir_down_q, dir_down_d;

    logic door_open_q, door_open_d;
    logic idle_q, idle_d;

    logic [DOOR_TIMER_W-1:0] door_timer_q, door_timer_d;
    logic [HOLD_TIMER_W-1:0] hold_timer_q, hold_timer_d;
    logic [ESTOP_BEEP_TIMER_W-1:0] estop_beep_timer_q, estop_beep_timer_d;

    logic hold_beep_q, hold_beep_d;
    logic estop_beep_q, estop_beep_d;
    logic door_hold_active_q, door_hold_active_d;

    logic estop_active_q, estop_active_d;
    logic estop_saved_dir_up_q, estop_saved_dir_up_d;
    logic estop_saved_dir_down_q, estop_saved_dir_down_d;
    lift_state_e estop_saved_state_q, estop_saved_state_d;
    logic [NUM_FLOORS-1:0] estop_saved_transit_q, estop_saved_transit_d;

    logic floor_has_request;
    logic any_request;
    logic can_move_up;
    logic can_move_down;

    assign pending_requests     = requests_q;
    assign transit_hall_pending = transit_q;
    assign max_request          = max_req_q;
    assign min_request          = min_req_q;
    assign current_floor        = floor_q;
    assign door_open            = door_open_q;
    assign idle                 = idle_q;
    assign moving_up            = (state_q == ST_MOVING_UP);
    assign moving_down          = (state_q == ST_MOVING_DOWN);
    assign service_dir_up       = dir_up_q;
    assign service_dir_down     = dir_down_q;
    assign estop_latched        = estop_active_q;
    assign hold_beep            = hold_beep_q;
    assign estop_beep           = estop_beep_q;
    assign door_hold_active     = door_hold_active_q;
    assign fsm_state            = state_q;

    function automatic logic [FLOOR_W-1:0] count_bits(
        input logic [NUM_FLOORS-1:0] bitmap
    );
        logic [FLOOR_W-1:0] total;
        total = '0;
        for (int i = 0; i < int'(NUM_FLOORS); i++) begin
            if (bitmap[i]) begin
                total = total + FLOOR_W'(1);
            end
        end
        return total;
    endfunction

    assign transit_full = count_bits(transit_q) >= FLOOR_W'(MAX_TRANSIT_STOPS);

    function automatic logic [FLOOR_W-1:0] scan_highest_request(
        input logic [NUM_FLOORS-1:0] req_bitmap
    );
        logic [FLOOR_W-1:0] result;
        logic               found;
        result = '0;
        found  = 1'b0;
        for (int i = int'(TOP_FLOOR); i >= 0; i--) begin
            if (!found && req_bitmap[i]) begin
                result = FLOOR_W'(i);
                found  = 1'b1;
            end
        end
        return result;
    endfunction

    function automatic logic [FLOOR_W-1:0] scan_lowest_request(
        input logic [NUM_FLOORS-1:0] req_bitmap,
        input logic                  skip_ground
    );
        logic [FLOOR_W-1:0] result;
        logic               found;
        int                 start_floor;
        result = MIN_INIT;
        found  = 1'b0;
        start_floor = skip_ground ? 1 : 0;
        for (int i = start_floor; i < int'(NUM_FLOORS); i++) begin
            if (!found && req_bitmap[i]) begin
                result = FLOOR_W'(i);
                found  = 1'b1;
            end
        end
        return result;
    endfunction

    function automatic logic [FLOOR_W-1:0] clamp_floor(input logic [FLOOR_W-1:0] floor);
        if (floor > FLOOR_W'(TOP_FLOOR)) begin
            return FLOOR_W'(TOP_FLOOR);
        end
        return floor;
    endfunction

    function automatic logic hall_call_allowed(
        input logic [FLOOR_W-1:0] floor,
        input logic               hall_up,
        input logic               cabin,
        input logic [FLOOR_W-1:0] cur_floor,
        input logic               cur_dir_up,
        input logic               cur_dir_down,
        input logic               cur_idle
    );
        if (cabin) begin
            return 1'b1;
        end

        if (cur_idle) begin
            return 1'b1;
        end

        if (floor > cur_floor) begin
            return hall_up && cur_dir_up;
        end

        if (floor < cur_floor) begin
            return !hall_up && cur_dir_down;
        end

        return 1'b1;
    endfunction

    function automatic logic [NUM_FLOORS-1:0] collect_new_requests(
        input logic [NUM_FLOORS-1:0] cabin_req,
        input logic [NUM_FLOORS-1:0] hall_up_req,
        input logic [NUM_FLOORS-1:0] hall_down_req,
        input logic                      req_pulse,
        input logic [FLOOR_W-1:0]        pulse_floor,
        input logic                      pulse_cabin,
        input logic                      pulse_hall_up,
        input logic [FLOOR_W-1:0]        cur_floor,
        input logic                      cur_dir_up,
        input logic                      cur_dir_down,
        input logic                      cur_idle
    );
        logic [NUM_FLOORS-1:0] additions;
        logic [FLOOR_W-1:0]    idx;

        additions = '0;

        for (int i = 0; i < int'(NUM_FLOORS); i++) begin
            if (cabin_req[i]) begin
                additions[i] = 1'b1;
            end

            if (hall_up_req[i] &&
                hall_call_allowed(FLOOR_W'(i), 1'b1, 1'b0, cur_floor, cur_dir_up, cur_dir_down, cur_idle)) begin
                additions[i] = 1'b1;
            end

            if (hall_down_req[i] &&
                hall_call_allowed(FLOOR_W'(i), 1'b0, 1'b0, cur_floor, cur_dir_up, cur_dir_down, cur_idle)) begin
                additions[i] = 1'b1;
            end
        end

        if (req_pulse) begin
            idx = clamp_floor(pulse_floor);
            if (pulse_cabin ||
                hall_call_allowed(idx, pulse_hall_up, 1'b0, cur_floor, cur_dir_up, cur_dir_down, cur_idle)) begin
                additions[idx] = 1'b1;
            end
        end

        return additions;
    endfunction

    function automatic logic is_cabin_addition(
        input int unsigned           floor_idx,
        input logic [NUM_FLOORS-1:0] cabin_req,
        input logic                  req_pulse,
        input logic [FLOOR_W-1:0]    pulse_floor,
        input logic                  pulse_cabin
    );
        if (cabin_req[floor_idx]) begin
            return 1'b1;
        end

        if (req_pulse && pulse_cabin && (FLOOR_W'(floor_idx) == clamp_floor(pulse_floor))) begin
            return 1'b1;
        end

        return 1'b0;
    endfunction

    always_comb begin
        logic [NUM_FLOORS-1:0] new_req_bits;
        logic [NUM_FLOORS-1:0] new_immediate;
        logic [NUM_FLOORS-1:0] new_transit;
        logic                  skip_ground_floor;

        skip_ground_floor      = 1'b0;

        state_d                = state_q;
        requests_d             = requests_q;
        transit_d              = transit_q;
        max_req_d              = max_req_q;
        min_req_d              = min_req_q;
        floor_d                = floor_q;
        dir_up_d               = dir_up_q;
        dir_down_d             = dir_down_q;
        door_open_d            = door_open_q;
        idle_d                 = idle_q;
        door_timer_d           = door_timer_q;
        hold_timer_d           = hold_timer_q;
        hold_beep_d            = hold_beep_q;
        estop_beep_d           = estop_beep_q;
        door_hold_active_d     = door_hold_active_q;
        estop_beep_timer_d     = estop_beep_timer_q;
        estop_active_d         = estop_active_q;
        estop_saved_dir_up_d   = estop_saved_dir_up_q;
        estop_saved_dir_down_d = estop_saved_dir_down_q;
        estop_saved_state_d    = estop_saved_state_q;
        estop_saved_transit_d  = estop_saved_transit_q;

        floor_has_request = requests_q[floor_q] | transit_q[floor_q];
        any_request       = |requests_q;
        can_move_up       = (floor_q < FLOOR_W'(TOP_FLOOR)) && !top_limit;
        can_move_down     = (floor_q > FLOOR_W'(0)) && !bottom_limit;

        new_req_bits  = collect_new_requests(
            cabin_buttons,
            hall_up_buttons,
            hall_down_buttons,
            req_valid,
            req_floor,
            req_cabin,
            req_hall_up,
            floor_q,
            dir_up_q,
            dir_down_q,
            idle_q
        );
        new_immediate = '0;
        new_transit   = '0;

        for (int i = 0; i < int'(NUM_FLOORS); i++) begin
            if (new_req_bits[i]) begin
                if (idle_q || is_cabin_addition(i, cabin_buttons, req_valid, req_floor, req_cabin)) begin
                    new_immediate[i] = 1'b1;
                end else begin
                    new_transit[i] = 1'b1;
                end
            end
        end

        if (|new_immediate) begin
            requests_d = requests_q | new_immediate;
            if (|requests_d) begin
                max_req_d = scan_highest_request(requests_d);
                min_req_d = scan_lowest_request(requests_d, skip_ground_floor);
            end else begin
                max_req_d = '0;
                min_req_d = MIN_INIT;
            end
        end

        if (|new_transit) begin
            logic [FLOOR_W-1:0] transit_count;
            logic [NUM_FLOORS-1:0] allowed_transit;
            transit_count   = count_bits(transit_q);
            allowed_transit = '0;
            for (int i = 0; i < int'(NUM_FLOORS); i++) begin
                if (new_transit[i] && (transit_count < FLOOR_W'(MAX_TRANSIT_STOPS))) begin
                    allowed_transit[i] = 1'b1;
                    transit_count      = transit_count + FLOOR_W'(1);
                end
            end
            transit_d = transit_q | allowed_transit;
        end

        if (reset) begin
            state_d        = ST_RESET;
            requests_d     = '0;
            transit_d      = '0;
            max_req_d      = '0;
            min_req_d      = MIN_INIT;
            floor_d        = '0;
            dir_up_d       = 1'b1;
            dir_down_d     = 1'b0;
            door_open_d    = 1'b0;
            idle_d         = 1'b1;
            door_timer_d   = '0;
            hold_timer_d   = '0;
            hold_beep_d    = 1'b0;
            estop_beep_d   = 1'b0;
            door_hold_active_d = 1'b0;
            estop_beep_timer_d = '0;
            estop_active_d = 1'b0;
        end else if (emergency_stop) begin
            if (!estop_active_q) begin
                estop_saved_dir_up_d   = dir_up_q;
                estop_saved_dir_down_d = dir_down_q;
                estop_saved_state_d    = state_q;
                estop_saved_transit_d  = transit_q;
            end

            estop_active_d = 1'b1;
            state_d        = ST_DOOR_CLOSED_IDLE;
            door_open_d    = 1'b0;
            idle_d         = 1'b1;
            door_timer_d   = '0;
            hold_timer_d   = '0;
            hold_beep_d    = 1'b0;
            door_hold_active_d = 1'b0;
            estop_beep_d   = 1'b1;
            estop_beep_timer_d = ESTOP_BEEP_TIMER_W'(ESTOP_BEEP_CYCLES);
            transit_d      = '0;
        end else if (estop_active_q) begin
            estop_active_d = 1'b0;
            dir_up_d       = estop_saved_dir_up_q;
            dir_down_d     = estop_saved_dir_down_q;
            state_d        = estop_saved_state_q;
            transit_d      = estop_saved_transit_q;
            idle_d         = (estop_saved_state_q != ST_MOVING_UP) &&
                             (estop_saved_state_q != ST_MOVING_DOWN);
        end else begin
            if (estop_beep_timer_q != ESTOP_BEEP_TIMER_W'(0)) begin
                estop_beep_d       = 1'b1;
                estop_beep_timer_d = estop_beep_timer_q - ESTOP_BEEP_TIMER_W'(1);
            end else begin
                estop_beep_d = 1'b0;
            end

            unique case (state_q)
                ST_RESET: begin
                    idle_d      = 1'b1;
                    door_open_d = 1'b0;
                    dir_up_d    = 1'b1;
                    dir_down_d  = 1'b0;
                    state_d     = ST_DOOR_CLOSED_IDLE;
                end

                ST_DOOR_CLOSED_IDLE: begin
                    idle_d       = 1'b1;
                    door_open_d  = 1'b0;
                    door_timer_d = '0;

                    if (|transit_q && !(|requests_q)) begin
                        requests_d = transit_q;
                        transit_d  = '0;
                        max_req_d  = scan_highest_request(requests_d);
                        min_req_d  = scan_lowest_request(requests_d, skip_ground_floor);
                    end

                    if (floor_has_request) begin
                        state_d     = ST_DOOR_OPEN_IDLE;
                        door_open_d = 1'b1;
                        requests_d[floor_q] = 1'b0;
                        transit_d[floor_q]  = 1'b0;
                        if (|requests_d) begin
                            max_req_d = scan_highest_request(requests_d);
                            min_req_d = scan_lowest_request(requests_d, skip_ground_floor);
                        end else begin
                            max_req_d = '0;
                            min_req_d = MIN_INIT;
                        end
                        door_timer_d = DOOR_TIMER_W'(DOOR_OPEN_CYCLES);
                    end else if (any_request) begin
                        if (dir_up_q && (max_req_q > floor_q) && can_move_up) begin
                            state_d = ST_MOVING_UP;
                            idle_d  = 1'b0;
                        end else if (dir_down_q && (min_req_q < floor_q) && can_move_down) begin
                            state_d = ST_MOVING_DOWN;
                            idle_d  = 1'b0;
                        end else if ((max_req_q == floor_q) && (min_req_q < floor_q)) begin
                            requests_d = requests_d | transit_d;
                            transit_d  = '0;
                            if (|requests_d) begin
                                max_req_d = scan_highest_request(requests_d);
                                min_req_d = scan_lowest_request(requests_d, skip_ground_floor);
                            end else begin
                                max_req_d = '0;
                                min_req_d = MIN_INIT;
                            end
                            state_d    = ST_DOWN_DIR_SETTER;
                            dir_up_d   = 1'b0;
                            dir_down_d = 1'b1;
                        end else if ((min_req_q == floor_q) && (max_req_q > floor_q)) begin
                            requests_d = requests_d | transit_d;
                            transit_d  = '0;
                            if (|requests_d) begin
                                max_req_d = scan_highest_request(requests_d);
                                min_req_d = scan_lowest_request(requests_d, skip_ground_floor);
                            end else begin
                                max_req_d = '0;
                                min_req_d = MIN_INIT;
                            end
                            state_d    = ST_UP_DIR_SETTER;
                            dir_up_d   = 1'b1;
                            dir_down_d = 1'b0;
                        end else if ((max_req_q > floor_q) && can_move_up) begin
                            dir_up_d   = 1'b1;
                            dir_down_d = 1'b0;
                            state_d    = ST_MOVING_UP;
                            idle_d     = 1'b0;
                        end else if ((min_req_q < floor_q) && can_move_down) begin
                            dir_up_d   = 1'b0;
                            dir_down_d = 1'b1;
                            state_d    = ST_MOVING_DOWN;
                            idle_d     = 1'b0;
                        end
                    end
                end

                ST_DOOR_OPEN_IDLE: begin
                    idle_d      = 1'b1;
                    door_open_d = 1'b1;

                    if (door_obstructed) begin
                        door_timer_d = DOOR_TIMER_W'(DOOR_OPEN_CYCLES);
                        if (hold_timer_q < HOLD_TIMER_W'(HOLD_BEEP_CYCLES)) begin
                            hold_timer_d = hold_timer_q + HOLD_TIMER_W'(1);
                        end
                        door_hold_active_d = 1'b1;
                        if (hold_timer_q >= HOLD_TIMER_W'(HOLD_BEEP_CYCLES)) begin
                            hold_beep_d = 1'b1;
                        end
                    end else begin
                        door_hold_active_d = 1'b0;
                        hold_beep_d        = 1'b0;
                        if (hold_timer_q != HOLD_TIMER_W'(0)) begin
                            hold_timer_d = '0;
                        end
                        if (door_timer_q != DOOR_TIMER_W'(0)) begin
                            door_timer_d = door_timer_q - DOOR_TIMER_W'(1);
                        end
                    end

                    if (floor_has_request) begin
                        requests_d[floor_q] = 1'b0;
                        transit_d[floor_q]  = 1'b0;
                        if (|requests_d) begin
                            max_req_d = scan_highest_request(requests_d);
                            min_req_d = scan_lowest_request(requests_d, skip_ground_floor);
                        end else begin
                            max_req_d = '0;
                            min_req_d = MIN_INIT;
                        end
                        door_timer_d = DOOR_TIMER_W'(DOOR_OPEN_CYCLES);
                    end else if ((door_timer_q == DOOR_TIMER_W'(1)) && !door_obstructed) begin
                        state_d      = ST_DOOR_CLOSED_IDLE;
                        door_open_d  = 1'b0;
                        door_timer_d = '0;
                        hold_timer_d = '0;
                        hold_beep_d  = 1'b0;
                        door_hold_active_d = 1'b0;
                    end
                end

                ST_MOVING_UP: begin
                    idle_d      = 1'b0;
                    door_open_d = 1'b0;

                    if (floor_has_request) begin
                        state_d     = ST_DOOR_OPEN_IDLE;
                        door_open_d = 1'b1;
                        idle_d      = 1'b1;
                        requests_d[floor_q] = 1'b0;
                        transit_d[floor_q]  = 1'b0;
                        if (|requests_d) begin
                            max_req_d = scan_highest_request(requests_d);
                            min_req_d = scan_lowest_request(requests_d, skip_ground_floor);
                        end else begin
                            max_req_d = '0;
                            min_req_d = MIN_INIT;
                        end
                        door_timer_d = DOOR_TIMER_W'(DOOR_OPEN_CYCLES);
                    end else if (can_move_up) begin
                        floor_d = floor_q + FLOOR_W'(1);
                    end else begin
                        requests_d = requests_d | transit_d;
                        transit_d  = '0;
                        if (|requests_d) begin
                            max_req_d = scan_highest_request(requests_d);
                            min_req_d = scan_lowest_request(requests_d, skip_ground_floor);
                        end else begin
                            max_req_d = '0;
                            min_req_d = MIN_INIT;
                        end
                        state_d    = ST_DOOR_CLOSED_IDLE;
                        idle_d     = 1'b1;
                        dir_up_d   = 1'b0;
                        dir_down_d = 1'b1;
                    end
                end

                ST_MOVING_DOWN: begin
                    idle_d      = 1'b0;
                    door_open_d = 1'b0;

                    if (floor_has_request) begin
                        state_d     = ST_DOOR_OPEN_IDLE;
                        door_open_d = 1'b1;
                        idle_d      = 1'b1;
                        requests_d[floor_q] = 1'b0;
                        transit_d[floor_q]  = 1'b0;
                        if (|requests_d) begin
                            max_req_d = scan_highest_request(requests_d);
                            min_req_d = scan_lowest_request(requests_d, skip_ground_floor);
                        end else begin
                            max_req_d = '0;
                            min_req_d = MIN_INIT;
                        end
                        door_timer_d = DOOR_TIMER_W'(DOOR_OPEN_CYCLES);
                    end else if (can_move_down) begin
                        floor_d = floor_q - FLOOR_W'(1);
                    end else begin
                        requests_d = requests_d | transit_d;
                        transit_d  = '0;
                        if (|requests_d) begin
                            max_req_d = scan_highest_request(requests_d);
                            min_req_d = scan_lowest_request(requests_d, skip_ground_floor);
                        end else begin
                            max_req_d = '0;
                            min_req_d = MIN_INIT;
                        end
                        state_d    = ST_DOOR_CLOSED_IDLE;
                        idle_d     = 1'b1;
                        dir_up_d   = 1'b1;
                        dir_down_d = 1'b0;
                    end
                end

                ST_UP_DIR_SETTER: begin
                    dir_up_d    = 1'b1;
                    dir_down_d  = 1'b0;
                    state_d     = ST_DOOR_OPEN_IDLE;
                    door_open_d = 1'b1;
                    idle_d      = 1'b1;

                    if (floor_has_request) begin
                        requests_d[floor_q] = 1'b0;
                        transit_d[floor_q]  = 1'b0;
                        if (|requests_d) begin
                            max_req_d = scan_highest_request(requests_d);
                            min_req_d = scan_lowest_request(requests_d, skip_ground_floor);
                        end else begin
                            max_req_d = '0;
                            min_req_d = MIN_INIT;
                        end
                    end

                    door_timer_d = DOOR_TIMER_W'(DOOR_OPEN_CYCLES);
                end

                ST_DOWN_DIR_SETTER: begin
                    dir_up_d    = 1'b0;
                    dir_down_d  = 1'b1;
                    state_d     = ST_DOOR_OPEN_IDLE;
                    door_open_d = 1'b1;
                    idle_d      = 1'b1;

                    if (floor_has_request) begin
                        requests_d[floor_q] = 1'b0;
                        transit_d[floor_q]  = 1'b0;
                        if (|requests_d) begin
                            max_req_d = scan_highest_request(requests_d);
                            min_req_d = scan_lowest_request(requests_d, skip_ground_floor);
                        end else begin
                            max_req_d = '0;
                            min_req_d = MIN_INIT;
                        end
                    end

                    door_timer_d = DOOR_TIMER_W'(DOOR_OPEN_CYCLES);
                end

                default: begin
                    state_d = ST_DOOR_CLOSED_IDLE;
                end
            endcase
        end
    end

    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            state_q                <= ST_RESET;
            requests_q             <= '0;
            transit_q              <= '0;
            max_req_q              <= '0;
            min_req_q              <= MIN_INIT;
            floor_q                <= '0;
            dir_up_q               <= 1'b1;
            dir_down_q             <= 1'b0;
            door_open_q            <= 1'b0;
            idle_q                 <= 1'b1;
            door_timer_q           <= '0;
            hold_timer_q           <= '0;
            hold_beep_q            <= 1'b0;
            estop_beep_q           <= 1'b0;
            door_hold_active_q     <= 1'b0;
            estop_beep_timer_q     <= '0;
            estop_active_q         <= 1'b0;
            estop_saved_dir_up_q   <= 1'b1;
            estop_saved_dir_down_q <= 1'b0;
            estop_saved_state_q    <= ST_DOOR_CLOSED_IDLE;
            estop_saved_transit_q  <= '0;
        end else begin
            state_q                <= state_d;
            requests_q             <= requests_d;
            transit_q              <= transit_d;
            max_req_q              <= max_req_d;
            min_req_q              <= min_req_d;
            floor_q                <= floor_d;
            dir_up_q               <= dir_up_d;
            dir_down_q             <= dir_down_d;
            door_open_q            <= door_open_d;
            idle_q                 <= idle_d;
            door_timer_q           <= door_timer_d;
            hold_timer_q           <= hold_timer_d;
            hold_beep_q            <= hold_beep_d;
            estop_beep_q           <= estop_beep_d;
            door_hold_active_q     <= door_hold_active_d;
            estop_beep_timer_q     <= estop_beep_timer_d;
            estop_active_q         <= estop_active_d;
            estop_saved_dir_up_q   <= estop_saved_dir_up_d;
            estop_saved_dir_down_q <= estop_saved_dir_down_d;
            estop_saved_state_q    <= estop_saved_state_d;
            estop_saved_transit_q  <= estop_saved_transit_d;
        end
    end

endmodule


module Lift16 (
    input  logic        clk,
    input  logic        reset,
    input  logic [3:0]  req_floor,
    output logic [1:0]  idle,
    output logic [1:0]  door,
    output logic [1:0]  Up,
    output logic [1:0]  Down,
    output logic [3:0]  current_floor,
    output logic [15:0] requests,
    output logic [3:0]  max_request,
    output logic [3:0]  min_request,
    input  logic        emergency_stop,
    output logic        hold_beep,
    output logic        estop_beep
);

    logic [3:0] req_floor_q;
    logic [3:0] cur_floor;
    logic         req_valid;
    logic         top_limit_i;
    logic         bottom_limit_i;
    logic         door_open;
    logic         idle_o;
    logic         moving_up;
    logic         moving_down;
    logic         service_dir_up;
    logic         service_dir_down;
    logic         estop_latched;
    logic         door_hold_active;
    logic [2:0]   fsm_state;
    logic [15:0]  transit_unused;

    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            req_floor_q <= 4'd0;
        end else begin
            req_floor_q <= req_floor;
        end
    end

    assign req_valid = (req_floor != req_floor_q) && !reset;

    elevator_controller #(
        .NUM_FLOORS(16),
        .DOOR_OPEN_CYCLES(10),
        .HOLD_BEEP_CYCLES(16),
        .ESTOP_BEEP_CYCLES(12),
        .FLOOR_W(4)
    ) u_ctrl (
        .clk(clk),
        .reset(reset),
        .req_valid(req_valid),
        .req_floor(req_floor),
        .req_cabin(1'b1),
        .req_hall_up(1'b0),
        .cabin_buttons('0),
        .hall_up_buttons('0),
        .hall_down_buttons('0),
        .emergency_stop(emergency_stop),
        .top_limit(top_limit_i),
        .bottom_limit(bottom_limit_i),
        .door_obstructed(1'b0),
        .door_open(door_open),
        .idle(idle_o),
        .moving_up(moving_up),
        .moving_down(moving_down),
        .service_dir_up(service_dir_up),
        .service_dir_down(service_dir_down),
        .current_floor(cur_floor),
        .pending_requests(requests),
        .transit_hall_pending(transit_unused),
        .max_request(max_request),
        .min_request(min_request),
        .estop_latched(estop_latched),
        .hold_beep(hold_beep),
        .estop_beep(estop_beep),
        .door_hold_active(door_hold_active),
        .fsm_state(fsm_state)
    );

    assign current_floor  = cur_floor;
    assign top_limit_i    = (cur_floor == 4'd15);
    assign bottom_limit_i = (cur_floor == 4'd0);

    assign idle = {1'b0, idle_o};
    assign door = {1'b0, door_open};
    assign Up   = {1'b0, service_dir_up};
    assign Down = {1'b0, service_dir_down};

endmodule
