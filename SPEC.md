# Elevator Control System — Design Specification

This document describes the RTL design: module hierarchy, interfaces, state machines, and control algorithms. It covers **design content only** (no testbench, simulation, or integration procedures).

---

## 1. Overview

The design is a **parameterized elevator control system** in two layers:

| Layer | Module(s) | Purpose |
|-------|-----------|---------|
| **Single-car controller** | `elevator_controller`, `Lift16` | One car: movement, doors, requests, emergency stop, door-hold alarm |
| **Multi-car bank** | `elevator_group`, `elevator_dispatch` | Six parallel cars with priority-based hall-call assignment |

**Default configuration:** 16 floors (`0` … `15`), 6 lifts, 10 clock cycles door-open time, 16 cycles until hold beep, 12 cycles estop beep.

**Implementation style:** Fully synchronous — registered state in `always_ff`, next-state logic in `always_comb`.

---

## 2. Hierarchy

### 2.1 Single-car

```
                    ┌──────────────────────────────────────────┐
  clk, reset ──────►│                                          │
  req_valid/floor ──►│         elevator_controller              ├──► door_open, idle
  cabin/hall btns ──►│  • Request queue (bitmap)                ├──► moving_up/down
  emergency_stop ───►│  • Transit hall-call buffer              ├──► current_floor
  top/bottom limit ─►│  • max/min request tracking            ├──► pending_requests
  door_obstructed ──►│  • 7-state FSM                           ├──► transit_hall_pending
                    │  • Door timer + hold/estop beep          ├──► hold_beep, estop_beep
                    └──────────────────────────────────────────┘
```

### 2.2 Multi-car

```
  Hall calls ──► FIFO queue ──► elevator_dispatch ──► selected lift (0..5)
        │                              │
        │                              ▼
        │              ┌─── elevator_controller (lift 0)
        │              ├─── elevator_controller (lift 1)
  Cabin buttons ───────├─── elevator_controller (lift 2)
  (per lift)           ├─── elevator_controller (lift 3)
                       ├─── elevator_controller (lift 4)
                       └─── elevator_controller (lift 5)
```

**Signal routing**
- **Hall calls** → FIFO → dispatcher → one-cycle `req_valid` pulse to the chosen lift.
- **Cabin buttons** → connected directly to that lift (bypass dispatch).

---

## 3. Module: `elevator_controller`

Single-car controller. Scales by changing `NUM_FLOORS`; FSM structure is unchanged.

### 3.1 Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `NUM_FLOORS` | 16 | Floors `0` … `NUM_FLOORS-1` |
| `DOOR_OPEN_CYCLES` | 10 | Door-open duration in clock cycles |
| `HOLD_BEEP_CYCLES` | 16 | Obstruction cycles before `hold_beep` asserts |
| `ESTOP_BEEP_CYCLES` | 12 | E-stop beep duration in clock cycles |
| `MAX_TRANSIT_STOPS` | 4 | Maximum hall calls buffered in `transit_hall_pending` while moving |
| `FLOOR_W` | `$clog2(NUM_FLOORS)` | Floor address width |
| `DOOR_TIMER_W` | derived | Door countdown register width |
| `HOLD_TIMER_W` | derived | Hold-obstruction accumulator width |
| `ESTOP_BEEP_TIMER_W` | derived | E-stop beep countdown width |

### 3.2 Interface

**Clock / reset:** `clk`, `reset` (async active-high)

**Requests (serialized and parallel inputs are OR-ed)**

| Port | Dir | Description |
|------|-----|-------------|
| `req_valid` | in | One-cycle pulse on button press |
| `req_floor[FLOOR_W-1:0]` | in | Target floor (serialized path) |
| `req_cabin` | in | `1` = cabin button; `0` = hall call |
| `req_hall_up` | in | Hall direction: `1` = up, `0` = down |
| `cabin_buttons[NUM_FLOORS-1:0]` | in | Level-sensitive cabin buttons |
| `hall_up_buttons[NUM_FLOORS-1:0]` | in | Level-sensitive hall up buttons |
| `hall_down_buttons[NUM_FLOORS-1:0]` | in | Level-sensitive hall down buttons |

**Safety**

| Port | Description |
|------|-------------|
| `emergency_stop` | Stop car, close doors, latch and save state |
| `top_limit` | Inhibit upward motion |
| `bottom_limit` | Inhibit downward motion |
| `door_obstructed` | Reload door-open timer while asserted; accumulate toward hold beep |

**Outputs**

| Port | Description |
|------|-------------|
| `door_open` | Doors open |
| `idle` | Not traveling between floors |
| `moving_up` / `moving_down` | Active during `ST_MOVING_UP` / `ST_MOVING_DOWN` |
| `service_dir_up` / `service_dir_down` | Preferred service direction |
| `current_floor[FLOOR_W-1:0]` | Present floor |
| `pending_requests[NUM_FLOORS-1:0]` | Immediate pending-stop bitmap |
| `transit_hall_pending[NUM_FLOORS-1:0]` | Hall calls buffered while car is moving |
| `max_request` / `min_request` | Highest / lowest pending floor (immediate queue only) |
| `estop_latched` | Emergency stop active |
| `hold_beep` | Intentional door hold alarm (sustained obstruction) |
| `estop_beep` | Audible alarm during e-stop recovery period |
| `door_hold_active` | Obstruction hold in progress (before beep threshold) |
| `transit_full` | Asserted when the transit buffer holds `MAX_TRANSIT_STOPS` entries |
| `fsm_state[2:0]` | Current FSM state encoding |

### 3.3 Internal state

| Structure | Role |
|-----------|------|
| `pending_requests[i]` | Floor `i` needs immediate service (cabin calls, idle hall calls, merged transit) |
| `transit_hall_pending[i]` | Hall call at floor `i` registered while car is moving; not yet in sweep bounds |
| `max_request` | Upper bound of `pending_requests`; rescanned after each clear |
| `min_request` | Lower bound of `pending_requests`; rescanned after each clear; **floor 0 is never skipped** |
| `service_dir_up/down` | Direction flags; after reset: up=`1`, down=`0` |
| Sentinels (no requests) | `max_request = 0`, `min_request = NUM_FLOORS-1` |

**Stop detection:** A floor is treated as a stop target when `pending_requests[floor] | transit_hall_pending[floor]` is set at the current floor, in **all** FSM states including `ST_MOVING_DOWN` and `ST_MOVING_UP`.

### 3.4 FSM

| Encoding | State | Description |
|----------|-------|-------------|
| 0 | `ST_RESET` | Reset held; exit when `reset` deasserts |
| 1 | `ST_DOOR_CLOSED_IDLE` | Stopped, doors closed, selecting next move |
| 2 | `ST_DOOR_OPEN_IDLE` | Stopped, doors open, timer counting |
| 3 | `ST_MOVING_UP` | Moving up one floor per clock if no stop here |
| 4 | `ST_MOVING_DOWN` | Moving down one floor per clock if no stop here |
| 5 | `ST_UP_DIR_SETTER` | At lower sweep bound; set direction up, open if requested |
| 6 | `ST_DOWN_DIR_SETTER` | At upper sweep bound; set direction down, open if requested |

**Key transitions**

| From | Condition | To | Action |
|------|-----------|-----|--------|
| `ST_DOOR_CLOSED_IDLE` | Stop at `current_floor` | `ST_DOOR_OPEN_IDLE` | Open doors, clear bit(s), start timer |
| `ST_DOOR_CLOSED_IDLE` | Pending above, direction up | `ST_MOVING_UP` | — |
| `ST_DOOR_CLOSED_IDLE` | Pending below, direction down | `ST_MOVING_DOWN` | — |
| `ST_DOOR_CLOSED_IDLE` | `max_request == current_floor` | `ST_DOWN_DIR_SETTER` | Merge transit → pending, reverse to down |
| `ST_DOOR_CLOSED_IDLE` | `min_request == current_floor` | `ST_UP_DIR_SETTER` | Merge transit → pending, reverse to up |
| `ST_MOVING_UP/DOWN` | Stop at `current_floor` | `ST_DOOR_OPEN_IDLE` | Stop, open, clear, timer |
| `ST_MOVING_UP/DOWN` | No stop, limit OK | same | ±1 floor |
| `ST_MOVING_UP/DOWN` | No stop, at limit | `ST_DOOR_CLOSED_IDLE` | Merge transit → pending, flip direction |
| `ST_DOOR_OPEN_IDLE` | Timer expired, not obstructed | `ST_DOOR_CLOSED_IDLE` | Close doors |
| any (normal) | `emergency_stop` | `ST_DOOR_CLOSED_IDLE` | Save state/direction, hold, start estop beep |
| e-stop latched | `emergency_stop` deassert | saved state | Restore and resume |

### 3.5 Request acceptance

New requests are collected from parallel buttons and the serialized pulse path, then split:

| Call type | Car state | Destination |
|-----------|-----------|-------------|
| Cabin (any time) | Any | `pending_requests` immediately |
| Hall, car idle | Idle | `pending_requests` immediately |
| Hall, car moving | Moving | `transit_hall_pending` (if directionally allowed and buffer not full) |

When the transit buffer already holds `MAX_TRANSIT_STOPS` entries, additional hall calls are **dropped** (not added to transit or pending) and `transit_full` is asserted.

**Hall-call gating while moving** (`hall_call_allowed`):

| Car state | Accepted hall calls |
|-----------|---------------------|
| Idle | All |
| Moving up | Up-calls above `current_floor` |
| Moving down | Down-calls below `current_floor` |
| Same floor as car | Always (either direction) |

When the car reverses at a sweep bound or hits a travel limit, all `transit_hall_pending` bits are merged into `pending_requests` and transit is cleared before bounds are rescanned.

### 3.6 Movement algorithm

Direction-persistent, proximity-based scheduling:

1. Move in the current direction, stopping at every floor with a pending or transit request.
2. Continue until the directional sweep bound (`max_request` going up, `min_request` going down).
3. `min_request` scan always includes floor 0 — ground floor is never excluded from the downward sweep.
4. Reverse direction at the bound via the direction-setter states (transit merged first).
5. Nearest-in-path service follows naturally from floor-by-floor travel (no separate FCFS queue per car).

### 3.7 Door logic

- Arrival at a requested floor → `door_open=1`, clear request(s), load timer with `DOOR_OPEN_CYCLES`.
- Timer counts down each clock; at zero (with `door_obstructed` deasserted) → doors close, return to `ST_DOOR_CLOSED_IDLE`.
- `door_obstructed` during open → reload timer, set `door_hold_active`, increment hold accumulator.
- After `HOLD_BEEP_CYCLES` consecutive obstructed cycles → assert `hold_beep` (doors remain open).

### 3.8 Emergency stop

1. **Assert:** latch e-stop, save FSM state, `service_dir_up/down`, and **`transit_hall_pending` bitmap**, force idle with doors closed, start `estop_beep` for `ESTOP_BEEP_CYCLES`.
2. **Hold** while `emergency_stop` is asserted.
3. **Deassert:** clear latch, restore saved direction, FSM state, and **transit bitmap**; `estop_beep` counts down independently.

---

## 4. Module: `Lift16`

Compatibility wrapper around `elevator_controller` (legacy port names, fixed 16 floors).

| Port | Mapping |
|------|---------|
| `req_floor[3:0]` | Edge-detected as `req_valid`; always treated as cabin call |
| `idle`, `door`, `Up`, `Down` | 2-bit `{1'b0, signal}` |
| `requests[15:0]` | `pending_requests` |
| `top_limit` / `bottom_limit` | Derived from `current_floor == 15` / `== 0` |
| `hold_beep`, `estop_beep` | Passed through from controller |

For hall-call behaviour and transit buffering, instantiate `elevator_controller` directly.

---

## 5. Module: `elevator_dispatch`

Registered round-robin pointer plus combinational scoring. Selects the lift with the **lowest score** for one hall call.

### 5.1 Availability

A lift is **unavailable** for assignment when `estop_latched` **or** `hold_beep` is asserted. Unavailable lifts receive score `{SCORE_W{1'b1}}`.

### 5.2 Score bands

| Band | Condition | Metric |
|------|-----------|--------|
| 0 | Idle at call floor | `lift_id` (tie-break only) |
| 1 | Idle elsewhere | `(distance << 3) + (pending_count << 2)` |
| 2 | Moving favorably toward call | `(distance << 1) + pending_count` |
| 3 | Unfavorable direction or heavy load | `(distance << 3) + (load << 2) + 10` |

**Composite score:** `(band << 8) + (metric << LIFT_W) + rotated_lift_id`

**Tie-break:** `rr_ptr` rotates after each valid assignment so same-floor idle ties cycle across cars.

### 5.3 Favorable movement

| Call vs car | Call type | Favorable when |
|-------------|-----------|----------------|
| Above | Up | `moving_up` or `service_dir_up`, not moving down |
| Below | Down | `moving_down` or `service_dir_down`, not moving up |
| Same floor | Either | Always |

### 5.4 Interface

| Port | Description |
|------|-------------|
| `call_floor`, `call_up`, `call_valid` | Hall call to score |
| `lift_floor[i]`, `lift_idle[i]`, `lift_moving_up/down[i]` | Per-lift motion |
| `lift_dir_up/down[i]` | Per-lift service direction |
| `lift_estop[i]`, `lift_hold_beep[i]`, `lift_pending[i]` | Availability and load |
| `selected_lift[LIFT_W-1:0]` | Winning lift index |
| `assign_valid` | Valid winner exists |
| `lift_score[i]`, `selected_score` | Per-lift and winning scores |

---

## 6. Module: `elevator_group`

Top level: `NUM_LIFTS` instances of `elevator_controller` plus hall-call queue and dispatch.

### 6.1 Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `NUM_LIFTS` | 6 | Parallel cars |
| `NUM_FLOORS` | 16 | Floors per car |
| `DOOR_OPEN_CYCLES` | 10 | Per-car door time |
| `HOLD_BEEP_CYCLES` | 16 | Per-car hold-beep threshold |
| `ESTOP_BEEP_CYCLES` | 12 | Per-car e-stop beep duration |
| `HALL_QUEUE_DEPTH` | 8 | Hall-call FIFO depth |

### 6.2 Hall-call inputs

| Source | Mechanism |
|--------|-----------|
| Serialized | `hall_req_valid` + `hall_req_floor` + `hall_req_up` |
| Parallel up | Rising edge on `hall_up_buttons[f]` (highest floor wins if multiple) |
| Parallel down | Rising edge on `hall_down_buttons[f]` (lowest floor wins if multiple) |

Duplicate `{floor, direction}` entries are not enqueued twice. FIFO overflow drops new entries (`hall_queue_full`). Button latches prevent re-enqueue while a button remains pressed.

### 6.3 Dispatch FSM

| State | Behaviour |
|-------|-----------|
| `DS_IDLE` | Wait for non-empty queue → `DS_ASSIGN` |
| `DS_ASSIGN` | Run `elevator_dispatch`, pulse selected lift, dequeue head; stay in `DS_ASSIGN` if queue remains |

Per-lift hall button inputs are tied off inside the group; hall traffic enters only through the FIFO/dispatch path.

### 6.4 Interface (group-level)

**Inputs:** hall-call ports above; `cabin_buttons[NUM_LIFTS][NUM_FLOORS]`; per-lift `emergency_stop`, `door_obstructed`.

**Outputs (per lift `i`):** same as `elevator_controller` (including `hold_beep`, `estop_beep`, `door_hold_active`, `transit_hall_pending`).

**Group alarms:** `group_hold_alarm` (OR of all `hold_beep`), `group_estop_alarm` (OR of all `estop_beep`).

**Dispatch status:** `last_assigned_lift`, `last_assigned_floor`, `last_assigned_up`, `hall_queue_full`, `hall_queue_count`.

---

## 7. Design assumptions

| Topic | Assumption |
|-------|------------|
| Clock | One floor step per clock while moving (behavioural travel model) |
| Reset | Async assert; all lifts start idle at floor 0 |
| Serialized requests | Single-cycle `req_valid` pulse |
| Doors | Single `door_open` flag; no separate door motor FSM |
| Multi-car | Each lift maintains an independent `current_floor` |
| Shafts | No inter-lift collision avoidance |
| Hold beep | Lift is unavailable for new hall dispatch while `hold_beep` is active |

---

## 8. Design scope

The RTL **includes:**
- Parameterized floor count and multi-car dispatch (default 16 × 6)
- Direction-aware request filtering and sweep scheduling
- Transit hall-call buffer with merge at direction reversal
- Door timer, obstruction extension, and hold-beep alarm
- Emergency stop with state/direction save-restore and estop beep
- Limit-sensor gating
- Round-robin hall-call dispatch with load- and distance-aware scoring

The RTL **does not include:**
- Motor drive / PWM / acceleration profiles
- Shared-shaft or anti-collision logic between cars
- External back-pressure when the hall FIFO is full
- Persistent hall-lamp or button-latch outputs to the building interface
