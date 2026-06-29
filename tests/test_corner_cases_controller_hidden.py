"""Hidden single-car corner-case tests (elevator_controller).

Golden RTL must pass; baseline sources/ must fail multiple scenarios.
"""

from __future__ import annotations

import os
from pathlib import Path

import cocotb
from cocotb.triggers import FallingEdge, RisingEdge
from cocotb_tools.runner import get_runner

from elevator_utils import (
    TOP_FLOOR,
    apply_limits,
    has_floor_request,
    pending_bitmap,
    pulse_request,
    reset_controller,
    start_clock,
    transit_bitmap,
    wait_cycles,
    wait_door_open_at,
    wait_for_floor,
    wait_serve_at,
    wait_until_moving_down,
    wait_until_moving_down_above,
    wait_until_moving_up,
    wait_until_no_requests,
)

PROJ_PATH = Path(__file__).resolve().parent.parent
RTL_ROOT = Path(os.getenv("ELEVATOR_RTL_ROOT", "sources"))
SOURCES = [PROJ_PATH / RTL_ROOT / "elevator_controller.sv"]
BUILD_PARAMS = {
    "NUM_FLOORS": 16,
    "DOOR_OPEN_CYCLES": 4,
    "HOLD_BEEP_CYCLES": 8,
    "ESTOP_BEEP_CYCLES": 6,
    "MAX_TRANSIT_STOPS": 4,
}


# --- Direction & request gating ------------------------------------------------


@cocotb.test()
async def test_wrong_direction_hall_rejected_while_moving_up(dut):
    await start_clock(dut)
    await reset_controller(dut)
    await pulse_request(dut, TOP_FLOOR)
    await wait_until_moving_up(dut)
    for _ in range(40):
        await RisingEdge(dut.clk)
        apply_limits(dut)
        if int(dut.current_floor.value) >= 3:
            break
    await pulse_request(dut, 8, cabin=0, hall_up=0)
    await RisingEdge(dut.clk)
    apply_limits(dut)
    assert not has_floor_request(dut, 8)


@cocotb.test()
async def test_same_floor_hall_opens_doors_while_idle(dut):
    await start_clock(dut)
    await reset_controller(dut)
    await pulse_request(dut, 7, cabin=0, hall_up=1)
    await wait_door_open_at(dut, 7)
    assert int(dut.idle.value) == 1


@cocotb.test()
async def test_cabin_call_accepted_when_transit_buffer_full(dut):
    await start_clock(dut)
    await reset_controller(dut)
    await pulse_request(dut, 10)
    await wait_until_moving_up(dut)
    for _ in range(50):
        await RisingEdge(dut.clk)
        apply_limits(dut)
        if int(dut.current_floor.value) >= 2:
            break
    for f in (11, 12, 13, 14, 15):
        await pulse_request(dut, f, cabin=0, hall_up=1)
        await RisingEdge(dut.clk)
        apply_limits(dut)
    assert int(dut.transit_full.value) == 1
    await pulse_request(dut, 6)
    await RisingEdge(dut.clk)
    assert pending_bitmap(dut) & (1 << 6)


@cocotb.test()
async def test_opposite_hall_directions_same_floor(dut):
    await start_clock(dut)
    await reset_controller(dut)
    await pulse_request(dut, 5, cabin=0, hall_up=1)
    await RisingEdge(dut.clk)
    await pulse_request(dut, 5, cabin=0, hall_up=0)
    await RisingEdge(dut.clk)
    apply_limits(dut)
    assert pending_bitmap(dut) & (1 << 5)
    await wait_serve_at(dut, 5)


@cocotb.test()
async def test_serialized_and_parallel_cabin_or(dut):
    await start_clock(dut)
    await reset_controller(dut)
    await pulse_request(dut, 4, cabin=1)
    dut.cabin_buttons.value = 1 << 4
    await RisingEdge(dut.clk)
    apply_limits(dut)
    await wait_serve_at(dut, 4)


# --- Sweep & scheduling --------------------------------------------------------


@cocotb.test()
async def test_reversal_at_top_limit_with_lower_pending(dut):
    await start_clock(dut)
    await reset_controller(dut)
    await pulse_request(dut, TOP_FLOOR)
    await wait_serve_at(dut, TOP_FLOOR)
    await pulse_request(dut, 3)
    await wait_for_floor(dut, 3, max_cycles=800)
    assert int(dut.current_floor.value) == 3


@cocotb.test()
async def test_reversal_at_bottom_limit_with_upper_pending(dut):
    await start_clock(dut)
    await reset_controller(dut)
    await pulse_request(dut, 12)
    await wait_for_floor(dut, 12, max_cycles=800)
    assert int(dut.current_floor.value) == 12


@cocotb.test()
async def test_stops_at_every_floor_in_upward_path(dut):
    await start_clock(dut)
    await reset_controller(dut)
    for f in (3, 5, 7):
        await pulse_request(dut, f)
    for f in (3, 5, 7):
        await wait_serve_at(dut, f)


@cocotb.test()
async def test_transit_buffer_capacity_enforced(dut):
    await start_clock(dut)
    await reset_controller(dut)
    await pulse_request(dut, 10)
    await wait_until_moving_up(dut)
    for _ in range(50):
        await RisingEdge(dut.clk)
        apply_limits(dut)
        if int(dut.current_floor.value) >= 2:
            break
    for f in (11, 12, 13, 14, 15):
        await pulse_request(dut, f, cabin=0, hall_up=1)
        await RisingEdge(dut.clk)
        apply_limits(dut)
    assert bin(transit_bitmap(dut)).count("1") == BUILD_PARAMS["MAX_TRANSIT_STOPS"]
    assert (transit_bitmap(dut) >> 15) & 1 == 0


@cocotb.test()
async def test_multiple_transit_merged_at_reversal(dut):
    await start_clock(dut)
    await reset_controller(dut)
    await pulse_request(dut, 10)
    await wait_until_moving_up(dut)
    await pulse_request(dut, 15, cabin=0, hall_up=1)
    await RisingEdge(dut.clk)
    assert transit_bitmap(dut) & (1 << 15)
    await wait_serve_at(dut, 10)
    await wait_for_floor(dut, 15, max_cycles=1200)


@cocotb.test()
async def test_ground_floor_served_on_downward_sweep(dut):
    await start_clock(dut)
    await reset_controller(dut)
    await pulse_request(dut, TOP_FLOOR)
    await wait_serve_at(dut, TOP_FLOOR)
    await pulse_request(dut, 0)
    await wait_until_moving_down_above(dut, 10)
    await pulse_request(dut, 12)
    await RisingEdge(dut.clk)
    assert int(dut.min_request.value) == 0
    await wait_for_floor(dut, 0, max_cycles=800)


@cocotb.test()
async def test_transit_hall_stops_while_moving_down(dut):
    await start_clock(dut)
    await reset_controller(dut)
    await pulse_request(dut, TOP_FLOOR)
    await wait_serve_at(dut, TOP_FLOOR)
    await pulse_request(dut, 3)
    await wait_until_moving_down_above(dut, 8)
    await pulse_request(dut, 8, cabin=0, hall_up=0)
    await wait_for_floor(dut, 8, max_cycles=500)


@cocotb.test()
async def test_min_request_includes_floors_zero_and_one(dut):
    await start_clock(dut)
    await reset_controller(dut)
    await pulse_request(dut, TOP_FLOOR)
    await wait_serve_at(dut, TOP_FLOOR)
    await pulse_request(dut, 0)
    await pulse_request(dut, 1)
    await wait_until_moving_down(dut)
    await RisingEdge(dut.clk)
    assert int(dut.min_request.value) <= 1


# --- Door & timing -------------------------------------------------------------


@cocotb.test()
async def test_request_during_door_open_reloads_timer(dut):
    await start_clock(dut)
    await reset_controller(dut)
    await pulse_request(dut, 4)
    await wait_door_open_at(dut, 4)
    await pulse_request(dut, 4)
    await wait_cycles(dut, BUILD_PARAMS["DOOR_OPEN_CYCLES"] + 1)
    assert int(dut.door_open.value) == 1


@cocotb.test()
async def test_obstruction_release_then_doors_close(dut):
    await start_clock(dut)
    await reset_controller(dut)
    await pulse_request(dut, 2)
    await wait_door_open_at(dut, 2)
    dut.door_obstructed.value = 1
    await wait_cycles(dut, BUILD_PARAMS["DOOR_OPEN_CYCLES"] + 2)
    dut.door_obstructed.value = 0
    await wait_cycles(dut, BUILD_PARAMS["DOOR_OPEN_CYCLES"] + 2)
    assert int(dut.door_open.value) == 0


@cocotb.test()
async def test_hold_beep_clears_when_obstruction_ends(dut):
    await start_clock(dut)
    await reset_controller(dut)
    await pulse_request(dut, 1)
    await wait_door_open_at(dut, 1)
    dut.door_obstructed.value = 1
    for _ in range(BUILD_PARAMS["HOLD_BEEP_CYCLES"] + 4):
        await RisingEdge(dut.clk)
        apply_limits(dut)
        if int(dut.hold_beep.value) == 1:
            break
    dut.door_obstructed.value = 0
    await wait_cycles(dut, 4)
    assert int(dut.hold_beep.value) == 0


@cocotb.test()
async def test_back_to_back_floor_service(dut):
    await start_clock(dut)
    await reset_controller(dut)
    await pulse_request(dut, 2)
    await pulse_request(dut, 3)
    await wait_serve_at(dut, 2)
    await wait_serve_at(dut, 3)


# --- Emergency stop ------------------------------------------------------------


@cocotb.test()
async def test_transit_preserved_through_estop(dut):
    await start_clock(dut)
    await reset_controller(dut)
    await pulse_request(dut, TOP_FLOOR)
    await wait_serve_at(dut, TOP_FLOOR)
    await pulse_request(dut, 0)
    await wait_until_moving_down_above(dut, 8)
    await pulse_request(dut, 8, cabin=0, hall_up=0)
    await RisingEdge(dut.clk)
    assert transit_bitmap(dut) & (1 << 8)
    dut.emergency_stop.value = 1
    await wait_cycles(dut, 4)
    dut.emergency_stop.value = 0
    await wait_cycles(dut, 4)
    await wait_for_floor(dut, 8, max_cycles=800)


@cocotb.test()
async def test_estop_mid_travel_resumes_service(dut):
    await start_clock(dut)
    await reset_controller(dut)
    await pulse_request(dut, 11)
    await wait_until_moving_up(dut)
    dut.emergency_stop.value = 1
    await wait_cycles(dut, 5)
    dut.emergency_stop.value = 0
    await wait_for_floor(dut, 11, max_cycles=800)


@cocotb.test()
async def test_estop_during_door_open_resumes(dut):
    await start_clock(dut)
    await reset_controller(dut)
    await pulse_request(dut, 5)
    await wait_door_open_at(dut, 5)
    dut.emergency_stop.value = 1
    await wait_cycles(dut, 4)
    dut.emergency_stop.value = 0
    await wait_cycles(dut, 4)
    await wait_until_no_requests(dut)


@cocotb.test()
async def test_estop_bounce_does_not_lose_pending(dut):
    await start_clock(dut)
    await reset_controller(dut)
    await pulse_request(dut, 9)
    await wait_until_moving_up(dut)
    for _ in range(3):
        dut.emergency_stop.value = 1
        await wait_cycles(dut, 2)
        dut.emergency_stop.value = 0
        await wait_cycles(dut, 2)
    await wait_for_floor(dut, 9, max_cycles=800)


@cocotb.test()
async def test_estop_with_full_transit_buffer(dut):
    await start_clock(dut)
    await reset_controller(dut)
    await pulse_request(dut, 10)
    await wait_until_moving_up(dut)
    for _ in range(50):
        await RisingEdge(dut.clk)
        if int(dut.current_floor.value) >= 2:
            break
    for f in (11, 12, 13, 14, 15):
        await pulse_request(dut, f, cabin=0, hall_up=1)
        await RisingEdge(dut.clk)
    assert int(dut.transit_full.value) == 1
    saved_transit = transit_bitmap(dut)
    dut.emergency_stop.value = 1
    await wait_cycles(dut, 3)
    dut.emergency_stop.value = 0
    await wait_cycles(dut, 2)
    assert transit_bitmap(dut) == saved_transit
    await wait_serve_at(dut, 10)


# --- Limits & safety -----------------------------------------------------------


@cocotb.test()
async def test_top_limit_inhibits_upward_motion(dut):
    await start_clock(dut)
    await reset_controller(dut)
    await pulse_request(dut, TOP_FLOOR)
    await wait_for_floor(dut, TOP_FLOOR)
    await pulse_request(dut, TOP_FLOOR)
    await wait_cycles(dut, 5)
    assert int(dut.current_floor.value) == TOP_FLOOR
    assert int(dut.moving_up.value) == 0


@cocotb.test()
async def test_bottom_limit_inhibits_downward_motion(dut):
    await start_clock(dut)
    await reset_controller(dut)
    await pulse_request(dut, 0)
    await wait_for_floor(dut, 0)
    await pulse_request(dut, 0)
    await wait_cycles(dut, 5)
    assert int(dut.current_floor.value) == 0
    assert int(dut.moving_down.value) == 0


@cocotb.test()
async def test_reset_clears_all_state(dut):
    await start_clock(dut)
    await reset_controller(dut)
    await pulse_request(dut, 8)
    await wait_cycles(dut, 5)
    dut.reset.value = 1
    await wait_cycles(dut, 2)
    dut.reset.value = 0
    await wait_cycles(dut, 2)
    assert int(dut.current_floor.value) == 0
    assert pending_bitmap(dut) == 0
    assert transit_bitmap(dut) == 0


@cocotb.test()
async def test_bounds_rescan_after_clear(dut):
    await start_clock(dut)
    await reset_controller(dut)
    await pulse_request(dut, 5)
    await pulse_request(dut, 12)
    await RisingEdge(dut.clk)
    assert int(dut.max_request.value) == 12
    assert int(dut.min_request.value) == 5
    await wait_serve_at(dut, 5)
    await RisingEdge(dut.clk)
    assert int(dut.min_request.value) == 12
    assert int(dut.max_request.value) == 12


# --- Complex traffic stories (single car) --------------------------------------


@cocotb.test()
async def test_ping_pong_direction_sweep(dut):
    await start_clock(dut)
    await reset_controller(dut)
    for f in (2, 14, 3, 13, 4):
        await pulse_request(dut, f)
    for f in (2, 3, 4, 13, 14):
        await wait_serve_at(dut, f)


@cocotb.test()
async def test_inter_floor_traffic_cluster(dut):
    await start_clock(dut)
    await reset_controller(dut)
    for f in (4, 6, 5, 7):
        await pulse_request(dut, f)
    for f in (4, 5, 6, 7):
        await wait_serve_at(dut, f)
    assert int(dut.current_floor.value) in (4, 5, 6, 7)


@cocotb.test()
async def test_express_cabin_to_top(dut):
    await start_clock(dut)
    await reset_controller(dut)
    for f in (2, 4, 6):
        await pulse_request(dut, f)
    await pulse_request(dut, TOP_FLOOR)
    await wait_serve_at(dut, TOP_FLOOR)
    assert int(dut.current_floor.value) == TOP_FLOOR


def test_corner_cases_controller_hidden_runner():
    sim = os.getenv("SIM", "icarus")
    runner = get_runner(sim)
    runner.build(
        sources=SOURCES,
        hdl_toplevel="elevator_controller",
        parameters=BUILD_PARAMS,
        always=True,
    )
    runner.test(
        hdl_toplevel="elevator_controller",
        test_module=Path(__file__).stem,
        parameters=BUILD_PARAMS,
    )
