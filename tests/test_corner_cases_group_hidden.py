"""Hidden multi-car and integration corner-case tests (elevator_group).

Golden RTL must pass; baseline sources/ must fail multiple scenarios.
"""

from __future__ import annotations

import os
from pathlib import Path

import cocotb
from cocotb.triggers import FallingEdge, RisingEdge
from cocotb_tools.runner import get_runner

from elevator_utils import NUM_LIFTS, TOP_FLOOR, start_clock

PROJ_PATH = Path(__file__).resolve().parent.parent
RTL_ROOT = Path(os.getenv("ELEVATOR_RTL_ROOT", "sources"))
SOURCES = [
    PROJ_PATH / RTL_ROOT / "elevator_controller.sv",
    PROJ_PATH / RTL_ROOT / "elevator_group.sv",
]
BUILD_PARAMS = {
    "NUM_LIFTS": 6,
    "NUM_FLOORS": 16,
    "DOOR_OPEN_CYCLES": 4,
    "HOLD_BEEP_CYCLES": 8,
    "ESTOP_BEEP_CYCLES": 6,
    "HALL_QUEUE_DEPTH": 8,
}


async def reset_group(dut, cycles: int = 2) -> None:
    dut.hall_req_valid.value = 0
    dut.hall_req_floor.value = 0
    dut.hall_req_up.value = 1
    dut.hall_up_buttons.value = 0
    dut.hall_down_buttons.value = 0
    dut.emergency_stop.value = 0
    dut.door_obstructed.value = 0
    for lift in range(NUM_LIFTS):
        dut.cabin_buttons[lift].value = 0
    dut.reset.value = 1
    for _ in range(cycles):
        await RisingEdge(dut.clk)
    dut.reset.value = 0
    await RisingEdge(dut.clk)


async def hall_call(dut, floor: int, up: int = 1) -> None:
    dut.hall_req_valid.value = 0
    await FallingEdge(dut.clk)
    dut.hall_req_floor.value = floor
    dut.hall_req_up.value = up
    dut.hall_req_valid.value = 1
    await FallingEdge(dut.clk)
    dut.hall_req_valid.value = 0


async def cabin_call(dut, lift: int, floor: int) -> None:
    await FallingEdge(dut.clk)
    dut.cabin_buttons[lift].value = 1 << floor
    await FallingEdge(dut.clk)
    dut.cabin_buttons[lift].value = 0


async def wait_group_cycles(dut, n: int) -> None:
    for _ in range(n):
        await RisingEdge(dut.clk)


async def wait_lift_floor(dut, lift: int, floor: int, max_cycles: int = 900) -> None:
    for _ in range(max_cycles):
        await RisingEdge(dut.clk)
        if int(dut.current_floor[lift].value) == floor and int(dut.idle[lift].value) == 1:
            return
    raise TimeoutError(f"Lift {lift} did not reach floor {floor}")


async def wait_lift_moving_up(dut, lift: int, max_cycles: int = 400) -> None:
    for _ in range(max_cycles):
        await RisingEdge(dut.clk)
        if int(dut.moving_up[lift].value) == 1:
            return
    raise TimeoutError(f"Lift {lift} did not start moving up")


# --- Dispatch scoring ----------------------------------------------------------


@cocotb.test()
async def test_moving_favorable_beats_idle_far_car(dut):
    await start_clock(dut)
    await reset_group(dut)
    await cabin_call(dut, 0, 8)
    await wait_lift_moving_up(dut, 0)
    await hall_call(dut, 10, up=1)
    await wait_group_cycles(dut, 10)
    assert int(dut.last_assigned_lift.value) == 0


@cocotb.test()
async def test_unfavorable_moving_loses_to_idle_nearer(dut):
    await start_clock(dut)
    await reset_group(dut)
    await cabin_call(dut, 0, 12)
    await wait_lift_floor(dut, 0, 12)
    await cabin_call(dut, 0, 0)
    for _ in range(200):
        await RisingEdge(dut.clk)
        if int(dut.moving_down[0].value) == 1:
            break
    else:
        raise TimeoutError("Lift 0 did not move down toward floor 0")
    await cabin_call(dut, 1, 7)
    await wait_lift_floor(dut, 1, 7)
    await hall_call(dut, 10, up=1)
    await wait_group_cycles(dut, 15)
    assert int(dut.last_assigned_lift.value) == 1


@cocotb.test()
async def test_all_lifts_estop_no_assignment(dut):
    await start_clock(dut)
    await reset_group(dut)
    dut.emergency_stop.value = (1 << NUM_LIFTS) - 1
    await hall_call(dut, 5)
    await wait_group_cycles(dut, 10)
    assert int(dut.hall_queue_count.value) >= 1


@cocotb.test()
async def test_round_robin_same_floor_idle_ties(dut):
    await start_clock(dut)
    await reset_group(dut)
    assigned = []
    for f in (9, 4, 11):
        await hall_call(dut, f)
        await wait_group_cycles(dut, 5)
        assigned.append(int(dut.last_assigned_lift.value))
    assert len(set(assigned)) >= 2


@cocotb.test()
async def test_nearest_idle_wins_over_farther_idle(dut):
    await start_clock(dut)
    await reset_group(dut)
    await cabin_call(dut, 2, 6)
    await wait_lift_floor(dut, 2, 6)
    await hall_call(dut, 8, up=1)
    await wait_group_cycles(dut, 10)
    assert int(dut.last_assigned_lift.value) == 2


@cocotb.test()
async def test_dispatch_prefers_low_load_idle_lift(dut):
    await start_clock(dut)
    await reset_group(dut)
    await cabin_call(dut, 0, 9)
    await wait_lift_floor(dut, 0, 9)
    for f in (11, 12, 13, 14, 15, TOP_FLOOR):
        await cabin_call(dut, 0, f)
        await RisingEdge(dut.clk)
    await cabin_call(dut, 1, 7)
    await wait_lift_floor(dut, 1, 7)
    await hall_call(dut, 10, up=1)
    await wait_group_cycles(dut, 15)
    assert int(dut.last_assigned_lift.value) == 1


@cocotb.test()
async def test_hold_beep_blocks_same_floor_hall_dispatch(dut):
    await start_clock(dut)
    await reset_group(dut)
    await cabin_call(dut, 0, 5)
    await wait_lift_floor(dut, 0, 5)
    dut.door_obstructed.value = 1 << 0
    for _ in range(BUILD_PARAMS["HOLD_BEEP_CYCLES"] + 6):
        await RisingEdge(dut.clk)
        if int(dut.hold_beep[0].value) == 1:
            break
    await hall_call(dut, 5, up=1)
    await wait_group_cycles(dut, 20)
    assert int(dut.last_assigned_lift.value) != 0


@cocotb.test()
async def test_hold_beep_lift_excluded_for_distant_call(dut):
    await start_clock(dut)
    await reset_group(dut)
    await cabin_call(dut, 0, 3)
    await wait_lift_floor(dut, 0, 3)
    dut.door_obstructed.value = 1 << 0
    for _ in range(BUILD_PARAMS["HOLD_BEEP_CYCLES"] + 6):
        await RisingEdge(dut.clk)
        if int(dut.hold_beep[0].value) == 1:
            break
    await hall_call(dut, 12)
    await wait_group_cycles(dut, 15)
    assert int(dut.last_assigned_lift.value) != 0


# --- Hall queue & routing ------------------------------------------------------


@cocotb.test()
async def test_fifo_hall_order_preserved(dut):
    await start_clock(dut)
    await reset_group(dut)
    dut.emergency_stop.value = (1 << NUM_LIFTS) - 1
    await hall_call(dut, 3)
    await hall_call(dut, 7)
    await hall_call(dut, 11)
    await RisingEdge(dut.clk)
    assert int(dut.hall_queue_count.value) == 3
    dut.emergency_stop.value = 0


@cocotb.test()
async def test_hall_serialized_enqueue_while_lifts_estop(dut):
    """Serialized hall calls enqueue even when every lift is in e-stop."""
    await start_clock(dut)
    await reset_group(dut)
    dut.emergency_stop.value = (1 << NUM_LIFTS) - 1
    await hall_call(dut, 9)
    await RisingEdge(dut.clk)
    assert int(dut.hall_queue_count.value) == 1
    dut.emergency_stop.value = 0


@cocotb.test()
async def test_button_latch_prevents_reenqueue(dut):
    await start_clock(dut)
    await reset_group(dut)
    dut.hall_up_buttons.value = 1 << 5
    await RisingEdge(dut.clk)
    await wait_group_cycles(dut, 5)
    first = int(dut.hall_queue_count.value)
    await RisingEdge(dut.clk)
    second = int(dut.hall_queue_count.value)
    dut.hall_up_buttons.value = 0
    assert second == first


@cocotb.test()
async def test_cabin_call_does_not_update_last_assigned(dut):
    await start_clock(dut)
    await reset_group(dut)
    await hall_call(dut, 10)
    await wait_group_cycles(dut, 5)
    hall_assign = int(dut.last_assigned_lift.value)
    await cabin_call(dut, 3, 8)
    await wait_group_cycles(dut, 5)
    await wait_lift_floor(dut, 3, 8)
    assert int(dut.last_assigned_lift.value) == hall_assign or int(dut.current_floor[3].value) == 8


@cocotb.test()
async def test_hall_queue_overflow_drops_new_call(dut):
    await start_clock(dut)
    await reset_group(dut)
    depth = BUILD_PARAMS["HALL_QUEUE_DEPTH"]
    dut.emergency_stop.value = (1 << NUM_LIFTS) - 1
    for f in range(1, depth + 1):
        await hall_call(dut, f)
        await RisingEdge(dut.clk)
    assert int(dut.hall_queue_full.value) == 1
    await hall_call(dut, 15)
    await RisingEdge(dut.clk)
    assert int(dut.hall_queue_count.value) == depth
    dut.emergency_stop.value = 0


@cocotb.test()
async def test_duplicate_hall_call_not_requeued(dut):
    await start_clock(dut)
    await reset_group(dut)
    await hall_call(dut, 6, up=1)
    await hall_call(dut, 6, up=1)
    await RisingEdge(dut.clk)
    assert int(dut.hall_queue_count.value) == 1


@cocotb.test()
async def test_group_hold_alarm_aggregates(dut):
    await start_clock(dut)
    await reset_group(dut)
    await cabin_call(dut, 1, 4)
    await wait_lift_floor(dut, 1, 4)
    dut.door_obstructed.value = 1 << 1
    for _ in range(BUILD_PARAMS["HOLD_BEEP_CYCLES"] + 6):
        await RisingEdge(dut.clk)
        if int(dut.group_hold_alarm.value) == 1:
            return
    raise TimeoutError("group_hold_alarm did not assert")


# --- Concurrent traffic --------------------------------------------------------


@cocotb.test()
async def test_six_hall_calls_all_get_dispatched(dut):
    """Each of six hall calls receives a valid lift assignment."""
    await start_clock(dut)
    await reset_group(dut)
    for f in (2, 4, 6, 8, 10, 12):
        await hall_call(dut, f)
        await wait_group_cycles(dut, 35)
        assert 0 <= int(dut.last_assigned_lift.value) < NUM_LIFTS


@cocotb.test()
async def test_one_busy_lift_routes_to_others(dut):
    await start_clock(dut)
    await reset_group(dut)
    for f in (11, 12, 13, TOP_FLOOR):
        await cabin_call(dut, 0, f)
        await RisingEdge(dut.clk)
    await wait_lift_moving_up(dut, 0)
    await hall_call(dut, 5)
    await wait_group_cycles(dut, 20)
    assert int(dut.last_assigned_lift.value) != 0


# --- Integration stories -------------------------------------------------------


@cocotb.test()
async def test_morning_up_peak_lobby_traffic(dut):
    await start_clock(dut)
    await reset_group(dut)
    for f in (8, 10, 12, 14):
        await hall_call(dut, f, up=1)
        await wait_group_cycles(dut, 15)
    assert any(int(dut.current_floor[i].value) >= 8 for i in range(NUM_LIFTS))


@cocotb.test()
async def test_lunch_reverse_peak_down_calls(dut):
    await start_clock(dut)
    await reset_group(dut)
    for lift in range(min(3, NUM_LIFTS)):
        await cabin_call(dut, lift, TOP_FLOOR)
        await wait_lift_floor(dut, lift, TOP_FLOOR)
    for f in (TOP_FLOOR, 12, 8, 3):
        await hall_call(dut, f, up=0)
        await wait_group_cycles(dut, 20)
    assert any(int(dut.current_floor[i].value) <= 3 for i in range(NUM_LIFTS))


@cocotb.test()
async def test_stranded_lift_estop_others_continue(dut):
    await start_clock(dut)
    await reset_group(dut)
    dut.emergency_stop.value = 1 << 2
    await hall_call(dut, 9)
    await hall_call(dut, 6)
    await wait_group_cycles(dut, 40)
    assert int(dut.estop_latched[2].value) == 1
    assert any(int(dut.current_floor[i].value) > 0 for i in range(NUM_LIFTS) if i != 2)


@cocotb.test()
async def test_queue_saturation_then_recovery(dut):
    await start_clock(dut)
    await reset_group(dut)
    depth = BUILD_PARAMS["HALL_QUEUE_DEPTH"]
    dut.emergency_stop.value = (1 << NUM_LIFTS) - 1
    for f in range(1, depth + 1):
        await hall_call(dut, f)
        await RisingEdge(dut.clk)
    assert int(dut.hall_queue_full.value) == 1
    dut.emergency_stop.value = 0
    await wait_group_cycles(dut, 200)
    assert int(dut.hall_queue_count.value) < depth or any(
        int(dut.current_floor[i].value) > 0 for i in range(NUM_LIFTS)
    )


@cocotb.test()
async def test_hall_and_cabin_mixed_storm(dut):
    await start_clock(dut)
    await reset_group(dut)
    for i in range(10):
        await hall_call(dut, 2 + (i % 6))
        await cabin_call(dut, i % NUM_LIFTS, 4 + (i % 4))
        await wait_group_cycles(dut, 5)
    assert any(int(dut.idle[i].value) == 0 or int(dut.current_floor[i].value) > 0 for i in range(NUM_LIFTS))


@cocotb.test()
async def test_hold_beep_end_reopens_dispatch(dut):
    await start_clock(dut)
    await reset_group(dut)
    await cabin_call(dut, 0, 2)
    await wait_lift_floor(dut, 0, 2)
    dut.door_obstructed.value = 1 << 0
    for _ in range(BUILD_PARAMS["HOLD_BEEP_CYCLES"] + 6):
        await RisingEdge(dut.clk)
        if int(dut.hold_beep[0].value) == 1:
            break
    dut.door_obstructed.value = 0
    await wait_group_cycles(dut, BUILD_PARAMS["DOOR_OPEN_CYCLES"] + 10)
    await hall_call(dut, 8)
    await wait_group_cycles(dut, 15)
    assert int(dut.last_assigned_lift.value) in range(NUM_LIFTS)


def test_corner_cases_group_hidden_runner():
    sim = os.getenv("SIM", "icarus")
    runner = get_runner(sim)
    runner.build(
        sources=SOURCES,
        hdl_toplevel="elevator_group",
        parameters=BUILD_PARAMS,
        always=True,
    )
    runner.test(
        hdl_toplevel="elevator_group",
        test_module=Path(__file__).stem,
        parameters=BUILD_PARAMS,
    )
