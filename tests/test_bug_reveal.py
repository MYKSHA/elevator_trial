"""Targeted cocotb tests that expose hidden defects in sources/ RTL.

These scenarios pass on golden/ clean RTL and fail on sources/ buggy RTL.
"""

from __future__ import annotations

import os
from pathlib import Path

import cocotb
from cocotb.triggers import FallingEdge, RisingEdge
from cocotb_tools.runner import get_runner

from elevator_utils import (
    TOP_FLOOR,
    NUM_LIFTS,
    apply_limits,
    pulse_request,
    reset_controller,
    start_clock,
    wait_for_floor,
)

PROJ_PATH = Path(__file__).resolve().parent.parent
RTL_ROOT = Path(os.getenv("ELEVATOR_RTL_ROOT", "sources"))
CTRL_SOURCES = [PROJ_PATH / RTL_ROOT / "elevator_controller.sv"]
GROUP_SOURCES = [
    PROJ_PATH / RTL_ROOT / "elevator_controller.sv",
    PROJ_PATH / RTL_ROOT / "elevator_group.sv",
]
BUILD_PARAMS = {
    "NUM_FLOORS": 16,
    "DOOR_OPEN_CYCLES": 4,
    "HOLD_BEEP_CYCLES": 8,
    "ESTOP_BEEP_CYCLES": 6,
    "MAX_TRANSIT_STOPS": 4,
}
GROUP_BUILD_PARAMS = {
    "NUM_LIFTS": 6,
    "NUM_FLOORS": 16,
    "DOOR_OPEN_CYCLES": 4,
    "HOLD_BEEP_CYCLES": 8,
    "ESTOP_BEEP_CYCLES": 6,
    "HALL_QUEUE_DEPTH": 8,
}


async def wait_serve_at(dut, floor: int) -> None:
    await wait_for_floor(dut, floor)
    assert int(dut.pending_requests.value) & (1 << floor) == 0


async def wait_until_moving_down_above(dut, min_floor: int, max_cycles: int = 500) -> int:
    """Return current floor once the car is moving down above min_floor."""
    for _ in range(max_cycles):
        await RisingEdge(dut.clk)
        apply_limits(dut)
        cur = int(dut.current_floor.value)
        if int(dut.moving_down.value) == 1 and cur > min_floor:
            return cur
    raise TimeoutError(f"Car did not move down above floor {min_floor}")


# --- Single-car bugs (elevator_controller) -----------------------------------


@cocotb.test()
async def test_ground_floor_served_on_downward_sweep(dut):
    """Bug 1: min-scan must not skip floor 0 while moving down."""
    await start_clock(dut)
    await reset_controller(dut)

    await pulse_request(dut, TOP_FLOOR)
    await wait_serve_at(dut, TOP_FLOOR)

    await pulse_request(dut, 0)
    await wait_until_moving_down_above(dut, 10)

    # Force a bounds refresh while descending; floor 0 must remain the minimum.
    await pulse_request(dut, 12)
    await RisingEdge(dut.clk)
    apply_limits(dut)
    assert int(dut.min_request.value) == 0
    assert int(dut.pending_requests.value) & 0x1

    await wait_for_floor(dut, 0, max_cycles=800)
    assert int(dut.pending_requests.value) & 0x1 == 0


@cocotb.test()
async def test_transit_hall_stops_while_moving_down(dut):
    """Bug 2: transit hall calls must stop the car while moving down."""
    await start_clock(dut)
    await reset_controller(dut)

    await pulse_request(dut, TOP_FLOOR)
    await wait_serve_at(dut, TOP_FLOOR)

    await pulse_request(dut, 3)
    await wait_until_moving_down_above(dut, 8)
    await pulse_request(dut, 8, cabin=0, hall_up=0)

    await wait_for_floor(dut, 8, max_cycles=500)
    assert int(dut.pending_requests.value) & (1 << 8) == 0
    assert int(dut.transit_hall_pending.value) & (1 << 8) == 0


# --- Group bug (elevator_group) ----------------------------------------------


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


async def wait_lift_floor(dut, lift: int, floor: int, max_cycles: int = 600) -> None:
    for _ in range(max_cycles):
        await RisingEdge(dut.clk)
        if int(dut.current_floor[lift].value) == floor and int(dut.idle[lift].value) == 1:
            return
    raise TimeoutError(f"Lift {lift} did not reach floor {floor}")


@cocotb.test()
async def test_hold_beep_blocks_same_floor_hall_dispatch(dut):
    """Bug 3: a lift in hold_beep must not take a hall call at its current floor."""
    await start_clock(dut)
    await reset_group(dut)

    await cabin_call(dut, 0, 5)
    await wait_lift_floor(dut, 0, 5)

    dut.door_obstructed.value = 1 << 0
    for _ in range(GROUP_BUILD_PARAMS["HOLD_BEEP_CYCLES"] + 6):
        await RisingEdge(dut.clk)
        if int(dut.hold_beep[0].value) == 1:
            break
    assert int(dut.hold_beep[0].value) == 1

    await hall_call(dut, 5, up=1)
    for _ in range(20):
        await RisingEdge(dut.clk)

    assert int(dut.last_assigned_lift.value) != 0


def test_bug_reveal_controller_runner():
    sim = os.getenv("SIM", "icarus")
    runner = get_runner(sim)
    runner.build(
        sources=CTRL_SOURCES,
        hdl_toplevel="elevator_controller",
        parameters=BUILD_PARAMS,
        always=True,
    )
    runner.test(
        hdl_toplevel="elevator_controller",
        test_module=Path(__file__).stem,
        parameters=BUILD_PARAMS,
        test_filter="test_ground_floor_served_on_downward_sweep|test_transit_hall_stops_while_moving_down",
    )


def test_bug_reveal_group_runner():
    sim = os.getenv("SIM", "icarus")
    runner = get_runner(sim)
    runner.build(
        sources=GROUP_SOURCES,
        hdl_toplevel="elevator_group",
        parameters=GROUP_BUILD_PARAMS,
        always=True,
    )
    runner.test(
        hdl_toplevel="elevator_group",
        test_module=Path(__file__).stem,
        parameters=GROUP_BUILD_PARAMS,
        test_filter="test_hold_beep_blocks_same_floor_hall_dispatch",
    )
