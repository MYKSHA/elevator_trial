"""Cocotb tests for the six-lift elevator_group top level."""

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


async def wait_lift_floor(dut, lift: int, floor: int, max_cycles: int = 600) -> None:
    for _ in range(max_cycles):
        await RisingEdge(dut.clk)
        if int(dut.current_floor[lift].value) == floor and int(dut.idle[lift].value) == 1:
            return
    raise TimeoutError(f"Lift {lift} did not reach floor {floor}")


@cocotb.test()
async def test_all_lifts_reset_at_floor_zero(dut):
    await start_clock(dut)
    await reset_group(dut)

    for lift in range(NUM_LIFTS):
        assert int(dut.current_floor[lift].value) == 0
        assert int(dut.idle[lift].value) == 1


@cocotb.test()
async def test_hall_call_assigned_and_served(dut):
    """First hall call from floor 0 should dispatch to a lift and reach floor 10."""
    await start_clock(dut)
    await reset_group(dut)

    await hall_call(dut, 10)
    await wait_group_cycles(dut, 5)
    assigned = int(dut.last_assigned_lift.value)
    assert 0 <= assigned < NUM_LIFTS

    await wait_lift_floor(dut, assigned, 10)
    assert int(dut.current_floor[assigned].value) == 10


@cocotb.test()
async def test_round_robin_hall_dispatch(dut):
    """Two hall calls with all lifts idle at floor 0 go to different cars."""
    await start_clock(dut)
    await reset_group(dut)

    await hall_call(dut, 9)
    await wait_group_cycles(dut, 3)
    first = int(dut.last_assigned_lift.value)

    await hall_call(dut, 4)
    await wait_group_cycles(dut, 3)
    second = int(dut.last_assigned_lift.value)

    assert first != second


@cocotb.test()
async def test_cabin_call_routes_to_lift(dut):
    """In-cabin button on lift 2 does not require hall dispatch."""
    await start_clock(dut)
    await reset_group(dut)

    await cabin_call(dut, 2, 11)
    await wait_lift_floor(dut, 2, 11)
    assert int(dut.current_floor[2].value) == 11


@cocotb.test()
async def test_group_scenario_sequence(dut):
    """Combined scenario with hall and cabin traffic across 16 floors."""
    await start_clock(dut)
    await reset_group(dut)

    await hall_call(dut, 10)
    await wait_group_cycles(dut, 25)
    await hall_call(dut, 5)
    await wait_group_cycles(dut, 25)

    await cabin_call(dut, 2, 13)
    await wait_group_cycles(dut, 50)

    await hall_call(dut, TOP_FLOOR)
    await hall_call(dut, 3, up=0)
    await wait_group_cycles(dut, 100)

    assert int(dut.current_floor[2].value) == TOP_FLOOR


@cocotb.test()
async def test_hold_beep_excluded_from_dispatch(dut):
    """A lift sounding hold_beep must not receive new hall assignments."""
    await start_clock(dut)
    await reset_group(dut)

    await cabin_call(dut, 0, 2)
    await wait_lift_floor(dut, 0, 2)

    mask = 1 << 0
    dut.door_obstructed.value = mask
    for _ in range(BUILD_PARAMS["HOLD_BEEP_CYCLES"] + 6):
        await RisingEdge(dut.clk)
        if int(dut.hold_beep[0].value) == 1:
            break
    assert int(dut.hold_beep[0].value) == 1

    await hall_call(dut, 8)
    await wait_group_cycles(dut, 8)
    assert int(dut.last_assigned_lift.value) != 0


def test_elevator_group_runner():
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
