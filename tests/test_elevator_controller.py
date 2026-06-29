"""Cocotb tests for the single-car elevator_controller module."""

from __future__ import annotations

import os
from pathlib import Path

import cocotb
from cocotb.triggers import RisingEdge
from cocotb_tools.runner import get_runner

from elevator_utils import (
    TOP_FLOOR,
    apply_limits,
    pulse_request,
    reset_controller,
    start_clock,
    wait_cycles,
    wait_for_floor,
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


async def wait_serve_at(dut, floor: int) -> None:
    """Wait until the car is idle at floor with that request cleared."""
    await wait_for_floor(dut, floor)
    assert int(dut.pending_requests.value) & (1 << floor) == 0


@cocotb.test()
async def test_reset_starts_at_floor_zero(dut):
    """After reset the car is idle at floor 0 with no pending requests."""
    await start_clock(dut)
    await reset_controller(dut)

    assert int(dut.current_floor.value) == 0
    assert int(dut.idle.value) == 1
    assert int(dut.pending_requests.value) == 0
    assert int(dut.transit_hall_pending.value) == 0
    assert int(dut.door_open.value) == 0
    assert int(dut.estop_latched.value) == 0
    assert int(dut.hold_beep.value) == 0
    assert int(dut.estop_beep.value) == 0


@cocotb.test()
async def test_serves_single_cabin_request(dut):
    """A cabin request to floor 5 is accepted and eventually served."""
    await start_clock(dut)
    await reset_controller(dut)

    await pulse_request(dut, 5)
    assert int(dut.pending_requests.value) & (1 << 5)

    await wait_serve_at(dut, 5)


@cocotb.test()
async def test_bounds_track_enqueued_requests(dut):
    """max_request/min_request reflect the pending cabin-call span."""
    await start_clock(dut)
    await reset_controller(dut)

    await pulse_request(dut, 3)
    await RisingEdge(dut.clk)
    apply_limits(dut)
    assert int(dut.max_request.value) == 3
    assert int(dut.min_request.value) == 3

    await pulse_request(dut, 12)
    await RisingEdge(dut.clk)
    apply_limits(dut)
    assert int(dut.max_request.value) == 12
    assert int(dut.min_request.value) == 3


@cocotb.test()
async def test_multiple_requests_served_in_order(dut):
    """Each cabin stop in the sweep sequence is served before the final floor."""
    await start_clock(dut)
    await reset_controller(dut)

    await pulse_request(dut, 2)
    await wait_serve_at(dut, 2)

    await pulse_request(dut, 7)
    await wait_serve_at(dut, 7)

    await pulse_request(dut, 5)
    await wait_serve_at(dut, 5)

    await pulse_request(dut, TOP_FLOOR)
    await wait_serve_at(dut, TOP_FLOOR)
    assert int(dut.current_floor.value) == TOP_FLOOR


@cocotb.test()
async def test_emergency_stop_holds_position(dut):
    """Emergency stop forces idle with doors closed; service resumes after release."""
    await start_clock(dut)
    await reset_controller(dut)

    await pulse_request(dut, 9)
    await wait_cycles(dut, 3)

    dut.emergency_stop.value = 1
    await wait_cycles(dut, 4)

    assert int(dut.idle.value) == 1
    assert int(dut.moving_up.value) == 0
    assert int(dut.moving_down.value) == 0
    assert int(dut.door_open.value) == 0
    assert int(dut.estop_beep.value) == 1

    dut.emergency_stop.value = 0
    await wait_cycles(dut, 2)

    await pulse_request(dut, 4)
    await wait_serve_at(dut, 4)


@cocotb.test()
async def test_hall_up_call_while_idle(dut):
    """Hall up-call from floor 0 to floor 8 is registered and served."""
    await start_clock(dut)
    await reset_controller(dut)

    await pulse_request(dut, 8, cabin=0, hall_up=1)
    await wait_serve_at(dut, 8)
    await wait_until_no_requests(dut)


@cocotb.test()
async def test_hall_down_call_while_idle(dut):
    """Hall down-call is accepted while the car is idle at the ground floor."""
    await start_clock(dut)
    await reset_controller(dut)

    await pulse_request(dut, 6, cabin=0, hall_up=0)
    await wait_serve_at(dut, 6)
    await wait_until_no_requests(dut)


@cocotb.test()
async def test_serves_lower_request_after_top_sweep(dut):
    """After visiting the top request, a lower pending cabin call is still served."""
    await start_clock(dut)
    await reset_controller(dut)

    await pulse_request(dut, TOP_FLOOR)
    await wait_serve_at(dut, TOP_FLOOR)
    await pulse_request(dut, 4)
    await wait_serve_at(dut, 4)
    await wait_until_no_requests(dut)


@cocotb.test()
async def test_door_opens_on_arrival(dut):
    """Doors open when the car stops at a requested floor."""
    await start_clock(dut)
    await reset_controller(dut)

    await pulse_request(dut, 3)
    for _ in range(400):
        await RisingEdge(dut.clk)
        apply_limits(dut)
        if int(dut.current_floor.value) == 3 and int(dut.door_open.value) == 1:
            return
    raise TimeoutError("Door did not open at floor 3")


@cocotb.test()
async def test_door_obstruction_extends_open_time(dut):
    """Asserting door_obstructed during open reloads the door timer."""
    await start_clock(dut)
    await reset_controller(dut)

    await pulse_request(dut, 2)
    await wait_for_floor(dut, 2)
    assert int(dut.door_open.value) == 1

    dut.door_obstructed.value = 1
    await wait_cycles(dut, BUILD_PARAMS["DOOR_OPEN_CYCLES"] + 2)
    assert int(dut.door_open.value) == 1

    dut.door_obstructed.value = 0
    await wait_cycles(dut, BUILD_PARAMS["DOOR_OPEN_CYCLES"] + 2)
    assert int(dut.door_open.value) == 0


@cocotb.test()
async def test_hold_beep_after_sustained_obstruction(dut):
    """Intentional door hold eventually asserts hold_beep while doors stay open."""
    await start_clock(dut)
    await reset_controller(dut)

    await pulse_request(dut, 1)
    await wait_for_floor(dut, 1)
    assert int(dut.door_open.value) == 1

    dut.door_obstructed.value = 1
    for _ in range(BUILD_PARAMS["HOLD_BEEP_CYCLES"] + 4):
        await RisingEdge(dut.clk)
        apply_limits(dut)
        if int(dut.hold_beep.value) == 1 and int(dut.door_hold_active.value) == 1:
            assert int(dut.door_open.value) == 1
            return
    raise TimeoutError("hold_beep did not assert during sustained obstruction")


def test_elevator_controller_runner():
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
