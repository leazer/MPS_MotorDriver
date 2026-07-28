import copy
import importlib.util
from pathlib import Path


MODULE_PATH = Path(__file__).parents[1] / "position_response_metrics.py"


def load_module():
    spec = importlib.util.spec_from_file_location(
        "position_response_metrics", MODULE_PATH
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def sample(timestamp_ms, target_mdeg, measured_mdeg, velocity_mdeg_s,
           iqref_ma=0, protocol_errors=3, rx_overflows=0,
           bus_off_events=0, tx_errors=0, fault=0, sync_age_ms=4,
           active=1, session=1, can_state=3):
    return {
        "type": "sample",
        "timestamp_ms": timestamp_ms,
        "target_mdeg": target_mdeg,
        "measured_mdeg": measured_mdeg,
        "velocity_mdeg_s": velocity_mdeg_s,
        "iqref_ma": iqref_ma,
        "protocol_errors": protocol_errors,
        "rx_overflows": rx_overflows,
        "bus_off_events": bus_off_events,
        "tx_errors": tx_errors,
        "fault": fault,
        "sync_age_ms": sync_age_ms,
        "active": active,
        "session": session,
        "can_state": can_state,
    }


def records():
    values = [
        sample(0, 0, 0, 10000, 20),
        sample(100, 3000, 2200, 10000, 120),
        sample(200, 5000, 5300, 0, 180),
        sample(300, 5000, 4200, 0, 140),
        sample(420, 5000, 4400, 0, 100),
        sample(520, 5000, 4800, 0, 80),
        sample(550, 5000, 4800, 0, 50),
        sample(575, 0, 0, 0, active=0, session=2),
        sample(600, 4000, 4700, -10000, -80),
        sample(800, 0, 1000, -10000, -160),
        sample(1000, -5000, -5300, 0, -180),
        sample(1100, -5000, -4200, 0, -140),
        sample(1220, -5000, -4400, 0, -100),
        sample(1320, -5000, -4800, 0, -80),
        sample(1350, -5000, -4800, 0, -50),
    ]
    values.append({
        "type": "final",
        "state": "DISABLED",
        "fault": 0,
        "enable": 0,
        "ccr1": 2812,
        "ccr2": 2812,
        "ccr3": 2812,
    })
    return values


def expect_value_error(module, values):
    try:
        module.summarize(values)
    except ValueError:
        return
    raise AssertionError("expected ValueError")


def main():
    module = load_module()
    baseline = records()
    metrics = module.summarize(baseline)
    assert metrics.command_span_deg == 10.0, metrics
    assert metrics.measured_span_deg == 9.6, metrics
    assert metrics.max_settle_ms == 320, metrics
    assert metrics.unsettled_endpoints == 0, metrics
    assert metrics.max_overshoot_deg == 0.3, metrics
    assert metrics.peak_abs_iq_a == 0.18, metrics
    assert metrics.discarded_spikes == 0, metrics

    torn_snapshot = copy.deepcopy(baseline)
    torn_snapshot.insert(3, sample(250, 0, 70000, 0, session=1))
    filtered = module.summarize(torn_snapshot)
    assert filtered.max_overshoot_deg == 0.3, filtered
    assert filtered.measured_span_deg == 9.6, filtered
    assert filtered.discarded_spikes == 1, filtered

    missing_reversal = baseline[:6] + [baseline[-1]]
    expect_value_error(module, missing_reversal)

    truncated = copy.deepcopy(baseline)
    del truncated[2]["measured_mdeg"]
    expect_value_error(module, truncated)

    counter_growth = copy.deepcopy(baseline)
    counter_growth[-2]["rx_overflows"] = 1
    expect_value_error(module, counter_growth)

    no_final_disabled = copy.deepcopy(baseline)
    no_final_disabled[-1]["state"] = "RUNNING"
    expect_value_error(module, no_final_disabled)

    print("position response metrics: PASS")


if __name__ == "__main__":
    main()
