"""Deterministic metrics for saved X-Track/Node 1 position-response logs."""

import argparse
import json
import statistics
from collections import namedtuple


PositionResponseMetrics = namedtuple(
    "PositionResponseMetrics",
    (
        "command_span_deg",
        "measured_span_deg",
        "max_settle_ms",
        "unsettled_endpoints",
        "max_overshoot_deg",
        "peak_abs_iq_a",
        "max_sync_age_ms",
        "protocol_error_delta",
        "rx_overflow_delta",
        "bus_off_delta",
        "tx_error_delta",
    ),
)

_SAMPLE_KEYS = (
    "timestamp_ms",
    "target_mdeg",
    "measured_mdeg",
    "velocity_mdeg_s",
    "iqref_ma",
    "protocol_errors",
    "rx_overflows",
    "bus_off_events",
    "tx_errors",
    "fault",
    "sync_age_ms",
    "active",
    "session",
    "can_state",
)
_COUNTERS = (
    "protocol_errors",
    "rx_overflows",
    "bus_off_events",
    "tx_errors",
)


def _checked_samples(records):
    if not isinstance(records, (list, tuple)) or len(records) < 2:
        raise ValueError("records must contain samples and a final safe state")
    final = records[-1]
    if not isinstance(final, dict) or final.get("type") != "final":
        raise ValueError("missing final safe-state record")
    if (final.get("state") != "DISABLED" or final.get("fault") != 0 or
            final.get("enable") != 0 or
            (final.get("ccr1"), final.get("ccr2"), final.get("ccr3")) !=
            (2812, 2812, 2812)):
        raise ValueError("final state is not DISABLED/EN-low/neutral PWM")

    samples = []
    previous_timestamp = None
    for record in records[:-1]:
        if not isinstance(record, dict) or record.get("type") != "sample":
            raise ValueError("invalid sample record")
        if any(key not in record for key in _SAMPLE_KEYS):
            raise ValueError("truncated telemetry sample")
        if any(isinstance(record[key], bool) or
               not isinstance(record[key], int) for key in _SAMPLE_KEYS):
            raise ValueError("telemetry fields must be integers")
        timestamp = record["timestamp_ms"]
        if previous_timestamp is not None and timestamp <= previous_timestamp:
            raise ValueError("timestamps must increase")
        if record["fault"] != 0:
            raise ValueError("faulted telemetry sample")
        previous_timestamp = timestamp
        if record["active"] not in (0, 1):
            raise ValueError("invalid position active flag")
        if record["active"] == 1:
            samples.append(record)
    if not samples:
        raise ValueError("no telemetry samples")
    for counter in _COUNTERS:
        if samples[-1][counter] != samples[0][counter]:
            raise ValueError("CAN error counter changed: " + counter)
    return samples


def _endpoint_holds(samples):
    holds = []
    index = 0
    while index < len(samples):
        session = samples[index]["session"]
        session_begin = index
        while index < len(samples) and samples[index]["session"] == session:
            index += 1
        session_end = index
        direction = 0
        cursor = session_begin
        while cursor < session_end:
            velocity = samples[cursor]["velocity_mdeg_s"]
            if velocity != 0:
                direction = 1 if velocity > 0 else -1
                cursor += 1
                continue
            begin = cursor
            while (cursor < session_end and
                   samples[cursor]["velocity_mdeg_s"] == 0):
                cursor += 1
            if direction != 0:
                holds.append((direction, begin, cursor, session_begin))
                direction = 0
    if (not any(item[0] > 0 for item in holds) or
            not any(item[0] < 0 for item in holds)):
        raise ValueError("missing positive/negative reversal holds")
    return holds


def _settle_ms(samples, begin, end):
    target = samples[begin]["target_mdeg"]
    for index in range(begin, end):
        if all(abs(samples[tail]["measured_mdeg"] - target) <= 500
               for tail in range(index, end)):
            return samples[index]["timestamp_ms"] - samples[begin]["timestamp_ms"]
    return None


def summarize(records):
    samples = _checked_samples(records)
    holds = _endpoint_holds(samples)
    endpoints = []
    plateaus = []
    settle_times = []
    overshoots = []
    for direction, begin, end, movement_begin in holds:
        endpoint = samples[begin]["target_mdeg"]
        endpoints.append(endpoint)
        settle_times.append(_settle_ms(samples, begin, end))
        terminal = [item["measured_mdeg"]
                    for item in samples[max(begin, end - 3):end]]
        plateaus.append(float(statistics.median(terminal)))
        measured = [item["measured_mdeg"]
                    for item in samples[movement_begin:end]]
        if direction > 0:
            overshoots.append(max(0, max(measured) - endpoint))
        else:
            overshoots.append(max(0, endpoint - min(measured)))

    deltas = {
        counter: samples[-1][counter] - samples[0][counter]
        for counter in _COUNTERS
    }
    return PositionResponseMetrics(
        command_span_deg=round((max(endpoints) - min(endpoints)) / 1000.0, 3),
        measured_span_deg=round((max(plateaus) - min(plateaus)) / 1000.0, 3),
        max_settle_ms=(max(value for value in settle_times
                           if value is not None)
                       if any(value is not None for value in settle_times)
                       else None),
        unsettled_endpoints=sum(value is None for value in settle_times),
        max_overshoot_deg=round(max(overshoots) / 1000.0, 3),
        peak_abs_iq_a=round(max(abs(item["iqref_ma"])
                                for item in samples) / 1000.0, 3),
        max_sync_age_ms=max(item["sync_age_ms"] for item in samples
                            if item["can_state"] == 3),
        protocol_error_delta=deltas["protocol_errors"],
        rx_overflow_delta=deltas["rx_overflows"],
        bus_off_delta=deltas["bus_off_events"],
        tx_error_delta=deltas["tx_errors"],
    )


def load_jsonl(path):
    with open(path, "r", encoding="utf-8") as stream:
        return [json.loads(line) for line in stream if line.strip()]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("log")
    args = parser.parse_args()
    print(json.dumps(summarize(load_jsonl(args.log))._asdict(),
                     sort_keys=True))


if __name__ == "__main__":
    main()
