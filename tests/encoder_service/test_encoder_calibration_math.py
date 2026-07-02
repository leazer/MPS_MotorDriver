def raw_delta(prev, now):
    value = (now - prev) & 0xFFFF
    if value >= 0x8000:
        value -= 0x10000
    return value


def unwrap(prev_unwrapped, prev_raw, now_raw):
    return prev_unwrapped + raw_delta(prev_raw, now_raw)


def test_raw_delta_wrap_forward():
    assert raw_delta(65530, 10) == 16


def test_raw_delta_wrap_reverse():
    assert raw_delta(10, 65530) == -16


def test_unwrap_forward_across_zero():
    assert unwrap(65530, 65530, 10) == 65546


def test_spike_threshold_rejects_large_jump():
    assert abs(raw_delta(1000, 50000)) > 1024


def build_simple_error_table(samples):
    first = samples[0]
    last = samples[-1]
    span = last - first
    bins = [[] for _ in range(256)]
    for i, raw in enumerate(samples):
        ref = first + (span * i) // (len(samples) - 1)
        err = raw - ref
        bins[(raw & 0xFFFF) >> 8].append(err)
    table = [0] * 256
    for idx, values in enumerate(bins):
        if values:
            table[idx] = ((sum(values) // len(values)) * 360000) // 65536
    return table


def test_error_table_zero_for_linear_ramp():
    samples = [i * 256 for i in range(256)]
    table = build_simple_error_table(samples)
    assert max(abs(v) for v in table) <= 6


if __name__ == "__main__":
    test_raw_delta_wrap_forward()
    test_raw_delta_wrap_reverse()
    test_unwrap_forward_across_zero()
    test_spike_threshold_rejects_large_jump()
    test_error_table_zero_for_linear_ramp()
    print("encoder math tests passed")
