from wbr.analysis.parity import compare


def test_saved_trace_parity():
    result = compare()
    assert set(result) == {"stand", "obstacle"}
    assert result["stand"]["max_abs_error"] <= 1e-7  # impact-free: rounding level on any machine
    assert result["obstacle"]["measured_metrics"]["pass"]  # impacts amplify platform rounding; acceptance rule only
    assert all(row["cad_reference_max_abs_difference"] > 0 for row in result.values())


def test_publish_scan():
    from publish_check import scan

    assert scan() == []
