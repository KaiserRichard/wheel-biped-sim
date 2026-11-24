def test_c_estimator_startup_packet_order_drift_gates_and_bounds(run_c_harness):
    run_c_harness("s10_estimator", includes=("estimation/s10",),
                  sources=("estimation/s10/s10_estimator.c",))
