def test_c_stop_position_reanchors_clamps_and_rearms(run_c_harness):
    run_c_harness("s8_stop_position", includes=("wbr/control/stop",))
