def test_c_wheel_loss_signed_and_independent_terms(run_c_harness):
    run_c_harness("s7_wheel_loss", includes=("wbr/control/wheel_loss",))
