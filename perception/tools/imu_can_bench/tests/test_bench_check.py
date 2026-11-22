import pathlib
import sys
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "pi"))
import bench_check as bc  # noqa: E402

BOOT = [
    "BOOT fw=imu_can_bench ver=1 sysclk=16000000 can_bps=500000 imu_hz=100",
    "PIN c1rx_idle=1 c2rx_idle=1 c1tx_c1rx=0 c1tx_c2rx=0 c2tx_c2rx=0 c2tx_c1rx=0 scl=1 sda=1",
    "IMUID addr=0x68 who=0x68 init=1",
    "LOOP c1_tx=10 c1_rx=10 c2_tx=10 c2_rx=10",
    "NORM c1_init=1 c2_init=1",
]


def stat(ms, match, esr1=0, boff=0, imu_err=0):
    sent = ms // 10
    return (f"STAT ms={ms} imu_ok=0 imu_err={imu_err} imu_stale=0 imu_reinit=0 imu_up=1 c1_req={sent} c1_busy=0 "
            f"c1_txok={sent} c1_txerr=0 c1_rx={match} c1_ovr=0 c1_other=0 c1_boff_n={boff} c1_esr={esr1:#010x} "
            f"c2_rx={sent} c2_req={sent} c2_busy=0 c2_txok={match} c2_txerr=0 c2_ovr=0 c2_other=0 c2_boff_n=0 "
            f"c2_esr=0x00000000 sent={sent} match={match} bad=0 uart_drop=0")


def imu_lines(n, az=8192, gx=-212):
    return [f"I,{1000 + 10 * i},1,-430,-95,{az},-1500,{gx},1,-12" for i in range(n)]


def verdicts(lines, **kw):
    res, m = bc.check(*bc.parse(lines), **kw)
    return {name: v for name, v, _ in res}, m


class BenchCheck(unittest.TestCase):
    def test_healthy_bench_passes(self):
        v, m = verdicts(["garbage partial li"] + BOOT + imu_lines(1000) + [stat(1000, 100), stat(11000, 1100)])
        self.assertTrue(all(x == "PASS" for x in v.values()), v)
        self.assertAlmostEqual(m["imu_rate_hz"], 100.0, places=1)
        self.assertAlmostEqual(m["gyro_bias_dps"][0], -212 / 65.5, places=2)

    def test_no_ack_fails_can(self):
        esr = (128 << 16) | (3 << 4) | 2  # TEC 128, LEC ack error, error passive
        v, _ = verdicts(BOOT + imu_lines(200) + [stat(1000, 0), stat(11000, 0, esr1=esr)])
        self.assertEqual(v["can_round_trip"], "FAIL")
        self.assertEqual(v["imu_rate"], "PASS")

    def test_expect_fail_mode_passes_on_failure(self):
        v, _ = verdicts(BOOT + imu_lines(200) + [stat(1000, 100), stat(11000, 150, boff=2)], expect_can_fail=True)
        self.assertEqual(v["can_fault_detected"], "PASS")

    def test_wrong_who_and_tilt(self):
        lines = [x.replace("who=0x68", "who=0x72") for x in BOOT] + imu_lines(200, az=4096)
        v, _ = verdicts(lines + [stat(1000, 100), stat(3000, 300)])
        self.assertEqual(v["imu_who_am_i"], "FAIL")
        self.assertEqual(v["imu_accel_1g"], "FAIL")

    def test_floating_rx_pin_reported(self):
        lines = [x.replace("c2rx_idle=1", "c2rx_idle=0") for x in BOOT]
        v, _ = verdicts(lines + imu_lines(200) + [stat(1000, 100), stat(3000, 300)])
        self.assertEqual(v["pins"], "FAIL")

    def test_esr_decode(self):
        self.assertEqual(bc.esr((5 << 24) | (130 << 16) | (3 << 4) | 4 | 2),
                         {"tec": 130, "rec": 5, "lec": 3, "boff": 1, "epv": 1})


if __name__ == "__main__":
    unittest.main()
