"""Bench link test verification on the host. DIAGNOSTIC tooling, not hardware
qualification: the simulator and QEMU check function, not real UART timing."""
import importlib.util
import json
import os
import shutil
import subprocess
import sys
import tempfile
import time
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parents[1]
PROTO = HERE.parent / 'link_proto'
spec = importlib.util.spec_from_file_location('link_bench', HERE / 'pi/link_bench.py')
lb = importlib.util.module_from_spec(spec)
spec.loader.exec_module(lb)


def frozen(name):
    for line in (PROTO / 'tests/vectors.txt').read_text().splitlines():
        if line.startswith(name + ' '):
            return bytes.fromhex(line.split()[1])
    raise KeyError(name)


class LiveFrameHandler(unittest.TestCase):
    def test_range_hysteresis_holds_obstacle_near_limit(self):
        from tools.obstacle_decision.detect import Cluster, FrameResult
        spec=importlib.util.spec_from_file_location('advisory_link_range',HERE/'pi/advisory_link.py')
        al=importlib.util.module_from_spec(spec); spec.loader.exec_module(al)
        state={}
        def filtered(distance):
            cluster=Cluster(distance,.1,.2,.1,20)
            frame=FrameResult(True,False,0,.3,0.,0.,cluster,[cluster])
            return al.in_range(frame,max_m=1.3,range_hysteresis=.1,state=state)
        for distance in (1.28,1.32,1.29,1.32,1.28):
            with self.subTest(distance=distance):
                self.assertIsNotNone(filtered(distance).nearest)
        self.assertIsNone(filtered(1.401).nearest)
        self.assertIsNone(filtered(1.32).nearest)
        self.assertIsNotNone(filtered(1.30).nearest)

    def test_fake_ros_image_flows_through_detector_and_fake_link(self):
        import importlib.util
        import numpy as np
        spec=importlib.util.spec_from_file_location('advisory_link',HERE/'pi/advisory_link.py')
        al=importlib.util.module_from_spec(spec); spec.loader.exec_module(al)
        class FakeLink:
            dropping=True; synced=False
            def send(self,wire,mark=False):
                self.wire=wire
                return time.monotonic_ns()
            def poll(self,wait=0): return []
        class FakeImage:
            height=24; width=32; step=64; encoding='16UC1'; is_bigendian=0
            data=bytes(24*64)
            class Header:
                class Stamp: sec=123; nanosec=456
                stamp=Stamp()
            header=Header()
        fake=FakeLink(); runner=al.Runner(fake)
        receipt=time.monotonic_ns()
        ps,ds,elapsed=al.handle_ros_frame(FakeImage(),
            np.array([[100.,0.,16.],[0.,100.,12.],[0.,0.,1.]]),runner,{}, {},receipt_pi_ns=receipt)
        self.assertIsInstance(ps,dict); self.assertIsInstance(ds,dict)
        self.assertGreaterEqual(elapsed,0); self.assertTrue(fake.wire.endswith(b'\0'))
        row=runner.rows[0]
        self.assertEqual(row['frame_stamp_ns'],123000000456)
        self.assertEqual(row['frame_receipt_pi_ns'],receipt)
        self.assertGreaterEqual(row['receipt_to_before_write_ns'],0)


class Decoder(unittest.TestCase):
    def test_frozen_c_status_vector(self):
        wire = frozen('status')
        rec = lb.decode(wire[:-1])
        self.assertEqual((rec['session'], rec['seq'], rec['sample_mcu_ns'], rec['last_health_seq'],
                          rec['action_mcu_ns'], rec['state'], rec['reason']),
                         (42, 10, 3000000, 8, 2000000, lb.STATES.index('ESTIMATE_VALID'), 0))

    def test_bench_health_round_trip_and_corruption(self):
        b = lb.Bench.__new__(lb.Bench)
        wire = b.health(77)
        self.assertEqual(len(wire), 102)  # SPEC max wire size; no zero bytes inside
        self.assertNotIn(0, wire[:-1])
        self.assertEqual(lb.decode(wire[:-1])['seq'], 77)
        for i in range(len(wire) - 1):
            bad = bytearray(wire[:-1])
            bad[i] ^= 0x80 if bad[i] ^ 0x80 else 0x40
            self.assertIsNone(lb.decode(bytes(bad)), i)

    def test_gpio_marker_ioctl_layout(self):
        """No GPIO chip here: fake the ioctls and check the uAPI v2 request bytes
        (offsets from linux/gpio.h on this host)."""
        calls = []
        def ioctl(fd, req, buf):
            calls.append((req, bytes(buf)))
            if req == lb.GpioMarker.GET_CHIPINFO:
                buf[32:43] = b'pinctrl-rp1'
            elif req == lb.GpioMarker.GET_LINE:
                lb.struct.pack_into('<i', buf, 588, 99)
            return 0
        saved = (lb.glob.glob, lb.os.open, lb.os.close, lb.fcntl.ioctl)
        lb.glob.glob, lb.os.open, lb.os.close, lb.fcntl.ioctl = (
            lambda p: ['/dev/gpiochip0'], lambda *a: 7, lambda fd: None, ioctl)
        try:
            m = lb.GpioMarker(17)
            m.toggle()
        finally:
            lb.glob.glob, lb.os.open, lb.os.close, lb.fcntl.ioctl = saved
        self.assertEqual(m.fd, 99)
        req = calls[1][1]
        self.assertEqual(len(req), 592)
        self.assertEqual(lb.struct.unpack_from('<I', req, 0)[0], 17)
        self.assertEqual(lb.struct.unpack_from('<Q', req, 288)[0], 8)
        self.assertEqual(lb.struct.unpack_from('<I', req, 560)[0], 1)
        self.assertEqual([c[1] for c in calls[2:]], [lb.struct.pack('<QQ', 0, 1), lb.struct.pack('<QQ', 1, 1)])

    def test_percentile_nearest_rank(self):
        v = list(range(1, 101))
        self.assertEqual((lb.percentile(v, 50), lb.percentile(v, 95), lb.percentile(v, 99)), (50, 95, 99))


def run_bench(port, out, *extra):
    cmd = [sys.executable, str(HERE / 'pi/link_bench.py'), '--port', port, '--out', out,
           '--duration', '3', '--fault-trials', '3', '--timeout-repeats', '2', *extra]
    subprocess.run(cmd, check=True, timeout=120, stdout=subprocess.DEVNULL)
    return json.loads(Path(out, 'summary.json').read_text())


class EndToEnd(unittest.TestCase):
    def check_function(self, s, real_clock=True):
        self.assertGreater(s['preflight']['status_records'], 0)
        self.assertEqual(s['preflight']['states'], ['NO_LINK'])
        self.assertEqual(s['rtt']['lost'], 0)
        self.assertEqual(s['rtt']['reply_states'], {'RESET_PENDING_REENABLE': s['rtt']['sent']})
        for v in s['faults'].values():
            self.assertEqual(v['observed_as_expected'], v['trials'])
        # The three deliberately injected faults must all be observed; retain
        # any additional wire errors in the summary rather than masking them.
        self.assertGreaterEqual(s['mcu_rejections_seen'].get('WIRE', 0), 3)
        self.assertEqual(s['mcu_rejections_seen'].get('SEQUENCE'), 3)
        self.assertEqual(s['mcu_rejections_seen'].get('SESSION'), 3)
        if real_clock:  # QEMU's TIM2 does not run at real time, so the 100 ms gap is not short there
            self.assertEqual(s['timeout']['100_ms']['no_link_declared'], 0)
        for v in s['timeout'].values():
            if real_clock:
                self.assertEqual(v['recovered_after_resume'], v['repeats'])
            else:  # QEMU time can jump; require the behaviour, not every trial's timing
                self.assertGreaterEqual(v['recovered_after_resume'], 1)
        if real_clock:
            self.assertEqual(s['timeout']['300_ms']['no_link_declared'], 2)
        else:
            self.assertGreaterEqual(s['timeout']['300_ms']['no_link_declared'], 1)
        self.assertEqual(s['pi_rx_wire_errors'], 0)
        self.assertEqual(s['mcu_reboots_seen'], 0)

    def test_host_simulator(self):
        subprocess.run(['make', '-s', '-C', str(HERE), 'host/sim'], check=True)
        with tempfile.TemporaryDirectory() as d:
            sim = subprocess.Popen([str(HERE / 'host/sim'), os.path.join(d, 'counters.txt')],
                                   stdout=subprocess.PIPE, text=True)
            try:
                port = sim.stdout.readline().strip()
                s = run_bench(port, os.path.join(d, 'out'))
            finally:
                sim.terminate()
                try: sim.wait(timeout=2)
                except subprocess.TimeoutExpired: sim.kill(); sim.wait(timeout=2)
                sim.stdout.close()
            self.check_function(s)
            # Real clock in the simulator: the MCU-side rule is 150 ms of silence.
            mcu = s['timeout']['300_ms']['mcu_silence_ms_accept_to_action']
            self.assertGreaterEqual(mcu['min'], 150.0)
            self.assertLess(mcu['max'], 200.0)
            counters = Path(d, 'counters.txt').read_text()
            wire_errors=int(counters.split('wire_errors=')[1].split()[0])
            self.assertGreaterEqual(wire_errors,3)
            self.assertIn('tx_dropped=0', counters)

    def test_advisory_synthetic_and_faults(self):
        subprocess.run(['make', '-s', '-C', str(HERE), 'host/sim'], check=True)
        with tempfile.TemporaryDirectory() as d:
            counters_path=os.path.join(d,'advisory-counters.txt')
            sim=subprocess.Popen([str(HERE/'host/sim'),counters_path],stdout=subprocess.PIPE,text=True)
            try:
                port=sim.stdout.readline().strip(); out=os.path.join(d,'advisory')
                subprocess.run([sys.executable,str(HERE/'pi/advisory_link.py'),'--port',port,
                    '--out',out,'--synthetic','30','0.5','--faults'],check=True,timeout=30,
                    stdout=subprocess.DEVNULL)
            finally:
                sim.terminate()
                try: sim.wait(timeout=2)
                except subprocess.TimeoutExpired: sim.kill(); sim.wait(timeout=2)
                sim.stdout.close()
            summary=json.loads(Path(out,'summary.json').read_text())
            self.assertGreater(summary['sent'],0)
            # One initial reply can be lost when the host PTY opens concurrently
            # with the simulator's first TX; later replies and sequence recovery are measured.
            self.assertLessEqual(summary['loss'],1)
            self.assertGreater(summary['stale_events'],0)
            self.assertTrue({'WIRE','FIELDS','SESSION','SEQUENCE'} <= set(summary['echo_result_codes']))
            self.assertGreaterEqual(summary['mcu_rejected_count'],4)
            with Path(out,'packets.csv').open(newline='') as f: rows=list(__import__('csv').DictReader(f))
            by_fault={r['fault']:int(r['echo_result']) for r in rows if r['fault'] and r['echo_result']}
            self.assertEqual({k:by_fault[k] for k in ('crc','old','session','nonfinite')},
                {'crc':1,'old':3,'session':2,'nonfinite':4})
            counters=Path(counters_path).read_text()
            self.assertIn('advisory_accepted=',counters)

    @unittest.skipUnless(shutil.which('qemu-system-arm') and shutil.which('arm-none-eabi-gcc'),
                         'needs qemu-system-arm and arm-none-eabi-gcc')
    def test_firmware_binary_in_qemu(self):
        """Same firmware sources (BENCH_EMU skips RCC/UID) on QEMU netduinoplus2 (STM32F405).
        QEMU does not model the F446 clock tree, so only behaviour is checked, not timing."""
        subprocess.run(['make', '-s', '-C', str(HERE / 'firmware'), 'build/link_bench_emu.elf'], check=True)
        q = subprocess.Popen(['qemu-system-arm', '-M', 'netduinoplus2', '-nographic', '-monitor', 'none',
                              '-kernel', str(HERE / 'firmware/build/link_bench_emu.elf'),
                              '-serial', 'pty', '-serial', 'null'],
                             stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        try:
            line = q.stdout.readline()
            port = line.split('redirected to ')[1].split()[0]
            time.sleep(0.5)
            with tempfile.TemporaryDirectory() as d:
                s = run_bench(port, os.path.join(d, 'out'))
        finally:
            q.kill()
            q.wait(timeout=5)
        self.check_function(s, real_clock=False)


if __name__ == '__main__':
    unittest.main()
