import csv
import json
import tempfile
import unittest
from pathlib import Path

from mcap.reader import make_reader

from tools.bench_twin_foxglove.make_mcap import convert


class MakeMcapTest(unittest.TestCase):
    def test_tiny_inputs_round_trip_and_layout_topic_coverage(self):
        with tempfile.TemporaryDirectory() as td:
            d=Path(td); packets=d/'packets.csv'; imu=d/'imu.csv'; bus=d/'bus.log'; fb=d/'fb.csv'; out=d/'out.mcap'
            with packets.open('w',newline='') as f:
                w=csv.writer(f); w.writerow(['seq','advisory','height_m','distance_m','frame_stamp_ns'])
                for i in range(4): w.writerow([i,0,.1,1.0,1_800_000_000_000_000_000+i*500_000_000])
            with imu.open('w',newline='') as f:
                w=csv.writer(f); w.writerow(['t_s','ax_g','ay_g','az_g','gx_dps','gy_dps','gz_dps'])
                for i in range(100): w.writerow([i*.01,0,0,1,0,0,0])
            bus.write_text('STAT ms=0 c1_busy=0 c1_txok=0 c1_txerr=0 c1_esr=0x00000000 c2_rx=0 c2_busy=0 c2_esr=0x00000000\nSTAT ms=1000 c1_busy=3 c1_txok=100 c1_txerr=2 c1_esr=0x00000000 c2_rx=100 c2_busy=4 c2_esr=0x00000000\n')
            fb.write_text('elapsed_s,ros_stamp_s,frame_id,left_position_rad,right_position_rad,left_velocity_rad_s,right_velocity_rad_s,left_effort_nm,right_effort_nm\n')
            result=convert(out,packets=packets,imu=imu,imu_offset=0.0,bus=bus,feedback=fb,jump_at=.2)
            with out.open('rb') as f:
                reader=make_reader(f); summary=reader.get_summary()
                schemas={s.id:s.name for s in summary.schemas.values()}
                counts={}
                stamps={}
                for _,channel,message in reader.iter_messages():
                    counts[channel.topic]=counts.get(channel.topic,0)+1
                    stamps.setdefault(channel.topic,[]).append(message.log_time)
                topics=set(counts)
                self.assertEqual(topics,set(result['counts']))
                self.assertEqual(schemas[summary.channels[next(c.id for c in summary.channels.values() if c.topic=='/body/imu')].schema_id],'wbr.bench.Imu')
                self.assertEqual(counts['/brain/advisory'],4)
                self.assertEqual(counts['/body/imu'],100)
                self.assertEqual(stamps['/brain/advisory'][0],1_800_000_000_000_000_000)
                self.assertEqual(stamps['/bus/stats'][-1],1_800_000_001_000_000_000)
            layout=json.loads((Path(__file__).resolve().parents[3]/'docs/bench_twin_foxglove/layout.json').read_text())
            refs=set()
            for cid,cfg in layout['configById'].items():
                refs.update(cfg.get('topics',{}))
                for key in ('topicPath',):
                    if key in cfg: refs.add(cfg[key])
                for path in cfg.get('paths',[]): refs.add(path['value'].split('.')[0])
                image=cfg.get('imageMode',{}).get('imageTopic')
                if image: refs.add(image)
            self.assertTrue(refs <= topics | {'/camera/camera/color/image_raw','/twin/image'})


if __name__=='__main__': unittest.main()
