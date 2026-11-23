"""Classical CAN standard data-frame load, including 3-bit intermission."""
import argparse
from dm_codec import worst_bits


def load(motors=2, frequency=500, bitrate=1_000_000, dlc=8, replies=1, retries_per_cycle=0):
    if motors<1 or frequency<=0 or bitrate<=0 or replies<0 or retries_per_cycle<0:
        raise ValueError('positive motors/rate/bitrate and nonnegative reply/retry count required')
    plain=47+8*dlc
    worst=worst_bits(dlc)
    return dict(unstuffed_pct=100*motors*(1+replies)*frequency*plain/bitrate,
                worst_pct=100*(motors*(1+replies)*worst+retries_per_cycle*(31+worst))*frequency/bitrate)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--motors',type=int,default=2);p.add_argument('--frequency',type=float,default=500)
    p.add_argument('--bitrate',type=int,default=1_000_000);p.add_argument('--dlc',type=int,default=8)
    p.add_argument('--replies',type=int,default=1);p.add_argument('--retries-per-cycle',type=int,default=0)
    print(load(**vars(p.parse_args())))
