"""Temporal advisory policy. Advisory is informational, never jump permission."""
from dataclasses import dataclass
from enum import IntEnum
import math

class Advisory(IntEnum):
    NONE=0; JUMP_CANDIDATE=1; STOP_CANDIDATE=2; UNCERTAIN=3

@dataclass
class DecisionParams:
    jump_min_h: float=.03  # ASSUMED, metres
    jump_max_h: float=.12  # ASSUMED, metres
    confirm_n: int=3       # ASSUMED
    confirm_m: int=5       # ASSUMED
    ema_alpha: float=.4    # ASSUMED
    near_m: float=1.5      # ASSUMED
    hysteresis_frames: int=2 # ASSUMED
    height_hysteresis_m: float=.01 # ASSUMED, metres

@dataclass
class Decision:
    advisory: Advisory
    distance_m: float=float('nan')
    height_m: float=float('nan')
    width_m: float=float('nan')
    confidence: float=0.
    time_to_contact_s: float=float('nan')

def update(frame, state=None, params=None, forward_speed_m_s=None):
    p=params or DecisionParams(); s={} if state is None else dict(state)
    hist=list(s.get('hist',[])); ob=frame.nearest
    candidate=ob is not None
    hist=(hist+[candidate])[-p.confirm_m:]; s['hist']=hist
    confirmed=sum(hist)>=p.confirm_n
    if candidate:
        vals=(ob.distance_m,ob.height_m,ob.width_m)
        old=s.get('ema',vals); s['ema']=tuple(p.ema_alpha*a+(1-p.ema_alpha)*b for a,b in zip(vals,old))
    dist,height,width=s.get('ema',(float('nan'),)*3)
    if frame.nearest is None and frame.plane_valid:
        target=Advisory.NONE
    elif not frame.plane_valid:
        target=Advisory.STOP_CANDIDATE if (math.isfinite(dist) and dist<=p.near_m) else Advisory.UNCERTAIN
    elif confirmed:
        prev=s.get('advisory',Advisory.NONE)
        h=p.height_hysteresis_m
        if not math.isfinite(height):
            target=Advisory.UNCERTAIN
        elif prev==Advisory.JUMP_CANDIDATE:
            target=(Advisory.STOP_CANDIDATE if height>p.jump_max_h+h else
                    Advisory.UNCERTAIN if height<p.jump_min_h-h else Advisory.JUMP_CANDIDATE)
        elif prev==Advisory.STOP_CANDIDATE:
            target=(Advisory.JUMP_CANDIDATE if height<p.jump_max_h-h and height>=p.jump_min_h+h else
                    Advisory.UNCERTAIN if height<p.jump_min_h+h else Advisory.STOP_CANDIDATE)
        else:
            # From NONE or UNCERTAIN (which also means "not confirmed yet") decide without a margin.
            target=(Advisory.STOP_CANDIDATE if height>p.jump_max_h else Advisory.JUMP_CANDIDATE if p.jump_min_h<=height<=p.jump_max_h else Advisory.UNCERTAIN)
    else: target=Advisory.UNCERTAIN if candidate else Advisory.NONE
    prev=s.get('advisory',Advisory.NONE); streak=s.get('clear_streak',0)
    if target==Advisory.NONE and prev!=Advisory.NONE:
        streak+=1; target=prev if streak<p.hysteresis_frames else Advisory.NONE
    else: streak=0
    s.update(advisory=target,clear_streak=streak)
    ttc=dist/forward_speed_m_s if forward_speed_m_s and forward_speed_m_s>0 and math.isfinite(dist) else float('nan')
    return Decision(target,dist,height,width,min(1.,sum(hist)/p.confirm_m),ttc),s
