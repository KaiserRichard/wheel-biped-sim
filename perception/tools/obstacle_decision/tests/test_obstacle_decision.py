import unittest, numpy as np
from tools.ground_plane.synth import render,Box
from tools.obstacle_decision.detect import process_frame,Params
from tools.obstacle_decision.decide import update,DecisionParams,Advisory
from tools.obstacle_decision.advisory_codec import encode,decode

K=np.array([[386.953,0,321.034],[0,386.953,241.040],[0,0,1.]])
def sequence(h=None,d=1.,pitch=0.,x=0.):
    boxes=() if h is None else (Box((x,d+.06,h/2),(.30,.12,h)),)
    dep,_,_=render(height=.30,pitch=pitch,boxes=boxes,sigma_d=.1)
    ps=Params(); st={}; ds={}; result=None
    for _ in range(5):
        fr,st=process_frame(dep,K,ps,st); result,ds=update(fr,ds,DecisionParams())
    return fr,result

class TestObstacleDecision(unittest.TestCase):
    def test_box_and_pitch(self):
        for pitch in (0.,3.):
            fr,dec=sequence(.05,pitch=pitch)
            self.assertIsNotNone(fr.nearest)
            self.assertAlmostEqual(fr.nearest.height_m,.05,delta=.01)
            self.assertAlmostEqual(fr.nearest.distance_m,1.,delta=.05)
            self.assertEqual(dec.advisory,Advisory.JUMP_CANDIDATE)
    def test_tall_box(self): self.assertEqual(sequence(.20)[1].advisory,Advisory.STOP_CANDIDATE)
    def test_empty_and_outside(self):
        self.assertEqual(sequence()[1].advisory,Advisory.NONE)
        self.assertEqual(sequence(.05,x=.7)[1].advisory,Advisory.NONE)
    def test_blip_not_confirmed(self):
        from types import SimpleNamespace
        state={}; params=DecisionParams(); out=None
        for exists in (True,False,False):
            f=SimpleNamespace(nearest=SimpleNamespace(distance_m=1.,height_m=.05,width_m=.3) if exists else None,plane_valid=True)
            out,state=update(f,state,params)
        self.assertNotEqual(out.advisory,Advisory.JUMP_CANDIDATE)
    def test_height_hysteresis_around_jump_max(self):
        from types import SimpleNamespace
        params=DecisionParams(confirm_n=1,confirm_m=1,ema_alpha=1.)
        state={}; outputs=[]
        for height in (.115,.125,.115,.125,.115,.125):
            frame=SimpleNamespace(nearest=SimpleNamespace(distance_m=1.,height_m=height,width_m=.3),plane_valid=True)
            decision,state=update(frame,state,params); outputs.append(decision.advisory)
        self.assertEqual(outputs,[Advisory.JUMP_CANDIDATE]*len(outputs))
        # A confirmed jump changes class above 12 cm + 1 cm, then remains STOP
        # while readings oscillate inside the hysteresis band.
        for height,expected in ((.131,Advisory.STOP_CANDIDATE),(.125,Advisory.STOP_CANDIDATE),
                                (.109,Advisory.JUMP_CANDIDATE),(.115,Advisory.JUMP_CANDIDATE)):
            frame=SimpleNamespace(nearest=SimpleNamespace(distance_m=1.,height_m=height,width_m=.3),plane_valid=True)
            decision,state=update(frame,state,params)
            self.assertEqual(decision.advisory,expected)
    def test_height_hysteresis_at_jump_min(self):
        from types import SimpleNamespace
        params=DecisionParams(confirm_n=1,confirm_m=1,ema_alpha=1.)
        state={}; outputs=[]
        for height in (.025,.035,.025,.035,.025):
            frame=SimpleNamespace(nearest=SimpleNamespace(distance_m=1.,height_m=height,width_m=.3),plane_valid=True)
            decision,state=update(frame,state,params); outputs.append(decision.advisory)
        # UNCERTAIN also means 'not confirmed yet', so the first confirmed frame decides without a margin;
        # after that the 1 cm hysteresis must hold JUMP (no flicker back to UNCERTAIN).
        self.assertEqual(outputs,[Advisory.UNCERTAIN]+[Advisory.JUMP_CANDIDATE]*(len(outputs)-1))
        frame=SimpleNamespace(nearest=SimpleNamespace(distance_m=1.,height_m=.045,width_m=.3),plane_valid=True)
        decision,state=update(frame,state,params)
        self.assertEqual(decision.advisory,Advisory.JUMP_CANDIDATE)
        frame=SimpleNamespace(nearest=SimpleNamespace(distance_m=1.,height_m=.025,width_m=.3),plane_valid=True)
        decision,state=update(frame,state,params)
        self.assertEqual(decision.advisory,Advisory.JUMP_CANDIDATE)
        frame=SimpleNamespace(nearest=SimpleNamespace(distance_m=1.,height_m=.019,width_m=.3),plane_valid=True)
        decision,state=update(frame,state,params)
        self.assertEqual(decision.advisory,Advisory.UNCERTAIN)
    def test_codec_round_trip(self):
        wire=encode(123,7,pi_monotonic_ns=1000,frame_stamp_ns=900,advisory=1,confidence=.8,
            distance_m=1.2,height_m=.05,width_m=.3,plane_valid=True,plane_pitch_rad=.02,plane_height_m=.30)
        got=decode(wire)
        self.assertEqual((got['session'],got['sequence'],got['advisory']),(123,7,1))
        self.assertAlmostEqual(got['height_m'],.05,places=6)
        self.assertAlmostEqual(got['plane_height_m'],.30,places=6)

if __name__=='__main__': unittest.main()
