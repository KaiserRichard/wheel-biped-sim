"""One setup path for design and assembly experiments."""
import json
from pathlib import Path
from .runner import Runner
from .plant import Sim
from .assembly import AssemblySim
from .assembly_sensors import AssemblySensors
from .sensors import Sensors
from . import scenarios
from wbr.dynamics import model, assembly
from wbr.control.lqr import design_gains
ROOT=Path(__file__).resolve().parents[2]

def experiment(behaviour='stand', plant='design', seed=0, delay_steps=0,
               delay_mode=None, phase_schedule=True, stop_on_fall=True):
    if plant not in ('design','assembly'):
        raise ValueError(plant)
    p=json.loads((ROOT/'models'/f'{plant}.json').read_text())
    backend=assembly if plant=='assembly' else model
    coef=assembly.coefficients() if plant=='assembly' else design_gains(p)[0]
    runner=Runner(plant=AssemblySim if plant=='assembly' else Sim,
                  backend=backend, initial_length=.14 if plant=='assembly' else .18,
                  sensor_class=AssemblySensors if plant=='assembly' else Sensors,
                  controller_options=dict(delay_mode=delay_steps>0 if delay_mode is None else delay_mode,
                     phase_schedule=phase_schedule,step_face_x=1. if behaviour=='obstacle' else None,
                     compact_jump=plant=='assembly'))
    metrics,trace,qpos=scenarios.ALL[behaviour](p,coef,runner=runner,seed=seed,
                            delay_steps=delay_steps,stop_on_fall=stop_on_fall)
    return metrics,runner.trace,qpos
