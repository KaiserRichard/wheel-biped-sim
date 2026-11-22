"""Headless contact physics scenarios. Rendering is imported only with --perception."""

import argparse
import csv
from pathlib import Path

import mujoco
import numpy as np

from .model import Plant, BOX_FACE, BOX_DEPTH, BOX_HEIGHT, R_WHEEL, DT
from .control import Controller

TRIGGER_GAP = 0.25  # ASSUMED axle-to-front-face distance at crouch request, m.
FIELDS = ('t', 'scenario', 'seed', 'x', 'z', 'pitch', 'vx', 'pitch_rate', 'l0', 'phase',
          'flight', 'perception', 'physics_warning', 'wheel_torque', 'hip_torque', 'leg_force', 'contact_left', 'contact_right',
          'base_ground', 'bad_box', 'wheel_box', 'wheel_left_x', 'wheel_right_x',
          'wheel_left_bottom', 'wheel_right_bottom', 'box_face', 'box_depth', 'box_height',
          'raw_advisory', 'gated_advisory', 'raw_confidence', 'gated_confidence', 'gate_phase')


class Perception:
    """Actual dynamic camera pose, detector and JumpGate; advisory-only output."""
    def __init__(self, plant, renderer=None):
        from tools.simcam.scenario import WIDTH, HEIGHT, intrinsics
        from tools.simcam.depth import clipping_distances
        from tools.jump_gate.gate import JumpGate
        self.renderer = renderer if renderer is not None else mujoco.Renderer(plant.model, height=HEIGHT, width=WIDTH)
        self.K, self.gate = intrinsics(), JumpGate()
        self.near, self.far = clipping_distances(plant.model.vis.map.znear, plant.model.vis.map.zfar, plant.model.stat.extent)
        self.detector, self.decision, self.previous = {}, {}, None
        self.was_airborne = False
        self.frame, self.result, self.stamp = None, None, None
        self.next_frame = 0.0

    def update(self, plant, data, phase, contact):
        from tools.simcam.run import update_camera
        from tools.simcam.depth import optical_depth
        from tools.obstacle_decision import detect, decide
        from tools.jump_gate.gate import Tick
        airborne = not (contact['left'] or contact['right'])
        if (phase == 'CROUCH' and self.previous != 'CROUCH') or (self.was_airborne and not airborne):
            self.detector, self.decision = {}, {}
            self.frame, self.result = None, None
        if data.time+1e-9 >= self.next_frame:
            update_camera(self.renderer, data, self.K, self.near)
            self.renderer.enable_depth_rendering()
            depth = optical_depth(self.renderer.render(), self.near, self.far)
            self.frame, self.detector = detect.process_frame(depth, self.K, state=self.detector)
            self.result, self.decision = decide.update(self.frame, self.decision, forward_speed_m_s=float(plant.state(data)[3]))
            self.stamp = float(data.time)
            self.next_frame += 0.1
        frame, decision = self.frame, self.result
        raw = decision.advisory if decision else decide.Advisory.UNCERTAIN
        confidence = decision.confidence if decision else 0.0
        status = dict(CROUCH=0, THRUST=1, RETRACT=2).get(phase, 3)
        gated = self.gate.update(Tick(float(data.time), status, not contact['left'], not contact['right'],
                                      0.0, float(np.degrees(plant.state(data)[5])), 0.0,
                                      bool(frame and frame.plane_valid), raw, confidence,
                                      plane_stamp_s=self.stamp))
        self.previous, self.was_airborne = phase, airborne
        return dict(raw_advisory=raw.name, gated_advisory=gated.advisory.name,
                    raw_confidence=confidence, gated_confidence=gated.confidence,
                    gate_phase=gated.phase.value)


def run(scenario, seed=0, out='docs/twin3d/runs', perception=False):
    rng = np.random.default_rng(seed)
    box_face = BOX_FACE + rng.uniform(-0.015, 0.015)
    plant = Plant(box_face if scenario == 'jump_box' else None)
    data = plant.data(pitch=rng.uniform(-0.02, 0.02))
    controller = Controller()
    duration = dict(stand=30.0, drive=6.0, jump_box=7.0)[scenario]
    camera = Perception(plant) if perception else None
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    path = out / f'{scenario}-{seed}.csv'
    with path.open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=FIELDS)
        writer.writeheader()
        for step in range(round(duration/DT)+1):
            state = plant.state(data)
            speed = 0.0 if scenario == 'stand' or data.time < 0.5 else 0.3
            if scenario == 'jump_box' and data.time >= 0.5:
                speed = 0.9
            trigger = scenario == 'jump_box' and box_face-state[0] <= TRIGGER_GAP
            flight = controller.update(plant, data, speed, trigger)
            contact = plant.contacts(data)
            row = dict(t=data.time, scenario=scenario, seed=seed, x=state[0], z=data.xpos[plant.model.body('base').id, 2],
                       pitch=state[2], vx=state[3], pitch_rate=state[5], l0=plant.length(data), phase=controller.jump.phase,
                       flight=int(flight), perception=int(perception), physics_warning=sum(int(w.number) for w in data.warning),
                       wheel_torque=data.ctrl[plant.u['wheel_left']]+data.ctrl[plant.u['wheel_right']],
                       hip_torque=data.ctrl[plant.u['hip_left']]+data.ctrl[plant.u['hip_right']],
                       leg_force=data.ctrl[plant.u['length_left']]+data.ctrl[plant.u['length_right']],
                       contact_left=int(contact['left']), contact_right=int(contact['right']),
                       base_ground=int(contact['base_ground']), bad_box=int(contact['bad_box']), wheel_box=int(contact['wheel_box']),
                       box_face=box_face if scenario == 'jump_box' else '', box_depth=BOX_DEPTH, box_height=BOX_HEIGHT)
            for side in ('left', 'right'):
                pos = data.xpos[plant.model.body('wheel_'+side).id]
                row['wheel_'+side+'_x'], row['wheel_'+side+'_bottom'] = pos[0], pos[2]-R_WHEEL
            if camera:
                row.update(camera.update(plant, data, controller.jump.phase, contact))
            writer.writerow(row)
            if abs(state[2]) >= np.pi/3 or not np.all(np.isfinite(data.qpos)):
                break
            if step < round(duration/DT):
                mujoco.mj_step(plant.model, data)
                # mj_step integrates qpos after its position/contact pipeline.
                # Refresh so camera, contacts and CSV all describe this timestamp.
                mujoco.mj_forward(plant.model, data)
    if camera:
        camera.renderer.close()
    return path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scenario', choices=('stand', 'drive', 'jump_box', 'all'), default='all')
    parser.add_argument('--seeds', type=int, default=10)
    parser.add_argument('--out', default='docs/twin3d/runs')
    parser.add_argument('--perception', action='store_true')
    args = parser.parse_args()
    for scenario in (('stand', 'drive', 'jump_box') if args.scenario == 'all' else (args.scenario,)):
        for seed in range(args.seeds):
            path = run(scenario, seed, args.out, args.perception)
            print(path, flush=True)


if __name__ == '__main__':
    main()
