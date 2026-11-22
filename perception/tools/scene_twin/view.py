"""MuJoCo recorded lab-day scene twin (visualisation only)."""
import argparse
import csv
from pathlib import Path
import time
from .replay import Replay, read_packets, bag_start
from .scene import make_scene, apply, camera


def record(a, replay, model, data):
    import mujoco
    from PIL import Image, ImageDraw
    out = Path(a.record)
    out.mkdir(parents=True, exist_ok=True)
    if any(out.iterdir()):
        raise ValueError(f'record directory must be empty: {out}')
    frames = []
    cam = camera(replay)
    with (out/'frames.csv').open('w', newline='') as f, mujoco.Renderer(model, 400, 720) as renderer:
        writer = csv.writer(f)
        writer.writerow(('png', 'playback_t_s', 'frame_stamp_ns', 'origin_relative_s',
                         'origin_status', 'packet_index'))
        count = int(replay.duration/a.speed*a.fps)+1
        for k in range(count):
            t = k*a.speed/a.fps
            i = replay.index_at(t)
            apply(model, data, replay, i)
            renderer.update_scene(data, camera=cam)
            img = Image.fromarray(renderer.render())
            draw = ImageDraw.Draw(img)
            draw.rectangle((0, 0, 720, 104), fill='white')
            draw.multiline_text((8, 5), replay.text(i), fill='black', spacing=3)
            name = f'frame_{k:04d}.png'
            img.save(out/name)
            writer.writerow((name, t, replay.packets[i].stamp,
                             (replay.packets[i].stamp-replay.bag_start_ns)/1e9,
                             'MEASURED bag log start' if replay.bag_origin_known else 'ASSUMED first packet', i))
            frames.append(img.convert('P', palette=Image.Palette.ADAPTIVE, colors=64))
    # ASSUMED rendering settings: reduce GIF spatial resolution if needed, retain all times.
    gif = out/'scene_twin.gif'
    while True:
        frames[0].save(gif, save_all=True, append_images=frames[1:],
                       duration=max(10, round(1000/a.fps/10)*10), loop=0, optimize=True)
        if gif.stat().st_size < 3_000_000:
            break
        if frames[0].width < 240:
            raise ValueError('GIF exceeds 3 MB; use lower --fps or higher --speed')
        frames = [im.resize((int(im.width*.8), int(im.height*.8))) for im in frames]
    print(f'{count} PNGs; GIF {gif.stat().st_size} bytes: {out}')


def window(a, replay, model, data):
    import mujoco.viewer
    cam = camera(replay)
    with mujoco.viewer.launch_passive(model, data, show_left_ui=False, show_right_ui=False) as v:
        v.cam.lookat[:] = cam.lookat
        v.cam.distance, v.cam.azimuth, v.cam.elevation = cam.distance, cam.azimuth, cam.elevation
        start = time.monotonic()
        while v.is_running():
            t = (time.monotonic()-start)*a.speed
            i = replay.index_at(min(t, replay.duration))
            with v.lock():
                apply(model, data, replay, i)
            v.set_texts((None, mujoco.mjtGridPos.mjGRID_TOPLEFT, replay.text(i), ''))
            v.sync()
            if t >= replay.duration:
                break
            time.sleep(1/a.fps)


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run', type=Path, required=True)
    p.add_argument('--record', type=Path)
    p.add_argument('--fps', type=float, default=10)  # ASSUMED playback display rate.
    p.add_argument('--speed', type=float, default=1)  # ASSUMED playback time scale.
    p.add_argument('--motion', choices=('auto', 'static', 'approach'), default='auto')
    a = p.parse_args(argv)
    import math
    if not all(math.isfinite(v) and v > 0 for v in (a.fps, a.speed)):
        p.error('--fps and --speed must be positive finite values')
    motion = a.motion if a.motion != 'auto' else ('approach' if a.run.name.startswith('4-approach') else 'static')
    replay = Replay(read_packets(a.run/'packets.csv'), bag_start(a.run), motion)
    model, data = make_scene(replay)
    print(f'{len(replay.packets)} packets, {replay.duration:.3f}s; {motion}; height {replay.height:.6f}m')
    (record if a.record else window)(a, replay, model, data)


if __name__ == '__main__':
    main()
