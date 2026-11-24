"""Reuse generated WBR XML in memory; never write the source model."""
import xml.etree.ElementTree as ET
from sim.models import make_wbr_3d_simple as gen
from .replay import BOX_DEPTH, BOX_WIDTH, CAMERA_OFFSET, COLORS


def build_xml(replay):
    # ASSUMED standing virtual leg length, inherited viewer convention.
    l0 = 0.12
    root = ET.fromstring(gen.build(l0))
    world = root.find('worldbody')
    for node in list(world):
        if node.get('name') in ('ghost_acc', 'ghost_gyro', 'upright_ref', 'demo', 'side'):
            world.remove(node)
    floor = world.find("geom[@name='floor']")
    floor.set('size', '4 4 0.01')  # room for robot + obstacle at up to ~2 m
    # Omit IMU drawing: this camera run has no IMU.
    robot = world.find("body[@name='robot']")
    for node in list(robot):
        if node.get('name', '').startswith(('imu', 'imu_sensor')):
            robot.remove(node)
    # ASSUMED front-top optical centre, z relative to wheel axle.
    camera_z = l0 + gen.HIP_BELOW_TOP
    ET.SubElement(robot, 'geom', name='assumed_camera', type='box',
                  pos=f'{CAMERA_OFFSET} 0 {camera_z}', size='0.012 0.025 0.012',
                  rgba='0.1 0.1 0.1 1', mass='0')
    ET.SubElement(robot, 'site', name='camera_origin', pos=f'{CAMERA_OFFSET} 0 {camera_z}', size='0.004')
    ET.SubElement(world, 'geom', name='obstacle', type='box',
                  pos=f'{replay.face+BOX_DEPTH/2} 0 {replay.height/2}',
                  size=f'{BOX_DEPTH/2} {BOX_WIDTH/2} {replay.height/2}', rgba='0.5 0.5 0.5 1')
    return ET.tostring(root, encoding='unicode')


def make_scene(replay):
    import mujoco
    model = mujoco.MjModel.from_xml_string(build_xml(replay))
    return model, mujoco.MjData(model)


def apply(model, data, replay, i):
    import mujoco
    robot = model.body('robot').mocapid[0]
    data.mocap_pos[robot] = (replay.x[i], 0, gen.R_WHEEL)  # INFERRED wheels touch floor.
    data.mocap_quat[robot] = (1, 0, 0, 0)  # ASSUMED upright, no IMU in these runs.
    model.geom('obstacle').rgba[:] = COLORS[replay.packets[i].advisory]
    mujoco.mj_forward(model, data)


def camera(replay):
    import mujoco
    cam = mujoco.MjvCamera()
    # ASSUMED display framing fits the complete trajectory and obstacle.
    lo, hi = min(replay.x)-0.25, max(max(replay.x)+0.25, replay.face+BOX_DEPTH)
    cam.lookat[:] = ((lo+hi)/2, 0, 0.12)
    cam.distance = max(1.0, (hi-lo)*0.95 + 0.4)
    cam.azimuth, cam.elevation = 125, -18   # behind-left of the robot, looking toward the obstacle
    return cam
