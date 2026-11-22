"""T_A_B maps points in B into A; metres, active rotations, integer ns."""
from dataclasses import dataclass
import numpy as np


def quaternion_order(q, source, target):
    """Explicit xyzw (ROS/TUM) <-> wxyz (Eigen constructor/ORB) conversion.

    Eigen coeffs() is xyzw; its constructor is wxyz. Never infer storage order.
    """
    if source not in {"xyzw", "wxyz"} or target not in {"xyzw", "wxyz"}:
        raise ValueError("quaternion order must be xyzw or wxyz")
    q = np.array(q, dtype=float, copy=True)
    if q.shape != (4,) or not np.isfinite(q).all():
        raise ValueError("expected four finite quaternion coefficients")
    if abs(np.linalg.norm(q) - 1) > 1e-3:
        raise ValueError("quaternion must have unit norm within 1e-3")
    q /= np.linalg.norm(q)
    if source == target:
        return q
    return q[[3, 0, 1, 2]] if target == "wxyz" else q[[1, 2, 3, 0]]


def quaternion_matrix(q, order):
    x, y, z, w = quaternion_order(q, order, "xyzw")
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


@dataclass(frozen=True, eq=False)
class Transform:
    parent: str
    child: str
    matrix: np.ndarray

    def __post_init__(self):
        if any(not isinstance(f, str) or not f.strip() for f in (self.parent, self.child)):
            raise ValueError("nonempty frame names required")
        m = np.array(self.matrix, dtype=float, copy=True)
        if m.shape != (4, 4) or not np.isfinite(m).all():
            raise ValueError("expected finite 4x4 transform")
        r = m[:3, :3]
        if (not np.allclose(m[3], [0, 0, 0, 1], atol=1e-9, rtol=0)
                or not np.allclose(r.T @ r, np.eye(3), atol=1e-6, rtol=0)
                or not np.isclose(np.linalg.det(r), 1, atol=1e-6, rtol=0)):
            raise ValueError("expected proper rigid transform")
        m.setflags(write=False)
        object.__setattr__(self, "matrix", m)

    @classmethod
    def from_pose(cls, parent, child, translation, quaternion, *, order):
        t = np.asarray(translation, dtype=float)
        if t.shape != (3,):
            raise ValueError("translation must have three components")
        m = np.eye(4)
        m[:3, :3] = quaternion_matrix(quaternion, order)
        m[:3, 3] = t
        return cls(parent, child, m)

    def __matmul__(self, other):
        if not isinstance(other, Transform):
            return NotImplemented
        if self.child != other.parent:
            raise ValueError(f"frame mismatch: {self.child!r} != {other.parent!r}")
        return Transform(self.parent, other.child, self.matrix @ other.matrix)

    def inverse(self):
        m = np.eye(4)
        r = self.matrix[:3, :3]
        m[:3, :3] = r.T
        m[:3, 3] = -r.T @ self.matrix[:3, 3]
        return Transform(self.child, self.parent, m)


def optical_to_body(body="camera_link", optical="camera_optical"):
    """REP-103 axis change at the same origin; this is NOT a measured mount."""
    return Transform(body, optical, [[0, 0, 1, 0], [-1, 0, 0, 0],
                                     [0, -1, 0, 0], [0, 0, 0, 1]])


def sdk_ir1_to_imu(rotation_column_major, translation):
    """SDK IR1->IMU extrinsic: p_imu = R p_camera + t, metres."""
    r = np.asarray(rotation_column_major, dtype=float)
    t = np.asarray(translation, dtype=float)
    if r.shape != (9,) or t.shape != (3,):
        raise ValueError("SDK extrinsic requires 9 rotation and 3 translation values")
    m = np.eye(4)
    m[:3, :3] = r.reshape(3, 3, order="F")
    m[:3, 3] = t
    return Transform("imu_camera", "camera_optical", m)


def base_imu_camera(T_base_camera, T_imu_camera):
    """T_base_imu_camera = T_base_camera @ inverse(T_imu_camera_camera).

    imu_camera is the camera's IMU frame, not the robot/controller body IMU.
    """
    if (T_base_camera.parent, T_base_camera.child) != ("base_link", "camera_optical"):
        raise ValueError("expected T_base_camera (base_link <- camera_optical)")
    if (T_imu_camera.parent, T_imu_camera.child) != ("imu_camera", "camera_optical"):
        raise ValueError("expected SDK IR1->IMU extrinsic")
    return T_base_camera @ T_imu_camera.inverse()


@dataclass(frozen=True)
class Sample:
    stamp_ns: int
    clock_domain: str
    map_id: str
    reset_counter: int
    pose: Transform

    def __post_init__(self):
        if type(self.stamp_ns) is not int or not -(2**63) <= self.stamp_ns < 2**63:
            raise ValueError("stamp_ns must be an int64 nanosecond integer")
        if any(not isinstance(s, str) or not s.strip() for s in (self.clock_domain, self.map_id)):
            raise ValueError("clock_domain and map_id must be nonempty strings")
        if type(self.reset_counter) is not int or self.reset_counter < 0:
            raise ValueError("reset_counter must be a nonnegative integer")


def convert(samples, T_base_sensor, *, world="slam_world"):
    """Return diagnostic poses and optional backward position differences.

    Differences are not qualified estimator velocity; loop jumps can exist inside
    an epoch. At every segment start velocity is unavailable (None).
    """
    if world != "slam_world":
        raise ValueError("diagnostic output must be slam_world; it cannot be called odom")
    if T_base_sensor.parent != "base_link":
        raise ValueError("mount parent must be base_link")
    previous = None
    segment = -1
    result = []
    for sample in samples:
        if (sample.pose.parent, sample.pose.child) != (world, T_base_sensor.child):
            raise ValueError("input must be sensor-to-slam_world pose matching mount")
        epoch = (sample.map_id, sample.reset_counter)
        boundary = previous is None or epoch != previous[0]
        # A domain switch always requires an explicit clock mapping, even on reset.
        if previous is not None and sample.clock_domain != previous[1].clock_domain:
            raise ValueError("cross-domain arithmetic refused; map clocks explicitly first")
        pose = sample.pose @ T_base_sensor.inverse()
        velocity = None
        if boundary:
            segment += 1
        else:
            dt_ns = sample.stamp_ns - previous[1].stamp_ns
            if dt_ns <= 0:
                raise ValueError("timestamps must strictly increase within each segment")
            with np.errstate(over="ignore", invalid="ignore"):
                velocity = ((pose.matrix[:3, 3] - previous[2].matrix[:3, 3])
                            / (dt_ns / 1_000_000_000))
            if not np.isfinite(velocity).all():
                raise ValueError("diagnostic velocity exceeds finite numeric range")
            velocity = velocity.tolist()
        result.append({"stamp_ns": sample.stamp_ns, "clock_domain": sample.clock_domain,
                       "map_id": sample.map_id, "reset_counter": sample.reset_counter,
                       "segment_id": segment, "reference_frame": world,
                       "child_frame": "base_link", "T_world_base": pose.matrix.tolist(),
                       "velocity_valid": velocity is not None,
                       "diagnostic_linear_velocity_world_m_s": velocity})
        previous = (epoch, sample, pose)
    return result
