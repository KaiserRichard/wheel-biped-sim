"""ROS-free reference mapping for EstimateSnapshot; proposed policy, no ROS imports.

Snapshots use the adapter's replay_fake JSONL keys. ClockEvidence must come from
a trusted mapper, never from guessing the magnitude of a device timestamp.
"""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class ClockEvidence:
    source_ns: int
    source_domain: str
    session: str
    target_domain: str
    generation: int
    revision: int
    ros_ns: int
    age_upper_ns: int
    uncertainty_ns: int


def integer(value, maximum):
    return type(value) is int and 0 <= value <= maximum


INT64_MAX = 2**63 - 1
UINT64_MAX = 2**64 - 1
ROS_MAX = 2147483647999999999


def pose_fields(transform, ros_ns):
    """T_world_camera -> PoseStamped fields, without inversion or axis guessing."""
    if type(ros_ns) is not int or not 0 <= ros_ns <= 2147483647 * 10**9 + 999999999:
        raise ValueError("ROS stamp outside nonnegative int32-second range")
    if transform["parent"] != "slam_world" or transform["child"] != "camera_left_optical":
        raise ValueError("bad_frame")
    m = transform["matrix"]
    if len(m) != 16 or not all(math.isfinite(v) for v in m):
        raise ValueError("bad_pose")
    if any(abs(m[12+j] - (1 if j == 3 else 0)) > 1e-6 for j in range(4)):
        raise ValueError("bad_pose")
    for i in range(3):
        for j in range(3):
            if abs(sum(m[4*k+i]*m[4*k+j] for k in range(3)) - (i == j)) > 1e-4:
                raise ValueError("bad_pose")
    det = (m[0]*(m[5]*m[10]-m[6]*m[9]) - m[1]*(m[4]*m[10]-m[6]*m[8])
           + m[2]*(m[4]*m[9]-m[5]*m[8]))
    if abs(det-1) >= 1e-4:
        raise ValueError("bad_pose")
    # Largest diagonal branch remains stable at 180 degrees.
    trace = m[0]+m[5]+m[10]
    if trace > 0:
        scale = 2*math.sqrt(trace+1)
        q = [(m[9]-m[6])/scale, (m[2]-m[8])/scale, (m[4]-m[1])/scale, scale/4]
    else:
        i = max(range(3), key=lambda k: m[5*k])
        j, k = (i+1) % 3, (i+2) % 3
        scale = 2*math.sqrt(1+m[5*i]-m[5*j]-m[5*k])
        q = [0.0]*4
        q[i] = scale/4
        q[j] = (m[4*i+j]+m[4*j+i])/scale
        q[k] = (m[4*i+k]+m[4*k+i])/scale
        q[3] = (m[4*k+j]-m[4*j+k])/scale
    norm = math.sqrt(sum(v*v for v in q))
    q = [v/norm for v in q]
    if q[3] < 0:
        q = [-v for v in q]
    return {"header": {"stamp": {"sec": ros_ns//10**9, "nanosec": ros_ns % 10**9},
                       "frame_id": "slam_world"},
            "pose": {"position": dict(zip("xyz", (m[3], m[7], m[11]))),
                     "orientation": dict(zip("xyzw", q))}}


class Gate:
    """One bound adapter session / ROS clock generation. Recreate on rebind.

    advance() returns (optional message fields, diagnostic reason). Invalid/reset
    events must be delivered even when source and sequence did not advance.
    """
    def __init__(self, session, source_domain, target_domain, generation,
                 max_age_ns, max_uncertainty_ns):
        self.session, self.domain, self.target = session, source_domain, target_domain
        self.generation = generation
        self.max_age, self.max_uncertainty = max_age_ns, max_uncertainty_ns
        self.sequence = 0
        self.source = None
        self.boundary = None
        self.blocked = False

    def advance(self, s, evidence):
        def reject(reason):
            return None, reason
        if self.blocked:
            return reject("regression_latched")
        if (not self.session or not self.domain or self.target not in ("ros_live", "ros_replay")
                or not integer(self.generation, UINT64_MAX)
                or not integer(self.max_age, INT64_MAX) or self.max_age == 0
                or not integer(self.max_uncertainty, INT64_MAX)):
            return reject("unconfigured")
        # Validate before watermarks/boundaries: malformed numbers are not events.
        if any(not integer(s[key], UINT64_MAX) for key in
               ('sequence', 'reset_counter', 'epoch_id')) or (
                s['map_id'] is not None and not integer(s['map_id'], UINT64_MAX)) or not integer(s['reasons'], 2**32 - 1):
            return reject("invalid_snapshot")
        if s['source_ns'] is not None and not integer(s['source_ns'], INT64_MAX):
            return reject("bad_source")
        if evidence is not None and any(
                not integer(getattr(evidence, key), maximum) for key, maximum in (
                    ('source_ns', INT64_MAX), ('generation', UINT64_MAX),
                    ('revision', UINT64_MAX), ('ros_ns', ROS_MAX),
                    ('age_upper_ns', INT64_MAX), ('uncertainty_ns', INT64_MAX))):
            return reject("clock_unavailable")
        if s["source_ns"] is not None and (s["source_ns"] < 0 or s["domain"] != self.domain):
            return reject("bad_source")
        boundary = (s["reset_counter"], s["epoch_id"], s["map_id"])
        if s["sequence"] < self.sequence or (self.boundary and
                (boundary[0] < self.boundary[0] or boundary[1] < self.boundary[1])):
            self.blocked = True
            return reject("regression_latched")
        changed = self.boundary is not None and boundary != self.boundary
        self.boundary = boundary
        fresh = s["sequence"] > self.sequence
        if fresh:
            self.sequence = s["sequence"]
            if s["source_ns"] is not None:
                if self.source is not None and s["source_ns"] <= self.source:
                    self.blocked = True
                    return reject("regression_latched")
                self.source = s["source_ns"]
        if changed:
            return reject("reset_boundary")
        if s["state"] != "TRACKING" or not s["valid"] or s["reasons"] != 0:
            return reject("invalid_snapshot")
        if s["source_ns"] is None or s["source_ns"] < 0 or s["domain"] != self.domain:
            return reject("bad_source")
        if evidence is None:
            return reject("clock_unavailable")
        if (evidence.source_ns, evidence.source_domain, evidence.session,
                evidence.target_domain, evidence.generation) != (
                s["source_ns"], s["domain"], self.session, self.target, self.generation):
            return reject("clock_identity_mismatch")
        if evidence.revision < 0 or evidence.age_upper_ns < 0 or evidence.uncertainty_ns < 0:
            return reject("clock_unavailable")
        if evidence.uncertainty_ns > self.max_uncertainty:
            return reject("clock_uncertain")
        if evidence.age_upper_ns > self.max_age:
            return reject("stale_source")
        available = s["covariance_availability"] == "AVAILABLE"
        cov = s.get("covariance")
        if s["covariance_availability"] not in ("AVAILABLE", "UNAVAILABLE") or (available and
                (cov is None or len(cov) != 36 or not all(math.isfinite(v) for v in cov))) or (not available and cov is not None):
            return reject("bad_covariance_metadata")
        if not fresh:
            return reject("duplicate")
        if s["T_world_camera"] is None:
            return reject("bad_pose")
        try:
            return pose_fields(s["T_world_camera"], evidence.ros_ns), "pose_published"
        except ValueError as error:
            return reject(str(error))
