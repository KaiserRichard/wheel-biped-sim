#!/usr/bin/env python3
"""Validated Kalibr calibration export. Conventions and source lines: README.md."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
from decimal import Decimal, InvalidOperation, ROUND_HALF_EVEN
import hashlib
import json
from pathlib import Path
import sys

import numpy as np
import yaml

VERSION = "1.0.0"
ORB_REV = "4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4"
ORB_URL = f"https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/{ORB_REV}/"
OV_URL = "https://github.com/rpng/open_vins/blob/v2.6.3/"
SOURCES = {
    "kalibr_conventions": "https://github.com/ethz-asl/kalibr/wiki/yaml-formats",
    "kalibr_noise": "https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model",
    "orb_cameras": ORB_URL + "src/Settings.cc#L184-L351",
    "orb_imu": ORB_URL + "src/Settings.cc#L414-L431",
    "orb_rectification": ORB_URL + "src/Settings.cc#L485-L525",
    "orb_noise_scaling": ORB_URL + "src/Tracking.cc#L603-L614",
    "openvins_transform": OV_URL + "ov_core/src/utils/opencv_yaml_parse.h#L448-L490",
    "openvins_noise": OV_URL + "ov_msckf/src/core/VioManagerOptions.h#L142-L152",
    "openvins_camera": OV_URL + "ov_msckf/src/core/VioManagerOptions.h#L205-L249",
}
UNITS = {
    "translation": "m", "intrinsics": "px", "resolution": "px",
    "timeshift": "s", "update_rate": "Hz", "gyroscope": "rad/s",
    "accelerometer": "m/s^2", "gyroscope_noise_density": "rad/s/sqrt(Hz)",
    "accelerometer_noise_density": "m/s^2/sqrt(Hz)",
    "gyroscope_random_walk": "rad/s^2/sqrt(Hz)",
    "accelerometer_random_walk": "m/s^3/sqrt(Hz)",
}
NOISE_KEYS = {
    "IMU.NoiseGyro": "gyroscope_noise_density",
    "IMU.NoiseAcc": "accelerometer_noise_density",
    "IMU.GyroWalk": "gyroscope_random_walk",
    "IMU.AccWalk": "accelerometer_random_walk",
    "IMU.Frequency": "update_rate",
}
# Operating choices from the inspected development candidate, not calibration.
TUNING = {
    "Camera.RGB": 1, "Stereo.ThDepth": 40.0, "IMU.InsertKFsWhenLost": 0,
    "ORBextractor.nFeatures": 1250, "ORBextractor.scaleFactor": 1.2,
    "ORBextractor.nLevels": 8, "ORBextractor.iniThFAST": 20,
    "ORBextractor.minThFAST": 7, "Viewer.KeyFrameSize": 0.05,
    "Viewer.KeyFrameLineWidth": 1.0, "Viewer.GraphLineWidth": 0.9,
    "Viewer.PointSize": 2.0, "Viewer.CameraSize": 0.08,
    "Viewer.CameraLineWidth": 3.0, "Viewer.ViewpointX": 0.0,
    "Viewer.ViewpointY": -0.7, "Viewer.ViewpointZ": -3.5,
    "Viewer.ViewpointF": 500.0,
}


class UniqueLoader(yaml.SafeLoader):
    """Reject ambiguous calibration files with duplicate keys."""


class QuotedString(str):
    """Distinguish quoted numeric strings from plain YAML exponents."""


def yaml_string(loader, node):
    value = loader.construct_scalar(node)
    return QuotedString(value) if node.style is not None else value


def unique_mapping(loader, node):
    result = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node)
        if not isinstance(key, str) or key in result:
            raise ValueError(f"invalid or duplicate YAML key: {key!r}")
        result[key] = loader.construct_object(value_node)
    return result


UniqueLoader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, unique_mapping)
UniqueLoader.add_constructor("tag:yaml.org,2002:str", yaml_string)


def read_yaml(raw: bytes, name: str) -> dict:
    text = raw.decode("utf-8")
    if text.startswith("%YAML:1.0"):
        text = text.split("\n", 1)[1] if "\n" in text else ""
    try:
        result = yaml.load(text, Loader=UniqueLoader)
    except yaml.YAMLError as exc:
        raise ValueError(f"{name}: invalid YAML: {exc}") from exc
    if not isinstance(result, dict):
        raise ValueError(f"{name}: expected a YAML mapping")
    return result


def required(mapping: dict, key: str, where: str):
    if key not in mapping:
        raise ValueError(f"{where}: missing {key}")
    return mapping[key]


def number(value, name: str, *, positive: bool = False) -> float:
    if isinstance(value, QuotedString):
        raise ValueError(f"{name}: numeric YAML fields must be unquoted for native engine loading")
    if isinstance(value, bool) or not isinstance(value, (int, float, str)):
        raise ValueError(f"{name}: expected a finite numeric SI value")
    try:
        result = float(value)
    except (ValueError, OverflowError) as exc:
        raise ValueError(f"{name}: expected a numeric SI value, without unit suffix") from exc
    if not np.isfinite(result) or (positive and result <= 0):
        raise ValueError(f"{name}: expected a finite {'positive ' if positive else ''}value")
    return result


def array(value, shape: tuple, name: str) -> np.ndarray:
    # Validate scalar types before NumPy can coerce booleans to numbers.
    flat = np.asarray(value, dtype=object)
    if flat.shape != shape:
        raise ValueError(f"{name}: expected shape {shape}, got {flat.shape}")
    return np.array([number(x, name) for x in flat.flat]).reshape(shape)


def transform(value, name: str, max_translation_m: float) -> np.ndarray:
    matrix = array(value, (4, 4), name)
    rotation = matrix[:3, :3]
    if not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-9, rtol=0):
        raise ValueError(f"{name}: invalid homogeneous bottom row")
    if (not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6, rtol=0)
            or abs(np.linalg.det(rotation) - 1) > 1e-6):
        raise ValueError(f"{name}: rotation must be orthonormal with determinant +1")
    if np.linalg.norm(matrix[:3, 3]) > max_translation_m:
        raise ValueError(f"{name}: translation exceeds {max_translation_m} m; check metre vs mm units")
    return matrix


def inverse(matrix: np.ndarray) -> np.ndarray:
    """Invert T_A_B without losing the rotated translation."""
    result = np.eye(4)
    result[:3, :3] = matrix[:3, :3].T
    result[:3, 3] = -result[:3, :3] @ matrix[:3, 3]
    return result


def check_units(mapping: dict, where: str):
    if "units" not in mapping:
        return
    declared = mapping["units"]
    if not isinstance(declared, dict):
        raise ValueError(f"{where}.units: expected mapping")
    for key, unit in declared.items():
        if key not in UNITS or unit != UNITS[key]:
            raise ValueError(f"{where}.units.{key}: unsupported unit {unit!r}; expected {UNITS.get(key)!r}")


def shift_record(value, name: str) -> dict:
    number(value, name)
    # Decimal conversion avoids epoch float arithmetic; offsets alone are rounded.
    try:
        seconds = Decimal(str(value))
        ns = seconds * 1_000_000_000
        rounded = ns.to_integral_value(rounding=ROUND_HALF_EVEN)
        if abs(rounded) > 2**63 - 1:
            raise ValueError(f"{name}: exceeds int64 nanoseconds")
    except InvalidOperation as exc:
        raise ValueError(f"{name}: invalid seconds") from exc
    return {"seconds": str(value), "nanoseconds": int(rounded),
            "rounding_error_ns": str(abs(ns - rounded)), "rounding": "half_even"}


def validate(camchain: dict, imu_doc: dict, target: str, max_translation_m: float) -> dict:
    check_units(camchain, "camchain")
    check_units(imu_doc, "imu")
    camera_keys = {key for key in camchain if key.startswith("cam")}
    if camera_keys != {"cam0", "cam1"}:
        raise ValueError("exactly cam0 and cam1 are required")
    cameras = []
    for name in ("cam0", "cam1"):
        cam = camchain[name]
        if not isinstance(cam, dict):
            raise ValueError(f"{name}: expected mapping")
        check_units(cam, name)
        if required(cam, "camera_model", name) != "pinhole":
            raise ValueError(f"{name}: unsupported camera_model; only pinhole is supported")
        model = required(cam, "distortion_model", name)
        if model not in {"radtan", "equidistant"}:
            raise ValueError(f"{name}: unsupported distortion_model {model!r}")
        if model == "equidistant" and target != "openvins":
            raise ValueError(f"{name}: equidistant is unsupported for ORB; KannalaBrandt8 mapping is not implemented")
        intrinsics = array(required(cam, "intrinsics", name), (4,), name + ".intrinsics")
        if np.any(intrinsics[:2] <= 0):
            raise ValueError(f"{name}: focal lengths must be positive pixels")
        distortion = array(required(cam, "distortion_coeffs", name), (4,), name + ".distortion_coeffs")
        resolution = required(cam, "resolution", name)
        if (not isinstance(resolution, list) or len(resolution) != 2
                or any(type(x) is not int or x <= 0 for x in resolution)):
            raise ValueError(f"{name}: resolution must be two positive integers [width,height]")
        if not (0 <= intrinsics[2] < resolution[0] and 0 <= intrinsics[3] < resolution[1]):
            raise ValueError(f"{name}: principal point outside resolution; check pixel units/order")
        pose = transform(required(cam, "T_cam_imu", name), name + ".T_cam_imu", max_translation_m)
        if "T_imu_cam" in cam:
            alternate = transform(cam["T_imu_cam"], name + ".T_imu_cam", max_translation_m)
            if not np.allclose(alternate @ pose, np.eye(4), atol=1e-6, rtol=0):
                raise ValueError(f"{name}: conflicting T_imu_cam and T_cam_imu")
        shift = shift_record(required(cam, "timeshift_cam_imu", name), name + ".timeshift_cam_imu")
        cameras.append({"intrinsics": intrinsics, "distortion": distortion,
                        "resolution": resolution, "pose": pose, "shift": shift})
    stereo = transform(required(camchain["cam1"], "T_cn_cnm1", "cam1"),
                       "cam1.T_cn_cnm1", max_translation_m)
    expected = cameras[1]["pose"] @ inverse(cameras[0]["pose"])
    if not np.allclose(stereo, expected, atol=1e-5, rtol=0):
        raise ValueError("cam1.T_cn_cnm1 is inconsistent with the two T_cam_imu transforms")
    baseline = float(np.linalg.norm(stereo[:3, 3]))
    if baseline <= 1e-9:
        raise ValueError("stereo baseline must be nonzero")
    if target != "openvins" and cameras[0]["resolution"] != cameras[1]["resolution"]:
        raise ValueError("ORB requires identical camera resolutions")
    if target != "orb" and Decimal(cameras[0]["shift"]["seconds"]) != Decimal(cameras[1]["shift"]["seconds"]):
        raise ValueError("OpenVINS loads only cam0 timeshift; unequal camera timeshifts are unsupported")
    if "imu0" in imu_doc:
        if any(key.startswith("imu") and key != "imu0" for key in imu_doc):
            raise ValueError("only imu0 is supported")
        if any(key in imu_doc for key in NOISE_KEYS.values()):
            raise ValueError("ambiguous flat and nested IMU fields")
        imu = imu_doc["imu0"]
    else:
        imu = imu_doc
    if not isinstance(imu, dict):
        raise ValueError("imu0: expected mapping")
    check_units(imu, "imu0")
    noise = {output: number(required(imu, key, "imu"), key, positive=True)
             for output, key in NOISE_KEYS.items()}
    return {"cameras": cameras, "stereo": stereo, "baseline_m": baseline, "noise": noise}


def scalar(value) -> str:
    if isinstance(value, str):
        return json.dumps(value)
    if isinstance(value, (int, np.integer)):
        return str(value)
    # OpenCV Settings.cc distinguishes real scalars from integer scalars.
    text = repr(float(value))
    if "e" in text and "." not in text.split("e", 1)[0]:
        text = text.replace("e", ".0e")
    return text


def orb_yaml(calibration: dict, fps: int) -> bytes:
    fields = {"File.version": "1.0", "Camera.type": "PinHole"}
    for index, camera in enumerate(calibration["cameras"], 1):
        for key, value in zip(("fx", "fy", "cx", "cy"), camera["intrinsics"]):
            fields[f"Camera{index}.{key}"] = float(value)
        for key, value in zip(("k1", "k2", "p1", "p2"), camera["distortion"]):
            fields[f"Camera{index}.{key}"] = float(value)
    width, height = calibration["cameras"][0]["resolution"]
    fields.update({"Camera.width": width, "Camera.height": height, "Camera.fps": fps})
    fields.update(TUNING)
    fields.update(calibration["noise"])
    # Settings.cc reads real fields and Sophus transforms as native float32.
    # Finite Python doubles can overflow there, or silently erase a nonzero value.
    def check_float32(value, name):
        with np.errstate(over="ignore", under="ignore"):
            native = np.asarray(value, dtype=np.float32)
        original = np.asarray(value)
        if not np.isfinite(native).all() or np.any((original != 0) & (native == 0)):
            raise ValueError(f"{name}: value overflows or underflows native float32")

    for key, value in fields.items():
        if isinstance(value, float):
            check_float32(value, key)
    for key, matrix in (("Stereo.T_c1_c2", inverse(calibration["stereo"])),
                        ("IMU.T_b_c1", inverse(calibration["cameras"][0]["pose"]))):
        check_float32(matrix, key)
    text = ["%YAML:1.0", "# Synthetic convention tests only; see provenance sidecar.",
            "# Feed raw pinhole/radtan images; ORB rectifies internally.",
            "# Timeshift must be applied at association; see provenance sidecar."]
    text.extend(f"{key}: {scalar(value)}" for key, value in fields.items())
    for key, matrix in (("Stereo.T_c1_c2", inverse(calibration["stereo"])),
                        ("IMU.T_b_c1", inverse(calibration["cameras"][0]["pose"]))):
        data = ", ".join(scalar(value) for value in matrix.flat)
        text.extend([f"{key}: !!opencv-matrix", "   rows: 4", "   cols: 4",
                     "   dt: f", f"   data: [{data}]"])
    return ("\n".join(text) + "\n").encode()


def openvins_yaml(raw: bytes, *, wrap_imu: bool) -> bytes:
    """Preserve input text/values; only add the OpenCV header/imu0 container."""
    text = raw.decode("utf-8")
    if text.startswith("%YAML:1.0"):
        text = text.split("\n", 1)[1] if "\n" in text else ""
    # A document marker belongs outside the newly added imu0 mapping.
    lines = [line for line in text.splitlines() if line.strip() not in {"---", "..."}]
    if wrap_imu:
        lines = ["imu0:"] + ["  " + line for line in lines]
    return ("%YAML:1.0\n" + "\n".join(lines) + "\n").encode()


def digest(raw: bytes) -> str:
    return hashlib.sha256(raw).hexdigest()


def convert(camchain_path: Path, imu_path: Path, output_dir: Path, *, target: str,
            fps: int | None, units_si: bool, max_translation_m: float = 1.0) -> list[Path]:
    if target not in {"orb", "openvins", "both"}:
        raise ValueError("target must be orb, openvins or both")
    if not units_si:
        raise ValueError("--units-si is required: attest metres, seconds, rad/s, m/s^2 and continuous noise densities")
    if target != "openvins" and (type(fps) is not int or fps <= 0):
        raise ValueError("--fps must be an explicit positive integer for ORB (not present in Kalibr)")
    max_translation_m = number(max_translation_m, "max_translation_m", positive=True)
    cam_raw, imu_raw = camchain_path.read_bytes(), imu_path.read_bytes()
    camchain, imu_doc = read_yaml(cam_raw, "camchain"), read_yaml(imu_raw, "imu")
    calibration = validate(camchain, imu_doc, target, max_translation_m)
    outputs = {}
    warnings = []
    if target != "openvins":
        outputs["orbslam3.yaml"] = orb_yaml(calibration, fps)
        warnings.append("ORB has no time-offset settings field. Shift camera stamps at association: "
                        "t_imu = t_cam + timeshift_cam_imu (Kalibr yaml-formats). "
                        "Apply each camera's shift once using integer nanoseconds in an explicitly mapped clock domain.")
        warnings.append("ORB tuning/viewer values are development operating choices, not calibrated values; review for your run.")
    if target != "orb":
        outputs["kalibr_imucam_chain.yaml"] = openvins_yaml(cam_raw, wrap_imu=False)
        outputs["kalibr_imu_chain.yaml"] = openvins_yaml(imu_raw, wrap_imu="imu0" not in imu_doc)
        warnings.append("OpenVINS v2.6.3 calibration files only: supply estimator config separately. "
                        "OpenVINS uses cam0 timeshift internally; do not also pre-shift its images. "
                        "Newer OpenVINS versions may require additional IMU intrinsic fields.")
    warnings.append("SI units are caller-attested; unlabelled YAML numbers cannot prove their physical units. "
                    "Magnitude/metadata guards do not qualify a calibration or verify delivered sample rate.")
    shifts = {f"cam{i}": cam["shift"] for i, cam in enumerate(calibration["cameras"])}
    if any(Decimal(shift["rounding_error_ns"]) != 0 for shift in shifts.values()):
        warnings.append("Sub-nanosecond timeshifts rounded half-even for integer-ns association; original seconds preserved.")
    provenance = {
        "tool": "calib_tools", "version": VERSION, "target": target,
        "generated_utc": datetime.now(timezone.utc).isoformat(),
        "converter_sha256": digest(Path(__file__).read_bytes()),
        "inputs": {"camchain": {"path": str(camchain_path.resolve()), "sha256": digest(cam_raw)},
                   "imu": {"path": str(imu_path.resolve()), "sha256": digest(imu_raw)}},
        "outputs": {name: {"sha256": digest(raw)} for name, raw in outputs.items()},
        "conventions": {"T_A_B": "maps points in B into A",
                        "orb_camera1": "Kalibr cam0", "orb_camera2": "Kalibr cam1",
                        "IMU.T_b_c1": "inverse(cam0.T_cam_imu); camera1 -> IMU",
                        "Stereo.T_c1_c2": "inverse(cam1.T_cn_cnm1); camera2 -> camera1",
                        "noise": "continuous densities copied; ORB scales noise by sqrt(f), walks by 1/sqrt(f)",
                        "time": "t_imu = t_cam + timeshift_cam_imu",
                        "clock_domain": "UNKNOWN: caller must establish a common/mapped domain before association"},
        "units": UNITS, "units_si_attested": units_si, "timeshift_cam_imu": shifts,
        "baseline_m": calibration["baseline_m"],
        "camera2_center_in_camera1_m": inverse(calibration["stereo"])[:3, 3].tolist(),
        "validation": {"status": "synthetic tests only; real calibration/engine loading UNKNOWN",
                       "rotation_atol": 1e-6, "chain_atol": 1e-5,
                       "max_translation_m": max_translation_m},
        "operating_choices": {"camera_fps": fps, "orb_tuning": TUNING} if target != "openvins" else {},
        "openvins_format_changes": {"opencv_header": True, "imu0_wrapper_added": "imu0" not in imu_doc}
        if target != "orb" else {},
        "sources": SOURCES, "warnings": warnings,
    }
    outputs["provenance.json"] = (json.dumps(provenance, indent=2, allow_nan=False) + "\n").encode()
    paths = [output_dir / name for name in outputs]
    # Validate every path before writing; never overwrite calibration inputs/results.
    input_paths = {camchain_path.resolve(), imu_path.resolve()}
    for path in paths:
        if path.resolve() in input_paths or path.exists() or path.is_symlink():
            raise ValueError(f"refusing to overwrite {path}")
    output_dir.mkdir(parents=True, exist_ok=True)
    for path in paths:
        with path.open("xb") as stream:
            stream.write(outputs[path.name])
    for warning in warnings:
        print("WARNING: " + warning, file=sys.stderr)
    return paths


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--camchain", required=True, type=Path)
    parser.add_argument("--imu", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--target", choices=("orb", "openvins", "both"), default="both")
    parser.add_argument("--fps", type=int, help="explicit delivered image rate for ORB; no inferred default")
    parser.add_argument("--units-si", action="store_true", help="attest SI units and continuous noise densities")
    parser.add_argument("--max-translation-m", type=float, default=1.0,
                        help="translation norm guard (default 1 m), an operating policy")
    args = parser.parse_args(argv)
    try:
        paths = convert(args.camchain, args.imu, args.output_dir, target=args.target,
                        fps=args.fps, units_si=args.units_si, max_translation_m=args.max_translation_m)
    except (ValueError, OSError, UnicodeError) as exc:
        parser.exit(2, f"error: {exc}\n")
    for path in paths:
        print(path)
    return 0


if __name__ == "__main__":
    sys.exit(main())
