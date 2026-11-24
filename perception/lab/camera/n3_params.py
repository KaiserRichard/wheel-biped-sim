"""Offline parameter comparator for the proposed N3 profile. UNTESTED ON HARDWARE."""
import argparse
import json
from pathlib import Path
import re


PROFILE = Path(__file__).resolve().parents[1] / "d435i_raw/profiles/n3_640x480_30.yaml"


def expected_parameters(profile):
    p = {k: v["value"] for k, v in profile["profile"].items()}
    if len(p["accel_hz_preference"]) != 1:
        raise ValueError("N3 requires one exact accel rate")
    return {
        "camera_name": "camera",
        "depth_module.infra_profile": f'{p["width"]}x{p["height"]}x{p["ir_hz"]}',
        "depth_module.infra1_format": "Y8", "depth_module.infra2_format": "Y8",
        "enable_infra": False, "enable_infra1": True, "enable_infra2": True,
        "enable_depth": False, "enable_color": False, "pointcloud.enable": False,
        "enable_sync": False, "enable_gyro": True, "enable_accel": True,
        "gyro_fps": p["gyro_hz"], "accel_fps": p["accel_hz_preference"][0],
        "depth_module.emitter_enabled": p["emitter"], "unite_imu_method": 0,
        "publish_tf": True,
    }


def flatten(mapping, prefix=""):
    result = {}
    for key, value in mapping.items():
        name = prefix + key
        if isinstance(value, dict):
            result.update(flatten(value, name + "."))
        else:
            if name in result:
                raise ValueError(f"Duplicate parameter: {name}")
            result[name] = value
    return result


def compare_dump(dump, profile):
    """Return mismatches; missing node/parameters fail closed. Accept nested ROS YAML."""
    params = flatten(dump["/camera/camera"]["ros__parameters"])
    errors = []
    for name, wanted in expected_parameters(profile).items():
        actual = params.get(name)
        if name == "depth_module.infra_profile" and isinstance(actual, str):
            actual = re.sub(r"\s+", "", actual).replace(",", "x").lower()
        if type(actual) is not type(wanted) or actual != wanted:
            errors.append(f"{name}: expected {wanted!r}, got {actual!r}")
    return errors


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", type=Path, default=PROFILE)
    parser.add_argument("--dump", type=Path)
    parser.add_argument("--arguments", action="store_true", help="emit ROS -p values, no ROS execution")
    args = parser.parse_args(argv)
    try:
        profile = json.loads(args.profile.read_text())
        if args.arguments:
            for name, value in expected_parameters(profile).items():
                print(f"{name}:={str(value).lower() if isinstance(value, bool) else value}")
            return 0
        # PyYAML ships in the intended ROS environment; never install it here.
        import yaml
        if args.dump is None:
            parser.error("--dump is required without --arguments")
        try:
            dump = yaml.safe_load(args.dump.read_text())
        except yaml.YAMLError as error:
            raise ValueError(f"Invalid parameter YAML: {error}") from error
        errors = compare_dump(dump, profile)
        print("\n".join(errors) if errors else "Parameter policy matches; UNTESTED ON HARDWARE")
        return 1 if errors else 0
    except (ImportError, OSError, ValueError, KeyError, TypeError) as error:
        print(f"Readback comparison unavailable/invalid: {error}")
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
