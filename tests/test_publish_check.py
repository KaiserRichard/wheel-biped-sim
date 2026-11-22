"""Invented fixtures for every audit probe, with accepted controls."""
import gzip
import json
import shutil
from pathlib import Path

import pytest

from publish_check import PNG_ALLOWLIST, ROOT, scan


def check_case(tmp_path, name, rejected, accepted, safe_name="doc.md"):
    path = tmp_path / name
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(rejected if isinstance(rejected, bytes) else rejected.encode())
    assert scan(tmp_path), name
    path.unlink()
    safe = tmp_path / safe_name
    safe.parent.mkdir(parents=True, exist_ok=True)
    safe.write_text(accepted)
    assert scan(tmp_path) == []


def test_windows_home(tmp_path):
    check_case(tmp_path, 'doc.md', 'C:' + '\\Users\\example\\notes', 'Use notes/input.csv')


def test_wsl_personal_path(tmp_path):
    check_case(tmp_path, 'doc.md', '/' + 'mnt/e/ExampleOwner/private', 'Use inputs/private.csv')


def test_licence_name(tmp_path):
    check_case(tmp_path, 'trial_FAKE.lic', 'invented', 'MIT License\nCopyright (c) 2026 Trần Nguyên Bình', 'LICENSE')


def test_mesh(tmp_path):
    check_case(tmp_path, 'robot.stl', 'solid fake', 'Primitive geometry is generated in Python.')


def test_cad_model(tmp_path):
    check_case(tmp_path, 'model.slx', b'FAKE model', 'No embedded CAD inputs.')


def test_paper(tmp_path):
    check_case(tmp_path, 'paper.pdf', b'%PDF fake', 'See the upstream paper at its original publisher.')


def test_github_token(tmp_path):
    check_case(tmp_path, 'doc.md', 'ghp' + '_' + 'FAKE' * 10, 'GitHub authentication is configured externally.')


def test_named_imu(tmp_path):
    check_case(tmp_path, 'doc.md', 'BMI' + '088', 'IMU sensor')


def test_student_id(tmp_path):
    check_case(tmp_path, 'doc.md', 'Student ID: ' + '2099' + '9999', '20261117 samples; 2026-11-17; version 26.2.0.3433840')


def test_account_label(tmp_path):
    check_case(tmp_path, 'doc.md', 'co' + 'dex-z', 'AI assistance was used; human and upstream credits are retained.')


def test_personal_home(tmp_path):
    check_case(tmp_path, 'doc.md', '/' + 'home/example/notes', 'Read notes/example.md')


def test_ip_address(tmp_path):
    check_case(tmp_path, 'doc.md', '192.' + '0.2.7', 'Python 3.11.18; version 1.2.3.4; 2026-11-17')


def test_reviewed_figure_directory(tmp_path):
    path = tmp_path / 'media/figures/unreviewed.png'
    path.parent.mkdir(parents=True)
    path.write_bytes(b'FAKE image')
    assert scan(tmp_path)
    path.unlink()
    for rel in PNG_ALLOWLIST:
        target = tmp_path / rel
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(ROOT / rel, target)
    assert scan(tmp_path) == []


@pytest.mark.parametrize('suffix', ['.STEP', '.stp', '.STL', '.OBJ', '.dae', '.SLX', '.PDF', '.lic', '.npz'])
def test_forbidden_extensions(tmp_path, suffix):
    check_case(tmp_path, 'input' + suffix, 'invented fixture', 'relative input.csv')


def test_trial_without_lic_extension(tmp_path):
    check_case(tmp_path, 'trial_example.txt', 'invented licence', 'MIT permission notice', 'LICENSE')


@pytest.mark.parametrize('prefix', ['github' + '_pat_', 'gh' + 'o_', 'sk' + '-', 'sk' + '-proj-', 'sk' + '-ant-api03-'])
def test_standalone_provider_tokens(tmp_path, prefix):
    check_case(tmp_path, 'doc.md', prefix + 'FAKE' * 12, 'Store credentials outside this tree.')


@pytest.mark.parametrize('value', ['cl' + 'aude-b', 'co' + 'dex-k', 'a' + 'gy', 'Student number: ' + '2099' + '9999', 'MSSV ' + '2099' + '9999', 'License number: ' + '1234' + '5678'])
def test_private_contexts(tmp_path, value):
    check_case(tmp_path, 'doc.md', value, 'MIT motor frames; MIT software license; upstream author seek.')


@pytest.mark.parametrize('value', ['BMI' + '-088', 'MPU' + '6050', 'ICM' + '-42688-P', 'BNO' + '085', 'LSM6' + 'DSOX', 'ADIS' + '16470'])
def test_sensor_spelling_variants(tmp_path, value):
    check_case(tmp_path, 'doc.md', value, 'IMU sensor')


def test_sensor_filename(tmp_path):
    check_case(tmp_path, 'BMI' + '088.md', 'generic contents', 'IMU sensor')


def test_json_escaped_windows_path(tmp_path):
    check_case(tmp_path, 'doc.json', json.dumps({'path': 'C:' + '\\Users\\example\\notes'}), '{}')


@pytest.mark.parametrize('value', ['/etc/input', '../private/input', 'C:' + '/data/input', 'https://example.org/file'])
def test_receipts_require_relative_paths(tmp_path, value):
    check_case(tmp_path, 'receipt.json', json.dumps({'nested': [{'input_path': value}]}), json.dumps({'input_path': 'inputs/data.csv'}), 'receipt.json')


@pytest.mark.parametrize('name', ['.venv', '.agents', '.co' + 'dex', '.cl' + 'aude', '__pycache__', '.pytest_cache'])
def test_environment_directories(tmp_path, name):
    path = tmp_path / name
    path.mkdir()
    assert scan(tmp_path)
    path.rmdir()
    assert scan(tmp_path) == []


def test_symlinks(tmp_path):
    (tmp_path / 'input.md').write_text('Safe fixture')
    for name, target in [('file-link', tmp_path / 'input.md'), ('dir-link', tmp_path), ('broken-link', tmp_path / 'missing')]:
        path = tmp_path / name
        path.symlink_to(target)
        assert scan(tmp_path)
        path.unlink()
    assert scan(tmp_path) == []


def test_changed_allowed_png(tmp_path):
    rel = next(iter(PNG_ALLOWLIST))
    path = tmp_path / rel
    path.parent.mkdir(parents=True)
    path.write_bytes((ROOT / rel).read_bytes() + b'changed')
    assert scan(tmp_path)


def test_compressed_text_bounded_and_scanned(tmp_path, monkeypatch):
    import publish_check
    path = tmp_path / 'fixture.csv.gz'
    path.write_bytes(gzip.compress(('192.' + '0.2.7').encode()))
    assert scan(tmp_path)  # No additions approved by default.
    monkeypatch.setattr(publish_check, 'GZIP_ALLOWLIST', {'fixture.csv.gz'})
    assert scan(tmp_path)  # Even an allowed container is inspected.
    path.write_bytes(gzip.compress(b't,value\n0,1\n'))
    assert scan(tmp_path) == []
    monkeypatch.setattr(publish_check, 'MAX_TEXT_BYTES', 128)
    path.write_bytes(gzip.compress(b'x' * 129))
    assert scan(tmp_path)
    path.write_bytes(b'not gzip')
    assert scan(tmp_path)


@pytest.mark.parametrize('value', ['a' + '@example.org', 'api_key=' + 'inventedvalue12345', 'con' + 'ductor', 'T' + '12', 'D435' + 'i 136'])
def test_existing_content_rules(tmp_path, value):
    check_case(tmp_path, 'doc.md', value, 'Upstream copyright 2024-2026; MIT License; version 3.11.18')


def test_windows_data_drive(tmp_path):
    check_case(tmp_path, 'doc.md', 'E:' + '\\ExampleOwner\\private', 'inputs/data.csv')


def test_mp4_metadata_and_compressed_samples(tmp_path):
    def atom(kind, data):
        return (len(data) + 8).to_bytes(4, 'big') + kind + data
    path = tmp_path / 'clip.mp4'
    sample = ('a' + 'gy').encode()
    path.write_bytes(atom(b'ftyp', b'isom') + atom(b'mdat', sample))
    assert scan(tmp_path) == []
    path.write_bytes(atom(b'ftyp', b'isom') + atom(b'mdat', sample) + atom(b'udta', ('C:' + '\\Users\\example').encode()))
    assert scan(tmp_path)


def test_credits_and_mit_packet_fields(tmp_path):
    path = tmp_path / 'doc.md'
    path.write_text('Cl' + 'aude and Co' + 'dex assistance. MIT mode P16 | V12 | Kp12 | Kd12 | T' + '12.')
    assert scan(tmp_path) == []
    path.write_text(path.read_text() + ' Internal label T' + '12.')
    assert scan(tmp_path)


def test_sensor_name_in_filename_with_underscores(tmp_path):
    check_case(tmp_path, 'driver_' + 'BMI' + '088_config.py', 'invented source', 'IMU sensor')


@pytest.mark.parametrize('key,value', [('student_id', '2099' + '9999'), ('license_number', '1234' + '5678')])
def test_private_identifiers_in_json(tmp_path, key, value):
    check_case(tmp_path, 'doc.json', json.dumps({key: value}), json.dumps({'version': '26.2.0.3433840', 'date': '2026-11-17'}))
