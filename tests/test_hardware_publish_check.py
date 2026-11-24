"""Publication coverage for the scoped design file exceptions."""
import pytest

import publish_check
from publish_check import scan


@pytest.mark.parametrize(("suffix", "header"), publish_check.HARDWARE_KICAD_HEADERS.items())
def test_kicad_size_exception_is_scoped(tmp_path, monkeypatch, suffix, header):
    monkeypatch.setattr(publish_check, "MAX_TEXT_BYTES", 128)
    monkeypatch.setattr(publish_check, "MAX_HARDWARE_KICAD_BYTES", 512)
    hardware = tmp_path / "hardware"
    hardware.mkdir()
    design = hardware / ("design" + suffix)
    safe = header + ' (version 20250114) (generator "' + "x" * 180 + '"))'
    design.write_text(safe, encoding="utf-8")
    assert scan(tmp_path) == []

    outside = tmp_path / design.name
    design.rename(outside)
    assert scan(tmp_path) == [f"oversize text or opaque data: {outside.name}"]
    outside.rename(design)

    design.write_text(safe + "x" * 512, encoding="utf-8")
    assert scan(tmp_path) == [f"oversize text or opaque data: hardware/{design.name}"]


@pytest.mark.parametrize("private_text", ["D:" + "/Private/design", "con" + "ductor", "BMI" + "088"])
def test_kicad_scans_beyond_normal_size_limit(tmp_path, monkeypatch, private_text):
    monkeypatch.setattr(publish_check, "MAX_TEXT_BYTES", 128)
    hardware = tmp_path / "hardware"
    hardware.mkdir()
    design = hardware / "design.kicad_sch"
    design.write_text('(kicad_sch (text "' + "x" * 180 + " " + private_text + '"))', encoding="utf-8")
    assert scan(tmp_path)


def test_oversize_other_hardware_text_is_rejected(tmp_path, monkeypatch):
    monkeypatch.setattr(publish_check, "MAX_TEXT_BYTES", 128)
    hardware = tmp_path / "hardware"
    hardware.mkdir()
    design = hardware / "design.kicad_mod"
    design.write_text("x" * 150, encoding="utf-8")
    assert scan(tmp_path) == ["oversize text or opaque data: hardware/design.kicad_mod"]


def make_pdf(path, metadata, text="Schematic"):
    pymupdf = pytest.importorskip("pymupdf")
    with pymupdf.open() as pdf:
        pdf.new_page().insert_text((72, 72), text)
        pdf.set_metadata(metadata)
        pdf.save(path)


def test_hardware_pdf_with_approved_metadata(tmp_path):
    hardware = tmp_path / "hardware"
    hardware.mkdir()
    schematic = hardware / "schematic.pdf"
    make_pdf(schematic, {
        "title": "WBR power board schematic", "creator": "Eeschema-PDF", "producer": "KiCad PDF",
    })
    assert scan(tmp_path) == []

    schematic.rename(tmp_path / schematic.name)
    assert scan(tmp_path) == ["prohibited file: schematic.pdf"]


@pytest.mark.parametrize(("field", "value"), [
    ("title", "Private Person"),
    ("author", "Private Person"),
    ("producer", "D:" + "/Private/Exporter"),
    ("creator", "/" + "home/private/Exporter"),
])
def test_hardware_pdf_rejects_private_metadata(tmp_path, field, value):
    hardware = tmp_path / "hardware"
    hardware.mkdir()
    make_pdf(hardware / "schematic.pdf", {field: value})
    assert scan(tmp_path) == ["unreadable or unapproved PDF metadata: hardware/schematic.pdf"]


def test_hardware_pdf_rejects_private_page_text(tmp_path):
    hardware = tmp_path / "hardware"
    hardware.mkdir()
    make_pdf(hardware / "schematic.pdf", {}, text="D:" + "/Private/design")
    assert scan(tmp_path) == ["private or secret-like text in hardware/schematic.pdf"]


def test_hardware_pdf_rejects_invalid_data(tmp_path):
    pytest.importorskip("pymupdf")
    hardware = tmp_path / "hardware"
    hardware.mkdir()
    (hardware / "schematic.pdf").write_bytes(b"%PDF-1.5\ninvalid")
    assert scan(tmp_path) == ["unreadable or unapproved PDF metadata: hardware/schematic.pdf"]
