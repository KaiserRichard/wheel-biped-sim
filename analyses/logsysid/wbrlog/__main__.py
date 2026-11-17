"""Offline conversion: python -m wbrlog INPUT OUTPUT [options]."""
import argparse
import json
from pathlib import Path

from . import importer, studio


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("input")
    p.add_argument("output")
    p.add_argument("--format", choices=("long", "wide", "wire"), default="long")
    p.add_argument("--meta", help="complete replacement META JSON")
    p.add_argument("--studio", help="M13 estimation CSV; supplies wide output grid")
    p.add_argument("--event", help="EVENT marking Studio time zero, normally studio_t0")
    p.add_argument("--offset", type=float, default=0.0, help="robot time of Studio zero in seconds")
    p.add_argument("--method", choices=("linear", "zoh", "angle"), default="zoh")
    p.add_argument("--dt", type=float, default=0.001)
    p.add_argument("--channels", nargs="+", help="wide export/comparison channels")
    args = p.parse_args(argv)
    src, dst = Path(args.input), Path(args.output)
    if src.resolve() == dst.resolve():
        p.error("input and output must differ")
    meta = json.loads(Path(args.meta).read_text(encoding="utf-8")) if args.meta else None
    ds = importer.import_log(src, meta_override=meta)
    if not ds.ch:
        p.error("no samples decoded")
    offset = studio.event_time(ds, args.event) if args.event else args.offset
    reference = studio.load_estimation_csv(args.studio) if args.studio else None
    names = args.channels or ds.names()
    if any(n not in ds.ch for n in names):
        p.error("requested channel is absent from log")
    receipt = {"source_sha256": importer.file_sha256(src), "offset_s": offset,
               "stats": ds.stats, "meta": ds.meta, "format": args.format,
               "method": args.method}
    if reference:
        shared = args.channels or sorted(set(ds.ch) & set(reference.ch))
        receipt["comparison"] = studio.compare(ds, reference, shared, offset, args.method)
        receipt["studio_sha256"] = importer.file_sha256(args.studio)
    for s in ds.ch.values():
        s.t = [t - offset for t in s.t]
    lo = min(s.t[0] for s in ds.ch.values())
    hi = max(s.t[-1] for s in ds.ch.values())
    if args.format == "long":
        importer.write_long_csv(ds, dst)
    elif args.format == "wire":
        studio.write_wire_csv(ds, dst, 0.0, hi)
    else:
        grid = (reference.ch["est.pitch"].t if reference else
                studio.uniform_grid(lo, hi, args.dt))
        importer.write_wide_csv(ds, dst, names, grid, args.method,
                                {"source_sha256": receipt["source_sha256"], "offset_s": offset})
    Path(str(dst) + ".json").write_text(json.dumps(receipt, indent=2), encoding="utf-8")
    print(f"Exported {len(ds.ch)} decoded channels to {dst}; receipt: {dst}.json")


if __name__ == "__main__":
    main()
