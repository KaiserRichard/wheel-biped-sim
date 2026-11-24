"""Explicit nearest-rank quantiles; integer nanosecond epochs never use floats."""

import math

UNAVAILABLE = "UNAVAILABLE"


def distribution(values):
    ordered = sorted(values)
    if not ordered:
        return {"status": UNAVAILABLE, "count": 0}
    return {"status": "AVAILABLE", "count": len(ordered),
            **{key: ordered[math.ceil(q * len(ordered)) - 1]
               for key, q in (("p50", .5), ("p95", .95), ("p99", .99), ("max", 1))}}
