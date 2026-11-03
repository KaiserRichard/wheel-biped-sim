#!/usr/bin/env bash
set -euo pipefail
python check.py
python -m pytest -q
