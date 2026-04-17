#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPORT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
PROJECT_DIR="$(cd "${REPORT_DIR}/.." && pwd)"
PYTHON_BIN="${PROJECT_DIR}/.venv/bin/python"

if [[ ! -x "${PYTHON_BIN}" ]]; then
  echo "Expected local virtualenv Python at ${PYTHON_BIN}" >&2
  exit 1
fi

"${PYTHON_BIN}" "${REPORT_DIR}/scripts/generate_assets.py"

cd "${REPORT_DIR}"
latexmk -pdf main.tex

echo "Built PDF: ${REPORT_DIR}/build/main.pdf"
