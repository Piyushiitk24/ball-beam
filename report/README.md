# Ball and Beam LaTeX Report

This report setup is self-contained inside the `report/` folder. It reads the experiment CSVs from this repository, generates figures and LaTeX-ready tables locally, and builds the final PDF into `report/build/main.pdf`.

## Folder layout

```text
report/
├── main.tex
├── preamble.tex
├── metadata.tex
├── chapters/
├── bib/
├── figures/
├── tables/
├── build/
└── scripts/
```

## Required tools

- A TeX distribution with `latexmk`, `pdflatex`, `biber`, and the packages used in `preamble.tex`
- Python in the repo-local `.venv`
- Python packages from `report/requirements.txt`

On macOS, a standard MacTeX installation already provides `latexmk`, `pdflatex`, `biber`, and the required LaTeX packages.

## Install Python dependencies

The report uses the repository's local virtual environment. Do not install Python packages globally.

```bash
cd /Users/piyush/Documents/PlatformIO/Projects/ball-beam
./.venv/bin/python -m pip install -r report/requirements.txt
```

## Build from the terminal

The simplest path is the wrapper script:

```bash
cd /Users/piyush/Documents/PlatformIO/Projects/ball-beam
./report/scripts/build_report.sh
```

That script:

1. Runs `report/scripts/generate_assets.py`
2. Writes plots into `report/figures/`
3. Writes LaTeX table fragments into `report/tables/`
4. Builds the document with `latexmk`

The final PDF is:

```text
report/build/main.pdf
```

## Build in VS Code

This repository includes `.vscode/settings.json` entries for LaTeX Workshop.

1. Open `report/main.tex`
2. Run `LaTeX Workshop: Build LaTeX project`
3. View the PDF in the VS Code tab viewer

LaTeX Workshop uses the local `.latexmkrc`, so auxiliary files and the PDF stay inside `report/build/`.

## Notes

- The analysis script reads telemetry directly from `telemetry_logs/20260411_142117/telemetry.csv`.
- The calibration plots are generated from `calibration_logs/combined_20260407_forward_reverse/combined_cleaned_calibration.csv`.
- If generated figures, tables, or metadata are missing, the LaTeX document still compiles by using placeholder blocks or safe defaults.
- Bibliography output prefers `biblatex` with `biber` in IEEE style. If `biber` output is unavailable, the document falls back to the manual bibliography file in `report/bib/manual_bibliography.tex`.
