# AE3200 DSE — Project 11: Tradeoff Sensitivity Analysis

This branch contains the code used to do sensitivity analysis of the trade-off criteria weights.

## Quickstart

1. Install dependencies:
```
pip install -r requirements.txt
```

2. Run the analysis with the example config:
```
python tradeoff_sensitivity.py propeller_design.json
```

Outputs
- PNG figures: `<title>_oat_sweep.png`, `<title>_mc_weights.png`, `<title>_mc_scores.png`
- Text report: `<title>_report.txt`
- LaTeX snippet: `<title>_sensitivity.tex`

Dependencies
- Python 3.8+
- numpy
- matplotlib