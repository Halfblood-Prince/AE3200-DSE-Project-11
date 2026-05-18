"""
tradeoff_sensitivity.py  –  Generic Trade-Off Sensitivity Analysis Tool
========================================================================
Usage:
    python tradeoff_sensitivity.py <config.json>

The JSON config file defines the trade-off table. Example:

{
  "title": "Propeller Design",
  "criteria": ["Thrust Coefficient", "Stability", "Efficiency", "Complexity"],
  "criteria_latex": ["C_T", "\\text{stab}", "\\text{eff}", "\\text{comp}"],
  "weights":  [2, 4, 3, 4],
  "options":  ["Conventional Propeller", "Toroidal Propeller"],
  "scores": [
    [3, 4],
    [3, 4],
    [3, 1],
    [5, 2]
  ],
  "adjustment_criterion_index": 3,
  "latex": {
    "label":   "propDesign",
    "caption_title": "propeller design trade-off",
    "subsection_title": "Propeller Design",
    "subsection_label": "subsec:propDesign_Sensitivity"
  },
  "weight_range": [1, 5],
  "score_range":  [1, 5],
  "monte_carlo_n": 50000
}

  title                      – used in chart titles and output file names
  criteria                   – criterion names (rows)
  criteria_latex             – LaTeX subscript for each weight column header,
                               e.g. "C_T" → $w_{C_T}$  (auto-generated if omitted)
  weights                    – baseline weight per criterion
  options                    – option/alternative names (columns)
  scores                     – 2-D list: rows = criteria, columns = options
  adjustment_criterion_index – index of the criterion that absorbs ±1 changes
                               when other weights are varied (default: last)
  latex.label                – prefix for \\label{tab:<label>_...}
  latex.caption_title        – phrase used inside \\caption{...}
  latex.subsection_title     – title for the \\subsection heading
  latex.subsection_label     – \\label for the \\subsection
  weight_range               – [min, max] allowed weight values  (default [1, 5])
  score_range                – [min, max] allowed score values   (default [1, 5])
  monte_carlo_n              – Monte Carlo sample count          (default 50 000)

Outputs (saved next to the config file):
    <title>_oat_sweep.png
    <title>_mc_weights.png
    <title>_mc_scores.png
    <title>_report.txt
    <title>_sensitivity.tex
"""

import sys
import json
import re
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt


# ──────────────────────────────────────────────────────────────────────────────
# HELPERS
# ──────────────────────────────────────────────────────────────────────────────

def safe_filename(text: str) -> str:
    return re.sub(r"[^\w]+", "_", text).strip("_").lower()


def wt(weights: np.ndarray, scores: np.ndarray) -> np.ndarray:
    return weights @ scores


def auto_latex_subscript(name: str) -> str:
    """Best-effort abbreviation: first word, lowercased, spaces stripped."""
    word = name.split()[0].lower()
    return f"\\text{{{word}}}"


# ──────────────────────────────────────────────────────────────────────────────
# LATEX GENERATION
# ──────────────────────────────────────────────────────────────────────────────

def generate_latex(cfg: dict, prefix: str) -> str:
    """
    Returns a complete LaTeX string containing:
      - \\subsection heading
      - Weight-variation table
      - Criterion-elimination table
    """
    criteria   = cfg["criteria"]
    weights    = np.array(cfg["weights"], dtype=float)
    options    = cfg["options"]
    scores     = np.array(cfg["scores"], dtype=float)
    n_c        = len(criteria)
    adj        = cfg.get("adjustment_criterion_index", n_c - 1)
    w_min, w_max = cfg.get("weight_range", [1, 5])

    lcfg       = cfg.get("latex", {})
    label_pfx  = lcfg.get("label", safe_filename(cfg["title"]))
    cap_title  = lcfg.get("caption_title", cfg["title"].lower())
    ss_title   = lcfg.get("subsection_title", cfg["title"])
    ss_label   = lcfg.get("subsection_label", f"subsec:{label_pfx}_Sensitivity")

    # LaTeX subscripts for weight column headers
    if "criteria_latex" in cfg:
        crit_lat = cfg["criteria_latex"]
    else:
        crit_lat = [auto_latex_subscript(c) for c in criteria]

    baseline   = wt(weights, scores)
    winner_idx = int(np.argmax(baseline))
    winner     = options[winner_idx]

    # ── helper: format a score row ────────────────────────────────────────────
    def score_cells(totals):
        return " & ".join(f"{int(t)}" for t in totals)

    def weight_cells(w):
        return " & ".join(f"{int(v)}" for v in w)

    # ── 1. Weight-variation table ─────────────────────────────────────────────
    # Column spec: criterion name | change | one col per weight | one col per option
    n_opt = len(options)
    col_spec = "ll" + "c" * n_c + "r" * n_opt

    # Header row – weight columns
    w_headers = " & ".join(f"$w_{{{s}}}$" for s in crit_lat)
    opt_headers = " & ".join(f"\\textbf{{{o}}}" for o in options)
    header = (
        f"    \\textbf{{Varied Criterion}} & \\textbf{{Change}}\n"
        f"    & {w_headers}\n"
        f"    & {opt_headers} \\\\"
    )

    var_rows = []
    # Baseline
    var_rows.append(
        f"    Baseline & --- & {weight_cells(weights)} & {score_cells(baseline)} \\\\"
    )
    var_rows.append("    \\midrule")

    adj_name   = criteria[adj]
    adj_reason = cfg.get("adjustment_reason", "")
    for c_idx, criterion in enumerate(criteria):
        first = True
        for delta in [+1, -1]:
            w = weights.copy()
            w[c_idx] += delta
            if c_idx != adj:
                w[adj] -= delta          # keep total sum constant
            w = np.clip(w, w_min, w_max)
            totals = wt(w, scores)
            sign = f"$+{delta}$" if delta > 0 else f"$-{abs(delta)}$"
            name_cell = criterion if first else ""
            first = False
            var_rows.append(
                f"    {name_cell} & {sign} & {weight_cells(w)} & {score_cells(totals)} \\\\"
            )

    var_body = "\n".join(var_rows)

    # ── 2. Criterion-elimination table ────────────────────────────────────────
    elim_col_spec = "l" + "c" * (n_opt + 1) + "l"
    elim_opt_headers = " & ".join(f"\\textbf{{{o}}}" for o in options)
    elim_header = (
        f"    \\textbf{{Eliminated Criterion}} & \\textbf{{Weight sum}}\n"
        f"    & {elim_opt_headers} & \\textbf{{Winner}} \\\\"
    )

    elim_rows = []
    # Baseline row
    elim_rows.append(
        f"    None (baseline) & {int(weights.sum())} & {score_cells(baseline)} & {winner} \\\\"
    )

    for c_idx, criterion in enumerate(criteria):
        w = weights.copy()
        w[c_idx] = 0
        totals = wt(w, scores)
        best = int(np.argmax(totals))
        if totals[best] == sorted(totals)[-2]:  # tie check
            result = "Tie"
        else:
            result = options[best]
        elim_rows.append(
            f"    {criterion} & {int(w.sum())} & {score_cells(totals)} & {result} \\\\"
        )

    elim_body = "\n".join(elim_rows)

    # ── Assemble full LaTeX ───────────────────────────────────────────────────
    tex = rf"""
\subsection{{Sensitivity Analysis}}
\label{{{ss_label}}}

The sensitivity analysis for the {cap_title} checks whether {winner} remains the preferred option when criterion weights are varied.
The baseline weighted scores are {" and ".join(f"{int(t)} for {o}" for o, t in zip(options, baseline))}.

Each criterion weight is varied by $\pm 1$ while the weight of \textit{{{adj_name}}} absorbs the compensating change, keeping the total weight sum constant.
{f"{adj_reason} Its weight is therefore considered the most subjective and is used as the compensating criterion." if adj_reason else ""}

\begin{{table}}[H]
    \centering
    \caption{{Sensitivity analysis: weight variation for {cap_title}}}
    \label{{tab:{label_pfx}_sensitivity_weights}}
    \begin{{tabular}}{{{col_spec}}}
    \toprule
{header}
    \\
    \midrule
{var_body}
    \bottomrule
    \end{{tabular}}
\end{{table}}

The second step eliminates each criterion by setting its weight to zero while keeping all other weights at their baseline values.
This tests whether the trade-off outcome depends on any single criterion.

\begin{{table}}[H]
    \centering
    \caption{{Sensitivity analysis: criterion elimination for {cap_title}}}
    \label{{tab:{label_pfx}_sensitivity_elimination}}
    \begin{{tabular}}{{{elim_col_spec}}}
    \toprule
{elim_header}
    \\
    \midrule
{elim_body}
    \bottomrule
    \end{{tabular}}
\end{{table}}
"""
    return tex.lstrip("\n")


# ──────────────────────────────────────────────────────────────────────────────
# NUMERICAL ANALYSIS
# ──────────────────────────────────────────────────────────────────────────────

def run_analysis(cfg: dict, out_dir: Path):
    title      = cfg["title"]
    criteria   = cfg["criteria"]
    weights    = np.array(cfg["weights"], dtype=float)
    options    = cfg["options"]
    scores     = np.array(cfg["scores"], dtype=float)
    w_min, w_max = cfg.get("weight_range", [1, 5])
    s_min, s_max = cfg.get("score_range",  [1, 5])
    N            = cfg.get("monte_carlo_n", 50_000)
    prefix       = out_dir / safe_filename(title)

    n_criteria = len(criteria)
    n_options  = len(options)

    assert scores.shape == (n_criteria, n_options), (
        f"scores must be {n_criteria}×{n_options}, got {scores.shape}")
    assert len(weights) == n_criteria, "weights length must match criteria"

    lines = []

    def log(text=""):
        print(text)
        lines.append(text)

    sep = "=" * 65

    # ── 1. Baseline ───────────────────────────────────────────────────────────
    baseline   = wt(weights, scores)
    winner_idx = int(np.argmax(baseline))
    col_w      = max(len(o) for o in options) + 2

    log(sep)
    log(f"TRADE-OFF: {title}")
    log(sep)
    log()
    log("1. BASELINE WEIGHTED SCORES")
    log(sep)
    for i, (opt, total) in enumerate(zip(options, baseline)):
        marker = "  <-- winner" if i == winner_idx else ""
        log(f"   {opt:<{col_w}}: {total:6.1f}{marker}")
    log(f"\n   Winner: {options[winner_idx]}\n")

    # ── 2. OAT Weight Sweep ───────────────────────────────────────────────────
    log(sep)
    log("2. ONE-AT-A-TIME WEIGHT SWEEP")
    log(sep)

    sweep  = np.arange(w_min, w_max + 0.01, 0.25)
    ncols  = min(3, n_criteria)
    nrows  = (n_criteria + ncols - 1) // ncols
    fig_oat, axes = plt.subplots(nrows, ncols, figsize=(5 * ncols, 4 * nrows))
    axes   = np.array(axes).flatten()
    palette    = plt.cm.tab10.colors
    opt_colors = [palette[i % 10] for i in range(n_options)]

    for c_idx, criterion in enumerate(criteria):
        ax = axes[c_idx]
        for a_idx, (opt, color) in enumerate(zip(options, opt_colors)):
            y = [wt(np.where(np.arange(n_criteria) == c_idx,
                             np.clip(w_val, w_min, w_max),
                             weights), scores)[a_idx]
                 for w_val in sweep]
            ax.plot(sweep, y, label=opt, color=color, linewidth=2)
        ax.axvline(weights[c_idx], color="grey", ls="--", alpha=0.6, label="Baseline")
        ax.set_title(criterion, fontsize=9, fontweight="bold")
        ax.set_xlabel("Weight")
        ax.set_ylabel("Total score")
        ax.legend(fontsize=7)
        ax.grid(True, alpha=0.3)
        log(f"   {criterion}: baseline weight = {weights[c_idx]:.0f}")

    for ax in axes[n_criteria:]:
        ax.set_visible(False)

    fig_oat.suptitle(f"OAT Weight Sweep – {title}", fontsize=12, fontweight="bold")
    plt.tight_layout()
    oat_path = f"{prefix}_oat_sweep.png"
    fig_oat.savefig(oat_path, dpi=150)
    plt.show()
    log(f"\n   Saved: {oat_path}\n")

    # ── 3. Rank-Reversal Thresholds ───────────────────────────────────────────
    log(sep)
    log("3. RANK-REVERSAL THRESHOLDS")
    log(sep)

    fine = np.arange(w_min, w_max + 0.001, 0.01)
    cw   = max(len(c) for c in criteria)
    for c_idx, criterion in enumerate(criteria):
        found = False
        for w_val in fine:
            w = weights.copy()
            w[c_idx] = w_val
            new_winner = int(np.argmax(wt(w, scores)))
            if new_winner != winner_idx:
                delta = w_val - weights[c_idx]
                log(f"   {criterion:<{cw}} : Δw = {delta:+.2f}  "
                    f"({weights[c_idx]:.0f} → {w_val:.2f})  "
                    f"→ {options[new_winner]}")
                found = True
                break
        if not found:
            log(f"   {criterion:<{cw}} : NO reversal in [{w_min}, {w_max}]")
    log()

    # ── 4. Monte Carlo – Weights ──────────────────────────────────────────────
    log(sep)
    log(f"4. MONTE CARLO WEIGHT PERTURBATION  (N = {N:,})")
    log(sep)

    rng       = np.random.default_rng(42)
    noise_w   = rng.uniform(-1, 1, size=(N, n_criteria))
    w_samples = np.clip(weights + noise_w, w_min, w_max)
    winners_w = np.argmax(w_samples @ scores, axis=1)
    win_w     = np.bincount(winners_w, minlength=n_options)
    pcts_w    = 100 * win_w / N

    for opt, pct in zip(options, pcts_w):
        log(f"   {opt:<{col_w}}: {pct:6.2f}%  {'█' * int(pct / 2)}")

    fig_mw, ax_mw = plt.subplots(figsize=(max(6, 2 * n_options + 2), 5))
    bars = ax_mw.bar(options, pcts_w, color=opt_colors, edgecolor="black")
    ax_mw.set_ylabel("Win probability (%)")
    ax_mw.set_ylim(0, 105)
    ax_mw.set_title(f"MC Weight Perturbation – {title}\n(N={N:,})", fontweight="bold")
    ax_mw.bar_label(bars, fmt="%.1f%%", fontsize=10, padding=3)
    ax_mw.grid(axis="y", alpha=0.3)
    plt.xticks(rotation=15, ha="right")
    plt.tight_layout()
    mw_path = f"{prefix}_mc_weights.png"
    fig_mw.savefig(mw_path, dpi=150)
    plt.show()
    log(f"\n   Saved: {mw_path}\n")

    # ── 5. Monte Carlo – Scores ───────────────────────────────────────────────
    log(sep)
    log(f"5. MONTE CARLO SCORE PERTURBATION  (N = {N:,})")
    log(sep)

    noise_s   = rng.choice([-1, 0, 1], size=(N, n_criteria, n_options))
    s_samples = np.clip(scores + noise_s, s_min, s_max)
    totals_s  = np.einsum("c,nco->no", weights, s_samples)
    winners_s = np.argmax(totals_s, axis=1)
    win_s     = np.bincount(winners_s, minlength=n_options)
    pcts_s    = 100 * win_s / N

    for opt, pct in zip(options, pcts_s):
        log(f"   {opt:<{col_w}}: {pct:6.2f}%  {'█' * int(pct / 2)}")

    fig_ms, ax_ms = plt.subplots(figsize=(max(6, 2 * n_options + 2), 5))
    bars3 = ax_ms.bar(options, pcts_s, color=opt_colors, edgecolor="black")
    ax_ms.set_ylabel("Win probability (%)")
    ax_ms.set_ylim(0, 105)
    ax_ms.set_title(f"MC Score Perturbation – {title}\n(N={N:,})", fontweight="bold")
    ax_ms.bar_label(bars3, fmt="%.1f%%", fontsize=10, padding=3)
    ax_ms.grid(axis="y", alpha=0.3)
    plt.xticks(rotation=15, ha="right")
    plt.tight_layout()
    ms_path = f"{prefix}_mc_scores.png"
    fig_ms.savefig(ms_path, dpi=150)
    plt.show()
    log(f"\n   Saved: {ms_path}\n")

    # ── 6. Summary ────────────────────────────────────────────────────────────
    log(sep)
    log("6. SUMMARY")
    log(sep)
    runner_up = int(np.argsort(baseline)[-2])
    log(f"   Winner          : {options[winner_idx]} (score {baseline[winner_idx]:.1f})")
    log(f"   Runner-up       : {options[runner_up]} (score {baseline[runner_up]:.1f})")
    log(f"   Margin          : {baseline[winner_idx] - baseline[runner_up]:.1f} points")
    log(f"   MC weight win % : {pcts_w[winner_idx]:.1f}%")
    log(f"   MC score  win % : {pcts_s[winner_idx]:.1f}%")
    log(sep)

    # ── Save text report ──────────────────────────────────────────────────────
    report_path = f"{prefix}_report.txt"
    Path(report_path).write_text("\n".join(lines))
    log(f"\n   Full report saved: {report_path}")

    # ── Generate LaTeX ────────────────────────────────────────────────────────
    tex = generate_latex(cfg, str(prefix))
    tex_path = f"{prefix}_sensitivity.tex"
    Path(tex_path).write_text(tex)
    print(f"   LaTeX saved     : {tex_path}")


# ──────────────────────────────────────────────────────────────────────────────
# ENTRY POINT
# ──────────────────────────────────────────────────────────────────────────────

def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)

    config_path = Path(sys.argv[1])
    if not config_path.exists():
        print(f"Error: file not found – {config_path}")
        sys.exit(1)

    with open(config_path) as f:
        cfg = json.load(f)

    run_analysis(cfg, config_path.parent)


if __name__ == "__main__":
    main()
