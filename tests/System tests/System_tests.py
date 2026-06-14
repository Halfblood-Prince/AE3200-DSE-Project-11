import pathlib
import sys

import matplotlib.pyplot as plt
import numpy as np
import pytest
from matplotlib.patches import Patch

from SALib.analyze.morris import analyze as morris_analyze
from SALib.sample.morris import sample as morris_sample



REPO_ROOT = pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT))

from Sizing_tool import run_sizing_tool
from propulsion.propClass import load_propeller_dict


def _get_sizing_inputs():
    return {
        "coaxial": True,
        "N_prop": 8,
        "flight_time": 7 / 60,
        "P_payload": 150,
        "P_avionics": 0,
        "Lipo_spec_energy": 230,
        "M_pay": 1.418,
    }


def _run_sizing_trace(MTOM_guess=1, invalid_mtom=None, **overrides):
    inputs = _get_sizing_inputs()
    inputs.update(overrides)
    trace_data = run_sizing_tool(
        MTOM_guess,
        test=True,
        return_trace_metadata=True,
        **inputs,
    )
    if isinstance(trace_data, dict):
        trace = {
            "mtom_list": list(trace_data["mtom_list"]),
            "p_prop_list": list(trace_data["p_prop_list"]),
            "m_battery_list": list(trace_data["m_battery_list"]),
            "m_structures_list": list(trace_data["m_structures_list"]),
            "residual_list": list(trace_data["residual_list"]),
            "final_mtom": trace_data["final_mtom"],
            "is_valid": trace_data["is_valid"],
            "failure_iteration": trace_data.get("failure_iteration"),
            "failure_mtom": trace_data.get("failure_mtom"),
        }
        if invalid_mtom is not None and not trace["is_valid"]:
            failure_anchor = trace["failure_mtom"]
            if failure_anchor is None:
                failure_anchor = trace["mtom_list"][-1] if trace["mtom_list"] else MTOM_guess
            trace["mtom_list"].append(float(invalid_mtom))
            trace["residual_list"].append(abs(float(invalid_mtom) - float(failure_anchor)))
            trace["final_mtom"] = float(invalid_mtom)
        return trace
    if trace_data is False:
        mtom_list = [MTOM_guess]
        residual_list = []
        final_mtom = np.nan
        if invalid_mtom is not None:
            final_mtom = float(invalid_mtom)
            mtom_list.append(final_mtom)
            residual_list.append(abs(final_mtom - MTOM_guess))
        return {
            "mtom_list": mtom_list,
            "p_prop_list": [],
            "m_battery_list": [],
            "m_structures_list": [],
            "residual_list": residual_list,
            "final_mtom": final_mtom,
            "is_valid": False,
            "failure_iteration": 1,
            "failure_mtom": MTOM_guess,
        }

    (
        mtom_list,
        p_prop_list,
        m_battery_list,
        m_structures_list,
        residual_list,
        final_mtom,
    ) = trace_data
    return {
        "mtom_list": mtom_list,
        "p_prop_list": p_prop_list,
        "m_battery_list": m_battery_list,
        "m_structures_list": m_structures_list,
        "residual_list": residual_list,
        "final_mtom": final_mtom,
        "is_valid": True,
        "failure_iteration": None,
        "failure_mtom": None,
    }


def _coerce_even_prop_count(value):
    prop_count = int(round(value))
    prop_count = max(4, prop_count)
    if prop_count % 2 != 0:
        prop_count += 1
    return prop_count


def _coerce_coaxial_switch(value):
    return bool(int(round(float(value))))


def _coerce_parameter_value(name, value):
    if name == "coaxial":
        return _coerce_coaxial_switch(value)
    return float(value)


def _format_parameter_value(name, value):
    if name == "coaxial":
        return str(bool(value))
    return f"{float(value):.3f}"


def _trace_impact_score(trace_a, trace_b, invalid_penalty):
    if (not trace_a["is_valid"]) or (not trace_b["is_valid"]):
        return float(invalid_penalty)

    mtom_a = np.asarray(trace_a["mtom_list"], dtype=float)
    mtom_b = np.asarray(trace_b["mtom_list"], dtype=float)
    delta_a = np.diff(mtom_a)
    delta_b = np.diff(mtom_b)
    if delta_a.size == 0:
        delta_a = np.array([0.0], dtype=float)
    if delta_b.size == 0:
        delta_b = np.array([0.0], dtype=float)

    mtom_len = max(len(mtom_a), len(mtom_b))
    delta_len = max(len(delta_a), len(delta_b))

    mtom_a = np.pad(mtom_a, (0, mtom_len - len(mtom_a)), mode="edge")
    mtom_b = np.pad(mtom_b, (0, mtom_len - len(mtom_b)), mode="edge")
    delta_a = np.pad(delta_a, (0, delta_len - len(delta_a)), mode="edge")
    delta_b = np.pad(delta_b, (0, delta_len - len(delta_b)), mode="edge")

    final_delta = abs(trace_a["final_mtom"] - trace_b["final_mtom"])
    path_delta = np.sum(np.abs(mtom_a - mtom_b))
    step_delta = np.sum(np.abs(delta_a - delta_b))
    return float(final_delta + path_delta + step_delta)


def _failure_plot_coordinates(trace, fallback_mtom):
    failure_iteration = trace.get("failure_iteration")
    if failure_iteration is None:
        failure_iteration = len(trace["mtom_list"]) if trace["mtom_list"] else 1

    failure_mtom = trace.get("failure_mtom")
    if failure_mtom is None:
        failure_mtom = trace["mtom_list"][-1] if trace["mtom_list"] else fallback_mtom

    return int(failure_iteration), float(failure_mtom)


def _trace_plot_coordinates(trace, fallback_mtom):
    mtom_trace = list(trace["mtom_list"])
    iterations = list(range(1, len(mtom_trace) + 1))

    if not trace["is_valid"]:
        failure_iteration, failure_mtom = _failure_plot_coordinates(trace, fallback_mtom)
        if not iterations or failure_iteration > iterations[-1]:
            iterations.append(failure_iteration)
            mtom_trace.append(failure_mtom)
        elif failure_iteration == iterations[-1]:
            mtom_trace[-1] = failure_mtom

    return iterations, mtom_trace


def _assert_converged_trace(trace, tolerance=0.001):
    assert trace["is_valid"], "Sizing trace should converge for this scenario"
    assert trace["residual_list"], "Converged traces should include a residual history"
    assert trace["residual_list"][-1] <= tolerance, (
        f"Final residual should be <= {tolerance}"
    )


def _assert_expected_endpoint_response(parameter_name, low_trace, high_trace):
    low_valid = low_trace["is_valid"]
    high_valid = high_trace["is_valid"]

    if parameter_name == "Lipo_spec_energy":
        if low_valid and not high_valid:
            raise AssertionError(
                "Higher battery specific energy should not make the design infeasible "
                "when the lower endpoint is feasible"
            )
        if not low_valid or not high_valid:
            return

        endpoint_delta = high_trace["final_mtom"] - low_trace["final_mtom"]
        assert endpoint_delta < 0, (
            "Higher battery specific energy should reduce MTOM"
        )
        return

    if not low_valid and high_valid:
        raise AssertionError(
            f"{parameter_name} should not become feasible only at the higher endpoint"
        )
    if not low_valid or not high_valid:
        return

    endpoint_delta = high_trace["final_mtom"] - low_trace["final_mtom"]
    expected_messages = {
        "flight_time": "Longer flight time should increase MTOM",
        "P_payload": "Higher payload power should increase MTOM",
        "P_avionics": "Higher avionics power should increase MTOM",
        "M_pay": "Higher payload mass should increase MTOM",
    }
    assert endpoint_delta > 0, expected_messages[parameter_name]


def _assert_nominal_trace_bounds(trace, mtom_bounds, max_iterations):
    _assert_converged_trace(trace)
    assert mtom_bounds[0] <= trace["final_mtom"] <= mtom_bounds[1], (
        f"Final MTOM should stay within {mtom_bounds}"
    )
    assert len(trace["mtom_list"]) <= max_iterations, (
        f"Sizing should converge within {max_iterations} iterations"
    )


def _style_iteration_axis(ax, title, ylabel, iterations):
    ax.set_title(title, loc="left", fontsize=11, fontweight="bold")
    ax.set_xlabel("Iteration")
    ax.set_ylabel(ylabel)
    if iterations:
        ax.set_xticks(iterations)
    ax.grid(True, linestyle="--", linewidth=0.7, alpha=0.35)
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)


def _plot_iteration_series(
    ax,
    iterations,
    values,
    *,
    color,
    marker,
    label,
    annotate_fmt="{:.3f}",
    linewidth=2.2,
):
    if not iterations:
        ax.text(
            0.5,
            0.5,
            "Converged in one iteration",
            transform=ax.transAxes,
            ha="center",
            va="center",
            fontsize=10,
            color="0.4",
        )
        return

    ax.plot(
        iterations,
        values,
        color=color,
        linewidth=linewidth,
        marker=marker,
        markersize=5,
        markerfacecolor="white",
        markeredgecolor=color,
        markeredgewidth=1.2,
        label=label,
    )
    ax.scatter([iterations[-1]], [values[-1]], color=color, s=46, zorder=3)
    ax.annotate(
        annotate_fmt.format(values[-1]),
        (iterations[-1], values[-1]),
        xytext=(6, 6),
        textcoords="offset points",
        fontsize=8,
        color=color,
    )
    ax.legend(frameon=False, loc="best")


def _plot_delta_series(
    ax,
    iterations,
    values,
    *,
    color,
    marker,
    label,
    annotate_fmt="{:.3f}",
):
    ax.axhline(0, color="0.5", linewidth=1.0, alpha=0.7)
    _plot_iteration_series(
        ax,
        iterations,
        values,
        color=color,
        marker=marker,
        label=label,
        annotate_fmt=annotate_fmt,
        linewidth=2.0,
    )


def test_SIZE_ST_01():
    # Base run test
    inputs = _get_sizing_inputs()
    assert run_sizing_tool(1, **inputs)


# def test_SIZE_ST_02(MTOM_guess=1, plot=False):
#     # Output convergence trend for MTOM and propulsive power
#     trace = _run_sizing_trace(MTOM_guess)
#     assert trace["is_valid"], "Default sizing run should be valid"
#     mtom_list = trace["mtom_list"]
#     p_prop_list = trace["p_prop_list"]
#     m_battery_list = trace["m_battery_list"]
#     m_structures_list = trace["m_structures_list"]
#     residual_list = trace["residual_list"]

#     iterations = list(range(1, len(mtom_list) + 1))
#     mtom_deltas = [mtom_list[i] - mtom_list[i - 1] for i in range(1, len(mtom_list))]
#     p_prop_deltas = [p_prop_list[i] - p_prop_list[i - 1] for i in range(1, len(p_prop_list))]
#     delta_iterations = list(range(2, len(mtom_list) + 1))

#     assert len(mtom_list) > 0, "Sizing tool should perform at least one iteration"
#     assert len(mtom_list) == len(p_prop_list) == len(m_battery_list) == len(m_structures_list) == len(residual_list)
#     assert all(np.isfinite(value) for value in mtom_list), "MTOM history should stay finite"
#     assert all(np.isfinite(value) for value in p_prop_list), "Power history should stay finite"
#     assert residual_list[-1] <= 0.001, "Final MTOM residual should satisfy the solver tolerance"
#     assert residual_list[-1] <= residual_list[0], "Final residual should improve from the initial residual"

#     if plot:
#         fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(
#             2,
#             2,
#             figsize=(13, 8),
#             constrained_layout=True,
#         )
#         fig.suptitle(
#             "ST_02 Convergence Overview\n"
#             f"MTOM guess = {MTOM_guess:.3f} kg, final MTOM = {trace['final_mtom']:.3f} kg, "
#             f"final residual = {residual_list[-1]:.4g}",
#             fontsize=14,
#             fontweight="bold",
#         )

#         _plot_iteration_series(
#             ax1,
#             iterations,
#             mtom_list,
#             color="tab:blue",
#             marker="o",
#             label="MTOM estimate",
#         )
#         _style_iteration_axis(ax1, "MTOM History", "MTOM (kg)", iterations)

#         _plot_iteration_series(
#             ax2,
#             iterations,
#             p_prop_list,
#             color="tab:green",
#             marker="s",
#             label="Propulsive power",
#             annotate_fmt="{:.1f}",
#         )
#         _style_iteration_axis(ax2, "Propulsive Power History", "Power (W)", iterations)

#         _plot_delta_series(
#             ax3,
#             delta_iterations,
#             mtom_deltas,
#             color="tab:blue",
#             marker="o",
#             label="Step change in MTOM",
#         )
#         _style_iteration_axis(ax3, "MTOM Step Change", "Delta MTOM (kg)", delta_iterations)

#         _plot_delta_series(
#             ax4,
#             delta_iterations,
#             p_prop_deltas,
#             color="tab:green",
#             marker="s",
#             label="Step change in power",
#             annotate_fmt="{:.1f}",
#         )
#         _style_iteration_axis(
#             ax4,
#             "Propulsive Power Step Change",
#             "Delta power (W)",
#             delta_iterations,
#         )

#         plt.show()


# def test_SIZE_ST_03(MTOM_guess=1, plot=False):
#     # Output change graphs for battery and structural mass
#     trace = _run_sizing_trace(MTOM_guess)
#     assert trace["is_valid"], "Default sizing run should be valid"
#     mtom_list = trace["mtom_list"]
#     p_prop_list = trace["p_prop_list"]
#     m_battery_list = trace["m_battery_list"]
#     m_structures_list = trace["m_structures_list"]
#     residual_list = trace["residual_list"]

#     iterations = list(range(1, len(m_battery_list) + 1))
#     battery_deltas = [
#         m_battery_list[i] - m_battery_list[i - 1] for i in range(1, len(m_battery_list))
#     ]
#     structures_deltas = [
#         m_structures_list[i] - m_structures_list[i - 1]
#         for i in range(1, len(m_structures_list))
#     ]
#     delta_iterations = list(range(2, len(m_battery_list) + 1))

#     assert len(m_battery_list) > 0, "Sizing tool should perform at least one iteration"
#     assert len(mtom_list) == len(p_prop_list) == len(m_battery_list) == len(m_structures_list) == len(residual_list)
#     assert all(np.isfinite(value) for value in m_battery_list), "Battery-mass history should stay finite"
#     assert all(np.isfinite(value) for value in m_structures_list), "Structure-mass history should stay finite"
#     assert all(value > 0 for value in m_battery_list), "Battery mass should remain positive"
#     assert all(value > 0 for value in m_structures_list), "Structure mass should remain positive"
#     assert residual_list[-1] <= 0.001, "Final MTOM residual should satisfy the solver tolerance"
#     assert residual_list[-1] <= residual_list[0], "Final residual should improve from the initial residual"

#     if plot:
#         fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(
#             2,
#             2,
#             figsize=(13, 8),
#             constrained_layout=True,
#         )
#         fig.suptitle(
#             "ST_03 Mass Convergence Overview\n"
#             f"MTOM guess = {MTOM_guess:.3f} kg, final battery mass = {m_battery_list[-1]:.3f} kg, "
#             f"final structure mass = {m_structures_list[-1]:.3f} kg",
#             fontsize=14,
#             fontweight="bold",
#         )

#         _plot_iteration_series(
#             ax1,
#             iterations,
#             m_battery_list,
#             color="tab:orange",
#             marker="o",
#             label="Battery mass",
#         )
#         _style_iteration_axis(ax1, "Battery Mass History", "Battery mass (kg)", iterations)

#         _plot_iteration_series(
#             ax2,
#             iterations,
#             m_structures_list,
#             color="tab:purple",
#             marker="s",
#             label="Structure mass",
#         )
#         _style_iteration_axis(ax2, "Structure Mass History", "Structure mass (kg)", iterations)

#         _plot_delta_series(
#             ax3,
#             delta_iterations,
#             battery_deltas,
#             color="tab:orange",
#             marker="o",
#             label="Step change in battery mass",
#         )
#         _style_iteration_axis(
#             ax3,
#             "Battery Mass Step Change",
#             "Delta battery mass (kg)",
#             delta_iterations,
#         )

#         _plot_delta_series(
#             ax4,
#             delta_iterations,
#             structures_deltas,
#             color="tab:purple",
#             marker="s",
#             label="Step change in structure mass",
#         )
#         _style_iteration_axis(
#             ax4,
#             "Structure Mass Step Change",
#             "Delta structure mass (kg)",
#             delta_iterations,
#         )

#         plt.show()
def test_SIZE_ST_02(MTOM_guess=1, plot=False):
    # Output convergence trend for MTOM and propulsive power
    trace = _run_sizing_trace(MTOM_guess)
    assert trace["is_valid"], "Default sizing run should be valid"
    mtom_list = trace["mtom_list"]
    p_prop_list = trace["p_prop_list"]
    m_battery_list = trace["m_battery_list"]
    m_structures_list = trace["m_structures_list"]
    residual_list = trace["residual_list"]

    iterations = list(range(1, len(mtom_list) + 1))
    mtom_deltas = [mtom_list[i] - mtom_list[i - 1] for i in range(1, len(mtom_list))]
    p_prop_deltas = [p_prop_list[i] - p_prop_list[i - 1] for i in range(1, len(p_prop_list))]
    delta_iterations = list(range(2, len(mtom_list) + 1))

    assert len(mtom_list) > 0, "Sizing tool should perform at least one iteration"
    assert len(mtom_list) == len(p_prop_list) == len(m_battery_list) == len(m_structures_list) == len(residual_list)
    assert all(np.isfinite(value) for value in mtom_list), "MTOM history should stay finite"
    assert all(np.isfinite(value) for value in p_prop_list), "Power history should stay finite"
    assert residual_list[-1] <= 0.001, "Final MTOM residual should satisfy the solver tolerance"
    assert residual_list[-1] <= residual_list[0], "Final residual should improve from the initial residual"

    if plot:
        tolerance = 0.001
        plotting_floor = tolerance / 10.0

        mtom_array = np.asarray(mtom_list, dtype=float)
        p_prop_array = np.asarray(p_prop_list, dtype=float)

        normalized_mtom = mtom_array / mtom_array[-1]
        normalized_p_prop = p_prop_array / p_prop_array[-1]

        mtom_relative_residuals = np.abs(np.diff(mtom_array)) / np.maximum(
            np.abs(mtom_array[1:]),
            np.finfo(float).eps,
        )
        p_prop_relative_residuals = np.abs(np.diff(p_prop_array)) / np.maximum(
            np.abs(p_prop_array[1:]),
            np.finfo(float).eps,
        )

        plotted_mtom_residuals = np.maximum(
            mtom_relative_residuals,
            plotting_floor,
        )
        plotted_p_prop_residuals = np.maximum(
            p_prop_relative_residuals,
            plotting_floor,
        )

        fig, (ax1, ax2) = plt.subplots(
            2,
            1,
            figsize=(8, 7),
            sharex=True,
            constrained_layout=True,
        )

        fig.suptitle(
            "ST_02 Top-Level Output Convergence\n"
            f"MTOM guess = {MTOM_guess:.3f} kg, "
            f"final MTOM = {trace['final_mtom']:.3f} kg, "
            f"final residual = {residual_list[-1]:.4g}",
            fontsize=14,
            fontweight="bold",
        )

        ax1.plot(
            iterations,
            normalized_mtom,
            color="tab:blue",
            marker="o",
            linewidth=2,
            label="MTOM",
        )
        ax1.plot(
            iterations,
            normalized_p_prop,
            color="tab:green",
            marker="s",
            linewidth=2,
            label="Propulsive power",
        )
        ax1.axhline(
            1.0,
            color="black",
            linestyle="--",
            linewidth=1.2,
            label="Final converged value",
        )
        ax1.set_ylabel("Normalized output (-)")
        ax1.set_title("Normalized Output History")
        ax1.grid(True, alpha=0.3)
        ax1.legend(frameon=False, ncols=3)

        ax2.semilogy(
            delta_iterations,
            plotted_mtom_residuals,
            color="tab:blue",
            marker="o",
            linewidth=2,
            label="MTOM residual",
        )
        ax2.semilogy(
            delta_iterations,
            plotted_p_prop_residuals,
            color="tab:green",
            marker="s",
            linewidth=2,
            label="Propulsive-power residual",
        )
        ax2.axhline(
            tolerance,
            color="black",
            linestyle="--",
            linewidth=1.2,
            label=f"Tolerance = {tolerance:.0e}",
        )
        ax2.set_xlabel("Iteration (-)")
        ax2.set_ylabel("Relative successive residual (-)")
        ax2.set_title("Output Residual Convergence")
        ax2.set_xticks(iterations)
        ax2.grid(True, which="both", alpha=0.3)
        ax2.legend(frameon=False)

        plt.show()


def test_SIZE_ST_03(MTOM_guess=1, plot=False):
    # Output change graphs for battery and structural mass
    trace = _run_sizing_trace(MTOM_guess)
    assert trace["is_valid"], "Default sizing run should be valid"
    mtom_list = trace["mtom_list"]
    p_prop_list = trace["p_prop_list"]
    m_battery_list = trace["m_battery_list"]
    m_structures_list = trace["m_structures_list"]
    residual_list = trace["residual_list"]

    iterations = list(range(1, len(m_battery_list) + 1))
    battery_deltas = [
        m_battery_list[i] - m_battery_list[i - 1] for i in range(1, len(m_battery_list))
    ]
    structures_deltas = [
        m_structures_list[i] - m_structures_list[i - 1]
        for i in range(1, len(m_structures_list))
    ]
    delta_iterations = list(range(2, len(m_battery_list) + 1))

    assert len(m_battery_list) > 0, "Sizing tool should perform at least one iteration"
    assert len(mtom_list) == len(p_prop_list) == len(m_battery_list) == len(m_structures_list) == len(residual_list)
    assert all(np.isfinite(value) for value in m_battery_list), "Battery-mass history should stay finite"
    assert all(np.isfinite(value) for value in m_structures_list), "Structure-mass history should stay finite"
    assert all(value > 0 for value in m_battery_list), "Battery mass should remain positive"
    assert all(value > 0 for value in m_structures_list), "Structure mass should remain positive"
    assert residual_list[-1] <= 0.001, "Final MTOM residual should satisfy the solver tolerance"
    assert residual_list[-1] <= residual_list[0], "Final residual should improve from the initial residual"

    if plot:
        tolerance = 0.001
        plotting_floor = tolerance / 10.0

        battery_array = np.asarray(m_battery_list, dtype=float)
        structures_array = np.asarray(m_structures_list, dtype=float)

        battery_relative_residuals = np.abs(
            np.diff(battery_array)
        ) / np.maximum(
            np.abs(battery_array[1:]),
            np.finfo(float).eps,
        )

        structures_relative_residuals = np.abs(
            np.diff(structures_array)
        ) / np.maximum(
            np.abs(structures_array[1:]),
            np.finfo(float).eps,
        )

        plotted_battery_residuals = np.maximum(
            battery_relative_residuals,
            plotting_floor,
        )
        plotted_structures_residuals = np.maximum(
            structures_relative_residuals,
            plotting_floor,
        )

        fig, ax = plt.subplots(
            figsize=(8, 4.8),
            constrained_layout=True,
        )

        fig.suptitle(
            "ST_03 Internal Coupling Convergence\n"
            f"MTOM guess = {MTOM_guess:.3f} kg, "
            f"final battery mass = {m_battery_list[-1]:.3f} kg, "
            f"final structure mass = {m_structures_list[-1]:.3f} kg",
            fontsize=14,
            fontweight="bold",
        )

        ax.semilogy(
            delta_iterations,
            plotted_battery_residuals,
            color="tab:orange",
            marker="o",
            linewidth=2,
            label="Battery-mass residual",
        )
        ax.semilogy(
            delta_iterations,
            plotted_structures_residuals,
            color="tab:purple",
            marker="s",
            linewidth=2,
            label="Structure-mass residual",
        )
        ax.axhline(
            tolerance,
            color="black",
            linestyle="--",
            linewidth=1.2,
            label=f"Tolerance = {tolerance:.0e}",
        )

        ax.set_xlabel("Iteration (-)")
        ax.set_ylabel("Relative successive residual (-)")
        ax.set_title("Internal Coupling Residuals")
        ax.set_xticks(iterations)
        ax.grid(True, which="both", alpha=0.3)
        ax.legend(frameon=False)

        plt.show()


def _plot_st04_coaxial_axis(
    ax,
    *,
    baseline_architecture_delta,
    coaxial_final_delta_mean,
    coaxial_final_delta_median,
    coaxial_final_mtom_deltas,
    coaxial_plot_records,
):
    sample_count = len(coaxial_plot_records)
    invalid_true = sum(
        not record["coaxial_true"]["is_valid"] for record in coaxial_plot_records
    )
    invalid_flat = sum(
        not record["coaxial_false"]["is_valid"] for record in coaxial_plot_records
    )
    valid_records = []
    invalid_records = []
    for record in coaxial_plot_records:
        coaxial_trace = record["coaxial_true"]
        flat_trace = record["coaxial_false"]
        if coaxial_trace["is_valid"] and flat_trace["is_valid"]:
            valid_records.append(
                {
                    "sample_index": record["sample_index"],
                    "final_delta": (
                        flat_trace["final_mtom"] - coaxial_trace["final_mtom"]
                    ),
                }
            )
        else:
            invalid_records.append(record["sample_index"])

    valid_records.sort(key=lambda item: item["final_delta"])
    positions = np.arange(1, len(valid_records) + 1)
    sorted_final_deltas = np.asarray(
        [record["final_delta"] for record in valid_records],
        dtype=float,
    )
    delta_colors = [
        "tab:orange" if delta >= 0 else "tab:red"
        for delta in sorted_final_deltas
    ]
    ax.bar(
        positions,
        sorted_final_deltas,
        color=delta_colors,
        alpha=0.9,
    )
    ax.axhline(
        0.0,
        color="black",
        linewidth=1.1,
    )
    ax.axhline(
        baseline_architecture_delta,
        color="tab:blue",
        linestyle="--",
        linewidth=1.6,
        label=f"Baseline design = {baseline_architecture_delta:.3f} kg",
    )
    ax.axhline(
        coaxial_final_delta_mean,
        color="tab:green",
        linestyle=":",
        linewidth=1.8,
        label=f"Mean = {coaxial_final_delta_mean:.3f} kg",
    )
    ax.set_xlabel("Valid paired sample (sorted by flat - coaxial)")
    ax.set_ylabel("Final MTOM difference (kg)")
    ax.set_title(
        "Flat - Coaxial MTOM by Sample",
        fontsize=12,
        fontweight="bold",
    )
    if len(positions) <= 12:
        ax.set_xticks(positions)
    else:
        tick_indices = np.linspace(
            0,
            len(positions) - 1,
            num=min(6, len(positions)),
            dtype=int,
        )
        ax.set_xticks(positions[tick_indices])
    ax.set_ylim(-2.0, 2.)
    ax.grid(True, axis="y", alpha=0.35)

    summary_lines = [
        "Above zero means flat is heavier",
        f"Baseline design = {baseline_architecture_delta:.3f} kg",
        f"mean = {coaxial_final_delta_mean:.3f} kg",
        f"median = {coaxial_final_delta_median:.3f} kg",
        (
            f"flat heavier in "
            f"{int(np.sum(coaxial_final_mtom_deltas > 0))}/"
            f"{coaxial_final_mtom_deltas.size} valid pairs"
        ),
        (
            f"invalid pairs: coaxial={invalid_true}/{sample_count}, "
            f"flat={invalid_flat}/{sample_count}"
        ),
    ]
    if invalid_records:
        summary_lines.append(
            "invalid sample ids: "
            + ", ".join(str(sample_id) for sample_id in invalid_records)
        )

    ax.text(
        0.02,
        0.98,
        "\n".join(summary_lines),
        transform=ax.transAxes,
        va="top",
        ha="left",
        fontsize=8,
        bbox={
            "boxstyle": "round,pad=0.3",
            "facecolor": "white",
            "edgecolor": "0.8",
            "alpha": 0.92,
        },
    )
    ax.legend(
        handles=[
            Patch(
                facecolor="tab:orange",
                edgecolor="none",
                label="Flat heavier than coaxial",
            ),
            Patch(
                facecolor="tab:red",
                edgecolor="none",
                label="Flat lighter than coaxial",
            ),
            Patch(
                facecolor="0.8",
                edgecolor="none",
                label="Each bar = one valid ST_04 sample",
            ),
        ] + ax.lines,
        frameon=True,
        facecolor="white",
        edgecolor="0.82",
        framealpha=0.95,
        loc="lower right",
        ncol=1,
        fontsize=8,
    )


def test_SIZE_ST_04(MTOM_guess=1, plot=False):
    # Sensitivity test for converged MTOM with fixed MTOM guess, N_prop, and propeller file
    baseline_trace = _run_sizing_trace(MTOM_guess)
    _assert_converged_trace(baseline_trace)
    baseline_flat_trace = _run_sizing_trace(MTOM_guess, coaxial=False)
    _assert_converged_trace(baseline_flat_trace)
    baseline_architecture_delta = (
        baseline_flat_trace["final_mtom"] - baseline_trace["final_mtom"]
    )
    # Invalid runs represent infeasible designs, so they should dominate the
    # sensitivity metric instead of looking like mild perturbations.
    baseline_mtom = baseline_trace["final_mtom"]
    invalid_mtom_penalty = max(baseline_mtom * 25.0, baseline_mtom + 50.0)
    default_inputs = _get_sizing_inputs()
    problem = {
        "num_vars": 5,
        "names": [
            "flight_time",
            "P_payload",
            "P_avionics",
            "Lipo_spec_energy",
            "M_pay",
        ],
        "bounds": [
            [5 / 60, 14 / 60],
            [50, 500],
            [0, 200],
            [200, 320],
            [0.4, 2.5],
        ],
    }
    param_values = morris_sample(problem, N=4, num_levels=4, seed=0)
    mtom_outputs = []
    coaxial_deltas = []
    coaxial_final_mtom_deltas = []
    coaxial_invalid_cases = 0
    coaxial_plot_records = []
    endpoint_traces = {}

    for parameter_name, (lower_bound, upper_bound) in zip(problem["names"], problem["bounds"]):
        endpoint_traces[parameter_name] = {
            "low": _run_sizing_trace(
                MTOM_guess,
                **{parameter_name: _coerce_parameter_value(parameter_name, lower_bound)},
            ),
            "high": _run_sizing_trace(
                MTOM_guess,
                **{parameter_name: _coerce_parameter_value(parameter_name, upper_bound)},
            ),
        }

    for row in param_values:
        sampled_inputs = {
            "flight_time": float(row[0]),
            "P_payload": float(row[1]),
            "P_avionics": float(row[2]),
            "Lipo_spec_energy": float(row[3]),
            "M_pay": float(row[4]),
        }
        trace_selected = _run_sizing_trace(
            MTOM_guess,
            coaxial=default_inputs["coaxial"],
            **sampled_inputs,
        )
        trace_coaxial_true = _run_sizing_trace(
            MTOM_guess,
            coaxial=True,
            **sampled_inputs,
        )
        trace_coaxial_false = _run_sizing_trace(
            MTOM_guess,
            coaxial=False,
            **sampled_inputs,
        )
        coaxial_plot_records.append(
            {
                "sample_index": len(coaxial_plot_records) + 1,
                "sampled_inputs": sampled_inputs,
                "coaxial_true": trace_coaxial_true,
                "coaxial_false": trace_coaxial_false,
            }
        )
        mtom_outputs.append(
            _trace_impact_score(baseline_trace, trace_selected, invalid_mtom_penalty)
        )
        coaxial_deltas.append(
            _trace_impact_score(trace_coaxial_true, trace_coaxial_false, invalid_mtom_penalty)
        )
        if trace_coaxial_true["is_valid"] and trace_coaxial_false["is_valid"]:
            coaxial_final_mtom_deltas.append(
                trace_coaxial_false["final_mtom"] - trace_coaxial_true["final_mtom"]
            )
        else:
            coaxial_invalid_cases += 1

    mtom_outputs = np.asarray(mtom_outputs, dtype=float)
    coaxial_deltas = np.asarray(coaxial_deltas, dtype=float)
    coaxial_final_mtom_deltas = np.asarray(coaxial_final_mtom_deltas, dtype=float)
    sensitivity = morris_analyze(problem, param_values, mtom_outputs, num_levels=4)
    mu_star = np.asarray(sensitivity["mu_star"], dtype=float)
    sigma = np.asarray(sensitivity["sigma"], dtype=float)
    coaxial_invalid_fraction = coaxial_invalid_cases / len(coaxial_deltas)
    coaxial_mean_delta = float(np.mean(coaxial_deltas))
    coaxial_peak_delta = float(np.max(coaxial_deltas))
    coaxial_final_delta_mean = float(np.mean(coaxial_final_mtom_deltas))
    coaxial_final_delta_median = float(np.median(coaxial_final_mtom_deltas))
    flat_heavier_fraction = float(np.mean(coaxial_final_mtom_deltas > 0))
    # Treat coaxial as an architectural switch: its relevance is based on the
    # full trajectory change plus an added feasibility penalty when one branch
    # breaks the design.
    coaxial_mu_star = (
        coaxial_mean_delta
        + coaxial_peak_delta
        + coaxial_invalid_fraction * invalid_mtom_penalty
    )
    coaxial_sigma = float(np.std(coaxial_deltas))

    assert len(mtom_outputs) == len(param_values), "Sensitivity run should evaluate every sample"
    assert np.all(np.isfinite(mtom_outputs)), "Sensitivity outputs should stay finite"
    assert np.all(np.isfinite(coaxial_deltas)), "Coaxial paired-switch deltas should stay finite"
    assert mu_star.shape == (problem["num_vars"],), "Each sampled variable should get one mu_star value"
    assert sigma.shape == (problem["num_vars"],), "Each sampled variable should get one sigma value"
    assert np.all(np.isfinite(mu_star)), "Sensitivity indices should stay finite"
    assert np.all(np.isfinite(sigma)), "Sensitivity spread indices should stay finite"
    assert np.all(mu_star >= 0), "Morris mu_star values should be non-negative"
    assert np.all(sigma >= 0), "Morris sigma values should be non-negative"
    assert np.any(mu_star > 0), "At least one variable should influence MTOM"
    assert baseline_architecture_delta > 0, (
        "For the baseline mission, the flat configuration should converge "
        "to a higher MTOM than the coaxial configuration"
    )
    assert coaxial_final_mtom_deltas.size > 0, (
        "ST_04 should retain at least one valid coaxial-vs-flat paired sample"
    )
    assert coaxial_final_delta_mean > 0, (
        "Across valid ST_04 paired samples, flat should remain heavier on average "
        "than coaxial"
    )
    assert flat_heavier_fraction >= 0.75, (
        "Flat should be heavier in a clear majority of valid ST_04 paired samples"
    )

    for parameter_name, traces in endpoint_traces.items():
        _assert_expected_endpoint_response(
            parameter_name,
            traces["low"],
            traces["high"],
        )

    ranking_entries = list(zip(problem["names"], mu_star, sigma))
    ranking_entries.append(("coaxial", coaxial_mu_star, coaxial_sigma))
    ranking = sorted(
        ranking_entries,
        key=lambda item: item[1],
        reverse=True,
    )
    ranked_names = [item[0] for item in ranking]
    ranked_mu_star = [item[1] for item in ranking]

    if plot:
        plot_mode = "all" if plot is True else str(plot).strip().lower()

        if plot_mode == "coaxial":
            coaxial_fig, coaxial_ax = plt.subplots(
                figsize=(7.6, 4.8),
                constrained_layout=True,
            )
            _plot_st04_coaxial_axis(
                coaxial_ax,
                baseline_architecture_delta=baseline_architecture_delta,
                coaxial_final_delta_mean=coaxial_final_delta_mean,
                coaxial_final_delta_median=coaxial_final_delta_median,
                coaxial_final_mtom_deltas=coaxial_final_mtom_deltas,
                coaxial_plot_records=coaxial_plot_records,
            )
            plt.show()
            return

        ranking_fig, ranking_ax = plt.subplots(
            figsize=(10.5, 5.4),
            constrained_layout=True,
        )
        ranking_positions = np.arange(len(ranked_names))
        ranking_colors = [
            "#d17c24" if name == "coaxial" else "#4c78a8"
            for name in ranked_names
        ]
        ranking_bars = ranking_ax.barh(
            ranking_positions,
            ranked_mu_star,
            color=ranking_colors,
            edgecolor="white",
            linewidth=1.1,
            height=0.72,
        )
        ranking_ax.set_yticks(ranking_positions)
        ranking_ax.set_yticklabels(ranked_names)
        ranking_ax.invert_yaxis()
        ranking_ax.set_xlabel("Sensitivity score (higher = stronger MTOM effect)")
        ranking_ax.set_title(
            "ST_04 Sensitivity Ranking",
            loc="left",
            fontsize=13,
            fontweight="bold",
        )
        ranking_ax.grid(True, axis="x", linestyle="--", linewidth=0.8, alpha=0.35)
        ranking_ax.spines["top"].set_visible(False)
        ranking_ax.spines["right"].set_visible(False)
        ranking_ax.spines["left"].set_visible(False)
        max_ranked_mu_star = max(ranked_mu_star)
        ranking_ax.set_xlim(0.0, max_ranked_mu_star * 1.16)
        for bar, value in zip(ranking_bars, ranked_mu_star):
            ranking_ax.text(
                value + max_ranked_mu_star * 0.015,
                bar.get_y() + bar.get_height() / 2.0,
                f"{value:.1f}",
                va="center",
                ha="left",
                fontsize=9,
                color="0.25",
            )
        top_three = ranking[:3]
        delta_fig, axes = plt.subplots(
            1,
            len(top_three),
            figsize=(6 * len(top_three), 4),
            constrained_layout=True,
        )
        if len(top_three) == 1:
            axes = [axes]

        for ax, (parameter_name, _, _) in zip(axes, top_three):
            if parameter_name == "coaxial":
                _plot_st04_coaxial_axis(
                    ax,
                    baseline_architecture_delta=baseline_architecture_delta,
                    coaxial_final_delta_mean=coaxial_final_delta_mean,
                    coaxial_final_delta_median=coaxial_final_delta_median,
                    coaxial_final_mtom_deltas=coaxial_final_mtom_deltas,
                    coaxial_plot_records=coaxial_plot_records,
                )
                continue

            parameter_index = problem["names"].index(parameter_name)
            lower_bound, upper_bound = problem["bounds"][parameter_index]

            selected_value = default_inputs[parameter_name]
            scenario_traces = [
                (
                    f"Min = {_format_parameter_value(parameter_name, lower_bound)}",
                    _run_sizing_trace(
                        MTOM_guess,
                        **{parameter_name: _coerce_parameter_value(parameter_name, lower_bound)},
                    ),
                    "tab:blue",
                    "-",
                    1.8,
                    0.9,
                    2,
                ),
                (
                    f"Selected = {_format_parameter_value(parameter_name, selected_value)}",
                    baseline_trace,
                    "tab:red",
                    "-",
                    2.6,
                    1.0,
                    4,
                ),
                (
                    f"Max = {_format_parameter_value(parameter_name, upper_bound)}",
                    _run_sizing_trace(
                        MTOM_guess,
                        **{parameter_name: _coerce_parameter_value(parameter_name, upper_bound)},
                    ),
                    "tab:green",
                    "-.",
                    1.8,
                    0.9,
                    2,
                ),
            ]

            for label, trace, color, linestyle, linewidth, alpha, zorder in scenario_traces:
                iterations, mtom_trace = _trace_plot_coordinates(
                    trace,
                    MTOM_guess,
                )
                if not trace["is_valid"]:
                    label = f"{label} (invalid)"
                if mtom_trace:
                    ax.plot(
                        iterations,
                        mtom_trace,
                        marker="o",
                        color=color,
                        linestyle=linestyle,
                        linewidth=linewidth,
                        alpha=alpha,
                        label=label,
                        zorder=zorder,
                    )
                if not trace["is_valid"]:
                    failure_iteration, failure_mtom = _failure_plot_coordinates(
                        trace,
                        MTOM_guess,
                    )
                    ax.scatter(
                        [failure_iteration],
                        [failure_mtom],
                        marker="x",
                        s=90,
                        linewidths=2.0,
                        color=color,
                        alpha=alpha,
                        zorder=zorder + 1,
                    )

            ax.set_xlabel("Iteration")
            ax.set_ylabel("MTOM (kg)")
            ax.set_title(
                f"{parameter_name}\n"
                f"bounds=({_format_parameter_value(parameter_name, lower_bound)}, "
                f"{_format_parameter_value(parameter_name, upper_bound)}), "
                f"selected={_format_parameter_value(parameter_name, selected_value)}"
            )
            ax.grid(True)
            ax.legend()

        plt.show()


def plot_SIZE_ST_04_coaxial(MTOM_guess=1):
    test_SIZE_ST_04(MTOM_guess=MTOM_guess, plot="coaxial")

def test_SIZE_ST_05(plot=False):
    # Initial guess robustness / basin-of-convergence check
    initial_guesses = list(range(-5, 15))
    traces = [_run_sizing_trace(guess) for guess in initial_guesses]

    valid_pairs = [
        (guess, trace["final_mtom"])
        for guess, trace in zip(initial_guesses, traces)
        if trace["is_valid"]
    ]

    invalid_pairs = [
        (guess, trace)
        for guess, trace in zip(initial_guesses, traces)
        if not trace["is_valid"]
    ]

    assert valid_pairs, (
        "At least one initial guess should converge to a valid sizing result"
    )

    non_positive_invalid_guesses = [
        guess
        for guess, _trace in invalid_pairs
        if guess <= 0
    ]

    positive_valid_guesses = [
        pair[0]
        for pair in valid_pairs
        if pair[0] > 0
    ]

    positive_invalid_guesses = [
        guess
        for guess, _trace in invalid_pairs
        if guess > 0
    ]

    valid_results = np.asarray(
        [pair[1] for pair in valid_pairs],
        dtype=float,
    )

    positive_valid_results = np.asarray(
        [
            pair[1]
            for pair in valid_pairs
            if pair[0] > 0
        ],
        dtype=float,
    )

    assert non_positive_invalid_guesses == [
        guess
        for guess in initial_guesses
        if guess <= 0
    ], "Non-positive guesses should be rejected explicitly"

    assert positive_valid_guesses, (
        "At least one positive initial guess should converge"
    )

    assert np.all(np.isfinite(valid_results)), (
        "Valid initial guesses should produce finite MTOM results"
    )

    assert np.all(valid_results > 0), (
        "Valid initial guesses should produce positive MTOM results"
    )

    assert np.allclose(
        positive_valid_results,
        positive_valid_results[0],
        atol=0.001,
        rtol=0.0,
    ), (
        "All feasible positive guesses should converge "
        "to the same MTOM fixed point"
    )

    for guess, trace in zip(initial_guesses, traces):
        if trace["is_valid"]:
            _assert_converged_trace(
                trace,
                tolerance=0.001,
            )

    if positive_invalid_guesses:
        assert min(positive_invalid_guesses) > max(
            positive_valid_guesses
        ), (
            "Positive initial-guess failures should only appear "
            "beyond the feasible MTOM range"
        )

    if plot:
        tolerance = 0.001
        reference_mtom = positive_valid_results[0]

        valid_guesses = [
            guess
            for guess, trace in zip(initial_guesses, traces)
            if trace["is_valid"]
        ]

        valid_final_mtoms = [
            trace["final_mtom"]
            for trace in traces
            if trace["is_valid"]
        ]

        valid_iteration_counts = [
            len(trace["mtom_list"])
            for trace in traces
            if trace["is_valid"]
        ]

        invalid_guesses = [
            guess
            for guess, trace in zip(initial_guesses, traces)
            if not trace["is_valid"]
        ]

        maximum_mtom_difference = np.max(
            np.abs(
                np.asarray(valid_final_mtoms, dtype=float)
                - reference_mtom
            )
        )

        fig = plt.figure(
            figsize=(9, 8),
            constrained_layout=True,
        )

        grid = fig.add_gridspec(
            nrows=3,
            ncols=1,
            height_ratios=[3.0, 0.75, 2.3],
        )

        ax1 = fig.add_subplot(grid[0])
        ax1_zero = fig.add_subplot(
            grid[1],
            sharex=ax1,
        )
        ax2 = fig.add_subplot(
            grid[2],
            sharex=ax1,
        )

        fig.suptitle(
            "ST_05 Initial-Guess Robustness\n"
            f"Reference MTOM = {reference_mtom:.3f} kg, "
            f"maximum final variation = "
            f"{maximum_mtom_difference:.4g} kg",
            fontsize=14,
            fontweight="bold",
        )

        # ---------------------------------------------------------
        # Plot 1a: Zoomed converged-MTOM region
        # ---------------------------------------------------------
        ax1.axhspan(
            reference_mtom - tolerance,
            reference_mtom + tolerance,
            color="tab:green",
            alpha=0.25,
            label=r"$\pm 0.001$ kg tolerance",
            zorder=1,
        )

        ax1.axhline(
            reference_mtom,
            color="black",
            linestyle="--",
            linewidth=1.2,
            label="Reference solution",
            zorder=2,
        )

        ax1.plot(
            valid_guesses,
            valid_final_mtoms,
            marker="o",
            markersize=6,
            linewidth=2,
            color="tab:blue",
            label="Converged MTOM",
            zorder=3,
        )

        ax1.set_ylim(
            reference_mtom - 2.5 * tolerance,
            reference_mtom + 2.5 * tolerance,
        )

        ax1.set_ylabel("Converged MTOM (kg)")
        ax1.set_title("Final Solution Independence")
        ax1.grid(True, alpha=0.3)

        # Hide the lower edge because the axis is broken
        ax1.spines["bottom"].set_visible(False)
        ax1.tick_params(
            axis="x",
            which="both",
            bottom=False,
            labelbottom=False,
        )

        # ---------------------------------------------------------
        # Plot 1b: Invalid guesses at y = 0
        # ---------------------------------------------------------
        if invalid_guesses:
            ax1_zero.scatter(
                invalid_guesses,
                [0.0] * len(invalid_guesses),
                marker="x",
                s=70,
                linewidths=2,
                color="tab:red",
                label="Invalid initial guess",
                zorder=3,
            )

        ax1_zero.axhline(
            0.0,
            color="0.6",
            linewidth=0.8,
            zorder=1,
        )

        ax1_zero.set_ylim(-0.12, 0.12)
        ax1_zero.set_yticks([0.0])
        ax1_zero.set_ylabel("Invalid")
        ax1_zero.grid(True, axis="x", alpha=0.3)

        # Hide the upper edge because the axis is broken
        ax1_zero.spines["top"].set_visible(False)
        ax1_zero.tick_params(
            axis="x",
            which="both",
            bottom=False,
            labelbottom=False,
        )

        # Diagonal break marks
        break_size = 0.012

        ax1.plot(
            (-break_size, +break_size),
            (-break_size, +break_size),
            transform=ax1.transAxes,
            color="black",
            clip_on=False,
            linewidth=1.0,
        )
        ax1.plot(
            (1.0 - break_size, 1.0 + break_size),
            (-break_size, +break_size),
            transform=ax1.transAxes,
            color="black",
            clip_on=False,
            linewidth=1.0,
        )

        ax1_zero.plot(
            (-break_size, +break_size),
            (1.0 - break_size, 1.0 + break_size),
            transform=ax1_zero.transAxes,
            color="black",
            clip_on=False,
            linewidth=1.0,
        )
        ax1_zero.plot(
            (1.0 - break_size, 1.0 + break_size),
            (1.0 - break_size, 1.0 + break_size),
            transform=ax1_zero.transAxes,
            color="black",
            clip_on=False,
            linewidth=1.0,
        )

        # Combined legend from both broken-axis sections
        handles_top, labels_top = ax1.get_legend_handles_labels()
        handles_zero, labels_zero = ax1_zero.get_legend_handles_labels()

        ax1.legend(
            handles_top + handles_zero,
            labels_top + labels_zero,
            frameon=False,
            ncols=2,
            loc="upper left",
        )

        # ---------------------------------------------------------
        # Plot 2: Number of iterations versus initial guess
        # ---------------------------------------------------------
        ax2.plot(
            valid_guesses,
            valid_iteration_counts,
            marker="s",
            markersize=6,
            linewidth=2,
            color="tab:purple",
            label="Iterations to convergence",
            zorder=3,
        )

        if invalid_guesses:
            ax2.scatter(
                invalid_guesses,
                [0.0] * len(invalid_guesses),
                marker="x",
                s=70,
                linewidths=2,
                color="tab:red",
                label="Invalid initial guess",
                zorder=4,
            )

        maximum_iteration_count = max(valid_iteration_counts)

        ax2.set_ylim(
            -0.5,
            maximum_iteration_count + 1.0,
        )

        ax2.set_xlim(
            min(initial_guesses) - 0.5,
            max(initial_guesses) + 0.5,
        )

        ax2.set_xlabel("Initial MTOM guess (kg)")
        ax2.set_ylabel("Number of iterations (-)")
        ax2.set_title("Convergence Effort")
        ax2.set_xticks(initial_guesses)
        ax2.grid(True, alpha=0.3)
        ax2.legend(frameon=False)

        plt.show()


def test_SIZE_ST_06():
    # Nominal coaxial system regression check
    trace = _run_sizing_trace(1)

    _assert_nominal_trace_bounds(
        trace,
        mtom_bounds=(3.6, 4.0),
        max_iterations=8,
    )


def test_SIZE_ST_07():
    # Nominal non-coaxial configuration should also converge to a plausible MTOM
    trace = _run_sizing_trace(1, coaxial=False)

    _assert_nominal_trace_bounds(
        trace,
        mtom_bounds=(3.6, 4.2),
        max_iterations=8,
    )


def test_SIZE_ST_08():
    # Deliberately infeasible mission should fail cleanly with failure metadata
    trace = _run_sizing_trace(
        1,
        coaxial=True,
        N_prop=8,
        flight_time=0.40,
        P_payload=400,
        P_avionics=80,
        Lipo_spec_energy=150,
        M_pay=2.5,
    )

    assert not trace["is_valid"], "Extreme sizing case should be flagged as invalid"
    assert np.isnan(trace["final_mtom"]), "Invalid sizing runs should report NaN final MTOM"
    assert trace["failure_iteration"] is not None, "Failure metadata should record the failing iteration"
    assert trace["failure_mtom"] is not None, "Failure metadata should record the failing MTOM"
    assert trace["failure_iteration"] >= 1, "Failure iteration should be positive"
    assert trace["failure_mtom"] > 0, "Failure MTOM should remain physically positive"
    assert trace["mtom_list"], "Infeasible positive-guess runs should retain attempted MTOM history"
    assert trace["residual_list"], "Infeasible positive-guess runs should retain residual history"


    
if __name__ == "__main__":
    # test_SIZE_ST_02(MTOM_guess=7, plot=True)
    # test_SIZE_ST_03(MTOM_guess=7, plot=True)
    test_SIZE_ST_04(MTOM_guess=7, plot=True)
    # plot_SIZE_ST_04_coaxial(MTOM_guess=7)
    # test_SIZE_ST_05(plot=True)
    # raise SystemExit(pytest.main([__file__]))
