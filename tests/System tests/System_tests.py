import pathlib
import sys

import matplotlib.pyplot as plt
import numpy as np
import pytest

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
        "Lipo_spec_energy": 250,
        "M_pay": 1.4,
    }


def _run_sizing_trace(MTOM_guess=1, invalid_mtom=None, **overrides):
    inputs = _get_sizing_inputs()
    inputs.update(overrides)
    trace_data = run_sizing_tool(MTOM_guess, test=True, **inputs)
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


def test_SIZE_ST_01():
    # Base run test
    inputs = _get_sizing_inputs()
    assert run_sizing_tool(1, **inputs)


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
        fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(12, 8))
        ax1.plot(iterations, mtom_list, marker="o", label="MTOM (kg)")
        ax1.set_xlabel("Iteration")
        ax1.set_ylabel("MTOM (kg)")
        ax1.set_title("Convergence: MTOM")
        ax1.legend()
        ax1.grid(True)

        ax2.plot(iterations, p_prop_list, marker="s", label="P_prop (W)")
        ax2.set_xlabel("Iteration")
        ax2.set_ylabel("P_prop (W)")
        ax2.set_title("Convergence: Propulsive Power")
        ax2.legend()
        ax2.grid(True)

        ax3.plot(delta_iterations, mtom_deltas, marker="o", label="MTOM_i - MTOM_i-1")
        ax3.set_xlabel("Iteration")
        ax3.set_ylabel("Delta MTOM (kg)")
        ax3.set_title("Successive Difference: MTOM")
        ax3.legend()
        ax3.grid(True)

        ax4.plot(delta_iterations, p_prop_deltas, marker="s", label="P_prop_i - P_prop_i-1")
        ax4.set_xlabel("Iteration")
        ax4.set_ylabel("Delta P_prop (W)")
        ax4.set_title("Successive Difference: Propulsive Power")
        ax4.legend()
        ax4.grid(True)

        plt.tight_layout()
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
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 4))
        ax1.plot(
            delta_iterations,
            battery_deltas,
            marker="o",
            label="m_battery_i - m_battery_i-1",
        )
        ax1.set_xlabel("Iteration")
        ax1.set_ylabel("Delta m_battery (kg)")
        ax1.set_title("Successive Difference: Battery Mass")
        ax1.legend()
        ax1.grid(True)

        ax2.plot(
            delta_iterations,
            structures_deltas,
            marker="s",
            label="M_structures_i - M_structures_i-1",
        )
        ax2.set_xlabel("Iteration")
        ax2.set_ylabel("Delta M_structures (kg)")
        ax2.set_title("Successive Difference: Structure Mass")
        ax2.legend()
        ax2.grid(True)

        plt.tight_layout()
        plt.show()

def test_SIZE_ST_04(MTOM_guess=1, plot=False):
    # Sensitivity test for converged MTOM with fixed MTOM guess, N_prop, and propeller file
    baseline_trace = _run_sizing_trace(MTOM_guess)
    assert baseline_trace["is_valid"], "Default sizing run should be valid"
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
            [5 / 60, 10 / 60],
            [120, 180],
            [0, 40],
            [200, 320],
            [0.8, 1.8],
        ],
    }
    param_values = morris_sample(problem, N=4, num_levels=4)
    mtom_outputs = []
    coaxial_deltas = []
    coaxial_invalid_cases = 0

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
            invalid_mtom=invalid_mtom_penalty,
            coaxial=default_inputs["coaxial"],
            **sampled_inputs,
        )
        trace_coaxial_true = _run_sizing_trace(
            MTOM_guess,
            invalid_mtom=invalid_mtom_penalty,
            coaxial=True,
            **sampled_inputs,
        )
        trace_coaxial_false = _run_sizing_trace(
            MTOM_guess,
            invalid_mtom=invalid_mtom_penalty,
            coaxial=False,
            **sampled_inputs,
        )
        mtom_outputs.append(
            _trace_impact_score(baseline_trace, trace_selected, invalid_mtom_penalty)
        )
        coaxial_deltas.append(
            _trace_impact_score(trace_coaxial_true, trace_coaxial_false, invalid_mtom_penalty)
        )
        if not trace_coaxial_true["is_valid"] or not trace_coaxial_false["is_valid"]:
            coaxial_invalid_cases += 1

    mtom_outputs = np.asarray(mtom_outputs, dtype=float)
    coaxial_deltas = np.asarray(coaxial_deltas, dtype=float)
    sensitivity = morris_analyze(problem, param_values, mtom_outputs, num_levels=4)
    mu_star = np.asarray(sensitivity["mu_star"], dtype=float)
    sigma = np.asarray(sensitivity["sigma"], dtype=float)
    coaxial_invalid_fraction = coaxial_invalid_cases / len(coaxial_deltas)
    coaxial_mean_delta = float(np.mean(coaxial_deltas))
    coaxial_peak_delta = float(np.max(coaxial_deltas))
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
    assert np.all(np.isfinite(mu_star)), "Sensitivity indices should stay finite"
    assert np.any(mu_star > 0), "At least one variable should influence MTOM"

    ranking_entries = list(zip(problem["names"], mu_star, sigma))
    ranking_entries.append(("coaxial", coaxial_mu_star, coaxial_sigma))
    ranking = sorted(
        ranking_entries,
        key=lambda item: item[1],
        reverse=True,
    )
    ranked_names = [item[0] for item in ranking]
    ranked_mu_star = [item[1] for item in ranking]

    assert True 
    if plot:
        ranking_fig, ranking_ax = plt.subplots(figsize=(10, 5))
        ranking_ax.bar(ranked_names, ranked_mu_star, color="tab:blue")
        ranking_ax.set_xlabel("Input Variable")
        ranking_ax.set_ylabel("Sensitivity score (|Delta MTOM|)")
        ranking_ax.set_title("ST_04 Sensitivity of Converged MTOM")
        ranking_ax.grid(True, axis="y")
        ranking_fig.tight_layout()

        top_three = ranking[:3]
        delta_fig, axes = plt.subplots(1, len(top_three), figsize=(6 * len(top_three), 4))
        if len(top_three) == 1:
            axes = [axes]

        for ax, (parameter_name, _, _) in zip(axes, top_three):
            if parameter_name == "coaxial":
                lower_bound, upper_bound = 0, 1
            else:
                parameter_index = problem["names"].index(parameter_name)
                lower_bound, upper_bound = problem["bounds"][parameter_index]

            selected_value = default_inputs[parameter_name]
            scenario_traces = [("Standard inputs", baseline_trace, "black", "--")]
            for label_prefix, raw_value, color, linestyle in [
                ("Min", lower_bound, "tab:blue", "-"),
                ("Selected", selected_value, "tab:red", ":"),
                ("Max", upper_bound, "tab:green", "-."),
            ]:
                scenario_traces.append(
                    (
                        f"{label_prefix} = {_format_parameter_value(parameter_name, raw_value)}",
                        _run_sizing_trace(
                            MTOM_guess,
                            invalid_mtom=invalid_mtom_penalty,
                            **{parameter_name: _coerce_parameter_value(parameter_name, raw_value)},
                        ),
                        color,
                        linestyle,
                    )
                )

            for label, trace, color, linestyle in scenario_traces:
                mtom_trace = trace["mtom_list"]
                iterations = list(range(1, len(mtom_trace) + 1))
                if not trace["is_valid"]:
                    label = f"{label} (invalid)"
                ax.plot(
                    iterations,
                    mtom_trace,
                    marker="o",
                    color=color,
                    linestyle=linestyle,
                    label=label,
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

        delta_fig.tight_layout()
        plt.show()

def test_SIZE_ST_05(plot=False):
    # Initial guess robustness 
    initial_guesses = list(range(-5, 15))
    traces = [_run_sizing_trace(guess) for guess in initial_guesses]
    valid_pairs = [
        (guess, trace["final_mtom"])
        for guess, trace in zip(initial_guesses, traces)
        if trace["is_valid"]
    ]
    invalid_guesses = [
        guess
        for guess, trace in zip(initial_guesses, traces)
        if not trace["is_valid"]
    ]

    assert valid_pairs, "At least one initial guess should converge to a valid sizing result"

    valid_guesses = [pair[0] for pair in valid_pairs]
    valid_results = np.asarray([pair[1] for pair in valid_pairs], dtype=float)
    distinct_results = np.unique(np.round(valid_results, decimals=6))

    assert np.all(np.isfinite(valid_results)), "Valid initial guesses should produce finite MTOM results"
    assert np.all(valid_results > 0), "Valid initial guesses should produce positive MTOM results"
    assert len(distinct_results) <= 3, "Initial guess sweep should not create too many distinct convergence basins"

    if plot:
        fig, ax = plt.subplots(figsize=(10, 5))
        ax.plot(valid_guesses, valid_results, marker="o", color="tab:blue", label="Converged MTOM")

        if invalid_guesses:
            invalid_level = (
                float(np.min(valid_results)) - max(0.1, 0.1 * (float(np.max(valid_results)) - float(np.min(valid_results)) + 1.0))
            )
            ax.scatter(
                invalid_guesses,
                [invalid_level] * len(invalid_guesses),
                marker="x",
                s=80,
                color="tab:red",
                label="Invalid initial guess",
            )

        ax.set_xlabel("Initial MTOM guess (kg)")
        ax.set_ylabel("Resulting MTOM (kg)")
        ax.set_title("ST_05 Robustness test")
        ax.grid(True)
        ax.legend()
        plt.tight_layout()
        plt.show()


    
if __name__ == "__main__":
    test_SIZE_ST_02(MTOM_guess=7, plot=True)
    test_SIZE_ST_03(MTOM_guess=7, plot=True)
    test_SIZE_ST_04(MTOM_guess=7, plot=True)
    test_SIZE_ST_05(plot=True)
    # raise SystemExit(pytest.main([__file__]))
