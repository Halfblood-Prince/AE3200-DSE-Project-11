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


def _assert_converged_trace(trace, tolerance=0.001):
    assert trace["is_valid"], "Sizing trace should converge for this scenario"
    assert trace["residual_list"], "Converged traces should include a residual history"
    assert trace["residual_list"][-1] <= tolerance, (
        f"Final residual should be <= {tolerance}"
    )


def _assert_nominal_trace_bounds(trace, mtom_bounds, max_iterations):
    _assert_converged_trace(trace)
    assert mtom_bounds[0] <= trace["final_mtom"] <= mtom_bounds[1], (
        f"Final MTOM should stay within {mtom_bounds}"
    )
    assert len(trace["mtom_list"]) <= max_iterations, (
        f"Sizing should converge within {max_iterations} iterations"
    )


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
        ax1.set_xticks(iterations)
        ax1.set_xlabel("Iteration")
        ax1.set_ylabel("MTOM (kg)")
        ax1.set_title("Convergence: MTOM")
        ax1.legend()
        ax1.grid(True)

        ax2.plot(iterations, p_prop_list, marker="s", label="P_prop (W)")
        ax2.set_xticks(iterations)
        ax2.set_xlabel("Iteration")
        ax2.set_ylabel("P_prop (W)")
        ax2.set_title("Convergence: Propulsive Power")
        ax2.legend()
        ax2.grid(True)

        ax3.plot(delta_iterations, mtom_deltas, marker="o", label="MTOM_i - MTOM_i-1", color="orange")
        ax3.set_xticks(delta_iterations)
        
        ax3.set_xlabel("Iteration")
        ax3.set_ylabel("Delta MTOM (kg)")
        ax3.set_title("Successive Difference: MTOM")
        ax3.legend()
        ax3.grid(True)

        ax4.plot(delta_iterations, p_prop_deltas, marker="s", label="P_prop_i - P_prop_i-1", color="orange")
        ax4.set_xticks(delta_iterations)
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
            color="orange",
        )
        ax1.set_xlabel("Iteration")
        ax1.set_ylabel("Delta m_battery (kg)")
        ax1.set_title("Successive Difference: Battery Mass")
        ax1.set_xticks(delta_iterations)
        ax1.legend()
        ax1.grid(True)

        ax2.plot(
            delta_iterations,
            structures_deltas,
            marker="s",
            label="M_structures_i - M_structures_i-1",
            color="orange",
        )
        ax2.set_xlabel("Iteration")
        ax2.set_ylabel("Delta M_structures (kg)")
        ax2.set_title("Successive Difference: Structure Mass")
        ax2.set_xticks(delta_iterations)
        ax2.legend()
        ax2.grid(True)

        plt.tight_layout()
        plt.show()

def test_SIZE_ST_04(MTOM_guess=1, plot=False):
    # Sensitivity test for converged MTOM with fixed MTOM guess, N_prop, and propeller file
    baseline_trace = _run_sizing_trace(MTOM_guess)
    _assert_converged_trace(baseline_trace)
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
    param_values = morris_sample(problem, N=4, num_levels=4, seed=0)
    mtom_outputs = []
    coaxial_deltas = []
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
    assert mu_star.shape == (problem["num_vars"],), "Each sampled variable should get one mu_star value"
    assert sigma.shape == (problem["num_vars"],), "Each sampled variable should get one sigma value"
    assert np.all(np.isfinite(mu_star)), "Sensitivity indices should stay finite"
    assert np.all(np.isfinite(sigma)), "Sensitivity spread indices should stay finite"
    assert np.all(mu_star >= 0), "Morris mu_star values should be non-negative"
    assert np.all(sigma >= 0), "Morris sigma values should be non-negative"
    assert np.any(mu_star > 0), "At least one variable should influence MTOM"

    for parameter_name, traces in endpoint_traces.items():
        _assert_converged_trace(
            traces["low"],
            tolerance=0.001,
        )
        _assert_converged_trace(
            traces["high"],
            tolerance=0.001,
        )

    endpoint_deltas = {
        parameter_name: (
            traces["high"]["final_mtom"] - traces["low"]["final_mtom"]
        )
        for parameter_name, traces in endpoint_traces.items()
    }
    assert endpoint_deltas["flight_time"] > 0, "Longer flight time should increase MTOM"
    assert endpoint_deltas["P_payload"] > 0, "Higher payload power should increase MTOM"
    assert endpoint_deltas["P_avionics"] > 0, "Higher avionics power should increase MTOM"
    assert endpoint_deltas["Lipo_spec_energy"] < 0, (
        "Higher battery specific energy should reduce MTOM"
    )
    assert endpoint_deltas["M_pay"] > 0, "Higher payload mass should increase MTOM"

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
        ranking_fig, ranking_ax = plt.subplots(figsize=(10, 5))
        ranking_ax.bar(ranked_names, ranked_mu_star, color="tab:blue")
        ranking_ax.set_xlabel("Input Variable")
        ranking_ax.set_ylabel("Sensitivity score (|Delta MTOM|)")
        # ranking_ax.set_title("ST_04 Sensitivity of Converged MTOM")
        ranking_ax.grid(True, axis="y")
        ranking_fig.tight_layout()

        top_three = ranking[:3]
        delta_fig, axes = plt.subplots(1, len(top_three), figsize=(6 * len(top_three), 4))
        if len(top_three) == 1:
            axes = [axes]

        for ax, (parameter_name, _, _) in zip(axes, top_three):
            if parameter_name == "coaxial":
                sample_count = len(coaxial_plot_records)
                invalid_true = sum(
                    not record["coaxial_true"]["is_valid"] for record in coaxial_plot_records
                )
                invalid_false = sum(
                    not record["coaxial_false"]["is_valid"] for record in coaxial_plot_records
                )
                max_iterations = 1
                coaxial_styles = [
                    ("Coaxial on", "tab:blue", "coaxial_true"),
                    ("Coaxial off", "tab:orange", "coaxial_false"),
                ]

                for line_label, color, trace_key in coaxial_styles:
                    invalid_label = f"{line_label} invalid"
                    current_line_label = line_label
                    for record in coaxial_plot_records:
                        trace = record[trace_key]
                        mtom_trace = trace["mtom_list"]
                        iterations = list(range(1, len(mtom_trace) + 1))
                        if mtom_trace:
                            max_iterations = max(max_iterations, len(mtom_trace))
                            ax.plot(
                                iterations,
                                mtom_trace,
                                color=color,
                                linewidth=1.0,
                                alpha=0.25,
                                label=current_line_label,
                            )
                            current_line_label = None

                        if not trace["is_valid"]:
                            failure_iteration, failure_mtom = _failure_plot_coordinates(
                                trace,
                                MTOM_guess,
                            )
                            max_iterations = max(max_iterations, failure_iteration)
                            ax.scatter(
                                [failure_iteration],
                                [failure_mtom],
                                marker="x",
                                s=35,
                                linewidths=1.2,
                                color=color,
                                alpha=0.9,
                                label=invalid_label,
                            )
                            invalid_label = None

                ax.set_xlabel("Iteration")
                ax.set_ylabel("MTOM (kg)")
                ax.set_xticks(range(1, max_iterations + 1))
                ax.set_title(
                    "coaxial\n"
                    f"invalid: on={invalid_true}/{sample_count}, off={invalid_false}/{sample_count}"
                )
                ax.grid(True)
                ax.legend()
                continue

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

    assert valid_pairs, "At least one initial guess should converge to a valid sizing result"

    non_positive_invalid_guesses = [
        guess for guess, _trace in invalid_pairs if guess <= 0
    ]
    positive_valid_guesses = [pair[0] for pair in valid_pairs if pair[0] > 0]
    positive_invalid_guesses = [
        guess for guess, _trace in invalid_pairs if guess > 0
    ]
    valid_results = np.asarray([pair[1] for pair in valid_pairs], dtype=float)
    positive_valid_results = np.asarray(
        [pair[1] for pair in valid_pairs if pair[0] > 0],
        dtype=float,
    )

    assert non_positive_invalid_guesses == [guess for guess in initial_guesses if guess <= 0], (
        "Non-positive guesses should be rejected explicitly"
    )
    assert positive_valid_guesses, "At least one positive initial guess should converge"
    assert np.all(np.isfinite(valid_results)), "Valid initial guesses should produce finite MTOM results"
    assert np.all(valid_results > 0), "Valid initial guesses should produce positive MTOM results"
    assert np.allclose(
        positive_valid_results,
        positive_valid_results[0],
        atol=0.001,
        rtol=0.0,
    ), "All feasible positive guesses should converge to the same MTOM fixed point"

    for guess, trace in zip(initial_guesses, traces):
        if trace["is_valid"]:
            _assert_converged_trace(trace, tolerance=0.001)

    if positive_invalid_guesses:
        assert min(positive_invalid_guesses) > max(positive_valid_guesses), (
            "Positive initial-guess failures should only appear beyond the feasible MTOM range"
        )

    if plot:
        fig, ax = plt.subplots(figsize=(10, 5))
        ax.plot(
            positive_valid_guesses,
            positive_valid_results,
            marker="o",
            color="tab:blue",
            label="Converged MTOM",
        )

        if invalid_pairs:
            invalid_guesses = [guess for guess, _trace in invalid_pairs]
            ax.scatter(
                invalid_guesses,
                [0.0] * len(invalid_guesses),
                marker="x",
                s=80,
                color="tab:red",
                label="Invalid initial guess",
            )

        ax.set_xlabel("Initial MTOM guess (kg)")
        ax.set_ylabel("Resulting MTOM (kg)")
        # ax.set_title("ST_05 Robustness test")
        ax.grid(True)
        ax.legend()
        plt.tight_layout()
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
        mtom_bounds=(3.6, 4.1),
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
    # test_SIZE_ST_05(plot=True)
    # raise SystemExit(pytest.main([__file__]))
