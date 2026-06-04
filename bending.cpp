#include <cmath>
#include <cstddef>
#include <iomanip>
#include <iostream>
#include <limits>

#ifdef _WIN32
#include <windows.h>
#endif

namespace {

constexpr double PI = 3.141592653589793238462643383279502884;
constexpr double GRAVITY = 9.81;

struct Material {
    double density = 1600.0;          // kg/m^3
    double youngs_modulus = 140e9;    // Pa
    double failure_strain = 0.01;

    double failure_stress() const {
        return failure_strain * youngs_modulus;
    }
};

struct BendingConfig {
    double t = 0.002;                 // wall thickness [m]
    double R = 0.01;                  // outer radius [m]
    double L = 0.25;                  // beam/arm length [m]
    double T = 0.1;                   // propeller thrust [N]
    double safety_factor = 1.5;
    Material material;
};

struct BendingResult {
    double area = 0.0;
    double distributed_load = 0.0;
    double inertia = 0.0;
    double max_bending_moment = 0.0;
    double z_at_max_bending_moment = 0.0;
    double max_bending_stress = 0.0;
    double mass = 0.0;
};

struct SearchConfig {
    double r_min = 0.002;
    double r_max = 10.0;
    double r_step = 0.001;
    double t_min = 0.0005;
    double t_step = 0.005;
};

struct SearchResult {
    bool found = false;
    double R = 0.0;
    double t = 0.0;
    BendingResult bending;
    std::size_t checked_designs = 0;
};

double cross_section_area(double R, double t) {
    const double inner_radius = R - t;
    return PI * (R * R - inner_radius * inner_radius);
}

double area_moment_of_inertia(double R, double t) {
    const double inner_radius = R - t;
    const double outer_radius_squared = R * R;
    const double inner_radius_squared = inner_radius * inner_radius;
    return (PI / 4.0) *
           (outer_radius_squared * outer_radius_squared -
            inner_radius_squared * inner_radius_squared);
}

BendingResult calculate_bending(const BendingConfig& config) {
    BendingResult result;

    result.area = cross_section_area(config.R, config.t);
    result.distributed_load = config.material.density * GRAVITY * result.area;
    result.inertia = area_moment_of_inertia(config.R, config.t);
    result.mass = result.area * config.L * config.material.density;

    double max_abs_moment = -std::numeric_limits<double>::infinity();

    const auto consider_moment = [&](double remaining_length) {
        const double moment =
            config.T * remaining_length -
            0.5 * result.distributed_load * remaining_length * remaining_length;

        const double abs_moment = std::abs(moment);
        if (abs_moment > max_abs_moment) {
            max_abs_moment = abs_moment;
            result.max_bending_moment = moment;
            result.z_at_max_bending_moment = config.L - remaining_length;
        }
    };

    consider_moment(0.0);
    consider_moment(config.L);

    if (result.distributed_load > 0.0) {
        const double stationary_point = config.T / result.distributed_load;
        if (stationary_point > 0.0 && stationary_point < config.L) {
            consider_moment(stationary_point);
        }
    }

    result.max_bending_stress =
        std::abs(result.max_bending_moment) * config.R / result.inertia;

    return result;
}

SearchResult minimise_mass(const BendingConfig& base_config, const SearchConfig& search) {
    SearchResult best;
    double best_mass = std::numeric_limits<double>::infinity();
    const double allowable_stress =
        base_config.material.failure_stress() / base_config.safety_factor;

    for (std::size_t r_index = 0;; ++r_index) {
        const double R = search.r_min + static_cast<double>(r_index) * search.r_step;
        if (R >= search.r_max) {
            break;
        }

        for (std::size_t t_index = 0;; ++t_index) {
            const double t = search.t_min + static_cast<double>(t_index) * search.t_step;
            if (t >= R) {
                break;
            }

            BendingConfig config = base_config;
            config.R = R;
            config.t = t;

            const BendingResult bending = calculate_bending(config);
            ++best.checked_designs;

            if (bending.max_bending_stress <= allowable_stress && bending.mass < best_mass) {
                best.found = true;
                best.R = R;
                best.t = t;
                best.bending = bending;
                best_mass = bending.mass;
            }
        }
    }

    return best;
}

void print_bending_result(const BendingConfig& config, const BendingResult& result) {
    std::cout << std::setprecision(10);
    std::cout << "Cross-sectional area: " << result.area << " m^2\n";
    std::cout << "Distributed load w: " << result.distributed_load << " N/m\n";
    std::cout << "Second moment of area I: " << result.inertia << " m^4\n";
    std::cout << "Maximum bending moment: " << result.max_bending_moment
              << " Nm at z = " << result.z_at_max_bending_moment << " m\n";
    std::cout << "Maximum bending stress: " << result.max_bending_stress << " Pa\n";
    std::cout << "Maximum bending stress: " << result.max_bending_stress / 1e6 << " MPa\n";
    std::cout << "Design bending stress with SF: "
              << config.safety_factor * result.max_bending_stress / 1e6 << " MPa\n";
    std::cout << "Beam mass: " << result.mass << " kg\n";
}

void print_search_result(const BendingConfig& config, const SearchResult& result) {
    std::cout << "\nMass minimisation checked " << result.checked_designs << " designs.\n";

    if (!result.found) {
        std::cout << "No feasible design found for the configured search range.\n";
        return;
    }

    std::cout << "Optimal outer radius R: " << result.R << " m\n";
    std::cout << "Optimal wall thickness t: " << result.t << " m\n";
    std::cout << "Optimal beam mass: " << result.bending.mass << " kg\n";
    std::cout << "Safety-factor bending stress: "
              << config.safety_factor * result.bending.max_bending_stress / 1e6 << " MPa\n";
    std::cout << "Failure stress: " << config.material.failure_stress() / 1e6 << " MPa\n";
}

void pause_if_launched_directly() {
#ifdef _WIN32
    DWORD process_list[2];
    const DWORD process_count = GetConsoleProcessList(process_list, 2);

    if (process_count <= 1) {
        std::cout << "\nPress Enter to close...";
        std::cin.get();
    }
#endif
}

} // namespace

int main() {
    const BendingConfig config;
    const BendingResult baseline = calculate_bending(config);

    print_bending_result(config, baseline);

    const SearchResult optimum = minimise_mass(config, SearchConfig{});
    print_search_result(config, optimum);

    pause_if_launched_directly();

    return optimum.found ? 0 : 1;
}
