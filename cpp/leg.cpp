#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <unordered_map>

namespace {

constexpr double PI = 3.141592653589793238462643383279502884;
constexpr double GRAVITY = 9.81;

class Arguments {
public:
    Arguments(int argc, char* argv[], int start_index) {
        for (int index = start_index; index < argc; index += 2) {
            const std::string key = argv[index];
            if (key.rfind("--", 0) != 0 || index + 1 >= argc) {
                throw std::invalid_argument("Options must use --name value pairs.");
            }
            values_[key.substr(2)] = argv[index + 1];
        }
    }

    double number(const std::string& key, double fallback) const {
        const auto value = values_.find(key);
        return value == values_.end() ? fallback : std::stod(value->second);
    }

    int integer(const std::string& key, int fallback) const {
        const auto value = values_.find(key);
        return value == values_.end() ? fallback : std::stoi(value->second);
    }

private:
    std::unordered_map<std::string, std::string> values_;
};

struct LegResult {
    double angle_deg = 0.0;
    double radius = 0.0;
    double area = 0.0;
    double inertia = 0.0;
    double bending_force = 0.0;
    double bending_moment = 0.0;
    double axial_force = 0.0;
    double tip_deflection = 0.0;
    double bending_stress = 0.0;
    double compressive_stress = 0.0;
    double longitudinal_deflection = 0.0;
    double governing_stress = 0.0;
    double euler_buckling_load = 0.0;
    double buckling_margin = 0.0;
    double mass = 0.0;
    bool buckling_safe = false;
    bool feasible = false;
};

struct LegSearchResult {
    bool found = false;
    std::size_t checked_designs = 0;
    LegResult design;
};

class Leg {
public:
    double vehicle_mass = 4.5;
    double density = 2700.0;
    double youngs_modulus = 69e9;
    double yield_strength = 276e6;
    double angle_deg = 30.0;
    double radius = 0.005;
    double safety_factor = 1.5;
    double length = 0.1;
    double effective_length_factor = 1.0;
    double max_tip_deflection = 0.005e-3;
    double max_compressive_deformation = 0.01e-3;
    int number_of_legs = 2;

    LegResult calculate() const {
        validate();

        LegResult result;
        const double angle = angle_deg * PI / 180.0;
        result.angle_deg = angle_deg;
        result.radius = radius;
        result.area = PI * radius * radius;
        result.inertia = PI * std::pow(radius, 4) / 4.0;
        result.bending_force =
            vehicle_mass * GRAVITY * std::sin(angle) *
            safety_factor / static_cast<double>(number_of_legs);
        result.axial_force =
            vehicle_mass * GRAVITY * std::cos(angle) *
            safety_factor / static_cast<double>(number_of_legs);
        result.bending_moment = result.bending_force * length;
        result.tip_deflection =
            result.bending_force * std::pow(length, 3) /
            (3.0 * youngs_modulus * result.inertia);
        result.bending_stress =
            result.bending_moment * radius / result.inertia;
        result.compressive_stress =
            std::abs(result.axial_force) / result.area;
        result.longitudinal_deflection =
            std::abs(result.axial_force) * length /
            (youngs_modulus * result.area);
        result.governing_stress = std::max(
            std::abs(result.bending_stress),
            result.compressive_stress
        );

        const double effective_length =
            effective_length_factor * length;
        result.euler_buckling_load =
            PI * PI * youngs_modulus * result.inertia /
            (effective_length * effective_length);
        result.buckling_margin =
            result.axial_force == 0.0
                ? std::numeric_limits<double>::infinity()
                : result.euler_buckling_load /
                    std::abs(result.axial_force);
        result.buckling_safe =
            result.euler_buckling_load >= std::abs(result.axial_force);
        result.mass =
            result.area * length * density * safety_factor;
        result.feasible =
            result.tip_deflection <= max_tip_deflection &&
            result.longitudinal_deflection <=
                max_compressive_deformation &&
            result.governing_stress <= yield_strength &&
            result.buckling_safe;
        return result;
    }

    LegSearchResult minimise_mass(
        double angle_min,
        double angle_max,
        double angle_step,
        double radius_min,
        double radius_max,
        double radius_step
    ) const {
        if (angle_max < angle_min || angle_step <= 0.0 ||
            radius_min <= 0.0 || radius_max < radius_min ||
            radius_step <= 0.0) {
            throw std::invalid_argument("Invalid leg search range.");
        }

        LegSearchResult best;
        double best_mass = std::numeric_limits<double>::infinity();

        for (
            double candidate_angle = angle_min;
            candidate_angle <= angle_max + 0.5 * angle_step;
            candidate_angle += angle_step
        ) {
            for (
                double candidate_radius = radius_min;
                candidate_radius <= radius_max + 0.5 * radius_step;
                candidate_radius += radius_step
            ) {
                Leg candidate = *this;
                candidate.angle_deg = candidate_angle;
                candidate.radius = candidate_radius;
                const LegResult result = candidate.calculate();
                ++best.checked_designs;

                if (result.feasible && result.mass < best_mass) {
                    best.found = true;
                    best.design = result;
                    best_mass = result.mass;
                }
            }
        }
        return best;
    }

private:
    void validate() const {
        if (vehicle_mass < 0.0 || density <= 0.0 ||
            youngs_modulus <= 0.0 || yield_strength <= 0.0 ||
            radius <= 0.0 || safety_factor <= 0.0 || length <= 0.0 ||
            effective_length_factor <= 0.0 ||
            max_tip_deflection < 0.0 ||
            max_compressive_deformation < 0.0 ||
            number_of_legs <= 0) {
            throw std::invalid_argument("Invalid leg configuration.");
        }
    }
};

Leg leg_from_arguments(const Arguments& arguments) {
    Leg leg;
    leg.vehicle_mass = arguments.number("mass", leg.vehicle_mass);
    leg.radius = arguments.number("radius", leg.radius);
    leg.angle_deg = arguments.number("angle-deg", leg.angle_deg);
    leg.length = arguments.number("length", leg.length);
    leg.safety_factor =
        arguments.number("safety-factor", leg.safety_factor);
    leg.number_of_legs =
        arguments.integer("number-of-legs", leg.number_of_legs);
    leg.density = arguments.number("density", leg.density);
    leg.youngs_modulus =
        arguments.number("youngs-modulus", leg.youngs_modulus);
    leg.yield_strength =
        arguments.number("yield-strength", leg.yield_strength);
    leg.effective_length_factor = arguments.number(
        "effective-length-factor",
        leg.effective_length_factor
    );
    leg.max_tip_deflection = arguments.number(
        "max-tip-deflection",
        leg.max_tip_deflection
    );
    leg.max_compressive_deformation = arguments.number(
        "max-compressive-deformation",
        leg.max_compressive_deformation
    );
    return leg;
}

void print_number(double value) {
    if (std::isfinite(value)) {
        std::cout << value;
    } else {
        std::cout << "null";
    }
}

void print_result(const LegResult& result) {
    std::cout << std::setprecision(17)
              << "{"
              << "\"angle_deg\":" << result.angle_deg << ","
              << "\"radius\":" << result.radius << ","
              << "\"area\":" << result.area << ","
              << "\"inertia\":" << result.inertia << ","
              << "\"bending_force\":" << result.bending_force << ","
              << "\"bending_moment\":" << result.bending_moment << ","
              << "\"axial_force\":" << result.axial_force << ","
              << "\"tip_deflection\":" << result.tip_deflection << ","
              << "\"bending_stress\":" << result.bending_stress << ","
              << "\"compressive_stress\":"
              << result.compressive_stress << ","
              << "\"longitudinal_deflection\":"
              << result.longitudinal_deflection << ","
              << "\"governing_stress\":" << result.governing_stress << ","
              << "\"euler_buckling_load\":"
              << result.euler_buckling_load << ","
              << "\"buckling_margin\":";
    print_number(result.buckling_margin);
    std::cout << ",\"mass\":" << result.mass
              << ",\"buckling_safe\":"
              << (result.buckling_safe ? "true" : "false")
              << ",\"feasible\":"
              << (result.feasible ? "true" : "false")
              << "}";
}

void print_search_result(const LegSearchResult& result) {
    std::cout << std::setprecision(17)
              << "{"
              << "\"found\":" << (result.found ? "true" : "false") << ","
              << "\"checked_designs\":" << result.checked_designs;
    if (result.found) {
        std::cout << ",\"design\":";
        print_result(result.design);
    }
    std::cout << "}";
}

}  // namespace

int main(int argc, char* argv[]) {
    try {
        const std::string command = argc > 1 ? argv[1] : "calculate";
        const Arguments arguments(argc, argv, argc > 1 ? 2 : 1);
        const Leg leg = leg_from_arguments(arguments);

        if (command == "calculate") {
            print_result(leg.calculate());
        } else if (command == "optimize") {
            print_search_result(leg.minimise_mass(
                arguments.number("angle-min", 5.0),
                arguments.number("angle-max", 60.0),
                arguments.number("angle-step", 1.0),
                arguments.number("radius-min", 0.001),
                arguments.number("radius-max", 0.05),
                arguments.number("radius-step", 0.0001)
            ));
        } else {
            throw std::invalid_argument(
                "Expected command 'calculate' or 'optimize'."
            );
        }
        std::cout << '\n';
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "leg: " << error.what() << '\n';
        return 2;
    }
}
