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

private:
    std::unordered_map<std::string, std::string> values_;
};

struct ArmResult {
    double area = 0.0;
    double distributed_load = 0.0;
    double inertia = 0.0;
    double max_shear_force = 0.0;
    double z_at_max_shear_force = 0.0;
    double max_bending_moment = 0.0;
    double z_at_max_bending_moment = 0.0;
    double max_bending_stress = 0.0;
    double tip_deflection = 0.0;
    double mass = 0.0;
    bool feasible = false;
};

struct ArmSearchResult {
    bool found = false;
    std::size_t checked_designs = 0;
    double radius = 0.0;
    double thickness = 0.0;
    ArmResult design;
};

class Arm {
public:
    double thickness = 0.002;
    double radius = 0.01;
    double length = 0.25;
    double thrust = 10.0;
    double safety_factor = 1.5;
    double density = 1600.0;
    double youngs_modulus = 140e9;
    double failure_stress = 1.4e9;
    double gravity = 9.81;
    double max_tip_deflection = 0.005e-3;

    ArmResult calculate() const {
        validate();

        ArmResult result;
        const double inner_radius = radius - thickness;
        result.area = PI * (radius * radius - inner_radius * inner_radius);
        result.inertia = (PI / 4.0) *
            (std::pow(radius, 4) - std::pow(inner_radius, 4));
        result.distributed_load = density * gravity * result.area;
        result.mass = result.area * length * density;

        double max_abs_shear = -std::numeric_limits<double>::infinity();
        double max_abs_moment = -std::numeric_limits<double>::infinity();

        const auto consider_shear = [&](double remaining_length) {
            const double shear =
                -thrust + result.distributed_load * remaining_length;
            if (std::abs(shear) > max_abs_shear) {
                max_abs_shear = std::abs(shear);
                result.max_shear_force = shear;
                result.z_at_max_shear_force = length - remaining_length;
            }
        };

        const auto consider_moment = [&](double remaining_length) {
            const double moment =
                thrust * remaining_length -
                0.5 * result.distributed_load *
                    remaining_length * remaining_length;
            if (std::abs(moment) > max_abs_moment) {
                max_abs_moment = std::abs(moment);
                result.max_bending_moment = moment;
                result.z_at_max_bending_moment =
                    length - remaining_length;
            }
        };

        consider_shear(0.0);
        consider_shear(length);
        consider_moment(0.0);
        consider_moment(length);

        if (result.distributed_load > 0.0) {
            const double stationary_point =
                thrust / result.distributed_load;
            if (stationary_point > 0.0 && stationary_point < length) {
                consider_moment(stationary_point);
            }
        }

        result.max_bending_stress =
            std::abs(result.max_bending_moment) * radius / result.inertia;
        result.tip_deflection =
            result.distributed_load * std::pow(length, 4) /
                (8.0 * youngs_modulus * result.inertia) +
            thrust * std::pow(length, 3) /
                (3.0 * youngs_modulus * result.inertia);
        result.feasible =
            safety_factor * result.max_bending_stress <= failure_stress &&
            result.tip_deflection <= max_tip_deflection;
        return result;
    }

    ArmSearchResult minimise_mass(
        double radius_min,
        double radius_max,
        double radius_step,
        double thickness_min,
        double thickness_step
    ) const {
        if (radius_min <= 0.0 || radius_max < radius_min ||
            radius_step <= 0.0 || thickness_min <= 0.0 ||
            thickness_step <= 0.0) {
            throw std::invalid_argument("Invalid arm search range.");
        }

        ArmSearchResult best;
        double best_mass = std::numeric_limits<double>::infinity();

        for (
            double candidate_radius = radius_min;
            candidate_radius <= radius_max + 0.5 * radius_step;
            candidate_radius += radius_step
        ) {
            for (
                double candidate_thickness = thickness_min;
                candidate_thickness < candidate_radius;
                candidate_thickness += thickness_step
            ) {
                Arm candidate = *this;
                candidate.radius = candidate_radius;
                candidate.thickness = candidate_thickness;
                const ArmResult result = candidate.calculate();
                ++best.checked_designs;

                if (result.feasible && result.mass < best_mass) {
                    best.found = true;
                    best.radius = candidate_radius;
                    best.thickness = candidate_thickness;
                    best.design = result;
                    best_mass = result.mass;
                }
            }
        }
        return best;
    }

private:
    void validate() const {
        if (radius <= 0.0 || thickness <= 0.0 || thickness >= radius ||
            length <= 0.0 || safety_factor <= 0.0 || density <= 0.0 ||
            youngs_modulus <= 0.0 || failure_stress <= 0.0 ||
            gravity <= 0.0 || max_tip_deflection < 0.0) {
            throw std::invalid_argument("Invalid arm configuration.");
        }
    }
};

Arm arm_from_arguments(const Arguments& arguments) {
    Arm arm;
    arm.radius = arguments.number("radius", arm.radius);
    arm.thickness = arguments.number("thickness", arm.thickness);
    arm.length = arguments.number("length", arm.length);
    arm.thrust = arguments.number("thrust", arm.thrust);
    arm.safety_factor =
        arguments.number("safety-factor", arm.safety_factor);
    arm.density = arguments.number("density", arm.density);
    arm.youngs_modulus =
        arguments.number("youngs-modulus", arm.youngs_modulus);
    arm.failure_stress =
        arguments.number("failure-stress", arm.failure_stress);
    arm.gravity = arguments.number("gravity", arm.gravity);
    arm.max_tip_deflection =
        arguments.number("max-tip-deflection", arm.max_tip_deflection);
    return arm;
}

void print_result(const ArmResult& result) {
    std::cout << std::setprecision(17)
              << "{"
              << "\"area\":" << result.area << ","
              << "\"distributed_load\":" << result.distributed_load << ","
              << "\"inertia\":" << result.inertia << ","
              << "\"max_shear_force\":" << result.max_shear_force << ","
              << "\"z_at_max_shear_force\":"
              << result.z_at_max_shear_force << ","
              << "\"max_bending_moment\":"
              << result.max_bending_moment << ","
              << "\"z_at_max_bending_moment\":"
              << result.z_at_max_bending_moment << ","
              << "\"max_bending_stress\":"
              << result.max_bending_stress << ","
              << "\"tip_deflection\":" << result.tip_deflection << ","
              << "\"mass\":" << result.mass << ","
              << "\"feasible\":"
              << (result.feasible ? "true" : "false")
              << "}";
}

void print_search_result(const ArmSearchResult& result) {
    std::cout << std::setprecision(17)
              << "{"
              << "\"found\":" << (result.found ? "true" : "false") << ","
              << "\"checked_designs\":" << result.checked_designs;
    if (result.found) {
        std::cout << ",\"radius\":" << result.radius
                  << ",\"thickness\":" << result.thickness
                  << ",\"design\":";
        print_result(result.design);
    }
    std::cout << "}";
}

}  // namespace

int main(int argc, char* argv[]) {
    try {
        const std::string command = argc > 1 ? argv[1] : "calculate";
        const Arguments arguments(argc, argv, argc > 1 ? 2 : 1);
        const Arm arm = arm_from_arguments(arguments);

        if (command == "calculate") {
            print_result(arm.calculate());
        } else if (command == "optimize") {
            print_search_result(arm.minimise_mass(
                arguments.number("radius-min", 0.002),
                arguments.number("radius-max", 0.05),
                arguments.number("radius-step", 0.0001),
                arguments.number("thickness-min", 0.0005),
                arguments.number("thickness-step", 0.0001)
            ));
        } else {
            throw std::invalid_argument(
                "Expected command 'calculate' or 'optimize'."
            );
        }
        std::cout << '\n';
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "arm: " << error.what() << '\n';
        return 2;
    }
}
