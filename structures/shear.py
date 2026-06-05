import numpy as np


def shear_arm(V: int, t: float, tau_allow: float = 50000, SF = 1.5) -> float:
    """
    Required outer radius Ro for a thick hollow circular rod under transverse shear.

    Inputs:
        V         : transverse shear force [N]
        t         : wall thickness [m]
        tau_allow : allowable max shear stress [Pa]

    Returns:
        Ro : required outer radius [m]

    Uses:
        Ri = Ro - t
        tau_max = 4V(3Ro^2 - 3Ro t + t^2)
                  / [3π(Ro^4 - (Ro - t)^4)]
    """
    if V <= 0:
        raise ValueError("V must be positive.")
    if t <= 0:
        raise ValueError("t must be positive.")
    if tau_allow <= 0:
        raise ValueError("tau_allow must be positive.")

    # Minimum possible Ro is t, which gives Ri = 0, i.e. solid circle.
    tau_at_Ro_equals_t = 4 * V / (3 * np.pi * t**2)

    # If even the solid case is below allowable stress, Ro = t is enough.
    if tau_allow >= tau_at_Ro_equals_t:
        return t

    def tau_max(Ro: float) -> float:
        Ri = Ro - t
        return (
            4 * V * (Ro**2 + Ro * Ri + Ri**2)
            / (3 * np.pi * (Ro**4 - Ri**4))
        )

    def f(Ro: float) -> float:
        return tau_max(Ro) - tau_allow

    lo = t
    hi = 2 * t

    # Expand upper bound until stress is below allowable.
    while f(hi) > 0:
        hi *= 2

    # Bisection solve.
    for _ in range(200):
        mid = 0.5 * (lo + hi)

        if f(mid) > 0:
            lo = mid
        else:
            hi = mid

    return 0.5 * (lo + hi)

if __name__ == "__main__":
    # Example usage:
    V = 10  # N
    t = 0.001  # m
    tau_allow = 50e6  # Pa (50 MPa)
    required_Ro = shear_arm(V, t, tau_allow)
    print(f"Required outer radius Ro: {required_Ro:.4f} m")
