from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from scipy.integrate import solve_ivp


# Vehicle parameters
LF = 1.5                 # Center of gravity to front axle [m]
LR = 1.5                 # Center of gravity to rear axle [m]
L = LF + LR             # Wheelbase [m]
V = 5.0                 # Rear axle speed [m/s]

# Simulation settings
DURATION = 20.0          # [s]
DT = 0.01               # Output sampling interval [s]
TIME = np.linspace(0.0, DURATION, int(round(DURATION / DT)) + 1)

# Save figures beside this Python file.
OUTPUT_DIR = Path(__file__).resolve().parent / "figures"
OUTPUT_DIR.mkdir(exist_ok=True)


def pepy_model(t, state, delta):
    """Return [X_dot, Y_dot, psi_dot] for the Pepy model."""
    x, y, psi = state

    return [
        V * np.cos(psi),
        V * np.sin(psi),
        V / L * np.tan(delta),
    ]


def main():
    steering_angles = [0.1, 0.2, 0.3]
    initial_state = [0.0, 0.0, 0.0]

    fig, ax = plt.subplots(figsize=(8, 6))

    for delta in steering_angles:
        result = solve_ivp(
            fun=pepy_model,
            t_span=(0.0, DURATION),
            y0=initial_state,
            args=(delta,),
            t_eval=TIME,
            max_step=DT,
            rtol=1e-8,
            atol=1e-10,
        )

        if not result.success:
            raise RuntimeError(result.message)

        x, y, psi = result.y
        radius = L / np.tan(delta)

        ax.plot(
            x,
            y,
            label=f"delta={delta:.1f} rad, R={radius:.2f} m",
        )

        print(f"delta={delta:.1f} rad -> radius={radius:.2f} m")

    ax.scatter([0.0], [0.0], color="black", label="Start", zorder=3)
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_title("Pepy model: constant steering")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True)
    ax.legend()

    fig.tight_layout()

    output_file = OUTPUT_DIR / "1_4_A_constant.png"
    fig.savefig(output_file, dpi=200)
    plt.close(fig)

    print(f"Saved figure: {output_file}")


if __name__ == "__main__":
    main()