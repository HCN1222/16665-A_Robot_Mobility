"""Core 1.4: RK4 simulations, figures, and analytic/refinement checks."""
from pathlib import Path
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from bicycle import pepy

ROOT = Path(__file__).resolve().parent
FIG, DATA = ROOT/'figures', ROOT/'data'
FIG.mkdir(exist_ok=True)
DATA.mkdir(exist_ok=True)


def rk4(speed, steering, stop, step=.01):
    times = np.r_[np.arange(0, stop, step), stop]
    states = np.zeros((len(times), 3))
    for k, (t0, t1) in enumerate(zip(times[:-1], times[1:])):
        h, z = t1-t0, states[k]
        k1 = pepy(z, speed, steering(t0))
        k2 = pepy(z+h*k1/2, speed, steering(t0+h/2))
        k3 = pepy(z+h*k2/2, speed, steering(t0+h/2))
        k4 = pepy(z+h*k3, speed, steering(np.nextafter(t1, t0)))
        states[k+1] = z+h*(k1+2*k2+2*k3+k4)/6
    return times, states


def figure_save(fig, filename):
    fig.tight_layout()
    fig.savefig(FIG/filename, dpi=220, bbox_inches='tight')
    plt.close(fig)


def export(name, t, z):
    np.savetxt(DATA/name, np.column_stack([t, z]), delimiter=',',
               header='time_s,X_m,Y_m,yaw_rad', comments='')


def constant():
    fig, ax = plt.subplots(figsize=(6.5, 5))
    checks = {}
    for angle in [.10, .25, .45]:
        t, z = rk4(5., lambda time: angle, 40.)
        radius = 3/np.tan(angle)
        yaw = 5*t/radius
        truth = np.column_stack([radius*np.sin(yaw),
                                 radius*(1-np.cos(yaw)), yaw])
        error = float(np.max(abs(z-truth)))
        assert error < 1e-7
        checks[str(angle)] = dict(radius_m=float(radius),
                                 revolution_s=float(2*np.pi*radius/5),
                                 max_state_difference=error)
        ax.plot(z[:, 0], z[:, 1], label=f'{angle:.2f} rad; R={radius:.2f} m')
        export(f'constant_{angle}.csv', t, z)
    ax.set(xlabel='X [m]', ylabel='Y [m]', title='Constant steering: 5 m/s, 40 s')
    ax.set_aspect('equal', adjustable='box'); ax.legend(); ax.grid(alpha=.3)
    figure_save(fig, 'constant.png')
    return checks


def sinusoidal():
    fig, axes = plt.subplots(2, 2, figsize=(8, 6))
    checks = {}
    for ax, (speed, amp) in zip(axes.flat, [(2,.2), (5,.2), (2,.6), (5,.6)]):
        control = lambda time: amp*np.sin(time)
        t, z = rk4(speed, control, 4*np.pi)
        _, fine = rk4(speed, control, 4*np.pi, .005)
        discrepancy = float(np.max(abs(z[-1]-fine[-1])))
        assert discrepancy < 1e-6
        checks[f'V{speed}_a{amp}'] = discrepancy
        ax.plot(z[:, 0], z[:, 1])
        ax.set(title=f'V={speed} m/s, amplitude={amp} rad',
               xlabel='X [m]', ylabel='Y [m]')
        ax.set_aspect('equal', adjustable='box'); ax.grid(alpha=.3)
        export(f'sine_V{speed}_a{amp}.csv', t, z)
    figure_save(fig, 'sinusoidal.png')
    return checks


def square():
    def control(time):
        return .25 if int(np.floor(time/2)) % 2 == 0 else -.25
    t, z = rk4(4., control, 12.)
    fig, axes = plt.subplots(1, 2, figsize=(9, 3.5))
    axes[0].plot(z[:, 0], z[:, 1])
    axes[0].set(xlabel='X [m]', ylabel='Y [m]', title='Alternating circular arcs')
    axes[0].set_aspect('equal', adjustable='box')
    axes[1].step(t, [control(time) for time in t], where='post')
    axes[1].set(xlabel='Time [s]', ylabel='Steering [rad]', title='Ideal square wave')
    for ax in axes: ax.grid(alpha=.3)
    figure_save(fig, 'square.png')
    export('square.csv', t, z)


if __name__ == '__main__':
    checks = dict(constant=constant(), sinusoid_refinement=sinusoidal())
    square()
    (DATA/'kinematic_checks.json').write_text(json.dumps(checks, indent=2))
    print('Kinematic checks passed.')
