"""Core 2: independent implementation of assignment Equation (10).
Primary simulation: exact constant-input flow, 0.01-s output.
Independent verification: segmented solve_ivp on a finer time grid.
"""
import json
import numpy as np
from scipy.linalg import expm
from scipy.integrate import solve_ivp
from bicycle import Vehicle, lqr
from kinematics import ROOT, DATA, figure_save, plt

STEP, STOP = .01, 6.
VEHICLE = Vehicle()
EVENTS = np.array([5., 95.])/VEHICLE.speed
ANGLE = np.arctan2(5., 90.)
BOUNDS = np.r_[0., EVENTS[0], EVENTS[0]+STEP,
               EVENTS[1], EVENTS[1]+STEP, STOP]
INPUTS = np.array([0., ANGLE/STEP, 0., -ANGLE/STEP, 0.])


def reference(t):
    t = np.asarray(t)
    X = VEHICLE.speed*t
    Y = np.clip((X-5)/18, 0, 5)-5
    heading = ANGLE*(np.clip((t-EVENTS[0])/STEP, 0, 1)
                     -np.clip((t-EVENTS[1])/STEP, 0, 1))
    rate = np.zeros_like(t)
    for left, right, value in zip(BOUNDS[:-1], BOUNDS[1:], INPUTS):
        rate[(t >= left) & (t < right)] = value
    return X, Y, heading, rate


def exact_samples(F, D, samples):
    # Merge the output times with switching times, so no step straddles a jump.
    merged = np.unique(np.r_[samples, BOUNDS])
    states = np.zeros((len(merged), 4))
    augmented = np.zeros((5, 5))
    augmented[:4, :4], augmented[:4, 4] = F, D
    flows = {}
    for j in range(len(merged)-1):
        left, right = merged[j:j+2]
        duration = round(float(right-left), 14)
        if duration not in flows:
            flows[duration] = expm(augmented*duration)
        E = flows[duration]
        segment = np.searchsorted(BOUNDS, (left+right)/2, side='right')-1
        states[j+1] = E[:4, :4]@states[j]+E[:4, 4]*INPUTS[segment]
    return states[np.searchsorted(merged, samples)]


def independent_ode(F, D, times):
    result = np.zeros((len(times), 4))
    initial = np.zeros(4)
    for left, right, value in zip(BOUNDS[:-1], BOUNDS[1:], INPUTS):
        sol = solve_ivp(lambda time, z: F@z+D*value,
                        (left, right), initial, dense_output=True,
                        rtol=2e-11, atol=2e-13, max_step=.002)
        assert sol.success
        chosen = (times >= left) & (times <= right)
        result[chosen] = sol.sol(times[chosen]).T
        initial = sol.y[:, -1]
    return result


def reconstruct(t, states):
    Xd, Yd, psid, _ = reference(t)
    psi = psid+states[:, 2]
    # Lecture 6 p31; positive e1 denotes vehicle-left from reference point.
    X = Xd-states[:, 0]*np.sin(psi)
    Y = Yd+states[:, 0]*np.cos(psi)
    recovered = -(X-Xd)*np.sin(psi)+(Y-Yd)*np.cos(psi)
    return X, Y, psi, recovered


def main():
    weights = json.loads((ROOT/'design.json').read_text())
    F, D, K = lqr(VEHICLE, weights['Q_diagonal'])
    poles = np.linalg.eigvals(F)
    assert np.all(poles.real < 0)
    t = np.arange(round(STOP/STEP)+1)*STEP
    x = exact_samples(F, D, t)
    dense_t = np.unique(np.r_[np.linspace(0, STOP, 60001), BOUNDS, EVENTS+1, 100/30])
    dense_x = independent_ode(F, D, dense_t)
    independent_at_samples = independent_ode(F, D, t)
    solver_error = float(np.max(abs(x-independent_at_samples)))
    assert solver_error < 1e-8
    _, _, _, rd = reference(dense_t)
    steer = -x@K
    steer_rate = -(dense_x@F.T+rd[:, None]*D)@K
    # Add both limits at each switch; steering is continuous in this model.
    endpoint_rates = []
    xb = exact_samples(F, D, BOUNDS)
    for j in range(1, len(BOUNDS)-1):
        endpoint_rates.extend([abs(K@(F@xb[j]+D*u))
                               for u in INPUTS[j-1:j+1]])
    rate_peak = float(max(np.max(abs(steer_rate)), *endpoint_rates))
    settling = []
    for start, end in zip(EVENTS, [EVENTS[1], STOP]):
        in_segment = (dense_t >= start) & (dense_t < end)
        outside = (abs(dense_x[:, 0]) > .002) | (abs(dense_x[:, 2]) > .0007)
        failures = np.flatnonzero(in_segment & outside)
        settling.append(float(dense_t[failures[-1]+1]-start) if failures.size else 0.)
    X, Y, psi, recovered = reconstruct(t, x)
    Xd, Yd, psid, _ = reference(t)
    geometry_error = float(np.max(abs(recovered-x[:, 0])))
    assert geometry_error < 1e-12
    terminal_t = np.array([100/30])
    terminal_x = exact_samples(F, D, terminal_t)
    terminal = reconstruct(terminal_t, terminal_x)
    sharp_heading = np.where((dense_t >= EVENTS[0]) & (dense_t < EVENTS[1]), ANGLE, 0.)
    finite_heading = reference(dense_t)[2]+dense_x[:, 2]
    diagnostics = dict(
        Q=weights['Q_diagonal'], R=1, K=K.tolist(),
        closed_loop_poles=[dict(real=float(z.real), imag=float(z.imag)) for z in poles],
        sample_interval_s=STEP, verification_interval_s=.0001,
        solver_max_difference=solver_error, geometry_max_difference=geometry_error,
        max_e1_m=float(np.max(abs(dense_x[:, 0]))),
        max_e2_rad=float(np.max(abs(dense_x[:, 2]))),
        max_continuous_steering_rate_rad_s=rate_peak,
        max_sampled_steering_rate_rad_s=float(np.max(abs(np.diff(steer)/STEP))),
        settling_s=settling,
        at_nominal_X100=dict(X=float(terminal[0][0]), Y=float(terminal[1][0]),
                            heading=float(terminal[2][0])),
        at_simulation_end=dict(X=float(X[-1]), Y=float(Y[-1]), heading=float(psi[-1])),
        max_heading_error_against_ideal_tangent_rad=float(np.max(abs(finite_heading-sharp_heading))),
        continuous_heading_corner_lower_bound_rad=float(ANGLE/2))
    diagnostics['course_model_checks'] = dict(
        heading=diagnostics['max_e2_rad'] <= .01,
        steering_rate=rate_peak <= 25,
        settling=max(settling) <= 1,
        initial_state=bool(np.all(x[0] == 0)))
    assert all(diagnostics['course_model_checks'].values())
    diagnostics['literal_checks'] = dict(
        ideal_tangent_heading=diagnostics['max_heading_error_against_ideal_tangent_rad'] <= .01,
        X100_within_error_bands=bool(abs(terminal_x[0, 0]) <= .002 and
                                     abs(terminal_x[0, 2]) <= .0007))
    (DATA/'lane_validation.json').write_text(json.dumps(diagnostics, indent=2))
    np.savetxt(DATA/'lane_samples.csv',
               np.column_stack([t, x, steer, Xd, Yd, X, Y, psi]), delimiter=',',
               header='t,e1,e1dot,e2,e2dot,delta,X_des,Y_des,X,Y,psi', comments='')
    fig, axes = plt.subplots(3, 1, figsize=(8, 7), sharex=True)
    for ax, values, label, limits in zip(axes,
            [dense_x[:, 0], dense_x[:, 2], steer_rate],
            ['e1 [m]', 'e2 [rad]', 'Steering rate [rad/s]'],
            [[.002], [.0007,.01], [25]]):
        ax.plot(dense_t, values)
        for threshold in limits:
            ax.axhline(threshold, color='red', ls='--', lw=.7)
            ax.axhline(-threshold, color='red', ls='--', lw=.7)
        for event in EVENTS:
            ax.axvline(event, color='gray', ls=':')
            ax.axvline(event+1, color='green', ls=':')
        ax.set_ylabel(label); ax.grid(alpha=.3)
    axes[-1].set_xlabel('Time [s]')
    figure_save(fig, 'lane_errors.png')
    fig, axes = plt.subplots(2, 1, figsize=(7, 4.4), sharex=True)
    for ax, col, limit in zip(axes, [0,2], [.002,.0007]):
        for event in EVENTS:
            sel=(dense_t>=event+.8)&(dense_t<=event+1.3)
            ax.plot(dense_t[sel]-event,dense_x[sel,col],label=f'Transition {event:.3f} s')
        ax.axhspan(-limit,limit,color='green',alpha=.12)
        ax.axvline(1,color='black',ls=':'); ax.grid(alpha=.3)
        ax.set_ylabel('e1 [m]' if col==0 else 'e2 [rad]')
    axes[0].legend(); axes[-1].set_xlabel('Time after transition [s]')
    figure_save(fig,'lane_settling.png')
    for filename, xr, yr in [('lane_global.png',(0,180),(-5.4,.5)),
                             ('lane_start.png',(2,23),(-5.2,-3.8)),
                             ('lane_end.png',(89,128),(-.4,.2))]:
        fig, ax = plt.subplots(figsize=(8, 3.3))
        ax.plot(Xd, Yd, 'k--', label='Desired polyline')
        ax.plot(X, Y, label='Course-model reconstruction')
        ax.set(xlim=xr, ylim=yr, xlabel='X [m]', ylabel='Y [m]')
        ax.legend(); ax.grid(alpha=.3)
        figure_save(fig, filename)
    fig, axes = plt.subplots(2,1,figsize=(8,4.7),sharex=True)
    axes[0].plot(dense_t, reference(dense_t)[2],label='Finite-input heading')
    axes[0].plot(dense_t,sharp_heading,'--',label='Ideal polyline tangent')
    axes[0].set_ylabel('Reference [rad]'); axes[0].legend()
    axes[1].plot(dense_t,finite_heading-sharp_heading)
    axes[1].axhline(.01,color='red',ls='--'); axes[1].axhline(-.01,color='red',ls='--')
    axes[1].set(xlabel='Time [s]',ylabel='Error to ideal tangent [rad]')
    for ax in axes:ax.grid(alpha=.3)
    figure_save(fig,'reference_audit.png')
    print(json.dumps(diagnostics, indent=2))


if __name__ == '__main__':
    main()
