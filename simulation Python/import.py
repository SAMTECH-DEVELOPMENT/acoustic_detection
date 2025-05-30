import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, CheckButtons

# Simulation parameters
fs = 8000           # Sampling frequency (Hz)
f_sig = 1000        # Signal frequency (Hz)
c = 343             # Speed of sound (m/s)
N = 8               # Number of microphones
radius = 0.05       # Array radius (m)
theta_sensors = np.arange(N) * 2 * np.pi / N
sensor_pos = np.vstack((radius * np.cos(theta_sensors),
                        radius * np.sin(theta_sensors)))  # (2, N)

# Noise source settings
noise_dirs = [-60, 100]   # angles in degrees
noise_dists = [2.0, 3.0]  # distances in meters

def compute_source_snapshot(angle_deg, dist):
    """Compute complex snapshot for a single source."""
    phi = np.deg2rad(angle_deg)
    source_pos = np.array([dist * np.cos(phi), dist * np.sin(phi)]).reshape(2, 1)
    distances = np.linalg.norm(sensor_pos - source_pos, axis=0)
    taus = distances / c
    return (1 / distances) * np.exp(-1j * 2 * np.pi * f_sig * taus)

def compute_snapshot(angle_deg, dist, filter_noise):
    """Combine drone and noise snapshots based on filter flag."""
    snap = compute_source_snapshot(angle_deg, dist)
    if not filter_noise:
        for nd, dd in zip(noise_dirs, noise_dists):
            snap += compute_source_snapshot(nd, dd)
    return snap

def compute_pseudospectrum(snapshot):
    """Compute delay-and-sum beamforming pseudospectrum."""
    angles_deg = np.linspace(-90, 90, 181)
    angles = np.deg2rad(angles_deg)
    psd = []
    for phi in angles:
        phase_shifts = (sensor_pos[0] * np.cos(phi) + sensor_pos[1] * np.sin(phi)) / c
        steering = np.exp(-1j * 2 * np.pi * f_sig * phase_shifts)
        psd.append(np.abs(np.vdot(steering, snapshot))**2)
    return angles_deg, np.array(psd)

# Initial settings
init_angle = 30
init_dist = 1.0
filter_initial = False
snapshot = compute_snapshot(init_angle, init_dist, filter_initial)
angles_deg, ps = compute_pseudospectrum(snapshot)
angles_rad = np.deg2rad(angles_deg)

# Create figure and subplots
fig = plt.figure(figsize=(16, 5))
ax_geom   = fig.add_subplot(1, 3, 1)
ax_beam   = fig.add_subplot(1, 3, 2)
ax_polar  = fig.add_subplot(1, 3, 3, projection='polar')
plt.subplots_adjust(left=0.05, right=0.75, bottom=0.25)

# Plot microphone geometry
ax_geom.plot(sensor_pos[0], sensor_pos[1], 'bo', label='Mics')
drone_pt, = ax_geom.plot(init_dist*np.cos(np.deg2rad(init_angle)),
                          init_dist*np.sin(np.deg2rad(init_angle)),
                          'r*', markersize=15, label='Drone')
noise_points = []
for nd, dd in zip(noise_dirs, noise_dists):
    x, y = dd * np.cos(np.deg2rad(nd)), dd * np.sin(np.deg2rad(nd))
    noise_points.append(ax_geom.plot(x, y, 'g^', markersize=10, label='Bird')[0])
ax_geom.set_aspect('equal')
ax_geom.set_xlim(-5, 5)
ax_geom.set_ylim(-5, 5)
ax_geom.set_title('Array Geometry with Drone & Birds')
ax_geom.legend(loc='upper left')

# Beamforming line plot
beam_line, = ax_beam.plot(angles_deg, ps, '-')
ax_beam.set_xlabel('Angle()')
ax_beam.set_ylabel('Power (a.u.)')
ax_beam.set_title('Beamforming feedback')
ax_beam.set_ylim(0, ps.max()*1.1)

# Polar heatmap
cmap = plt.cm.viridis
normed = ps / ps.max()
bars = ax_polar.bar(angles_rad, ps, width=np.deg2rad(1), bottom=0,
                    color=cmap(normed), alpha=0.8)
ax_polar.set_title('Directional Heatmap')
ax_polar.set_ylim(0, ps.max()*1.1)

# Sliders
ax_angle = plt.axes([0.05, 0.15, 0.6, 0.03])
ax_dist  = plt.axes([0.05, 0.10, 0.6, 0.03])
slider_angle = Slider(ax_angle, 'Angle ()', -90, 90, valinit=init_angle)
slider_dist  = Slider(ax_dist,  'Distance (m)', 0.5, 5.0, valinit=init_dist)

# Filter toggle
ax_check = plt.axes([0.80, 0.40, 0.15, 0.15])
check = CheckButtons(ax_check, ['Filter Birds'], [filter_initial])

def update(val):
    angle = slider_angle.val
    dist = slider_dist.val
    use_filter = check.get_status()[0]
    snap = compute_snapshot(angle, dist, use_filter)
    angles_deg, ps = compute_pseudospectrum(snap)
    angles_rad = np.deg2rad(angles_deg)

    # Update drone point
    drone_pt.set_data(dist*np.cos(np.deg2rad(angle)),
                      dist*np.sin(np.deg2rad(angle)))
    # Show/hide bird markers
    for pt in noise_points:
        pt.set_visible(not use_filter)

    # Update beam line
    beam_line.set_ydata(ps)
    ax_beam.set_ylim(0, ps.max()*1.1)

    # Update heatmap bars
    normed = ps / ps.max()
    for bar, height, col in zip(bars, ps, normed):
        bar.set_height(height)
        bar.set_facecolor(cmap(col))
    ax_polar.set_ylim(0, ps.max()*1.1)

    fig.canvas.draw_idle()

slider_angle.on_changed(update)
slider_dist.on_changed(update)
check.on_clicked(update)

plt.show()
