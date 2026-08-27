import numpy as np
import matplotlib.pyplot as plt

COUNTS_PER_REV = 2400
TWO_PI = 2 * np.pi

dt = 0.001
t = np.arange(0, 10, dt)

#pretend the pendulum follows sinusoidal motion
theta_true = 0.5 * np.sin(2 * np.pi * t)

#exact derivative
omega_true = np.pi * np.cos(2 * np.pi * t)

#simulate encoder measurements
counts = np.round(theta_true / TWO_PI * COUNTS_PER_REV)

theta_measured = counts * TWO_PI / COUNTS_PER_REV

#backwards finite difference derivative
omega_raw = np.zeros_like(theta_measured)

omega_raw[1:] = (
    theta_measured[1:] - theta_measured[:-1]
) / dt

# 1st order low pass filter
tau = 0.02
alpha = dt / (tau + dt)

omega_filtered = np.zeros_like(omega_raw)

for k in range(1, len(omega_raw)):
    omega_filtered[k] = (omega_filtered[k-1] + alpha * (omega_raw[k] - omega_filtered[k-1]))

plt.figure()
plt.plot(t, omega_true, label="true velocity")
plt.plot(t, omega_raw, label="raw finite difference")
plt.plot(t, omega_filtered, label="filtered estimate")

plt.xlabel("Time (s)")
plt.ylabel("Angular velocity (rad/s)")
plt.legend()
plt.show()

