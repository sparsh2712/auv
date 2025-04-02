import numpy as np
import matplotlib.pyplot as plt

c = 1400
fs = 200000
peak_height = 1

pinger_pos = np.array([0, 25, 2])
hydrophone_pos = np.array([[1, 0, 0], [0, 0, 0], [1, 1, 0]])

distance = np.linalg.norm(hydrophone_pos - pinger_pos, axis=1)

time_delays = distance / c
time_delays += np.random.rand()+0.5
data = np.zeros((3, 2 * fs))

for i in range(3):
    t = np.linspace(0, 2, 2 * fs)
    index = int(time_delays[i] * fs)
    data[i, index] = peak_height

np.save('sim_data.npy', data)

fig, axes = plt.subplots(3, 1, figsize=(10, 6), sharex=True)

for i in range(3):
    axes[i].plot(data[i], label=f'Hydrophone {i+1}')
    axes[i].set_title(f'Hydrophone {i+1}')
    axes[i].set_ylabel('Amplitude')
    axes[i].legend()

plt.xlabel('Sample Index')
plt.tight_layout()
plt.show()
