import control as ct
import numpy as np
import matplotlib.pyplot as plt

G = ct.tf([0.93], [0.044, 1])
t = np.linspace(0, 0.3, 8192)
t, y = ct.step_response(G, T=t)

fourier_y = np.fft.fft(y)
freqs = np.fft.fftfreq(len(fourier_y), d=t[1] - t[0])

freq_band = (freqs >= 0) & (freqs <= 10000)
fourier_y = fourier_y[freq_band]
freqs = freqs[freq_band]

bode = ct.bode(G, dB=True, Hz=True, plot=True, label='Bode Plot', grid=True)

_, axs = plt.subplots(2, 1, figsize=(10, 8))
axs[0].plot(t, y)
axs[0].set_title('Respuesta a escalón')
axs[0].set_xlabel('Time (t)')
axs[0].set_ylabel('Response')
axs[0].grid()

axs[1].plot(freqs, 20 * np.log10(np.abs(fourier_y))) # Magnitud en dB
axs[1].set_yscale('log')
axs[1].set_xscale('log')
axs[1].set_title('Diagrama de Bode (Magnitud)')
axs[1].set_xlabel('Frecuencia (Hz)')
axs[1].set_ylabel('Magnitud (dB)')
axs[1].grid(True, which='both', axis='both')

plt.tight_layout()
plt.show()