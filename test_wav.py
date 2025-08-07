import numpy as np
import soundfile as sf

DURATION = 1.0  # seconds
FREQ = 440.0    # Hz
sample_rates = [8000, 16000, 22050, 32000, 44100, 48000]

for sr in sample_rates:
    t = np.linspace(0, DURATION, int(sr * DURATION), endpoint=False)
    x = 0.5 * np.sin(2 * np.pi * FREQ * t)
    sf.write(f'test-{sr}.wav', x, sr)
    print(f'Wrote test-{sr}.wav')
