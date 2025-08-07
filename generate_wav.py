import numpy as np
import soundfile as sf

for sr in [16000, 22050, 44100, 48000]:
    t = np.linspace(0, 1, sr, endpoint=False)
    x = 0.5 * np.sin(2 * np.pi * 440 * t)
    sf.write(f'test-{sr}.wav', x, sr)