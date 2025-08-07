import os
import glob
import soundfile as sf
import numpy as np
import sounddevice as sd
import scipy.signal

# Change this to wherever your TTS node writes WAVs
WAV_DIR = os.path.expanduser('~/projects/servo-skull')
TARGET_SR = 48000

def find_speaker_device():
    devices = sd.query_devices()
    # Try to find a device with 'speaker' or 'output' in the name, fallback to default
    print("Available audio devices:")
    print("\n".join(f"{idx}: {dev['name']} ({dev['max_output_channels']} channels)" for idx, dev in enumerate(devices)))
    for idx, dev in enumerate(devices):
        name = dev.get('name', '').lower()
        if dev['max_output_channels'] > 0 and ('speaker' in name or 'output' in name):
            return idx
    # Fallback: return default output device
    return sd.default.device[1]

def find_latest_wav(wav_dir):
    files = glob.glob(os.path.join(wav_dir, '*.wav'))
    if not files:
        raise FileNotFoundError(f'No WAV files found in {wav_dir}')
    return max(files, key=os.path.getmtime)

def resample_audio(audio, orig_sr, target_sr):
    if orig_sr == target_sr:
        return audio
    num_samples = int(len(audio) * target_sr / orig_sr)
    if audio.ndim == 1:
        return scipy.signal.resample(audio, num_samples)
    else:
        return scipy.signal.resample(audio, num_samples, axis=0)

def main():
    wav_path = find_latest_wav(WAV_DIR)
    print(f'Loading: {wav_path}')
    audio, sr = sf.read(wav_path, always_2d=False)
    print(f'Original sample rate: {sr}, shape: {audio.shape}')
    audio_rs = resample_audio(audio, sr, TARGET_SR)
    devices = sd.query_devices()
    print("Available audio devices:")
    for idx, dev in enumerate(devices):
        print(f"{idx}: {dev['name']} ({dev['max_output_channels']} channels)")
    try:
        device_idx = int(input("Select output device index (or blank for default): ") or -1)
    except Exception:
        device_idx = -1
    if device_idx < 0 or device_idx >= len(devices):
        device = None  # Use default
        print("Using default output device.")
        max_channels = 2
    else:
        device = device_idx
        max_channels = devices[device_idx]['max_output_channels']
        print(f'Using audio output device: {device}')
    # Force stereo if device expects it
    if max_channels >= 2 and (audio_rs.ndim == 1 or audio_rs.shape[-1] == 1):
        print("Duplicating mono audio to stereo for output device...")
        audio_rs = np.stack([audio_rs, audio_rs], axis=-1)
    print(f'Playing at {TARGET_SR} Hz, shape: {audio_rs.shape} ...')
    sd.play(audio_rs, samplerate=TARGET_SR, blocking=True, device=device)
    print('Done.')

if __name__ == '__main__':
    main()
