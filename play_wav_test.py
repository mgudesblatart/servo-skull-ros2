import sounddevice as sd
import soundfile as sf
import sys
import glob

# Usage: python3 play_wav_test.py [glob_pattern]
# Example: python3 play_wav_test.py 'test-*.wav'

def play_wav(filename):
    try:
        data, sr = sf.read(filename)
        print(f'Playing {filename} at {sr} Hz...')
        sd.play(data, sr)
        sd.wait()
        print(f'SUCCESS: {filename} played at {sr} Hz')
    except Exception as e:
        print(f'ERROR: {filename} failed to play at {sr if "sr" in locals() else "unknown"} Hz: {e}')

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 play_wav_test.py [glob_pattern]")
        sys.exit(1)
    pattern = sys.argv[1]
    files = glob.glob(pattern)
    if not files:
        print(f'No files found for pattern: {pattern}')
        sys.exit(1)
    for f in files:
        play_wav(f)

if __name__ == '__main__':
    main()
