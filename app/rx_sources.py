"""
Receive-side audio sources for LoFT8.

Every source delivers 12 kHz mono audio into a SlotRecorder, which hands
back one complete 15-second FT8 slot at a time for decoding.

  SoundCardSource  any audio input: a radio's speaker/data jack, or an SDR
                   program (SDR++, SDR#, SDRangel) through a virtual audio
                   cable, set to USB mode on the FT8 frequency.
  RtlSdrSource     an RTL-SDR USB dongle driven directly: tunes the
                   frequency, demodulates upper sideband, no other software.
"""
import threading
import time

import numpy as np

import ft8rx

SR = ft8rx.SAMPLE_RATE


class SlotRecorder:
    """Collects audio with timestamps and cuts it into 15 s FT8 slots."""

    def __init__(self, clock, on_slot, on_spectrum=None):
        self.clock = clock                    # function -> corrected UTC seconds
        self.on_slot = on_slot                # callback(slot_start, audio)
        self.on_spectrum = on_spectrum        # callback(power_db_row) for the waterfall
        self.lock = threading.Lock()
        self.buf = np.zeros(0, dtype=np.float32)
        self.buf_start = None                 # UTC time of buf[0]
        self.level = 0.0
        self._spec_acc = np.zeros(0, dtype=np.float32)

    def push(self, audio):
        """audio: float array at 12 kHz that ends 'now'."""
        now = self.clock()
        audio = np.asarray(audio, dtype=np.float32)
        if len(audio) == 0:
            return
        self.level = float(np.sqrt(np.mean(audio ** 2)))
        with self.lock:
            if self.buf_start is None:
                self.buf_start = now - len(audio) / SR
            else:
                # gently re-anchor the timeline to the clock (handles drift/dropouts)
                expected_end = self.buf_start + (len(self.buf) + len(audio)) / SR
                err = now - expected_end
                if abs(err) > 0.5:
                    self.buf = np.zeros(0, dtype=np.float32)
                    self.buf_start = now - len(audio) / SR
                else:
                    self.buf_start += err * 0.05
            self.buf = np.concatenate([self.buf, audio])
            self._cut_slots()
        if self.on_spectrum:
            self._spec_acc = np.concatenate([self._spec_acc, audio])
            n = 2048
            while len(self._spec_acc) >= n:
                seg = self._spec_acc[:n]
                self._spec_acc = self._spec_acc[n // 2:]
                spec = np.abs(np.fft.rfft(seg * np.hanning(n))) ** 2
                # 0..3000 Hz
                row = 10 * np.log10(spec[: int(3000 * n / SR)] + 1e-12)
                self.on_spectrum(row)

    def _cut_slots(self):
        # first slot boundary we have the start of
        while True:
            first = np.ceil(self.buf_start / 15.0) * 15.0
            start_idx = int(round((first - self.buf_start) * SR))
            need = start_idx + int(14.8 * SR)       # decode as soon as the signals are over
            if len(self.buf) < need:
                # drop audio older than the next slot we could use
                if start_idx > 0 and start_idx < len(self.buf):
                    pass
                return
            audio = self.buf[start_idx:start_idx + 15 * SR].copy()
            self.on_slot(first, audio)
            drop = start_idx + int(14.5 * SR)
            self.buf = self.buf[drop:]
            self.buf_start += drop / SR


# ------------------------------------------------------------------ sound card

class SoundCardSource:
    name = "Sound card"

    def __init__(self, recorder, device=None):
        import sounddevice as sd             # imported here so the app runs without it
        self.sd = sd
        self.recorder = recorder
        self.device = device
        self.stream = None

    @staticmethod
    def list_devices():
        try:
            import sounddevice as sd
        except Exception:
            return []
        out = []
        for i, d in enumerate(sd.query_devices()):
            if d["max_input_channels"] > 0:
                out.append((i, d["name"]))
        return out

    def start(self):
        info = self.sd.query_devices(self.device, "input")
        rate = int(info["default_samplerate"])
        for r in (48000, 12000, 24000, 44100, rate):
            try:
                self.sd.check_input_settings(device=self.device, samplerate=r, channels=1)
                rate = r
                break
            except Exception:
                continue
        self.rate = rate
        self.carry = np.zeros(0)

        def cb(indata, frames, t, status):
            x = indata[:, 0].astype(np.float64)
            self.recorder.push(self._to12k(x))

        self.stream = self.sd.InputStream(device=self.device, channels=1, samplerate=rate,
                                          blocksize=int(rate * 0.1), callback=cb)
        self.stream.start()

    def _to12k(self, x):
        if self.rate == SR:
            return x
        if self.rate % SR == 0:
            # stateful decimation: keep filter history between blocks
            d = self.rate // SR
            if not hasattr(self, "_taps"):
                self._taps = ft8rx.lowpass_taps(0.45 / d, 24 * d + 1)
                self._hist = np.zeros(len(self._taps) - 1)
                self._phase = 0
            xx = np.concatenate([self._hist, x])
            y = np.convolve(xx, self._taps, mode="valid")
            self._hist = xx[-(len(self._taps) - 1):]
            out = y[self._phase::d]
            self._phase = (self._phase - len(y)) % d
            return out
        return ft8rx.resample(x, self.rate, SR)

    def stop(self):
        if self.stream:
            self.stream.stop()
            self.stream.close()
            self.stream = None


# ------------------------------------------------------------------ RTL-SDR

class _Fir:
    """Stateful complex FIR filter + decimator."""

    def __init__(self, taps, decim):
        self.taps = taps.astype(np.complex128)
        self.decim = decim
        self.hist = np.zeros(len(taps) - 1, dtype=np.complex128)
        self.phase = 0

    def process(self, x):
        xx = np.concatenate([self.hist, x])
        y = np.convolve(xx, self.taps, mode="valid")
        self.hist = xx[-(len(self.taps) - 1):]
        out = y[self.phase::self.decim]
        self.phase = (self.phase - len(y)) % self.decim
        return out


class UsbDemod:
    """Complex IQ at fs (dial frequency at +offset_hz) -> 12 kHz USB audio."""

    def __init__(self, fs=240000, offset_hz=60000):
        self.fs = fs
        self.decim = fs // SR
        self.offset = offset_hz
        self.n = 0
        self.stage1 = _Fir(ft8rx.lowpass_taps(4500 / fs, 8 * self.decim + 1), self.decim)
        self.stage2 = _Fir(ft8rx.lowpass_taps(1450 / SR, 161), 1)
        self.n2 = 0

    def process(self, iq):
        iq = np.asarray(iq, dtype=np.complex128)
        # bring the dial frequency down to 0 Hz, and the middle of the 0-3 kHz
        # passband (1500 Hz) to 0 so one symmetric filter keeps just the upper sideband
        idx = self.n + np.arange(len(iq))
        self.n += len(iq)
        lo = np.exp(-2j * np.pi * (self.offset + 1500.0) * idx / self.fs)
        y = self.stage1.process(iq * lo)                # 12 kHz complex, centred on 1500 Hz
        y = self.stage2.process(y)                      # keep -1450..+1450 -> 50..2950 Hz audio
        idx2 = self.n2 + np.arange(len(y))
        self.n2 += len(y)
        audio = np.real(y * np.exp(2j * np.pi * 1500.0 * idx2 / SR))
        self.n %= self.fs * 1000
        self.n2 %= SR * 1000
        return audio


class RtlSdrSource:
    name = "RTL-SDR"

    def __init__(self, recorder, dial_hz, ppm=0, gain="auto"):
        from rtlsdr import RtlSdr                # pyrtlsdr; needs librtlsdr installed
        self.RtlSdr = RtlSdr
        self.recorder = recorder
        self.dial_hz = dial_hz
        self.ppm = int(ppm)
        self.gain = gain
        self.sdr = None
        self.running = False
        self.demod = UsbDemod()

    def start(self):
        sdr = self.RtlSdr()
        sdr.sample_rate = self.demod.fs
        if self.ppm:
            sdr.freq_correction = self.ppm
        sdr.center_freq = self.dial_hz - self.demod.offset
        sdr.gain = self.gain
        self.sdr = sdr
        self.running = True
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        n = 24 * 1024                            # ~0.1 s per read
        while self.running:
            try:
                iq = self.sdr.read_samples(n)
            except Exception as e:
                self.error = str(e)
                self.running = False
                break
            self.recorder.push(self.demod.process(iq) * 4.0)

    def stop(self):
        self.running = False
        time.sleep(0.2)
        if self.sdr:
            try:
                self.sdr.close()
            except Exception:
                pass
            self.sdr = None


def rtl_available():
    try:
        from rtlsdr import RtlSdr  # noqa: F401
        return True, ""
    except Exception as e:
        return False, str(e)
