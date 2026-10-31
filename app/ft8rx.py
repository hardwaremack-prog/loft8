"""
FT8 decoder (Python + numpy).

Give it 15 seconds of audio at 12 kHz (USB audio from a receiver tuned to
an FT8 channel) and it returns the decoded messages, like WSJT-X does.

Algorithm follows Karlis Goba's ft8_lib (MIT license): an oversampled
waterfall, Costas-array sync search, soft-decision belief-propagation LDPC
decoding, CRC check and WSJT-X message unpacking. A second pass subtracts
already-decoded signals to dig out weaker ones underneath.
"""
import numpy as np

import ft8

SAMPLE_RATE = 12000
BLOCK = 1920                 # samples per FT8 symbol at 12 kHz
TIME_OSR = 2
FREQ_OSR = 2
SUB = BLOCK // TIME_OSR
NFFT = BLOCK * FREQ_OSR
F_MIN, F_MAX = 200.0, 3000.0
MIN_BIN = int(F_MIN * ft8.SYMBOL_PERIOD)
MAX_BIN = int(F_MAX * ft8.SYMBOL_PERIOD) + 1
NUM_BINS = MAX_BIN - MIN_BIN
MIN_SCORE = 10
MAX_CANDIDATES = 140
LDPC_ITERS = 25
DT_ZERO = 0.65               # waterfall time of a signal that starts exactly on time (0.5 s)

COSTAS = np.array(ft8.COSTAS)
GRAY = np.array(ft8.GRAY)

# ---------------------------------------------------------------- LDPC tables
_NM = [[4, 31, 59, 91, 92, 96, 153], [5, 32, 60, 93, 115, 146], [6, 24, 61, 94, 122, 151], [7, 33, 62, 95, 96, 143], [8, 25, 63, 83, 93, 96, 148], [6, 32, 64, 97, 126, 138], [5, 34, 65, 78, 98, 107, 154], [9, 35, 66, 99, 139, 146], [10, 36, 67, 100, 107, 126], [11, 37, 67, 87, 101, 139, 158], [12, 38, 68, 102, 105, 155], [13, 39, 69, 103, 149, 162], [8, 40, 70, 82, 104, 114, 145], [14, 41, 71, 88, 102, 123, 156], [15, 42, 59, 106, 123, 159], [1, 33, 72, 106, 107, 157], [16, 43, 73, 108, 141, 160], [17, 37, 74, 81, 109, 131, 154], [11, 44, 75, 110, 121, 166], [45, 55, 64, 111, 130, 161, 173], [8, 46, 71, 112, 119, 166], [18, 36, 76, 89, 113, 114, 143], [19, 38, 77, 104, 116, 163], [20, 47, 70, 92, 138, 165], [2, 48, 74, 113, 128, 160], [21, 45, 78, 83, 117, 121, 151], [22, 47, 58, 118, 127, 164], [16, 39, 62, 112, 134, 158], [23, 43, 79, 120, 131, 145], [19, 35, 59, 73, 110, 125, 161], [20, 36, 63, 94, 136, 161], [14, 31, 79, 98, 132, 164], [3, 44, 80, 124, 127, 169], [19, 46, 81, 117, 135, 167], [7, 49, 58, 90, 100, 105, 168], [12, 50, 61, 118, 119, 144], [13, 51, 64, 114, 118, 157], [24, 52, 76, 129, 148, 149], [25, 53, 69, 90, 101, 130, 156], [20, 46, 65, 80, 120, 140, 170], [21, 54, 77, 100, 140, 171], [35, 82, 133, 142, 171, 174], [14, 30, 83, 113, 125, 170], [4, 29, 68, 120, 134, 173], [1, 4, 52, 57, 86, 136, 152], [26, 51, 56, 91, 122, 137, 168], [52, 84, 110, 115, 145, 168], [7, 50, 81, 99, 132, 173], [23, 55, 67, 95, 172, 174], [26, 41, 77, 109, 141, 148], [2, 27, 41, 61, 62, 115, 133], [27, 40, 56, 124, 125, 126], [18, 49, 55, 124, 141, 167], [6, 33, 85, 108, 116, 156], [28, 48, 70, 85, 105, 129, 158], [9, 54, 63, 131, 147, 155], [22, 53, 68, 109, 121, 174], [3, 13, 48, 78, 95, 123], [31, 69, 133, 150, 155, 169], [12, 43, 66, 89, 97, 135, 159], [5, 39, 75, 102, 136, 167], [2, 54, 86, 101, 135, 164], [15, 56, 87, 108, 119, 171], [10, 44, 82, 91, 111, 144, 149], [23, 34, 71, 94, 127, 153], [11, 49, 88, 92, 142, 157], [29, 34, 87, 97, 147, 162], [30, 50, 60, 86, 137, 142, 162], [10, 53, 66, 84, 112, 128, 165], [22, 57, 85, 93, 140, 159], [28, 32, 72, 103, 132, 166], [28, 29, 84, 88, 117, 143, 150], [1, 26, 45, 80, 128, 147], [17, 27, 89, 103, 116, 153], [51, 57, 98, 163, 165, 172], [21, 37, 73, 138, 152, 169], [16, 47, 76, 130, 137, 154], [3, 24, 30, 72, 104, 139], [9, 40, 90, 106, 134, 151], [15, 58, 60, 74, 111, 150, 163], [18, 42, 79, 144, 146, 152], [25, 38, 65, 99, 122, 160], [17, 42, 75, 129, 170, 172]]
_NM_PAD = np.full((83, 7), -1, dtype=np.int64)
for _m, _row in enumerate(_NM):
    _NM_PAD[_m, :len(_row)] = np.array(_row) - 1
_NM_MASK = _NM_PAD >= 0
_MN = [[] for _ in range(174)]
for _m, _row in enumerate(_NM):
    for _n in _row:
        _MN[_n - 1].append(_m)
_MN = np.array(_MN)                                     # (174, 3)
# edge (m, n_idx) -> (n, m_idx)
_EDGE_N = np.where(_NM_MASK, _NM_PAD, 0)
_EDGE_MIDX = np.zeros((83, 7), dtype=np.int64)
for _m in range(83):
    for _j in range(7):
        if _NM_MASK[_m, _j]:
            _EDGE_MIDX[_m, _j] = list(_MN[_NM_PAD[_m, _j]]).index(_m)


def _parity_errors(plain):
    bits = np.where(_NM_MASK, plain[_EDGE_N], 0)
    return int(np.count_nonzero(bits.sum(axis=1) & 1))


def bp_decode(codeword, max_iters=LDPC_ITERS):
    """Belief propagation. codeword: 174 log-likelihoods. Returns (bits, errors)."""
    tov = np.zeros((174, 3))
    min_errors = 83
    plain = np.zeros(174, dtype=np.int64)
    for _ in range(max_iters):
        total = codeword + tov.sum(axis=1)
        plain = (total > 0).astype(np.int64)
        if plain.sum() == 0:
            break
        errors = _parity_errors(plain)
        if errors < min_errors:
            min_errors = errors
            if errors == 0:
                break
        # bits -> checks
        tnm = total[_EDGE_N] - tov[_EDGE_N, _EDGE_MIDX]
        toc = np.where(_NM_MASK, np.tanh(-tnm / 2), 1.0)
        # checks -> bits: product of all the other edges on the check
        pre = np.cumprod(np.hstack([np.ones((83, 1)), toc[:, :-1]]), axis=1)
        shifted = np.hstack([toc[:, 1:], np.ones((83, 1))])
        suf = np.cumprod(shifted[:, ::-1], axis=1)[:, ::-1]
        tmn = pre * suf
        val = -2 * np.arctanh(np.clip(tmn, -0.9999999, 0.9999999))
        tov[_EDGE_N[_NM_MASK], _EDGE_MIDX[_NM_MASK]] = val[_NM_MASK]
    return plain, min_errors


# ---------------------------------------------------------------- waterfall

class Waterfall:
    """mag[block, time_sub, freq_sub, bin] in 0.5 dB steps (0..255), like ft8_lib."""

    def __init__(self, audio):
        audio = np.asarray(audio, dtype=np.float64)
        num_blocks = min(len(audio) // BLOCK, int(15.0 / ft8.SYMBOL_PERIOD))
        nframes = num_blocks * TIME_OSR
        padded = np.concatenate([np.zeros(NFFT), audio[:num_blocks * BLOCK]])
        idx = (np.arange(nframes)[:, None] + 1) * SUB + np.arange(NFFT)[None, :]
        window = (2.0 / NFFT) * np.sin(np.pi * np.arange(NFFT) / NFFT) ** 2
        spec = np.fft.rfft(padded[idx] * window, axis=1)
        p = spec.real ** 2 + spec.imag ** 2
        bins = (np.arange(MIN_BIN, MAX_BIN)[None, :] * FREQ_OSR + np.arange(FREQ_OSR)[:, None])  # (fs, nb)
        db = 10 * np.log10(1e-12 + p[:, bins])               # (frames, fs, nb)
        scaled = np.clip((2 * db + 240).astype(np.int64), 0, 255)
        self.mag = scaled.reshape(num_blocks, TIME_OSR, FREQ_OSR, NUM_BINS)
        self.num_blocks = num_blocks
        self.max_db = float(db.max()) if db.size else -120.0

    def magdb(self):
        return self.mag * 0.5 - 120.0


def find_candidates(wf):
    mag = wf.mag
    nb = NUM_BINS - 7
    cands = []
    for ts in range(TIME_OSR):
        for fs in range(FREQ_OSR):
            plane = mag[:, ts, fs, :]                      # (blocks, bins)
            for t0 in range(-10, 20):
                score = np.zeros(nb, dtype=np.int64)
                navg = 0
                for m in range(3):
                    for k in range(7):
                        block = 36 * m + k
                        babs = t0 + block
                        if babs < 0 or babs >= wf.num_blocks:
                            continue
                        sm = COSTAS[k]
                        row = plane[babs]
                        here = row[sm:sm + nb]
                        if sm > 0:
                            score += here - row[sm - 1:sm - 1 + nb]; navg += 1
                        if sm < 7:
                            score += here - row[sm + 1:sm + 1 + nb]; navg += 1
                        if k > 0 and babs > 0:
                            score += here - plane[babs - 1][sm:sm + nb]; navg += 1
                        if k + 1 < 7 and babs + 1 < wf.num_blocks:
                            score += here - plane[babs + 1][sm:sm + nb]; navg += 1
                if navg == 0:
                    continue
                score = np.trunc(score / navg).astype(np.int64)
                good = np.nonzero(score >= MIN_SCORE)[0]
                for f0 in good:
                    cands.append((int(score[f0]), t0, ts, int(f0), fs))
    cands.sort(key=lambda c: -c[0])
    return cands[:MAX_CANDIDATES]


def _extract_logl(wf, cand):
    score, t0, ts, f0, fs = cand
    mag = wf.magdb()
    out = np.zeros(174)
    for k in range(58):
        sym = k + (7 if k < 29 else 14)
        b = t0 + sym
        if b < 0 or b >= wf.num_blocks:
            continue
        s2 = mag[b, ts, fs, f0 + GRAY]
        out[3 * k + 0] = max(s2[4:8]) - max(s2[0:4])
        out[3 * k + 1] = max(s2[[2, 3, 6, 7]]) - max(s2[[0, 1, 4, 5]])
        out[3 * k + 2] = max(s2[[1, 3, 5, 7]]) - max(s2[[0, 2, 4, 6]])
    var = (np.sum(out ** 2) - np.sum(out) ** 2 / 174) / 174
    if var > 0:
        out *= np.sqrt(24.0 / var)
    return out


# ---------------------------------------------------------------- unpacking

_A1, _A2, _A3, _A4, _A38, _FULL = ft8._A1, ft8._A2, ft8._A3, ft8._A4, ft8._A38, ft8._FULL


class CallHash:
    """Remembers full callsigns so hashed ones (<...>) can be shown."""

    def __init__(self):
        self.h22 = {}

    def save(self, call):
        call = call.strip("<>")
        try:
            self.h22[ft8._hash22(call)] = call
        except ft8.FT8Error:
            pass

    def lookup22(self, n22):
        c = self.h22.get(n22)
        return f"<{c}>" if c else "<...>"

    def lookup12(self, n12):
        for k, c in self.h22.items():
            if k >> 10 == n12:
                return f"<{c}>"
        return "<...>"


def _unpack28(n28, ip, i3, hashes):
    if n28 < ft8.NTOKENS:
        if n28 == 0:
            return "DE"
        if n28 == 1:
            return "QRZ"
        if n28 == 2:
            return "CQ"
        if n28 <= 1002:
            return "CQ %03d" % (n28 - 3)
        if n28 <= 532443:
            n = n28 - 1003
            s = ""
            for _ in range(4):
                s = _A4[n % 27] + s
                n //= 27
            return "CQ " + s.strip()
        return None
    n28 -= ft8.NTOKENS
    if n28 < ft8.MAX22:
        return hashes.lookup22(n28)
    n = n28 - ft8.MAX22
    c = [""] * 6
    c[5] = _A4[n % 27]; n //= 27
    c[4] = _A4[n % 27]; n //= 27
    c[3] = _A4[n % 27]; n //= 27
    c[2] = _A3[n % 10]; n //= 10
    c[1] = _A2[n % 36]; n //= 36
    if n >= 37:
        return None
    c[0] = _A1[n]
    call = "".join(c)
    if call.startswith("3D0") and call[3] != " ":
        call = "3DA0" + call[3:].strip()
    elif call[0] == "Q" and call[1].isalpha():
        call = "3X" + call[1:].strip()
    else:
        call = call.strip()
    if len(call) < 3:
        return None
    if ip:
        call += "/R" if i3 == 1 else "/P"
    hashes.save(call)
    return call


def _unpackgrid(g, ir):
    if g <= ft8.MAXGRID4:
        n = g
        d3 = n % 10; n //= 10
        d2 = n % 10; n //= 10
        c1 = chr(65 + n % 18); n //= 18
        c0 = chr(65 + n % 18)
        grid = f"{c0}{c1}{d2}{d3}"
        return ("R " + grid) if ir else grid
    irpt = g - ft8.MAXGRID4
    if irpt == 1:
        return ""
    if irpt == 2:
        return "RRR"
    if irpt == 3:
        return "RR73"
    if irpt == 4:
        return "73"
    return ("R" if ir else "") + "%+03d" % (irpt - 35)


def unpack(payload, hashes):
    """77-bit payload -> message text (or None)."""
    i3 = payload & 7
    n3 = (payload >> 3) & 7
    if i3 == 0 and n3 == 0:
        v = payload >> 6
        s = ""
        for _ in range(13):
            s = _FULL[v % 42] + s
            v //= 42
        return s.strip()
    if i3 in (1, 2):
        igrid = (payload >> 3) & 0x7FFF
        ir = (payload >> 18) & 1
        n29b = (payload >> 19) & ((1 << 29) - 1)
        n29a = (payload >> 48) & ((1 << 29) - 1)
        a = _unpack28(n29a >> 1, n29a & 1, i3, hashes)
        b = _unpack28(n29b >> 1, n29b & 1, i3, hashes)
        if a is None or b is None:
            return None
        if b.startswith("CQ") or (b in ("DE", "QRZ")):
            return None
        extra = _unpackgrid(igrid, ir)
        return " ".join(x for x in (a, b, extra) if x)
    if i3 == 4:
        icq = (payload >> 3) & 1
        nrpt = (payload >> 4) & 3
        iflip = (payload >> 6) & 1
        n58 = (payload >> 7) & ((1 << 58) - 1)
        n12 = (payload >> 65) & 0xFFF
        s = ""
        for _ in range(11):
            s = _A38[n58 % 38] + s
            n58 //= 38
        call58 = s.strip()
        hashes.save(call58)
        if icq:
            return "CQ " + call58
        other = hashes.lookup12(n12)
        to, de = (call58, other) if iflip else (other, call58)
        extra = {1: "RRR", 2: "RR73", 3: "73"}.get(nrpt, "")
        return " ".join(x for x in (to, de, extra) if x)
    return None   # contest / telemetry types not shown


# ---------------------------------------------------------------- decoding

def _check_crc(bits):
    payload = int("".join(map(str, bits[:77])), 2)
    crc = int("".join(map(str, bits[77:91])), 2)
    return payload if ft8._crc14_ft8lib(payload) == crc else None


def _snr(wf, cand, tones):
    score, t0, ts, f0, fs = cand
    lin = 10 ** (wf.magdb()[:, ts, fs, :] / 10)
    sig, n = 0.0, 0
    for i, t in enumerate(tones):
        b = t0 + i
        if 0 <= b < wf.num_blocks:
            sig += lin[b, f0 + t]
            n += 1
    if n == 0:
        return -24
    sig /= n
    noise = np.median(lin)
    snr = 10 * np.log10(max(sig / noise - 1, 1e-3)) - 26.0
    return int(round(max(-24, min(40, snr))))


def decode(audio, hashes=None, passes=2):
    """Decode one 15 s slot. audio: float array at 12 kHz starting at the slot boundary.
    Returns list of dicts: snr, dt, freq, text, payload."""
    if hashes is None:
        hashes = CallHash()
    audio = np.array(audio, dtype=np.float64)
    results = {}
    for p in range(passes):
        wf = Waterfall(audio)
        found_new = []
        for cand in find_candidates(wf):
            score, t0, ts, f0, fs = cand
            logl = _extract_logl(wf, cand)
            bits, errors = bp_decode(logl)
            if errors > 0:
                continue
            payload = _check_crc(bits)
            if payload is None or payload in results:
                continue
            text = unpack(payload, hashes)
            if text is None:
                continue
            tones = ft8.encode_tones(payload)
            freq = (MIN_BIN + f0 + fs / FREQ_OSR) / ft8.SYMBOL_PERIOD
            tsec = (t0 + ts / TIME_OSR) * ft8.SYMBOL_PERIOD
            results[payload] = {"snr": _snr(wf, cand, tones), "dt": round(tsec - DT_ZERO, 1),
                                "freq": int(round(freq)), "text": text, "payload": payload,
                                "tones": tones, "tsec": tsec}
            found_new.append(results[payload])
        if not found_new or p == passes - 1:
            break
        for r in found_new:
            subtract_signal(audio, r["tones"], r["freq"], r["tsec"] - (DT_ZERO - 0.5))
    # second chance for hashed calls learned later in the slot
    for r in results.values():
        if "<...>" in r["text"]:
            t = unpack(r["payload"], hashes)
            if t:
                r["text"] = t
    out = sorted(results.values(), key=lambda r: r["freq"])
    for r in out:
        r.pop("tsec", None)
    return out


def subtract_signal(audio, tones, freq_hz, t_start):
    """Estimate and remove one decoded FT8 signal (amplitude/phase fitted per symbol,
    then smoothed) so weaker signals under it can decode on the next pass."""
    n_sps = BLOCK
    start = int(round(t_start * SAMPLE_RATE))
    best = None
    # refine timing a little: the waterfall only knows time to 80 ms
    for shift in range(-480, 481, 120):
        s = start + shift
        e = _fit_energy(audio, tones, freq_hz, s)
        if best is None or e > best[0]:
            best = (e, s)
    start = best[1]
    n = len(tones) * n_sps
    lo, hi = max(0, start), min(len(audio), start + n)
    if hi - lo < n_sps * 20:
        return
    t = np.arange(start, start + n)
    finc = freq_hz + np.repeat(np.array(tones) * ft8.TONE_SPACING, n_sps)
    ref = np.exp(1j * 2 * np.pi * np.cumsum(finc) / SAMPLE_RATE)[lo - start:hi - start]
    seg = audio[lo:hi]
    # complex amplitude per symbol via correlation, smoothed over 3 symbols
    nsym = len(seg) // n_sps
    amps = np.array([np.vdot(ref[i * n_sps:(i + 1) * n_sps], seg[i * n_sps:(i + 1) * n_sps]) * 2 / n_sps
                     for i in range(nsym)])
    if nsym >= 3:
        amps = np.convolve(amps, np.ones(3) / 3, mode="same")
    env = np.repeat(amps, n_sps)
    m = min(len(env), len(seg))
    seg[:m] -= np.real(env[:m] * ref[:m])


def _fit_energy(audio, tones, freq_hz, start):
    n_sps = BLOCK
    tot = 0.0
    for i in range(0, len(tones), 4):
        s = start + i * n_sps
        if s < 0 or s + n_sps > len(audio):
            continue
        t = np.arange(n_sps)
        f = freq_hz + tones[i] * ft8.TONE_SPACING
        r = np.exp(1j * 2 * np.pi * f * t / SAMPLE_RATE)
        tot += abs(np.vdot(r, audio[s:s + n_sps])) ** 2
    return tot


def read_wav(path):
    """Read a WAV file -> float mono audio at 12 kHz."""
    import wave
    with wave.open(path, "rb") as w:
        sr, ch, sw, n = w.getframerate(), w.getnchannels(), w.getsampwidth(), w.getnframes()
        raw = w.readframes(n)
    dtype = {1: np.uint8, 2: np.int16, 4: np.int32}[sw]
    x = np.frombuffer(raw, dtype=dtype).astype(np.float64)
    if sw == 1:
        x -= 128
    x = x.reshape(-1, ch).mean(axis=1) / float(2 ** (8 * sw - 1))
    if sr != SAMPLE_RATE:
        x = resample(x, sr, SAMPLE_RATE)
    return x


def resample(x, sr_in, sr_out):
    """Windowed-sinc resampler (good enough for audio-band FT8)."""
    from math import gcd
    g = gcd(int(sr_in), int(sr_out))
    up, down = int(sr_out) // g, int(sr_in) // g
    if up == 1:
        h = lowpass_taps(0.45 / down, 32 * down + 1)
        y = np.convolve(x, h, mode="same")
        return y[::down]
    # generic: linear interpolate after anti-alias filtering (rare case)
    cutoff = 0.45 * min(1.0, sr_out / sr_in)
    y = np.convolve(x, lowpass_taps(cutoff, 129), mode="same")
    t_out = np.arange(int(len(x) * sr_out / sr_in)) * sr_in / sr_out
    return np.interp(t_out, np.arange(len(y)), y)


def lowpass_taps(cutoff, ntaps):
    """cutoff as a fraction of the sample rate (0..0.5)."""
    n = np.arange(ntaps) - (ntaps - 1) / 2
    h = 2 * cutoff * np.sinc(2 * cutoff * n) * np.blackman(ntaps)
    return h / h.sum()


if __name__ == "__main__":
    import sys, time
    hashes = CallHash()
    for p in sys.argv[1:]:
        t = time.time()
        res = decode(read_wav(p), hashes)
        for r in res:
            print("%+3d %4.1f %4d ~  %s" % (r["snr"], r["dt"], r["freq"], r["text"]))
        print("-- %d decodes in %.1f s" % (len(res), time.time() - t))
