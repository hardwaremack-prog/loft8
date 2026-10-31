"""
FT8 encoder (pure Python, no dependencies).

Turns an FT8 message like "CQ K1ABC FN42" into the 79 tone numbers (0-7)
that get transmitted, 160 ms each, 6.25 Hz apart.

Follows the WSJT-X / FT8 protocol. Packing, CRC and LDPC logic mirror
Karlis Goba's ft8_lib (MIT license, https://github.com/kgoba/ft8_lib),
and the output was checked tone-for-tone against it.
"""
import math
import re
import struct
import wave

TONE_SPACING = 6.25          # Hz
SYMBOL_PERIOD = 0.160        # s
NUM_SYMBOLS = 79
TX_DURATION = NUM_SYMBOLS * SYMBOL_PERIOD   # 12.64 s
SLOT_SECONDS = 15.0
TX_START_OFFSET = 0.5        # FT8 transmissions start 0.5 s into the slot

COSTAS = (3, 1, 4, 0, 6, 5, 2)
GRAY = (0, 1, 3, 2, 5, 6, 4, 7)

NTOKENS = 2063592
MAX22 = 4194304
MAXGRID4 = 32400

_LDPC_GEN_HEX = (
    "8329ce11bf31eaf509f27fc0",
    "761c264e25c2593354931320",
    "dc265902fb277c6410a1bdc0",
    "1b3f417858cd2dd33ec7f620",
    "09fda4fee04195fd034783a0",
    "077cccc11b8873ed5c3d48a0",
    "29b62afe3ca036f4fe1a9da0",
    "6054faf5f35d96d3b0c8c3e0",
    "e20798e4310eed27884ae900",
    "775c9c08e80e26ddae563180",
    "b0b811028c2bf997213487c0",
    "18a0c9231fc60adf5c5ea320",
    "76471e8302a0721e01b12b80",
    "ffbccb80ca8341fafb47b2e0",
    "66a72a158f9325a2bf671700",
    "c4243689fe85b1c51363a180",
    "0dff739414d1a1b34b1c2700",
    "15b48830636c8b99894972e0",
    "29a89c0d3de81d665489b0e0",
    "4f126f37fa51cbe61bd6b940",
    "99c47239d0d97d3c84e09400",
    "1919b75119765621bb4f1e80",
    "09db12d731faee0b86df6b80",
    "488fc33df43fbdeea4eafb40",
    "827423ee40b675f756eb5fe0",
    "abe197c484cb74757144a9a0",
    "2b500e4bc0ec5a6d2bdbdd00",
    "c474aa53d702187616693600",
    "8eba1a13db3390bd6718cec0",
    "753844673a27782cc42012e0",
    "06ff83a145c37035a5c12680",
    "3b37417858cc2dd33ec3f620",
    "9a4a5a28ee17ca9c324842c0",
    "bc29f465309c977e89610a40",
    "2663ae6ddf8b5ce2bb294880",
    "46f231efe457034c18144180",
    "3fb2ce85abe9b0c72e06fbe0",
    "de87481f282c153971a0a2e0",
    "fcd7ccf23c69fa99bba14120",
    "f0261447e9490ca8e474cec0",
    "4410115818196f95cdd70120",
    "088fc31df4bfbde2a4eafb40",
    "b8fef1b6307729fb0a078c00",
    "5afea7acccb77bbc9d99a900",
    "49a7016ac653f65ecdc90760",
    "1944d085be4e7da8d6cc7d00",
    "251f62adc4032f0ee7140020",
    "56471f8702a0721e00b12b80",
    "2b8e4923f2dd51e2d537fa00",
    "6b550a40a66f4755de95c260",
    "a18ad28d4e27fe92a4f6c840",
    "10c2e586388cb82a3d807580",
    "ef34a41817ee02133db2eb00",
    "7e9c0c54325a9c15836e0000",
    "3693e572d1fde4cdf079e860",
    "bfb2cec5abe1b0c72e07fbe0",
    "7ee18230c583cccc57d4b080",
    "a066cb2fedafc9f526641260",
    "bb23725abc47cc5f4cc4cd20",
    "ded9dba3bee40c59b5609b40",
    "d9a7016ac653e6decdc90360",
    "9ad46aed5f707f280ab5fc40",
    "e5921c77822587316d7d3c20",
    "4f14da8242a8b86dca733520",
    "8b8b507ad467d4441df770e0",
    "22831c9cf1169467ad04b680",
    "213b838fe2ae54c38ee71800",
    "5d926b6dd71f085181a4e120",
    "66ab79d4b29ee6e69509e560",
    "958148682d748a38dd68baa0",
    "b8ce020cf069c32a723ab140",
    "f4331d6d461607e957527460",
    "6da23ba424b9596133cf9c80",
    "a636bcbc7b30c5fbeae67fe0",
    "5cb0d86a07df654a9089a200",
    "f11f106848780fc9ecdd80a0",
    "1fbb5364fb8d2c9d730d5ba0",
    "fcb86bc70a50c9d02a5d0340",
    "a534433029eac15f322e34c0",
    "c989d9c7c3d3b8c55d751300",
    "7bb38b2f0186d46643ae9620",
    "2644ebadeb44b9467d1f42c0",
    "608cc857594bfbb55d696000",
)
# 83 rows x 91 bits (stored left-aligned in 96 bits)
_LDPC_GEN = [int(h, 16) >> 5 for h in _LDPC_GEN_HEX]

_A1 = " 0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ"   # alnum + space
_A2 = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ"    # alnum
_A3 = "0123456789"
_A4 = " ABCDEFGHIJKLMNOPQRSTUVWXYZ"             # letters + space
_FULL = " 0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ+-./?"
_A38 = " 0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ/"


class FT8Error(ValueError):
    pass


# ---------------------------------------------------------------- packing

def _pack_basecall(call):
    """Standard callsign -> 28-bit number, or -1."""
    n = len(call)
    if n <= 2:
        return -1
    c6 = None
    if call.startswith("3DA0") and 4 < n <= 7:
        c6 = "3D0" + call[4:]
    elif call.startswith("3X") and call[2:3].isalpha() and n <= 7:
        c6 = "Q" + call[2:]
    elif call[2:3].isdigit() and n <= 6:
        c6 = call
    elif call[1:2].isdigit() and n <= 5:
        c6 = " " + call
    if c6 is None:
        return -1
    c6 = c6.ljust(6)
    if len(c6) != 6:
        return -1
    tables = (_A1, _A2, _A3, _A4, _A4, _A4)
    v = 0
    for ch, tab, base in zip(c6, tables, (37, 36, 10, 27, 27, 27)):
        i = tab.find(ch)
        if i < 0:
            return -1
        v = v * base + i
    return v


def is_standard_call(call):
    base = call[:-2] if call.endswith(("/P", "/R")) else call
    return _pack_basecall(base) >= 0


def _hash22(call):
    n58 = 0
    for ch in call[:11]:
        j = _A38.find(ch)
        if j < 0:
            raise FT8Error("Bad character in callsign: %r" % call)
        n58 = n58 * 38 + j
    for _ in range(11 - min(len(call), 11)):
        n58 *= 38
    return ((47055833459 * n58) & ((1 << 64) - 1)) >> (64 - 22)


def _cq_modifier(word):
    """'DX' / 'POTA' / '123' after CQ -> value, else -1."""
    if not word or len(word) > 4 + 0 and not word.isdigit() and len(word) > 4:
        return -1
    if word.isdigit() and len(word) == 3:
        return int(word)
    if word.isalpha() and 1 <= len(word) <= 4:
        m = 0
        for ch in word:
            m = 27 * m + (ord(ch) - ord("A") + 1)
        return 1000 + m
    return -1


def _pack28(tok):
    """Returns (n28, ip) or raises."""
    if tok == "DE":
        return 0, 0
    if tok == "QRZ":
        return 1, 0
    if tok == "CQ":
        return 2, 0
    if tok.startswith("CQ "):
        v = _cq_modifier(tok[3:])
        if v < 0:
            raise FT8Error("Unsupported CQ modifier: %s" % tok)
        return 3 + v, 0
    ip = 0
    base = tok
    if tok.endswith("/P") or tok.endswith("/R"):
        ip, base = 1, tok[:-2]
    n = _pack_basecall(base)
    if n >= 0:
        return NTOKENS + MAX22 + n, ip
    if 3 <= len(tok) <= 11:
        return NTOKENS + _hash22(tok), 0
    raise FT8Error("Can't encode callsign: %s" % tok)


_REPORT_RE = re.compile(r"^(R)?([+-])(\d{2})$")
_GRID_RE = re.compile(r"^[A-R]{2}[0-9]{2}$")


def _packgrid(extra):
    """Returns 16-bit value (ir<<15 | g15), or None if not a valid extra."""
    if extra == "":
        return MAXGRID4 + 1
    if extra == "RRR":
        return MAXGRID4 + 2
    if extra == "RR73":
        return MAXGRID4 + 3
    if extra == "73":
        return MAXGRID4 + 4
    if _GRID_RE.match(extra):
        g = (ord(extra[0]) - 65) * 18 + (ord(extra[1]) - 65)
        return (g * 10 + int(extra[2])) * 10 + int(extra[3])
    if extra.startswith("R ") or (extra.startswith("R") and _GRID_RE.match(extra[1:])):
        return None   # "R + grid" isn't used in standard FT8 QSOs
    m = _REPORT_RE.match(extra)
    if m:
        dd = int(m.group(3)) * (-1 if m.group(2) == "-" else 1)
        if not -30 <= dd <= 30:
            return None
        v = MAXGRID4 + 35 + dd
        return v | 0x8000 if m.group(1) else v
    return None


def _encode_std(call_to, call_de, extra):
    n28a, ipa = _pack28(call_to)
    n28b, ipb = _pack28(call_de)
    g = _packgrid(extra)
    if g is None:
        raise FT8Error("Not a grid or report: %s" % extra)
    i3 = 1
    if call_to.endswith("/P") or call_de.endswith("/P"):
        if call_to.endswith("/R") or call_de.endswith("/R"):
            raise FT8Error("Can't mix /P and /R")
        i3 = 2
    n29a = (n28a << 1) | ipa
    n29b = (n28b << 1) | ipb
    return (n29a << 48) | (n29b << 19) | (g << 3) | i3


def _encode_nonstd(call_to, call_de, extra):
    icq = 1 if call_to == "CQ" else 0
    if not icq and len(call_to) < 3:
        raise FT8Error("Callsign too short")
    if icq:
        iflip, n12, call58 = 0, 0, call_de
    else:
        # the nonstandard call goes in plain text; the other one is hashed
        iflip = 1 if (not is_standard_call(call_to) and is_standard_call(call_de)) else 0
        call12, call58 = (call_de, call_to) if iflip else (call_to, call_de)
        n12 = _hash22(call12) >> 10
    if len(call58) > 11:
        raise FT8Error("Callsign too long")
    n58 = 0
    for ch in call58:
        j = _A38.find(ch)
        if j < 0:
            raise FT8Error("Bad character in callsign")
        n58 = n58 * 38 + j
    nrpt = 0 if icq else {"RRR": 1, "RR73": 2, "73": 3}.get(extra, 0)
    if not icq and extra not in ("", "RRR", "RR73", "73"):
        raise FT8Error("Nonstandard-call messages only allow RRR, RR73 or 73")
    if icq and extra:
        raise FT8Error("CQ from a nonstandard call can't include a grid")
    return (n12 << 65) | (n58 << 7) | (iflip << 6) | (nrpt << 4) | (icq << 3) | 4


def _encode_free(text):
    if len(text) > 13:
        raise FT8Error("Free text is limited to 13 characters")
    v = 0
    for ch in text.ljust(13):
        j = _FULL.find(ch)
        if j < 0:
            raise FT8Error("Free text can only use A-Z 0-9 space + - . / ?")
        v = v * 42 + j
    return v << 6   # n3 = 0, i3 = 0


def pack_message(text):
    """Message text -> (77-bit int, kind). kind is 'standard', 'nonstandard' or 'free text'."""
    msg = " ".join(text.upper().split())
    if not msg:
        raise FT8Error("Message is empty")
    toks = msg.split(" ")
    # CQ with modifier ("CQ DX K1ABC FN42") is one token
    if toks[0] == "CQ" and len(toks) >= 3 and _cq_modifier(toks[1]) >= 0:
        toks = ["CQ " + toks[1]] + toks[2:]
    if 2 <= len(toks) <= 3:
        call_to, call_de = toks[0], toks[1]
        extra = toks[2] if len(toks) == 3 else ""
        if call_to in ("CQ", "QRZ", "DE") or call_to.startswith("CQ ") or is_standard_call(call_to):
            if is_standard_call(call_de) and _packgrid(extra) is not None:
                if not (call_to == "CQ" and "/" in call_de and not call_de.endswith(("/P", "/R"))):
                    return _encode_std(call_to, call_de, extra), "standard"
        def compound(c):
            return "/" in c or (7 <= len(c) <= 11 and any(ch.isdigit() for ch in c))
        if (compound(call_de) or compound(call_to)) and re.fullmatch(r"[A-Z0-9/]{3,11}", call_de) \
                and (call_to == "CQ" or re.fullmatch(r"[A-Z0-9/]{3,11}", call_to)):
            try:
                return _encode_nonstd(call_to, call_de, extra), "nonstandard"
            except FT8Error:
                pass
    return _encode_free(msg), "free text"


# ---------------------------------------------------------------- FEC

def _crc14_ft8lib(payload77):
    # ft8_lib feeds the message bytes into the remainder register (bytewise
    # augmented form), which is equivalent to a plain CRC over 77 bits + 5 zeros
    # followed by 14 more zeros. Implemented exactly that way here:
    data = payload77 << 5            # 82 bits
    nbytes = 12
    buf = (data << (96 - 82)).to_bytes(nbytes, "big")
    rem = 0
    idx = 0
    for b in range(82):
        if b % 8 == 0:
            rem ^= buf[idx] << 6
            idx += 1
        if rem & 0x2000:
            rem = (rem << 1) ^ 0x2757
        else:
            rem <<= 1
    return rem & 0x3FFF


def encode_tones(payload77):
    """77-bit payload -> list of 79 tones."""
    crc = _crc14_ft8lib(payload77)
    a91 = (payload77 << 14) | crc
    parity = 0
    for row in _LDPC_GEN:
        parity = (parity << 1) | (bin(a91 & row).count("1") & 1)
    cw = (a91 << 83) | parity        # 174 bits
    data = []
    for k in range(58):
        b3 = (cw >> (174 - 3 * (k + 1))) & 7
        data.append(GRAY[b3])
    return list(COSTAS) + data[:29] + list(COSTAS) + data[29:] + list(COSTAS)


def message_to_tones(text):
    payload, kind = pack_message(text)
    return encode_tones(payload), kind


# ---------------------------------------------------------------- audio preview

RADIO_STEP_HZ = 32e6 / (1 << 25)   # SX126x synthesizer step, ~0.954 Hz


def synth_wav(tones, path, audio_hz=1500.0, quantize_like_radio=False,
              sample_rate=12000, slot_aligned=True):
    """Write a 15 s mono WAV (GFSK, BT=2 like WSJT-X) for decoding tests.
    quantize_like_radio rounds each tone to the radio's 0.954 Hz steps."""
    n_sps = int(round(sample_rate * SYMBOL_PERIOD))
    bt = 2.0
    k = math.pi * math.sqrt(2.0 / math.log(2.0))
    pulse = []
    for i in range(3 * n_sps):
        t = i / n_sps - 1.5
        pulse.append((math.erf(k * bt * (t + 0.5)) - math.erf(k * bt * (t - 0.5))) / 2)

    def toneval(s):
        f = s * TONE_SPACING
        if quantize_like_radio:
            f = round((audio_hz + f) / RADIO_STEP_HZ) * RADIO_STEP_HZ - audio_hz
        return f

    n_sym = len(tones)
    total = (n_sym + 2) * n_sps
    dphi = [2 * math.pi * audio_hz / sample_rate] * total
    hmod = 2 * math.pi / sample_rate
    if quantize_like_radio:
        # radio hops frequency instantly (plain FSK), so no smoothing
        for i in range(n_sym):
            for j in range(n_sps):
                dphi[(i + 1) * n_sps + j] += hmod * toneval(tones[i])
    else:
        for i in range(n_sym):
            off = i * n_sps
            for j in range(3 * n_sps):
                dphi[off + j] += hmod * TONE_SPACING * tones[i] * pulse[j]
        for j in range(2 * n_sps):
            dphi[j] += hmod * TONE_SPACING * tones[0] * pulse[j + n_sps]
            dphi[n_sym * n_sps + j] += hmod * TONE_SPACING * tones[-1] * pulse[j]
    sig = []
    phi = 0.0
    for i in range(n_sps, total - n_sps):
        sig.append(math.sin(phi))
        phi = (phi + dphi[i]) % (2 * math.pi)
    lead = int(sample_rate * TX_START_OFFSET) if slot_aligned else 0
    out_len = int(sample_rate * SLOT_SECONDS)
    samples = [0.0] * lead + sig
    samples = (samples + [0.0] * out_len)[:out_len]
    with wave.open(path, "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(sample_rate)
        w.writeframes(b"".join(struct.pack("<h", int(s * 0.5 * 32767)) for s in samples))


if __name__ == "__main__":
    import sys
    for m in sys.argv[1:]:
        t, kind = message_to_tones(m)
        print("".join(map(str, t)), kind)
