#!/usr/bin/env python3
"""
LoFT8 — send FT8 from a Meshtastic LoRa board (SX1262).

The app encodes FT8 messages, keeps FT8 slot timing (with an NTP clock
check), and tells the board's LoFT8 firmware which tones to play.
Works on Windows and Raspberry Pi OS.

Needs: Python 3.9+, tkinter, pyserial (and esptool only for flashing).
"""
import json
import os
import queue
import socket
import struct
import subprocess
import sys
import threading
import time
import tkinter as tk

import numpy as np
from tkinter import filedialog, messagebox, ttk

import ft8
import ft8rx
import rx_sources

try:
    import serial
    import serial.tools.list_ports
except ImportError:                       # handled nicely in the UI
    serial = None

APP_NAME = "LoFT8"
APP_DIR = os.path.dirname(os.path.abspath(__file__))
FW_DIR = os.path.join(APP_DIR, "firmware")
CONFIG_PATH = os.path.join(os.path.expanduser("~"), ".loft8.json")

BOARDS = {
    "Heltec V3 (also Wireless Stick Lite V3 / Tracker)": ("esp32s3", "loft8-heltec-v3.bin"),
    "LilyGO T-Beam v1.1/v1.2 (SX1262)": ("esp32", "loft8-tbeam-sx1262.bin"),
    "LilyGO T3-S3 (SX1262)": ("esp32s3", "loft8-t3s3-sx1262.bin"),
}

FREQ_PRESETS = {
    "70 cm FT8 — 432.174 MHz": 432.174,
    "Custom": None,
}

DEFAULTS = {
    "callsign": "", "grid": "", "their_call": "", "report": "-10",
    "preset": "70 cm FT8 — 432.174 MHz", "dial_mhz": "432.174",
    "audio_hz": "1500", "cal_hz": "0", "power": 10, "slot": "Even (:00 / :30)",
    "repeat_every": "2", "repeat_count": "5", "board": list(BOARDS)[0],
    "licensed": False, "free_text": "", "beep_mhz": "446.000", "rx_source": "", "rtl_ppm": "0", "rtl_gain": "auto",
}

# colors
BG = "#11161c"; PANEL = "#1a222b"; FG = "#e6edf3"; MUTED = "#8b98a5"
ACCENT = "#ffb020"; GOOD = "#3fb950"; BAD = "#f85149"; TXRED = "#ff5a5a"


# ------------------------------------------------------------------ clock

def sntp_offset(server="pool.ntp.org", timeout=2.0):
    """Seconds to ADD to the local clock to get true time."""
    pkt = b"\x1b" + 47 * b"\0"
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.settimeout(timeout)
        t1 = time.time()
        s.sendto(pkt, (server, 123))
        data, _ = s.recvfrom(48)
        t4 = time.time()
    if len(data) < 48:
        raise OSError("short NTP reply")
    def ts(off):
        sec, frac = struct.unpack("!II", data[off:off + 8])
        return sec - 2208988800 + frac / 2**32
    t2, t3 = ts(32), ts(40)
    return ((t2 - t1) + (t3 - t4)) / 2


# ------------------------------------------------------------------ app

class LoFT8App:
    def __init__(self, root):
        self.root = root
        self.cfg = dict(DEFAULTS)
        self.load_cfg()
        self.ser = None
        self.rx = queue.Queue()
        self.clock_offset = 0.0
        self.clock_checked = False
        self.tx_job = None            # dict while a send/beacon is scheduled
        self.transmitting = False
        self.tones = None

        self.rx_src = None
        self.rx_mode = None
        self.hashes = ft8rx.CallHash()
        self.decode_jobs = queue.Queue()
        self.spec_rows = []
        self.lora = {"samples": [], "slot": None}
        threading.Thread(target=self.decode_worker, daemon=True).start()
        root.title(f"{APP_NAME} — FT8 from your LoRa radio")
        root.configure(bg=BG)
        root.geometry("1060x880")
        root.minsize(940, 820)
        self.build_style()
        self.build_ui()
        self.refresh_ports()
        self.update_preview()
        root.after(100, self.poll)
        root.after(300, self.tick)
        threading.Thread(target=self.check_clock, daemon=True).start()
        root.protocol("WM_DELETE_WINDOW", self.on_close)

    # ---------------------------------------------------------- config
    def load_cfg(self):
        try:
            with open(CONFIG_PATH) as f:
                self.cfg.update(json.load(f))
        except Exception:
            pass

    def save_cfg(self):
        for k, v in self.vars.items():
            self.cfg[k] = v.get()
        try:
            with open(CONFIG_PATH, "w") as f:
                json.dump(self.cfg, f, indent=1)
        except Exception:
            pass

    # ---------------------------------------------------------- UI
    def build_style(self):
        st = ttk.Style()
        try:
            st.theme_use("clam")
        except tk.TclError:
            pass
        st.configure(".", background=BG, foreground=FG, fieldbackground=PANEL,
                     bordercolor="#2d3843", lightcolor=PANEL, darkcolor=PANEL,
                     font=("Segoe UI", 10) if sys.platform == "win32" else ("DejaVu Sans", 10))
        st.configure("Panel.TFrame", background=PANEL)
        st.configure("Panel.TLabel", background=PANEL, foreground=FG)
        st.configure("Muted.TLabel", background=PANEL, foreground=MUTED)
        st.configure("Head.TLabel", background=PANEL, foreground=ACCENT, font=("", 11, "bold"))
        st.configure("Title.TLabel", background=BG, foreground=FG, font=("", 18, "bold"))
        st.configure("Sub.TLabel", background=BG, foreground=MUTED)
        st.configure("TButton", background="#26313c", foreground=FG, padding=(10, 5))
        st.map("TButton", background=[("active", "#33414f"), ("disabled", "#1f272f")],
               foreground=[("disabled", "#56616c")])
        st.configure("Accent.TButton", background=ACCENT, foreground="#111")
        st.map("Accent.TButton", background=[("active", "#ffc24d"), ("disabled", "#5c4a20")])
        st.configure("Stop.TButton", background="#7a2323", foreground=FG)
        st.map("Stop.TButton", background=[("active", "#963030")])
        st.configure("TCheckbutton", background=PANEL, foreground=FG)
        st.configure("TRadiobutton", background=PANEL, foreground=FG)
        st.configure("TCombobox", fieldbackground=PANEL, background="#26313c", foreground=FG,
                     arrowcolor=FG)
        st.configure("TEntry", fieldbackground="#0d1217", foreground=FG, insertcolor=FG)
        st.configure("Horizontal.TScale", background=PANEL, troughcolor="#0d1217")
        st.configure("Horizontal.TProgressbar", troughcolor="#0d1217", background=TXRED, bordercolor=PANEL)
        st.map("TCombobox", fieldbackground=[("readonly", "#0d1217")], foreground=[("readonly", FG)],
               selectbackground=[("readonly", "#0d1217")], selectforeground=[("readonly", FG)])
        st.configure("TNotebook", background=BG, borderwidth=0, tabmargins=(0, 0, 0, 0))
        st.configure("TNotebook.Tab", background="#1f2831", foreground=MUTED, padding=(18, 7),
                     font=("", 11, "bold"))
        st.map("TNotebook.Tab", background=[("selected", PANEL)], foreground=[("selected", ACCENT)])
        st.configure("Treeview", background="#0d1217", fieldbackground="#0d1217", foreground=FG,
                     rowheight=22, borderwidth=0,
                     font=("Consolas", 10) if sys.platform == "win32" else ("DejaVu Sans Mono", 10))
        st.configure("Treeview.Heading", background="#26313c", foreground=MUTED, relief="flat")
        st.map("Treeview", background=[("selected", "#2b4a73")])
        self.root.option_add("*TCombobox*Listbox.background", PANEL)
        self.root.option_add("*TCombobox*Listbox.foreground", FG)

    def panel(self, parent, title):
        f = ttk.Frame(parent, style="Panel.TFrame", padding=12)
        ttk.Label(f, text=title, style="Head.TLabel").grid(row=0, column=0, columnspan=6, sticky="w", pady=(0, 6))
        return f

    def build_ui(self):
        c = self.cfg
        self.vars = {
            "callsign": tk.StringVar(value=c["callsign"]), "grid": tk.StringVar(value=c["grid"]),
            "their_call": tk.StringVar(value=c["their_call"]), "report": tk.StringVar(value=c["report"]),
            "preset": tk.StringVar(value=c["preset"]), "dial_mhz": tk.StringVar(value=c["dial_mhz"]),
            "audio_hz": tk.StringVar(value=c["audio_hz"]), "cal_hz": tk.StringVar(value=c["cal_hz"]),
            "power": tk.IntVar(value=c["power"]), "slot": tk.StringVar(value=c["slot"]),
            "repeat_every": tk.StringVar(value=c["repeat_every"]),
            "repeat_count": tk.StringVar(value=c["repeat_count"]),
            "board": tk.StringVar(value=c["board"]), "licensed": tk.BooleanVar(value=c["licensed"]),
            "free_text": tk.StringVar(value=c["free_text"]),
        }
        v = self.vars
        self.msg_var = tk.StringVar()

        # header
        top = ttk.Frame(self.root, padding=(16, 12, 16, 4))
        top.pack(fill="x")
        ttk.Label(top, text="LoFT8", style="Title.TLabel").pack(side="left")
        ttk.Label(top, text="   FT8 for SX1262 Meshtastic boards", style="Sub.TLabel").pack(side="left", pady=(6, 0))
        self.clock_lbl = tk.Label(top, text="clock: checking…", bg=BG, fg=MUTED, font=("", 10))
        self.clock_lbl.pack(side="right")
        self.utc_lbl = tk.Label(top, text="", bg=BG, fg=FG, font=("Consolas" if sys.platform == "win32" else "DejaVu Sans Mono", 14, "bold"))
        self.utc_lbl.pack(side="right", padx=14)

        body = ttk.Frame(self.root, padding=(16, 6))
        body.pack(fill="both", expand=True)
        body.columnconfigure(0, weight=1, uniform="c")
        body.columnconfigure(1, weight=1, uniform="c")
        body.rowconfigure(1, weight=1)

        # --- radio panel
        rp = self.panel(body, "Radio")
        rp.grid(row=0, column=0, sticky="nsew", padx=(0, 6), pady=(0, 10))
        ttk.Label(rp, text="Port", style="Panel.TLabel").grid(row=1, column=0, sticky="w")
        self.port_cb = ttk.Combobox(rp, width=22, state="readonly")
        self.port_cb.grid(row=1, column=1, columnspan=2, sticky="we", padx=4)
        ttk.Button(rp, text="↻", width=3, command=self.refresh_ports).grid(row=1, column=3)
        self.conn_btn = ttk.Button(rp, text="Connect", command=self.toggle_connect)
        self.conn_btn.grid(row=1, column=4, padx=(6, 0))
        self.radio_lbl = ttk.Label(rp, text="Not connected", style="Muted.TLabel")
        self.radio_lbl.grid(row=2, column=0, columnspan=5, sticky="w", pady=(4, 6))
        ttk.Label(rp, text="Power", style="Panel.TLabel").grid(row=3, column=0, sticky="w")
        pw = ttk.Scale(rp, from_=-9, to=22, variable=v["power"], orient="horizontal",
                       command=lambda e: self.pwr_lbl.config(text=f"{int(float(e))} dBm"))
        pw.grid(row=3, column=1, columnspan=3, sticky="we", padx=4)
        pw.bind("<ButtonRelease-1>", lambda e: self.send_power())
        self.pwr_lbl = ttk.Label(rp, text=f"{v['power'].get()} dBm", style="Panel.TLabel", width=7)
        self.pwr_lbl.grid(row=3, column=4, sticky="w")
        ttk.Label(rp, text="Firmware", style="Panel.TLabel").grid(row=4, column=0, sticky="w", pady=(8, 0))
        ttk.Combobox(rp, textvariable=v["board"], values=list(BOARDS), state="readonly", width=30)\
            .grid(row=4, column=1, columnspan=3, sticky="we", padx=4, pady=(8, 0))
        ttk.Button(rp, text="Flash", command=self.flash).grid(row=4, column=4, padx=(6, 0), pady=(8, 0))
        rp.columnconfigure(1, weight=1)

        # --- frequency panel
        fp = self.panel(body, "Frequency")
        fp.grid(row=0, column=1, sticky="nsew", padx=(6, 0), pady=(0, 10))
        ttk.Label(fp, text="Band", style="Panel.TLabel").grid(row=1, column=0, sticky="w")
        pc = ttk.Combobox(fp, textvariable=v["preset"], values=list(FREQ_PRESETS), state="readonly", width=24)
        pc.grid(row=1, column=1, columnspan=3, sticky="we", padx=4)
        pc.bind("<<ComboboxSelected>>", self.on_preset)
        ttk.Label(fp, text="Dial (MHz)", style="Panel.TLabel").grid(row=2, column=0, sticky="w", pady=3)
        ttk.Entry(fp, textvariable=v["dial_mhz"], width=12).grid(row=2, column=1, sticky="w", padx=4)
        ttk.Label(fp, text="Audio offset (Hz)", style="Panel.TLabel").grid(row=2, column=2, sticky="e")
        ttk.Entry(fp, textvariable=v["audio_hz"], width=7).grid(row=2, column=3, sticky="w", padx=4)
        ttk.Label(fp, text="Calibration (Hz)", style="Panel.TLabel").grid(row=3, column=0, sticky="w", pady=3)
        ttk.Entry(fp, textvariable=v["cal_hz"], width=12).grid(row=3, column=1, sticky="w", padx=4)
        tb = ttk.Frame(fp, style="Panel.TFrame")
        tb.grid(row=3, column=2, columnspan=2, sticky="e", padx=4)
        ttk.Button(tb, text="Beep test", command=self.beep_test).pack(side="left", padx=(0, 6))
        ttk.Button(tb, text="Tune 10 s", command=self.tune).pack(side="left")
        self.freq_lbl = ttk.Label(fp, text="", style="Muted.TLabel", wraplength=440)
        self.freq_lbl.grid(row=4, column=0, columnspan=4, sticky="w", pady=(4, 0))
        fp.columnconfigure(1, weight=1)
        for key in ("dial_mhz", "audio_hz", "cal_hz"):
            v[key].trace_add("write", lambda *a: self.update_freq_label())

        # --- tabs
        self.nb = ttk.Notebook(body)
        self.nb.grid(row=1, column=0, columnspan=2, sticky="nsew")
        txtab = ttk.Frame(self.nb, padding=(0, 10, 0, 0))
        rxtab = ttk.Frame(self.nb, padding=(0, 10, 0, 0))
        self.nb.add(txtab, text="Transmit")
        self.nb.add(rxtab, text="Receive")
        self.txtab = txtab
        txtab.columnconfigure(0, weight=1, uniform="t")
        txtab.columnconfigure(1, weight=1, uniform="t")
        txtab.rowconfigure(1, weight=1)

        # --- message panel
        mp = self.panel(txtab, "Message")
        mp.grid(row=0, column=0, columnspan=2, sticky="nsew", pady=(0, 10))
        row = ttk.Frame(mp, style="Panel.TFrame")
        row.grid(row=1, column=0, columnspan=6, sticky="we")
        for i, (lbl, key, w) in enumerate((("My call", "callsign", 10), ("Grid", "grid", 6),
                                           ("Their call", "their_call", 10), ("Report", "report", 5))):
            ttk.Label(row, text=lbl, style="Panel.TLabel").pack(side="left", padx=(0 if i == 0 else 12, 4))
            e = ttk.Entry(row, textvariable=v[key], width=w)
            e.pack(side="left")
            v[key].trace_add("write", lambda *a, k=key: self.force_upper(k))
        qr = ttk.Frame(mp, style="Panel.TFrame")
        qr.grid(row=2, column=0, columnspan=6, sticky="we", pady=(10, 4))
        for name, fn in (("CQ", lambda m, g, t, r: f"CQ {m} {g}"),
                         ("Answer", lambda m, g, t, r: f"{t} {m} {g}"),
                         ("Report", lambda m, g, t, r: f"{t} {m} {r}"),
                         ("R+Report", lambda m, g, t, r: f"{t} {m} R{r}"),
                         ("RR73", lambda m, g, t, r: f"{t} {m} RR73"),
                         ("73", lambda m, g, t, r: f"{t} {m} 73")):
            ttk.Button(qr, text=name, command=lambda f=fn: self.quick(f)).pack(side="left", padx=(0, 6))
        ttk.Label(mp, text="Send", style="Panel.TLabel").grid(row=3, column=0, sticky="w", pady=(6, 0))
        me = ttk.Entry(mp, textvariable=self.msg_var, font=("Consolas" if sys.platform == "win32" else "DejaVu Sans Mono", 13))
        me.grid(row=3, column=1, columnspan=5, sticky="we", padx=(6, 0), pady=(6, 0))
        self.msg_var.trace_add("write", lambda *a: self.update_preview())
        self.kind_lbl = ttk.Label(mp, text="", style="Muted.TLabel")
        self.kind_lbl.grid(row=4, column=1, columnspan=5, sticky="w", padx=6, pady=(3, 0))
        self.canvas = tk.Canvas(mp, height=72, bg="#0d1217", highlightthickness=0)
        self.canvas.grid(row=5, column=0, columnspan=6, sticky="we", pady=(8, 0))
        self.canvas.bind("<Configure>", lambda e: self.draw_tones())
        mp.columnconfigure(5, weight=1)

        # --- send panel + log
        sp = self.panel(txtab, "Send")
        sp.grid(row=1, column=0, sticky="nsew", padx=(0, 6))
        ttk.Label(sp, text="Slot", style="Panel.TLabel").grid(row=1, column=0, sticky="w")
        ttk.Combobox(sp, textvariable=v["slot"], state="readonly", width=18,
                     values=["Even (:00 / :30)", "Odd (:15 / :45)", "Next free slot"])\
            .grid(row=1, column=1, columnspan=3, sticky="w", padx=4)
        ttk.Label(sp, text="Beacon: every", style="Panel.TLabel").grid(row=2, column=0, sticky="w", pady=6)
        ttk.Entry(sp, textvariable=v["repeat_every"], width=4).grid(row=2, column=1, sticky="w", padx=4)
        ttk.Label(sp, text="cycles,", style="Panel.TLabel").grid(row=2, column=2, sticky="w")
        bc = ttk.Frame(sp, style="Panel.TFrame"); bc.grid(row=2, column=3, sticky="w")
        ttk.Entry(bc, textvariable=v["repeat_count"], width=4).pack(side="left", padx=4)
        ttk.Label(bc, text="times", style="Panel.TLabel").pack(side="left")
        ttk.Checkbutton(sp, text="I hold an amateur license for this frequency",
                        variable=v["licensed"]).grid(row=3, column=0, columnspan=4, sticky="w", pady=(4, 8))
        br = ttk.Frame(sp, style="Panel.TFrame"); br.grid(row=4, column=0, columnspan=4, sticky="we")
        self.send_btn = ttk.Button(br, text="Send once", style="Accent.TButton", command=lambda: self.schedule(1))
        self.send_btn.pack(side="left")
        self.beacon_btn = ttk.Button(br, text="Start beacon", command=lambda: self.schedule(None))
        self.beacon_btn.pack(side="left", padx=6)
        ttk.Button(br, text="STOP", style="Stop.TButton", command=self.stop_all).pack(side="left")
        self.state_lbl = tk.Label(sp, text="Idle", bg=PANEL, fg=MUTED, font=("", 13, "bold"), anchor="w")
        self.state_lbl.grid(row=5, column=0, columnspan=4, sticky="we", pady=(12, 0))
        self.progress = ttk.Progressbar(sp, maximum=79)
        self.progress.grid(row=6, column=0, columnspan=4, sticky="we", pady=(6, 0))
        pb = ttk.Frame(sp, style="Panel.TFrame")
        pb.grid(row=7, column=0, columnspan=4, sticky="w", pady=(12, 0))
        ttk.Button(pb, text="▶ Play on speakers", command=self.play_preview).pack(side="left")
        ttk.Button(pb, text="Save WAV preview…", command=self.save_wav).pack(side="left", padx=6)
        sp.columnconfigure(3, weight=1)

        lp = self.panel(txtab, "Log")
        lp.grid(row=1, column=1, sticky="nsew", padx=(6, 0))
        self.log_txt = tk.Text(lp, height=10, bg="#0d1217", fg=FG, relief="flat", wrap="word", highlightthickness=0,
                               font=("Consolas" if sys.platform == "win32" else "DejaVu Sans Mono", 9))
        self.log_txt.grid(row=1, column=0, sticky="nsew")
        lp.rowconfigure(1, weight=1); lp.columnconfigure(0, weight=1)
        self.log_txt.tag_config("tx", foreground=TXRED)
        self.log_txt.tag_config("ok", foreground=GOOD)
        self.log_txt.tag_config("err", foreground=BAD)
        self.log_txt.tag_config("dim", foreground=MUTED)

        self.build_rx_tab(rxtab)
        self.update_freq_label()
        if not self.msg_var.get():
            self.msg_var.set(self.cfg.get("free_text") or "")
        if not self.msg_var.get() and v["callsign"].get():
            self.quick(lambda m, g, t, r: f"CQ {m} {g}")
        if serial is None:
            self.log("pyserial is not installed — run the installer script or: pip install pyserial", "err")

    def force_upper(self, key):
        var = self.vars[key]
        s = var.get()
        if s != s.upper():
            var.set(s.upper())

    def log(self, text, tag=None):
        stamp = time.strftime("%H:%M:%S", time.gmtime(self.now()))
        self.log_txt.insert("end", f"{stamp}  ", "dim")
        self.log_txt.insert("end", text + "\n", tag)
        self.log_txt.see("end")

    # ---------------------------------------------------------- time
    def now(self):
        return time.time() + self.clock_offset

    def check_clock(self):
        try:
            off = sntp_offset()
            self.clock_offset = off
            self.clock_checked = True
            self.rx.put(("clock", off))
        except Exception as e:
            self.rx.put(("clock_err", str(e)))

    # ---------------------------------------------------------- freq
    def on_preset(self, _=None):
        f = FREQ_PRESETS.get(self.vars["preset"].get())
        if f:
            self.vars["dial_mhz"].set(f"{f:.3f}")

    def freq_plan(self):
        dial = float(self.vars["dial_mhz"].get()) * 1e6
        audio = float(self.vars["audio_hz"].get())
        cal = float(self.vars["cal_hz"].get() or 0)
        if not 200 <= audio <= 3000:
            raise ValueError("Audio offset should be 200–3000 Hz")
        f0 = dial + audio + cal
        if not 150e6 <= f0 <= 960e6:
            raise ValueError("SX1262 radios cover 150–960 MHz")
        return dial, audio, cal, f0

    def update_freq_label(self):
        try:
            dial, audio, cal, f0 = self.freq_plan()
            self.freq_lbl.config(text=f"RF tones {f0/1e6:.6f} – {(f0+43.75)/1e6:.6f} MHz  "
                                      f"(shows at {audio:.0f} Hz in a USB receiver on {dial/1e6:.3f})")
        except Exception as e:
            self.freq_lbl.config(text=f"⚠ {e}")

    # ---------------------------------------------------------- message
    def quick(self, fn):
        v = self.vars
        m, g, t, r = v["callsign"].get().strip(), v["grid"].get().strip()[:4], \
            v["their_call"].get().strip(), v["report"].get().strip()
        if r and r[0] not in "+-":
            r = "+" + r
        if r and len(r) == 2:
            r = r[0] + "0" + r[1]
        self.msg_var.set(" ".join(fn(m, g, t, r).split()))

    def update_preview(self):
        text = self.msg_var.get().strip()
        self.tones = None
        if not text:
            self.kind_lbl.config(text="Type a message or use the buttons above") if hasattr(self, "kind_lbl") else None
        else:
            try:
                tones, kind = ft8.message_to_tones(text)
                self.tones = tones
                warn = ""
                if kind == "free text":
                    warn = "  — sent as free text (13 characters max, no callsign ID)"
                self.kind_lbl.config(text=f"✓ FT8 {kind} message{warn}", foreground=GOOD if not warn else ACCENT)
            except ft8.FT8Error as e:
                self.kind_lbl.config(text=f"✗ {e}", foreground=BAD)
        if hasattr(self, "canvas"):
            self.draw_tones()

    def draw_tones(self, highlight=-1):
        c = self.canvas
        c.delete("all")
        w = max(c.winfo_width(), 100)
        h = int(c["height"])
        if not self.tones:
            c.create_text(w // 2, h // 2, text="tone preview", fill="#3a4651")
            return
        cw = w / 79
        rh = h / 8
        for i, t in enumerate(self.tones):
            sync = i < 7 or 36 <= i < 43 or i >= 72
            col = "#5b8cff" if sync else ACCENT
            if i < highlight:
                col = TXRED
            y = h - (t + 1) * rh
            c.create_rectangle(i * cw + 1, y + 1, (i + 1) * cw - 1, y + rh - 1, fill=col, outline="")

    # ---------------------------------------------------------- serial
    def refresh_ports(self):
        if serial is None:
            return
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_cb["values"] = ports
        if ports and self.port_cb.get() not in ports:
            # prefer USB serial adapters
            pref = [p for p in ports if any(s in p for s in ("USB", "ACM", "COM"))]
            self.port_cb.set((pref or ports)[-1])

    def toggle_connect(self):
        if self.ser:
            self.disconnect()
        else:
            self.connect()

    def connect(self):
        if serial is None:
            messagebox.showerror(APP_NAME, "pyserial is missing. Install it with: pip install pyserial")
            return
        port = self.port_cb.get()
        if not port:
            messagebox.showinfo(APP_NAME, "Plug in the board and pick its port.")
            return
        try:
            self.ser = serial.Serial(port, 115200, timeout=0.2)
        except Exception as e:
            messagebox.showerror(APP_NAME, f"Couldn't open {port}:\n{e}")
            return
        self.conn_btn.config(text="Disconnect")
        self.radio_lbl.config(text=f"Opened {port}, waiting for LoFT8 firmware…")
        threading.Thread(target=self.reader, args=(self.ser,), daemon=True).start()
        self.root.after(1500, lambda: self.cmd("HELLO"))
        self.root.after(1700, self.send_power)

    def disconnect(self):
        s, self.ser = self.ser, None
        if s:
            try:
                s.write(b"STOP\n")
                s.close()
            except Exception:
                pass
        self.conn_btn.config(text="Connect")
        self.radio_lbl.config(text="Not connected", foreground=MUTED)

    def reader(self, s):
        buf = b""
        while self.ser is s:
            try:
                data = s.read(256)
            except Exception as e:
                self.rx.put(("serial_err", str(e)))
                return
            if not data:
                continue
            buf += data
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                self.rx.put(("line", line.decode(errors="replace").strip()))

    def cmd(self, text):
        if not self.ser:
            return False
        try:
            self.ser.write((text + "\n").encode())
            self.ser.flush()
            return True
        except Exception as e:
            self.log(f"Serial write failed: {e}", "err")
            self.disconnect()
            return False

    def send_power(self):
        p = int(round(self.vars["power"].get()))
        self.vars["power"].set(p)
        self.pwr_lbl.config(text=f"{p} dBm")
        self.cmd(f"PWR {p}")

    # ---------------------------------------------------------- sending
    def ready_to_tx(self):
        v = self.vars
        if not self.ser:
            messagebox.showinfo(APP_NAME, "Connect to the radio first.")
            return False
        if not v["licensed"].get():
            messagebox.showwarning(APP_NAME, "Transmitting FT8 requires an amateur radio license.\n"
                                   "Tick the license box once you're sure you're allowed on this frequency.")
            return False
        if not ft8.is_standard_call(v["callsign"].get().strip()) and "/" not in v["callsign"].get():
            messagebox.showwarning(APP_NAME, "Enter your callsign first (it identifies your station).")
            return False
        try:
            self.freq_plan()
        except ValueError as e:
            messagebox.showerror(APP_NAME, str(e))
            return False
        if not self.tones:
            messagebox.showerror(APP_NAME, "The message can't be sent as FT8 — check the red note under it.")
            return False
        msg = self.msg_var.get().upper()
        if v["callsign"].get().strip() not in msg.split():
            if not messagebox.askyesno(APP_NAME, "This message doesn't contain your callsign, so it doesn't "
                                       "identify your station. Send it anyway?\n\n"
                                       "(Remember to send your call at least every 10 minutes.)"):
                return False
        if not self.clock_checked:
            self.log("Clock hasn't been checked against NTP — make sure your PC clock is right.", "err")
        return True

    def next_slot(self, after):
        slot = self.vars["slot"].get()
        t = (int(after // 15) + 1) * 15
        if slot.startswith("Even"):
            while (t // 15) % 2:
                t += 15
        elif slot.startswith("Odd"):
            while not (t // 15) % 2:
                t += 15
        return t

    def schedule(self, count):
        if not self.ready_to_tx():
            return
        try:
            every = max(1, int(self.vars["repeat_every"].get()))
            total = count if count else max(1, int(self.vars["repeat_count"].get()))
        except ValueError:
            messagebox.showerror(APP_NAME, "Beacon settings must be whole numbers.")
            return
        self.save_cfg()
        start = self.next_slot(self.now() + 0.3)
        self.tx_job = {"text": self.msg_var.get().strip().upper(), "tones": list(self.tones),
                       "next": start, "every": every * 30 if self.vars["slot"].get()[0] in "EO" else every * 15,
                       "left": total, "sent": 0, "total": total}
        when = time.strftime("%H:%M:%S", time.gmtime(start))
        self.log(f"Queued “{self.tx_job['text']}” ×{total} — first slot {when} UTC")
        self.send_btn.state(["disabled"]); self.beacon_btn.state(["disabled"])

    def fire(self):
        job = self.tx_job
        _, _, _, f0 = self.freq_plan()
        tones = "".join(map(str, job["tones"]))
        if self.cmd(f"TX {f0:.1f} {tones}"):
            self.transmitting = True
            self.tx_started = self.now()
            job["sent"] += 1
            self.log(f"TX {job['sent']}/{job['total']}  {job['text']}  @ {f0/1e6:.6f} MHz", "tx")
        job["left"] -= 1
        if job["left"] > 0:
            job["next"] += job["every"]
        else:
            job["next"] = None

    def stop_all(self):
        self.tx_job = None
        self.transmitting = False
        self.cmd("STOP")
        self.send_btn.state(["!disabled"]); self.beacon_btn.state(["!disabled"])
        self.state_lbl.config(text="Stopped", fg=MUTED)
        self.progress["value"] = 0
        self.draw_tones()

    def tune(self):
        if not self.ready_to_tx():
            return
        dial, audio, cal, f0 = self.freq_plan()
        self.cmd(f"TUNE {f0:.1f} 10")
        self.log(f"Tune carrier {f0/1e6:.6f} MHz for 10 s — measure it, then set Calibration = "
                 f"(where it should be) − (where it showed up)", "tx")

    def beep_test(self):
        v = self.vars
        if not self.ser:
            messagebox.showinfo(APP_NAME, "Connect to the radio first.")
            return
        if not v["licensed"].get():
            messagebox.showwarning(APP_NAME, "Tick the license box on the Transmit tab first.")
            return
        call = v["callsign"].get().strip().upper()
        if not call:
            messagebox.showwarning(APP_NAME, "Enter your callsign on the Transmit tab first (the beep test sends it in Morse).")
            return
        from tkinter import simpledialog
        mhz = simpledialog.askstring(
            APP_NAME, "Beep test: the board sends a 1-second tone, then\n"
            f"“VVV DE {call}” in Morse, as FM you can hear on a handheld.\n\n"
            "Set your 70 cm handheld to FM on this frequency (MHz):\n"
            "(446.000 is the US national 70 cm FM simplex frequency)",
            initialvalue=self.cfg.get("beep_mhz", "446.000"), parent=self.root)
        if not mhz:
            return
        try:
            hz = float(mhz) * 1e6
            if not 150e6 <= hz <= 960e6:
                raise ValueError
        except ValueError:
            messagebox.showerror(APP_NAME, "That isn't a frequency the SX1262 can use (150–960 MHz).")
            return
        self.cfg["beep_mhz"] = mhz.strip()
        self.save_cfg()
        try:
            cal = float(v["cal_hz"].get() or 0)
        except ValueError:
            cal = 0
        self.beep_pending = True
        self.cmd(f"MCW {hz + cal:.0f} 700 15 VVV DE {call}")
        self.log(f"Beep test on {hz/1e6:.4f} MHz FM — listen on your handheld (about 8 seconds).", "tx")

    def play_preview(self):
        if not self.tones:
            messagebox.showerror(APP_NAME, "Enter a valid message first.")
            return
        import tempfile
        path = os.path.join(tempfile.gettempdir(), "loft8_play.wav")
        ft8.synth_wav(self.tones, path, audio_hz=1000.0, slot_aligned=False)
        try:
            if sys.platform == "win32":
                import winsound
                winsound.PlaySound(path, winsound.SND_FILENAME | winsound.SND_ASYNC)
            else:
                import shutil
                player = shutil.which("paplay") or shutil.which("pw-play") or shutil.which("aplay")
                if not player:
                    raise RuntimeError("no audio player found (aplay/paplay)")
                subprocess.Popen([player, path], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            self.log("Playing what this FT8 message sounds like (13 s).", "ok")
        except Exception as e:
            messagebox.showerror(APP_NAME, f"Couldn't play audio: {e}")

    def save_wav(self):
        if not self.tones:
            messagebox.showerror(APP_NAME, "Enter a valid message first.")
            return
        path = filedialog.asksaveasfilename(defaultextension=".wav", initialfile="loft8_preview.wav",
                                            filetypes=[("WAV audio", "*.wav")])
        if not path:
            return
        try:
            audio = float(self.vars["audio_hz"].get())
        except ValueError:
            audio = 1500
        ft8.synth_wav(self.tones, path, audio_hz=audio, quantize_like_radio=True)
        self.log(f"Saved {os.path.basename(path)} — open it in WSJT-X (File → Open) to test-decode.", "ok")

    # ---------------------------------------------------------- flashing
    def flash(self):
        board = self.vars["board"].get()
        chip, fname = BOARDS[board]
        path = os.path.join(FW_DIR, fname)
        port = self.port_cb.get()
        if not os.path.exists(path):
            messagebox.showerror(APP_NAME, f"Firmware file missing:\n{path}")
            return
        if not port:
            messagebox.showinfo(APP_NAME, "Plug in the board and pick its port first.")
            return
        if not messagebox.askyesno(APP_NAME, f"Flash LoFT8 firmware for:\n{board}\n\non {port}?\n\n"
                                   "This replaces Meshtastic on that board. You can put Meshtastic "
                                   "back any time at flasher.meshtastic.org."):
            return
        self.disconnect()
        self.log(f"Flashing {fname} to {port}…")
        import importlib.util, shutil
        if importlib.util.find_spec("esptool"):
            tool = [sys.executable, "-m", "esptool"]
        else:
            exe = shutil.which("esptool") or shutil.which("esptool.py")
            tool = [exe] if exe else ["esptool-missing"]
        verb = "write_flash"
        try:
            import importlib.metadata
            if int(importlib.metadata.version("esptool").split(".")[0]) >= 5:
                verb = "write-flash"
        except Exception:
            pass
        args = tool + ["--chip", chip, "--port", port, "--baud", "460800", verb, "0x0", path]

        def run():
            try:
                p = subprocess.Popen(args, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                                     creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))
                last = ""
                for line in p.stdout:
                    line = line.strip()
                    if line and ("%" not in line or line.endswith("100 %)") or "100%" in line):
                        self.rx.put(("flashlog", line))
                    last = line
                rc = p.wait()
                self.rx.put(("flashdone", rc))
            except (FileNotFoundError, OSError):
                self.rx.put(("flashdone", -1))
        threading.Thread(target=run, daemon=True).start()


    # ---------------------------------------------------------- receive tab
    def build_rx_tab(self, tab):
        v = self.vars
        v["rx_source"] = tk.StringVar(value=self.cfg.get("rx_source", ""))
        v["rtl_ppm"] = tk.StringVar(value=self.cfg.get("rtl_ppm", "0"))
        v["rtl_gain"] = tk.StringVar(value=self.cfg.get("rtl_gain", "auto"))
        tab.columnconfigure(0, weight=1)
        tab.rowconfigure(2, weight=1)

        cp = self.panel(tab, "Listen")
        cp.grid(row=0, column=0, sticky="we", pady=(0, 10))
        ttk.Label(cp, text="Receiver", style="Panel.TLabel").grid(row=1, column=0, sticky="w")
        self.src_cb = ttk.Combobox(cp, textvariable=v["rx_source"], state="readonly", width=46)
        self.src_cb.grid(row=1, column=1, columnspan=3, sticky="we", padx=4)
        self.src_cb.bind("<<ComboboxSelected>>", lambda e: self.on_source_pick())
        ttk.Button(cp, text="↻", width=3, command=self.refresh_sources).grid(row=1, column=4)
        self.rx_btn = ttk.Button(cp, text="Start listening", style="Accent.TButton", command=self.toggle_rx)
        self.rx_btn.grid(row=1, column=5, padx=(8, 0))
        ttk.Button(cp, text="Decode WAV…", command=self.decode_wav_file).grid(row=1, column=6, padx=(6, 0))
        self.rtl_row = ttk.Frame(cp, style="Panel.TFrame")
        self.rtl_row.grid(row=2, column=1, columnspan=6, sticky="w", pady=(6, 0))
        ttk.Label(self.rtl_row, text="RTL-SDR ppm", style="Panel.TLabel").pack(side="left", padx=(4, 4))
        ttk.Entry(self.rtl_row, textvariable=v["rtl_ppm"], width=5).pack(side="left")
        ttk.Label(self.rtl_row, text="gain (dB or auto)", style="Panel.TLabel").pack(side="left", padx=(12, 4))
        ttk.Entry(self.rtl_row, textvariable=v["rtl_gain"], width=6).pack(side="left")
        self.rx_lbl = tk.Label(cp, text="", bg=PANEL, fg=MUTED, anchor="w", justify="left", wraplength=900)
        self.rx_lbl.grid(row=3, column=0, columnspan=7, sticky="we", pady=(8, 0))
        cp.columnconfigure(3, weight=1)

        # waterfall / signal meter
        wp = ttk.Frame(tab, style="Panel.TFrame", padding=(12, 8))
        wp.grid(row=1, column=0, sticky="we", pady=(0, 10))
        self.wf_hdr = ttk.Label(wp, text="Waterfall  0 – 3000 Hz", style="Head.TLabel")
        self.wf_hdr.pack(anchor="w")
        self.wf_canvas = tk.Canvas(wp, height=120, bg="#05080b", highlightthickness=0)
        self.wf_canvas.pack(fill="x", pady=(4, 0))
        self.wf_canvas.bind("<Configure>", lambda e: self.redraw_waterfall(force=True))
        self.wf_img = None
        self.wf_last = 0

        # decodes
        dp = self.panel(tab, "Decodes  (double-click to answer)")
        dp.grid(row=2, column=0, sticky="nsew")
        cols = ("utc", "db", "dt", "freq", "msg")
        self.tree = ttk.Treeview(dp, columns=cols, show="headings", height=8)
        for c, w, t, anchor in (("utc", 80, "UTC", "w"), ("db", 50, "dB", "e"), ("dt", 50, "DT", "e"),
                                ("freq", 60, "Freq", "e"), ("msg", 520, "Message", "w")):
            self.tree.heading(c, text=t, anchor=anchor)
            self.tree.column(c, width=w, anchor=anchor, stretch=(c == "msg"))
        self.tree.grid(row=1, column=0, sticky="nsew")
        sb = ttk.Scrollbar(dp, orient="vertical", command=self.tree.yview)
        sb.grid(row=1, column=1, sticky="ns")
        self.tree.configure(yscrollcommand=sb.set)
        dp.rowconfigure(1, weight=1)
        dp.columnconfigure(0, weight=1)
        self.tree.tag_configure("me", background="#5a1f22", foreground="#ffffff")
        self.tree.tag_configure("cq", foreground=GOOD)
        self.tree.tag_configure("sep", foreground="#4b5661")
        self.tree.tag_configure("lora", foreground=ACCENT)
        self.tree.bind("<Double-1>", self.on_decode_click)
        bar = ttk.Frame(dp, style="Panel.TFrame")
        bar.grid(row=2, column=0, columnspan=2, sticky="we", pady=(6, 0))
        ttk.Button(bar, text="Clear", command=lambda: self.tree.delete(*self.tree.get_children())).pack(side="left")
        self.rx_count_lbl = ttk.Label(bar, text="", style="Muted.TLabel")
        self.rx_count_lbl.pack(side="right")
        self.refresh_sources()

    SRC_RTL = "RTL-SDR dongle (tunes the dial frequency itself)"
    SRC_LORA = "LoRa board — signal detector only (can't decode)"

    def refresh_sources(self):
        items = [self.SRC_RTL, self.SRC_LORA]
        self.sound_devs = {}
        for idx, name in rx_sources.SoundCardSource.list_devices():
            label = f"Sound card: {name}"
            self.sound_devs[label] = idx
            items.append(label)
        if not self.sound_devs:
            items.append("Sound card (install the 'sounddevice' package to use)")
        self.src_cb["values"] = items
        if self.vars["rx_source"].get() not in items:
            self.vars["rx_source"].set(items[0])
        self.on_source_pick()

    def on_source_pick(self):
        src = self.vars["rx_source"].get()
        if src == self.SRC_RTL:
            self.rtl_row.grid()
            ok, err = rx_sources.rtl_available()
            self.rx_status("Plug in an RTL-SDR with a 70 cm antenna. LoFT8 tunes it to the Dial frequency and "
                           "demodulates USB itself." + ("" if ok else "  ⚠ RTL-SDR driver library not found — "
                                                        "see README (Windows: Zadig + pip install pyrtlsdr[lib])."),
                           MUTED if ok else ACCENT)
        else:
            self.rtl_row.grid_remove()
            if src == self.SRC_LORA:
                self.rx_status("Uses the connected LoFT8 board's own receiver as a signal meter. The SX1262 "
                               "can't separate FT8's 6.25 Hz tones, so it can only tell you a signal is there "
                               "(and how strong) — not what it says.", ACCENT)
            elif src.startswith("Sound card:"):
                self.rx_status("Feed it USB-mode audio from a radio tuned to the Dial frequency, or from an "
                               "SDR program through a virtual audio cable.", MUTED)
            else:
                self.rx_status("Sound card input needs: pip install sounddevice  (Pi: sudo apt install "
                               "python3-sounddevice)", ACCENT)

    def rx_status(self, text, color=MUTED):
        self.rx_lbl.config(text=text, fg=color)

    def toggle_rx(self):
        if self.rx_mode:
            self.stop_rx()
        else:
            self.start_rx()

    def start_rx(self):
        self.save_cfg()
        src = self.vars["rx_source"].get()
        try:
            dial = float(self.vars["dial_mhz"].get()) * 1e6
        except ValueError:
            messagebox.showerror(APP_NAME, "Set a valid Dial frequency first.")
            return
        self.spec_rows = []
        rec = rx_sources.SlotRecorder(self.now, self.on_slot_audio, self.on_spectrum)
        try:
            if src == self.SRC_RTL:
                ok, err = rx_sources.rtl_available()
                if not ok:
                    raise RuntimeError("RTL-SDR library not available: " + err)
                gain = self.vars["rtl_gain"].get().strip().lower()
                gain = "auto" if gain in ("", "auto") else float(gain)
                self.rx_src = rx_sources.RtlSdrSource(rec, dial, ppm=int(self.vars["rtl_ppm"].get() or 0), gain=gain)
                self.rx_src.start()
                self.rx_mode = "rtl"
                self.rx_status(f"Listening on {dial/1e6:.3f} MHz USB with the RTL-SDR. Decodes appear at the end "
                               "of each 15 s slot.", GOOD)
            elif src == self.SRC_LORA:
                if not self.ser:
                    messagebox.showinfo(APP_NAME, "Connect to the LoFT8 board first (Radio panel).")
                    return
                self.rx_mode = "lora"
                self.lora = {"samples": [], "slot": None, "floor": None}
                self.relisten()
                self.wf_hdr.config(text="LoRa board signal meter — this 15 s slot (dBm)")
                self.rx_status("Listening with the LoRa board. FT8-length bursts are flagged in the list "
                               "(strength only — the chip can't decode them).", ACCENT)
            elif src in self.sound_devs:
                self.rx_src = rx_sources.SoundCardSource(rec, self.sound_devs[src])
                self.rx_src.start()
                self.rx_mode = "sound"
                self.rx_status(f"Listening on {src[12:]} ({self.rx_src.rate} Hz). Decodes appear at the end of "
                               "each 15 s slot.", GOOD)
            else:
                self.on_source_pick()
                return
        except Exception as e:
            self.rx_src = None
            self.rx_mode = None
            messagebox.showerror(APP_NAME, f"Couldn't start the receiver:\n{e}")
            return
        if self.rx_mode != "lora":
            self.wf_hdr.config(text="Waterfall  0 – 3000 Hz   (orange line = your TX offset)")
        self.rx_btn.config(text="Stop listening")
        self.src_cb.state(["disabled"])

    def stop_rx(self):
        if self.rx_src:
            try:
                self.rx_src.stop()
            except Exception:
                pass
        if self.rx_mode == "lora" and self.ser and not self.transmitting:
            self.cmd("STOP")
        self.rx_src = None
        self.rx_mode = None
        self.rx_btn.config(text="Start listening")
        self.src_cb.state(["!disabled"])

    def relisten(self):
        if self.rx_mode == "lora" and self.ser and not self.transmitting:
            try:
                dial, audio, cal, f0 = self.freq_plan()
            except ValueError:
                return
            self.cmd(f"LISTEN {dial + 1500 + cal:.0f}")

    # audio thread -> decode thread -> UI
    def on_slot_audio(self, slot_start, audio):
        if self.decode_jobs.qsize() < 2:
            self.decode_jobs.put((slot_start, audio))

    def decode_worker(self):
        while True:
            slot_start, audio = self.decode_jobs.get()
            try:
                if np_rms(audio) < 1e-7:
                    res = []
                else:
                    res = ft8rx.decode(audio, self.hashes)
                self.rx.put(("decodes", (slot_start, res, None)))
            except Exception as e:
                self.rx.put(("decodes", (slot_start, [], f"decoder error: {e}")))

    def on_spectrum(self, row):
        self.spec_rows.append(row)
        if len(self.spec_rows) > 400:
            del self.spec_rows[:100]

    def show_decodes(self, slot_start, res, err, note=None):
        stamp = time.strftime("%H%M%S", time.gmtime(slot_start)) if slot_start else "file"
        my = self.vars["callsign"].get().strip().upper()
        if err:
            self.rx_status(err, BAD)
        summary = note or f"{len(res)} decode{'s' if len(res) != 1 else ''}"
        self.tree.insert("", "end", values=("—" * 3, "", "", "", f"{stamp[:2]}:{stamp[2:4]}:{stamp[4:]}  {summary}"),
                         tags=("sep",))
        for r in res:
            words = r["text"].split()
            tag = ()
            if my and my in [w.strip("<>") for w in words]:
                tag = ("me",)
            elif words and words[0] == "CQ":
                tag = ("cq",)
            self.tree.insert("", "end", values=(stamp, "%+d" % r["snr"], "%.1f" % r["dt"], r["freq"], r["text"]),
                             tags=tag)
            if tag == ("me",):
                self.log(f"Heard you: {r['text']}  ({r['snr']:+d} dB)", "ok")
        kids = self.tree.get_children()
        if len(kids) > 1500:
            self.tree.delete(*kids[:300])
        self.tree.yview_moveto(1.0)
        if self.rx_mode in ("rtl", "sound"):
            self.rx_count_lbl.config(text=f"last slot {stamp[:2]}:{stamp[2:4]}:{stamp[4:]} — {len(res)} decoded")

    def decode_wav_file(self):
        paths = filedialog.askopenfilenames(title="FT8 recordings (15 s WAV)", filetypes=[("WAV audio", "*.wav")])
        if not paths:
            return

        def run():
            for p in paths:
                try:
                    res = ft8rx.decode(ft8rx.read_wav(p), self.hashes)
                    self.rx.put(("decodes", (0, res, None)))
                except Exception as e:
                    self.rx.put(("decodes", (0, [], f"{os.path.basename(p)}: {e}")))
        self.rx_status(f"Decoding {len(paths)} file(s)…", MUTED)
        threading.Thread(target=run, daemon=True).start()

    # ------------------------------------------- answering from a decode
    def on_decode_click(self, event):
        item = self.tree.identify_row(event.y)
        if not item or "sep" in self.tree.item(item, "tags"):
            return
        utc, db, dt, freq, text = self.tree.item(item, "values")
        reply = self.reply_for(text, int(db) if db not in ("", "?") else -10)
        if not reply:
            return
        their, msg = reply
        self.vars["their_call"].set(their)
        try:
            self.vars["report"].set("%+03d" % max(-30, min(30, int(db))))
        except ValueError:
            pass
        self.msg_var.set(msg)
        # answer in the other half of the minute
        try:
            sec = int(str(utc)[4:6])
            self.vars["slot"].set("Odd (:15 / :45)" if sec % 30 == 0 else "Even (:00 / :30)")
        except ValueError:
            pass
        self.nb.select(self.txtab)
        self.log(f"Reply ready for {their}: “{msg}” — press Send once.", "ok")

    def reply_for(self, text, snr):
        my = self.vars["callsign"].get().strip().upper()
        grid = self.vars["grid"].get().strip().upper()[:4]
        rpt = "%+03d" % max(-30, min(30, snr))
        w = text.split()
        if not my:
            messagebox.showinfo(APP_NAME, "Enter your callsign on the Transmit tab first.")
            return None
        if len(w) < 2:
            return None
        if w[0] == "CQ":
            their = w[-2] if len(w) >= 3 and ft8._GRID_RE.match(w[-1]) else w[-1]
            return their, f"{their} {my} {grid}".strip()
        if w[0].strip("<>") == my:
            their = w[1].strip("<>")
            extra = w[2] if len(w) > 2 else ""
            if extra in ("RR73", "RRR", "73"):
                return their, f"{their} {my} 73"
            if extra.startswith("R") and len(extra) > 1 and extra[1] in "+-":
                return their, f"{their} {my} RR73"
            if extra[:1] in "+-":
                return their, f"{their} {my} R{rpt}"
            return their, f"{their} {my} {rpt}"
        their = w[1].strip("<>")
        if their in ("CQ", "DE", "QRZ") or their == my:
            their = w[0].strip("<>")
        return their, f"{their} {my} {grid}".strip()

    # ------------------------------------------- LoRa signal meter
    def on_lora_rssi(self, line):
        if self.rx_mode != "lora":
            return
        try:
            dbm = int(line[2:]) / 10.0
        except ValueError:
            return
        now = self.now()
        slot = int(now // 15) * 15
        L = self.lora
        if L["slot"] is None:
            L["slot"] = slot
        if slot != L["slot"]:
            self.finish_lora_slot()
            L["slot"] = slot
            L["samples"] = []
        L["samples"].append((now - slot, dbm))
        self.lora_now = dbm

    def finish_lora_slot(self):
        L = self.lora
        pts = L["samples"]
        if len(pts) < 200:          # not a full slot of readings
            return
        vals = np.array([p[1] for p in pts])
        tsec = np.array([p[0] for p in pts])
        floor_now = float(np.percentile(vals, 15))
        L["floor"] = floor_now if L.get("floor") is None else min(L["floor"] * 0.8 + floor_now * 0.2, floor_now)
        floor = L["floor"]
        win = (tsec > 1.2) & (tsec < 12.8)
        if win.sum() < 100:
            return
        above = vals[win] > floor + 3.0
        frac = float(above.mean())
        quiet = (tsec > 14.0)
        tail_quiet = quiet.sum() == 0 or float((vals[quiet] > floor + 3.0).mean()) < 0.5
        if frac > 0.8 and tail_quiet:
            strength = float(np.median(vals[win])) - floor
            note = f"LoRa detector: FT8-length signal, {np.median(vals[win]):.0f} dBm ({strength:+.0f} dB over noise)"
            self.show_decodes(L["slot"], [], None, note)
            # mark the row we just added as a detector row
            kids = self.tree.get_children()
            if kids:
                self.tree.item(kids[-1], tags=("lora",))
        self.rx_count_lbl.config(text=f"noise floor {floor:.0f} dBm")

    # ------------------------------------------- waterfall drawing
    def redraw_waterfall(self, force=False):
        c = self.wf_canvas
        w, h = max(c.winfo_width(), 50), int(c["height"])
        if self.rx_mode == "lora":
            self.draw_lora_meter(w, h)
            return
        rows = self.spec_rows[-h:]
        if not rows:
            if force:
                c.delete("all")
                c.create_text(w // 2, h // 2, text="waterfall appears while listening", fill="#3a4651")
            return
        arr = np.array(rows)
        # map 0..3000 Hz onto the canvas width
        xs = np.linspace(0, arr.shape[1] - 1, w).astype(int)
        arr = arr[:, xs][::-1]
        floor = np.median(arr)
        norm = np.clip((arr - floor + 3) / 25.0, 0, 1)
        img = np.zeros((h, w, 3), dtype=np.uint8)
        n = norm.shape[0]
        img[:n] = waterfall_colors(norm)
        hdr = f"P6 {w} {h} 255 ".encode()
        self.wf_img = tk.PhotoImage(data=hdr + img.tobytes(), format="PPM")
        c.delete("all")
        c.create_image(0, 0, anchor="nw", image=self.wf_img)
        try:
            x = float(self.vars["audio_hz"].get()) / 3000 * w
            c.create_line(x, 0, x, h, fill=ACCENT)
            c.create_line(x + 50 / 3000 * w, 0, x + 50 / 3000 * w, h, fill=ACCENT, dash=(2, 3))
        except ValueError:
            pass
        for f in range(500, 3000, 500):
            x = f / 3000 * w
            c.create_text(x, h - 8, text=str(f), fill="#7d8a96", font=("", 8))

    def draw_lora_meter(self, w, h):
        c = self.wf_canvas
        c.delete("all")
        pts = self.lora.get("samples", [])
        lo, hi = -135.0, -60.0
        if pts:
            vals = [p[1] for p in pts]
            lo = min(lo, min(vals) - 2)
            hi = max(min(vals) + 40, max(vals) + 5)
        def y(v):
            return h - 4 - (v - lo) / (hi - lo) * (h - 8)
        for sec in range(0, 16, 3):
            x = sec / 15 * w
            c.create_line(x, 0, x, h, fill="#17212a")
            c.create_text(x + 10, h - 8, text=f"{sec}s", fill="#4b5661", font=("", 8))
        x0, x1 = 0.5 / 15 * w, 13.14 / 15 * w
        c.create_rectangle(x0, 0, x1, h, outline="#2c3a46", dash=(2, 3))
        c.create_text(x0 + 4, 10, text="where an FT8 signal would be", anchor="w", fill="#4b5661", font=("", 8))
        if self.lora.get("floor") is not None:
            c.create_line(0, y(self.lora["floor"]), w, y(self.lora["floor"]), fill="#3a4651", dash=(4, 3))
        if len(pts) > 1:
            coords = []
            for t, v in pts[:: max(1, len(pts) // 600)]:
                coords += [t / 15 * w, y(v)]
            c.create_line(*coords, fill=ACCENT, width=1.5)
            c.create_text(w - 6, 10, text=f"{pts[-1][1]:.0f} dBm", anchor="e", fill=FG, font=("", 10, "bold"))

    # ---------------------------------------------------------- loops
    def poll(self):
        try:
            while True:
                kind, val = self.rx.get_nowait()
                if kind == "line":
                    self.on_line(val)
                elif kind == "clock":
                    good = abs(val) < 0.5
                    self.clock_lbl.config(text=f"clock {'OK' if good else 'corrected'} ({val:+.2f} s vs NTP)",
                                          fg=GOOD if good else ACCENT)
                    self.log(f"NTP clock offset {val:+.3f} s — timing corrected automatically.", "ok")
                elif kind == "clock_err":
                    self.clock_lbl.config(text="clock: NTP unreachable — using PC clock", fg=ACCENT)
                    self.log("Couldn't reach an NTP server. FT8 needs your clock within ~1 s — "
                             "sync it in your OS settings (Windows: Settings → Time → Sync now).", "err")
                elif kind == "serial_err":
                    self.log(f"Serial error: {val}", "err")
                    self.disconnect()
                elif kind == "decodes":
                    self.show_decodes(*val)
                elif kind == "rxerr":
                    self.rx_status(val, BAD)
                    self.stop_rx()
                elif kind == "flashlog":
                    self.log(val, "dim")
                elif kind == "flashdone":
                    if val == 0:
                        self.log("Flash complete. Press Connect.", "ok")
                    elif val == -1:
                        self.log("esptool not found — run: pip install esptool", "err")
                    else:
                        self.log("Flash failed. Hold the board's BOOT button while plugging it in, then retry.", "err")
        except queue.Empty:
            pass
        self.root.after(50, self.poll)

    def on_line(self, line):
        if not line:
            return
        if line.startswith("LOFT8"):
            ok = "NO-RADIO" not in line
            self.radio_lbl.config(text=("✓ " if ok else "⚠ ") + line.replace("LOFT8", "LoFT8 firmware"),
                                  foreground=GOOD if ok else BAD)
            if not ok:
                self.log("Firmware runs but can't find the SX1262 — wrong board image?", "err")
        elif line.startswith("R "):
            self.on_lora_rssi(line)
        elif line.startswith("LISTENON"):
            pass
        elif line.startswith("TXDONE"):
            self.transmitting = False
            self.relisten()
            self.progress["value"] = 79
            self.draw_tones()
            self.log("TX complete", "ok")
            if self.tx_job and self.tx_job["next"] is None:
                self.tx_job = None
                self.send_btn.state(["!disabled"]); self.beacon_btn.state(["!disabled"])
        elif line.startswith("ERR"):
            if "unknown command" in line and getattr(self, "beep_pending", False):
                self.log("This board has older LoFT8 firmware — click Flash to update it (beep test needs 1.2).", "err")
            else:
                self.log(line, "err")
            self.transmitting = False
        elif line.startswith(("OK", "TXON", "TUNEON", "TUNEDONE", "STOPPED")):
            if line.startswith("TUNE"):
                self.log(line.title(), "dim")
            if line.startswith("TUNEDONE"):
                self.relisten()
        elif line.startswith("MCWON"):
            self.state_lbl.config(text="● BEEP TEST", fg=TXRED)
        elif line.startswith("MCWDONE"):
            self.log("Beep test finished.", "ok")
            self.state_lbl.config(text="Idle", fg=MUTED)
            self.relisten()
        else:
            self.log(line, "dim")

    def tick(self):
        now = self.now()
        self.utc_lbl.config(text=time.strftime("%H:%M:%S UTC", time.gmtime(now)))
        if self.rx_mode and time.time() - self.wf_last > 0.25:
            self.wf_last = time.time()
            self.redraw_waterfall()
        job = self.tx_job
        if job and job["next"] is not None and not self.transmitting:
            fire_at = job["next"] + ft8.TX_START_OFFSET - 0.015   # tiny lead for USB latency
            wait = fire_at - now
            if wait <= 0:
                if wait > -2.0:
                    self.fire()
                else:   # missed (e.g. PC was busy) — skip to the next cycle
                    job["next"] += job["every"]
            else:
                self.state_lbl.config(text=f"Waiting {wait:4.1f} s for slot", fg=ACCENT)
                # sleep-precise scheduling for the final stretch
                self.root.after(max(1, min(200, int(wait * 1000) - 5)), self.tick)
                return
        if self.transmitting:
            sym = min(79, int((now - self.tx_started) / ft8.SYMBOL_PERIOD) + 1)
            self.progress["value"] = sym
            self.state_lbl.config(text="● TRANSMITTING", fg=TXRED)
            if sym % 4 == 0:
                self.draw_tones(highlight=sym)
            if now - self.tx_started > 15:     # firmware never answered
                self.transmitting = False
                self.log("No TXDONE from radio — check the connection.", "err")
        elif not (job and job["next"] is not None):
            if self.state_lbl.cget("text").startswith(("Waiting", "●")):
                self.state_lbl.config(text="Idle", fg=MUTED)
        self.root.after(100, self.tick)

    def on_close(self):
        self.stop_rx()
        self.save_cfg()
        self.disconnect()
        self.root.destroy()


def np_rms(x):
    import numpy as _np
    return float(_np.sqrt(_np.mean(_np.square(x)))) if len(x) else 0.0


def waterfall_colors(norm):
    """0..1 array -> RGB uint8 (black, deep blue, cyan, yellow, white)."""
    import numpy as _np
    stops = _np.array([[5, 8, 11], [20, 40, 120], [0, 170, 200], [255, 210, 40], [255, 255, 255]], float)
    pos = norm * (len(stops) - 1)
    i = _np.clip(pos.astype(int), 0, len(stops) - 2)
    f = (pos - i)[..., None]
    return (stops[i] * (1 - f) + stops[i + 1] * f).astype(_np.uint8)


def main():
    root = tk.Tk()
    LoFT8App(root)
    root.mainloop()


if __name__ == "__main__":
    main()
