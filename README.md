# LoFT8 — FT8 from your Meshtastic LoRa board

LoFT8 turns an SX1262-based Meshtastic board (Heltec V3, T-Beam SX1262, T3-S3) into a real **FT8 transmitter**, and decodes FT8 too. The desktop app builds the FT8 message, keeps the 15-second slot timing, and the board's firmware plays the 79 tones straight out of the LoRa chip's synthesizer. The Receive tab decodes FT8 from an RTL-SDR dongle, a radio or SDR program through the sound card, or WAV recordings.

Anyone running WSJT-X (or any FT8 decoder) on a receiver tuned to the same frequency in USB mode will decode you like any other FT8 station.

## What it can and can't do

- **The LoRa board can't decode FT8.** Its narrowest receive filter is about 4.8 kHz wide, so it can't tell FT8's 6.25 Hz-spaced tones apart. LoFT8 still uses it as a **signal detector**: it shows the channel strength live and flags FT8-length bursts in each slot. To actually *read* messages, use an RTL-SDR (about $35) or any receiver.
- **SX1262 boards only.** Their synthesizer moves in 0.95 Hz steps, fine enough for FT8. Older SX1276/SX1278 boards (Heltec V2, T-Beam SX1276, etc.) move in 61 Hz steps and can't do it.
- **Bands:** the chip covers 150–960 MHz. The ham FT8 channel in range is **70 cm, 432.174 MHz** (preset). 33 cm (902–928 MHz) works too but has no standard FT8 channel — use Custom and coordinate with whoever is listening.
- **Antenna note:** a 915 MHz-version board will still transmit on 432 MHz, but its filter/antenna aren't tuned for it, so expect much less output. A 433 MHz version board and a 70 cm antenna work best.
- **License:** you need an amateur license for the band. Your callsign is in your CQ and QSO messages, which identifies you.
- Flashing LoFT8 replaces Meshtastic on that board. Put Meshtastic back anytime at flasher.meshtastic.org.

## Setup

**Windows:** install Python 3 from python.org (tick "Add to PATH"), then double-click `app/Run LoFT8 (Windows).bat`. It installs pyserial, esptool, numpy, sounddevice and pyrtlsdr the first time.

**Raspberry Pi (Pi 400 / 500 / 5):** `cd app && ./install-pi.sh`, then open LoFT8 from the menu or Desktop. Log out and back in once so the USB port permission takes effect.

## First run

1. Plug the board in by USB, pick its port, choose your board under **Firmware**, click **Flash**. (If it fails, hold the board's BOOT button while plugging it in and try again.)
2. Click **Connect**. You should see `✓ LoFT8 firmware 1.2 … SX1262`. (Older version? Click Flash again to get the newest features.)
3. Enter your call and grid, tick the license box.
4. **Calibrate once:** click **Tune 10 s** and watch the carrier on a receiver/SDR. Set **Calibration (Hz)** to *(where it should be) − (where it showed up)*. Cheap boards are often off by a few hundred Hz to a couple of kHz at 432 MHz; the TCXO keeps it steady, so one calibration lasts.
5. Press **CQ** (or type a message) and **Send once**, or **Start beacon** to repeat every N cycles.

The app checks your clock against NTP at startup and corrects timing automatically. FT8 needs your clock within about a second.

## Quick "is it working?" checks

- **Beep test** (Frequency panel): the board transmits a 1-second tone and then `VVV DE <your call>` in Morse, as FM. Set any 70 cm handheld to FM on the frequency it asks for (446.000 MHz by default, the US 70 cm FM simplex frequency) and you'll hear it. The board's frequency error doesn't matter here, since FM is wide.
- **▶ Play on speakers** (Transmit tab): plays what your FT8 message sounds like, so you know what to listen for.

## Receiving (Receive tab)

Pick a **Receiver** and press **Start listening**. Decodes show up at the end of every 15 s slot, like WSJT-X. Messages with your call are highlighted red, CQs green. **Double-click** a decode to build the right reply (answer a CQ, send a report, R-report, RR73 or 73), set their call and report, and pick the opposite slot; then press **Send once** on the Transmit tab.

- **RTL-SDR dongle** — plug it in with a 70 cm antenna. LoFT8 tunes it to the Dial frequency and does the USB demodulation itself. Use **ppm** to correct the dongle's frequency error. Windows needs the WinUSB driver once: run Zadig, pick "Bulk-In, Interface (Interface 0)", install WinUSB.
- **Sound card** — audio from a radio in USB mode, or from SDR++ / SDR# / SDRangel through a virtual audio cable (VB-Cable on Windows).
- **LoRa board — signal detector** — uses the connected LoFT8 board. A strip chart shows the channel level for the current slot; a full-length burst in the FT8 window is listed as a detection with its strength. It pauses while you transmit and resumes after.
- **Decode WAV…** — decodes 15 s FT8 recordings (WSJT-X saves these).

The decoder is written in Python + numpy. On 60 real off-air test recordings it found 988 messages, versus 932 for ft8_lib's reference decoder (WSJT-X itself finds more, about 1,289).

## Messages

Standard FT8 messages work exactly like WSJT-X: `CQ K8ABC EN82`, `CQ POTA K8ABC EN82`, `W9XYZ K8ABC -07`, `R-07`, `RR73`, `73`, `/P` and `/R` calls, and compound calls like `VE3XYZ/W8`. Anything else goes as 13-character free text.

**Save WAV preview** writes the exact transmission (with the radio's frequency rounding) as audio. Open it in WSJT-X (File → Open) to see it decode before you go on the air.

## Files

- `app/` — the desktop app (`loft8_app.py`), FT8 encoder (`ft8.py`), FT8 decoder (`ft8rx.py`), receivers (`rx_sources.py`), and ready-to-flash firmware images.
- `firmware/` — Arduino source. Rebuild with `firmware/build.sh` (arduino-cli + ESP32 core 3.3.x), or open `LoFT8/LoFT8.ino` in the Arduino IDE and uncomment your board at the top of `board.h`.

## Serial protocol (for tinkering)

115200 baud, one command per line: `HELLO`, `PWR <dBm>`, `TX <tone0_hz> <79 digits>`, `TUNE <hz> <seconds>`, `LISTEN <hz>` (streams `R <dBm×10>` 50×/s), `MCW <hz> <tone_hz> <wpm> <text>` (FM beep + Morse), `STOP`, `STATUS`.

## Credits

FT8 protocol by K1JT and the WSJT-X team. Encoder and decoder follow Karlis Goba's ft8_lib (MIT); the encoder was verified tone-for-tone against it and the decoder against its real-world test recordings. Radio driver: RadioLib (MIT). T-Beam power management: XPowersLib (MIT).
