# ESP32 OLED Lyric Visualizer

Windows laptop plays MP3 + LRC lyrics; the ESP32 draws synchronized lyrics, title,
beat-reactive pixel animations on a 0.96" SSD1306 OLED over USB serial.

```
ESP32-Music-Player/
├── music.py              # PC app (pygame + pyserial)
├── requirements.txt
├── songs/ song.mp3 + song.lrc   (put YOUR mp3 here; name .lrc identically)
└── esp32/oled_player.ino # firmware
```

## 1. Wiring

| OLED (SSD1306 I2C) | ESP32 DevKit V1 |
|---|---|
| VCC | 3V3 |
| GND | GND |
| SDA | GPIO 21 |
| SCL | GPIO 22 |

Connect the ESP32 to the laptop with a **data-capable** micro-USB/USB-C cable.

## 2. Python setup (Windows)

1. Install Python 3.9-3.13 from python.org (tick **Add python.exe to PATH**).
2. In the project folder:
   ```
   python -m venv .venv
   .venv\Scripts\activate
   pip install -r requirements.txt
   ```

## 3. ESP32 setup (Arduino IDE 2.x)

1. File > Preferences > *Additional boards manager URLs*:
   `https://espressif.github.io/arduino-esp32/package_esp32_index.json`
2. Tools > Board > Boards Manager > install **esp32 by Espressif Systems**.
3. Tools > Manage Libraries > install **Adafruit SSD1306** and **Adafruit GFX Library**
   (accept "install all dependencies", e.g. Adafruit BusIO). `Wire` is built in.
4. Tools > Board > **ESP32 Dev Module** (or *DOIT ESP32 DEVKIT V1*), select the COM port.
5. Open `esp32/oled_player.ino`, click **Upload**. If it hangs on "Connecting...",
   hold the **BOOT** button until upload starts.
6. The OLED shows "Waiting for PC...". **Close the Arduino Serial Monitor** (it holds the port).

## 4. Run

1. Copy your song into `songs/` as `song.mp3` and a matching `song.lrc`
   (any names work as long as the base names match; all `.mp3` files in the folder are listed).
   The bundled `song.lrc` is a demo; its timings only fit a song you time it for.
2. `python music.py`  (port is auto-detected; or `python music.py --port COM5`)
3. Keys: **SPACE** play/pause/resume, **S** stop, **N** next, **B** previous,
   **, / <-** seek -5 s, **. / ->** seek +5 s, **number + ENTER** pick a song, **L** list, **Q** quit.

Useful options: `--list-ports`, `--no-serial` (music only), `--no-autoplay`,
`--volume 0.6`, `--songs D:\Music`, `--no-ascii-fold`.
Tune sync in `music.py`: `LYRIC_LEAD_MS` (higher = lyrics appear earlier).

## 5. What the OLED shows

Only big lyrics over an animated background, no other text.

* Lyrics use large 12x16 text. Words drop in one by one (flash + drop-in), timed across the line's duration.
* The PC analyzes the MP3 (bass energy + onset detection, needs `numpy`) and sends energy + beat flags.
  Backgrounds pulse, shake and flash (screen invert) on the beat.
* 7 background styles switch with every line: rings, equalizer wall, starfield, waves, rain, grid/checker, rays.
  Keywords pick a matching style: rain/tear, star/night/sky, love/heart, fire/light/electric, dance/beat/loud, wave/sea/fly, neon/city/pixel.
* Instrumental gaps show a big bouncing note. No PC connection: ambient animation only.
* Until beat analysis finishes (a few seconds), a steady 120 BPM pulse is used.

## 6. Serial protocol v2 (115200 8N1)

`$<payload>*<HH>\n`, `HH` = XOR of payload bytes (hex). Fields split by `|`, UTF-8.

| Dir | Payload | Meaning |
|---|---|---|
| PC>ESP | `H\|2` | hello / keep-alive every 1 s |
| ESP>PC | `A\|2` | ack |
| ESP>PC | `R\|2` | ESP32 booted: PC resends state |
| PC>ESP | `T\|title\|durMs` | new song (resets screen) |
| PC>ESP | `L\|line\|lineDurMs\|elapsedMs` | current lyric, on change only |
| PC>ESP | `P\|posMs\|durMs\|state` | state 0 stop, 1 play, 2 pause |
| PC>ESP | `E\|level\|beat` | energy 0-100 + beat flag, 20 Hz |

Notes: the built-in font is ASCII only (accents are folded, CJK/Cyrillic show `?`).
Beat flashes invert the screen briefly; lower `flashUntil` in `triggerBeat()` or remove it if you prefer no flashing.

## 7. Troubleshooting

| Problem | Fix |
|---|---|
| OLED blank, LED blinks | Wrong wiring/address. Try `OLED_ADDR 0x3D`; check 3V3, SDA=21, SCL=22; some modules need pull-ups. |
| `no serial port found` | Install the CP210x or CH340 USB driver; try another cable (must carry data); `python music.py --list-ports`. |
| `cannot use COMx: PermissionError` | Close Arduino Serial Monitor / any other program using the port. |
| `no answer from ESP32` | Firmware not uploaded, wrong baud, or board stuck in boot: press EN, re-run. |
| Board resets every time Python connects | Normal on some boards (DTR/RTS). The script waits and resyncs automatically. A 10 uF capacitor EN-GND stops it. |
| Upload fails | Hold BOOT during "Connecting...", select the right port/board, close Serial Monitor. |
| Lyrics early/late | Adjust `LYRIC_LEAD_MS`, or add `[offset:+/-ms]` to the LRC. |
| `No lyrics found` | `.lrc` must have the same base name as the `.mp3`; timestamps like `[mm:ss.xx]`. |
| `?` on the OLED | Characters outside the ASCII font (see notes). |
| No sound / audio error | Check the Windows default output device; try `--volume 1.0`. |
| Beats feel off | Beat analysis needs numpy (`pip install numpy`); tune `LYRIC_LEAD_MS`. |
| Choppy animation | Ensure 400 kHz I2C works with your module; lower speed in the `display(...)` constructor. |
