#!/usr/bin/env python3
"""
ESP32 OLED Synchronized Music Player  -  PC side (Windows / Python 3.9+)

Plays MP3 files with pygame, parses matching .lrc lyric files, and streams the
currently active lyric + playback progress to an ESP32 over USB serial.

Serial protocol v1 (115200 8N1), one ASCII line per frame:

    $<payload>*<HH>\\n          HH = XOR of all payload bytes, 2 hex digits

    PC  -> ESP32   H|2                       hello / keep-alive
    ESP32 -> PC    A|2                       hello acknowledged
    ESP32 -> PC    R|2                       ESP32 just booted (PC resends state)
    PC  -> ESP32   T|<title>|<durationMs>    new song loaded (resets the screen)
    PC  -> ESP32   L|<line>|<lineDurMs>|<elapsedMs>   current lyric (sent on change)
    PC  -> ESP32   P|<posMs>|<durMs>|<state> state 0=stop 1=play 2=pause
    PC  -> ESP32   E|<level 0-100>|<beat>    audio energy + beat flag (20 Hz)

Keyboard controls (while the console window is focused):
    SPACE play / pause / resume      S stop          N next      B previous
    , or <-  seek -5 s               . or ->  seek +5 s
    digits + ENTER  jump to that song number         L list songs
    H help                           Q quit
"""
from __future__ import annotations

import argparse
import bisect
import os
import queue
import re
import shutil
import sys
import threading
import time
import unicodedata
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Tuple

os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")

try:
    import pygame
except ImportError:  # pragma: no cover
    sys.exit("Missing dependency 'pygame'.  Run:  pip install -r requirements.txt")

try:
    import serial
    from serial.tools import list_ports
except ImportError:  # pragma: no cover
    sys.exit("Missing dependency 'pyserial'.  Run:  pip install -r requirements.txt")

try:
    from mutagen import File as MutagenFile
except ImportError:  # optional
    MutagenFile = None

# ---------------------------------------------------------------------------
# Configuration (can be tweaked here or overridden on the command line)
# ---------------------------------------------------------------------------
BAUD_RATE = 115200
PROTOCOL_VERSION = 2
SONGS_DIR = Path(__file__).resolve().parent / "songs"

LOOP_INTERVAL = 0.03          # main loop period (s)
PROGRESS_INTERVAL = 0.2       # progress frame period (s)  -> 5 Hz
MIN_LYRIC_INTERVAL = 0.04     # never send two lyric frames closer than this (s)
KEEPALIVE_INTERVAL = 1.0      # hello frame period (s)
RECONNECT_INTERVAL = 3.0      # retry period when the ESP32 is missing (s)
HANDSHAKE_TIMEOUT = 5.0       # wait for ESP32 to answer after opening port (s)
LYRIC_LEAD_MS = 120           # show lyrics this much early (serial + render latency)
SEEK_STEP_S = 5.0
MAX_FIELD_BYTES = 100         # lyric line (must be < LINE_MAX in the firmware)
ENERGY_INTERVAL = 0.05        # E frame period (s) -> 20 Hz
MAX_TITLE_BYTES = 48
KNOWN_USB_VIDS = {0x10C4, 0x1A86, 0x0403, 0x303A}   # CP210x, CH340, FTDI, Espressif


# ---------------------------------------------------------------------------
# Protocol helpers
# ---------------------------------------------------------------------------
def checksum(data: bytes) -> int:
    value = 0
    for byte in data:
        value ^= byte
    return value


def build_frame(*fields) -> bytes:
    """Build '$f0|f1|...*HH\\n' (UTF-8)."""
    payload = "|".join(str(f) for f in fields).encode("utf-8")
    return b"$" + payload + b"*" + f"{checksum(payload):02X}".encode("ascii") + b"\n"


def parse_frame(line: bytes) -> Optional[List[str]]:
    """Return the payload fields of a valid frame, otherwise None.

    The *last* '$' is used so boot-time noise in front of a frame is ignored
    (frames sent by the ESP32 never contain '$' inside the payload)."""
    line = line.strip()
    start = line.rfind(b"$")
    star = line.rfind(b"*")
    if start < 0 or star < start or len(line) - star - 1 != 2:
        return None
    payload = line[start + 1:star]
    try:
        received = int(line[star + 1:], 16)
    except ValueError:
        return None
    if received != checksum(payload):
        return None
    return payload.decode("utf-8", "replace").split("|")


_PUNCT_MAP = str.maketrans({
    "\u2018": "'", "\u2019": "'", "\u201a": "'", "\u201b": "'",
    "\u201c": '"', "\u201d": '"', "\u201e": '"', "\u201f": '"',
    "\u2013": "-", "\u2014": "-", "\u2015": "-", "\u2212": "-",
    "\u2026": "...", "\u2022": "*", "\u00a0": " ",
    "\u266a": "", "\u266b": "", "\u266c": "",
    "\u00df": "ss", "\u00e6": "ae", "\u00c6": "AE", "\u00f8": "o", "\u00d8": "O",
    "\u0111": "d", "\u0110": "D", "\u0142": "l", "\u0141": "L",
    "\u0153": "oe", "\u0152": "OE", "\u00fe": "th", "\u00de": "Th",
    "\u00a1": "!", "\u00bf": "?",
})


def to_display_text(text: str, ascii_fold: bool = True) -> str:
    """Make text safe for the protocol and (optionally) for the OLED's ASCII font.

    * removes control characters, replaces the field separator '|'
    * ascii_fold=True: "Café señor" -> "Cafe senor"; anything that still is not
      ASCII becomes '?' (the built-in 5x7 font only has ASCII/CP437 glyphs)
    * ascii_fold=False: text stays UTF-8 (the firmware folds Latin-1 itself)
    """
    text = unicodedata.normalize("NFKC", text)
    text = text.translate(_PUNCT_MAP)
    cleaned = []
    for ch in text:
        if ch in "\t\r\n":
            cleaned.append(" ")
        elif unicodedata.category(ch) in ("Cc", "Cf"):
            continue
        else:
            cleaned.append(ch)
    text = "".join(cleaned).replace("|", "/")
    if ascii_fold:
        decomposed = unicodedata.normalize("NFKD", text)
        text = "".join(c for c in decomposed if not unicodedata.combining(c))
        text = "".join(c if " " <= c <= "~" else "?" for c in text)
    return " ".join(text.split())


def truncate_bytes(text: str, limit: int) -> str:
    """Truncate to at most `limit` UTF-8 bytes without splitting a character."""
    raw = text.encode("utf-8")
    if len(raw) <= limit:
        return text
    return raw[: limit - 2].decode("utf-8", "ignore").rstrip() + ".."


def fmt_time(seconds: float) -> str:
    seconds = max(0, int(seconds))
    return f"{seconds // 60:02d}:{seconds % 60:02d}"


# ---------------------------------------------------------------------------
# LRC parsing
# ---------------------------------------------------------------------------
TIME_TAG = re.compile(r"\[(\d{1,3}):(\d{1,2})(?:[.:](\d{1,3}))?\]")
META_TAG = re.compile(r"^\[([A-Za-z#]+):(.*)\]\s*$")
WORD_TAG = re.compile(r"<\d{1,3}:\d{1,2}(?:[.:]\d{1,3})?>")   # enhanced-LRC word times


@dataclass
class Lyrics:
    times: List[float]          # seconds, sorted
    lines: List[str]
    title: str = ""
    artist: str = ""

    def __len__(self) -> int:
        return len(self.times)

    def window(self, pos_s: float) -> Tuple[int, str, str, str]:
        """(index, previous, current, next) lyric for a playback position."""
        idx = bisect.bisect_right(self.times, pos_s) - 1
        prev = self.lines[idx - 1] if idx - 1 >= 0 else ""
        cur = self.lines[idx] if idx >= 0 else ""
        nxt = self.lines[idx + 1] if idx + 1 < len(self.lines) else ""
        return idx, prev, cur, nxt

    def timing(self, idx: int, pos_s: float) -> Tuple[int, int]:
        """(line duration ms, elapsed ms) of lyric line `idx` at a playback position."""
        if idx < 0:
            return 0, 0
        start = self.times[idx]
        end = self.times[idx + 1] if idx + 1 < len(self.times) else start + 4.0
        return int((end - start) * 1000), max(0, int((pos_s - start) * 1000))


def read_text_file(path: Path) -> str:
    raw = path.read_bytes()
    if raw.startswith((b"\xff\xfe", b"\xfe\xff")):
        return raw.decode("utf-16")
    try:
        return raw.decode("utf-8-sig")
    except UnicodeDecodeError:
        return raw.decode("cp1252", errors="replace")


def parse_lrc(text: str) -> Lyrics:
    """Parse LRC text. Supports several timestamps per line, [offset:], fractions
    of 1-3 digits, metadata tags, enhanced <mm:ss.xx> word tags and empty lines."""
    entries: List[Tuple[float, str]] = []
    offset_ms = 0
    meta = {}
    for raw_line in text.splitlines():
        line = raw_line.strip()
        if not line:
            continue
        pos, stamps = 0, []
        while True:
            match = TIME_TAG.match(line, pos)
            if not match:
                break
            frac = match.group(3)
            stamps.append(int(match.group(1)) * 60 + int(match.group(2))
                          + (float("0." + frac) if frac else 0.0))
            pos = match.end()
        if stamps:
            lyric = WORD_TAG.sub("", line[pos:]).strip()
            entries.extend((t, lyric) for t in stamps)
            continue
        match = META_TAG.match(line)
        if match:
            key, value = match.group(1).lower(), match.group(2).strip()
            meta[key] = value
            if key == "offset":
                try:
                    offset_ms = int(value)
                except ValueError:
                    pass
    # LRC spec: positive offset = lyrics appear sooner
    entries = [(max(0.0, t - offset_ms / 1000.0), s) for t, s in entries]
    entries.sort(key=lambda e: e[0])            # stable: keeps file order for ties
    return Lyrics([e[0] for e in entries], [e[1] for e in entries],
                  meta.get("ti", ""), meta.get("ar", ""))


# ---------------------------------------------------------------------------
# Songs
# ---------------------------------------------------------------------------
@dataclass
class Song:
    path: Path
    title: str
    duration: float
    lyrics: Optional[Lyrics]
    lrc_path: Optional[Path]


def discover_songs(folder: Path) -> List[Path]:
    if not folder.is_dir():
        return []
    return sorted((p for p in folder.iterdir() if p.suffix.lower() == ".mp3"),
                  key=lambda p: p.name.lower())


def find_lrc(mp3: Path) -> Optional[Path]:
    for candidate in mp3.parent.iterdir():
        if candidate.suffix.lower() == ".lrc" and candidate.stem.lower() == mp3.stem.lower():
            return candidate
    return None


def read_audio_info(path: Path) -> Tuple[str, str, float]:
    """(title, artist, length_seconds) via mutagen; empty/0 when unknown."""
    if MutagenFile is None:
        return "", "", 0.0
    try:
        audio = MutagenFile(str(path), easy=True)
        if audio is None:
            return "", "", 0.0
        tags = audio.tags or {}
        title = (tags.get("title") or [""])[0]
        artist = (tags.get("artist") or [""])[0]
        length = float(audio.info.length) if audio.info and audio.info.length else 0.0
        return title, artist, length
    except Exception:
        return "", "", 0.0


def load_song(path: Path) -> Song:
    lrc_path = find_lrc(path)
    lyrics: Optional[Lyrics] = None
    if lrc_path:
        try:
            parsed = parse_lrc(read_text_file(lrc_path))
            lyrics = parsed if len(parsed) else None
        except (OSError, UnicodeError):
            lyrics = None
    tag_title, tag_artist, length = read_audio_info(path)
    if tag_title:
        title = f"{tag_artist} - {tag_title}" if tag_artist else tag_title
    elif lyrics and lyrics.title:
        title = f"{lyrics.artist} - {lyrics.title}" if lyrics.artist else lyrics.title
    else:
        title = path.stem
    if length <= 0:
        try:
            length = float(pygame.mixer.Sound(str(path)).get_length())
        except Exception:
            length = (lyrics.times[-1] + 10.0) if lyrics else 0.0
    return Song(path, title, length, lyrics, lrc_path)


# ---------------------------------------------------------------------------
# Beat / energy analysis (numpy) - runs in a background thread per song
# ---------------------------------------------------------------------------
class BeatMap:
    HOP = 0.025                                   # seconds per analysis frame

    def __init__(self) -> None:
        self.levels: List[int] = []               # 0-100 loudness per frame
        self.beats: List[float] = []              # beat times in seconds
        self.ready = False

    @classmethod
    def analyze(cls, path: Path) -> "BeatMap":
        bm = cls()
        try:
            import numpy as np
            sound = pygame.mixer.Sound(str(path))
            samples = pygame.sndarray.array(sound).astype(np.float32)
            if samples.ndim == 2:
                samples = samples.mean(axis=1)
            samples /= 32768.0
            rate = pygame.mixer.get_init()[0]
            hop = max(1, int(rate * cls.HOP))
            count = len(samples) // hop
            if count < 80:
                return bm
            frames = samples[: count * hop].reshape(count, hop)
            rms = np.sqrt((frames ** 2).mean(axis=1))
            ref = float(np.percentile(rms, 95)) or 1.0
            lv = np.clip(rms / ref, 0.0, 1.0)
            lv = np.convolve(lv, np.ones(3) / 3, mode="same")
            spec = np.abs(np.fft.rfft(frames * np.hanning(hop), axis=1))
            bin_hz = rate / hop
            lo = max(1, int(40 / bin_hz))
            hi = max(lo + 1, int(220 / bin_hz) + 1)
            bass = spec[:, lo:hi].sum(axis=1)
            flux = np.maximum(0.0, np.diff(bass, prepend=bass[0]))
            win = int(1.0 / cls.HOP)
            local = np.convolve(flux, np.ones(win) / win, mode="same")
            thresh = local * 1.6 + float(flux.mean()) * 0.3
            last = -1.0
            beats: List[float] = []
            for i in range(1, count - 1):
                t = i * cls.HOP
                if (flux[i] > thresh[i] and flux[i] >= flux[i - 1] and flux[i] > flux[i + 1]
                        and t - last >= 0.28):
                    beats.append(t)
                    last = t
            bm.levels = (lv * 100).astype(int).tolist()
            bm.beats = beats
            bm.ready = True
        except Exception:
            bm.ready = False
        return bm


# ---------------------------------------------------------------------------
# Audio player (pygame)
# ---------------------------------------------------------------------------
class Player:
    STOPPED, PLAYING, PAUSED = 0, 1, 2

    def __init__(self, paths: List[Path], volume: float):
        self.paths = paths
        self.index = -1
        self.song: Optional[Song] = None
        self.state = Player.STOPPED
        self._offset = 0.0            # seconds: where the current play() call started
        pygame.mixer.init(frequency=44100, size=-16, channels=2, buffer=1024)
        pygame.mixer.music.set_volume(max(0.0, min(1.0, volume)))

    def play(self, index: int) -> Song:
        song = load_song(self.paths[index])
        pygame.mixer.music.load(str(song.path))
        pygame.mixer.music.play()
        self.index, self.song = index, song
        self.state, self._offset = Player.PLAYING, 0.0
        return song

    def pause(self) -> None:
        if self.state == Player.PLAYING:
            pygame.mixer.music.pause()
            self.state = Player.PAUSED

    def resume(self) -> None:
        if self.state == Player.PAUSED:
            pygame.mixer.music.unpause()
            self.state = Player.PLAYING

    def stop(self) -> None:
        pygame.mixer.music.stop()
        self.state, self._offset = Player.STOPPED, 0.0

    def position(self) -> float:
        """Real playback position in seconds (the single source of truth for sync)."""
        if self.song is None or self.state == Player.STOPPED:
            return 0.0
        played_ms = max(0, pygame.mixer.music.get_pos())
        pos = self._offset + played_ms / 1000.0
        if self.song.duration > 0:
            pos = min(pos, self.song.duration)
        return pos

    def seek(self, delta: float) -> None:
        if self.song is None or self.state == Player.STOPPED:
            return
        limit = max(0.0, self.song.duration - 0.5) if self.song.duration > 0 else 1e9
        target = max(0.0, min(limit, self.position() + delta))
        was_paused = self.state == Player.PAUSED
        pygame.mixer.music.play(start=target)
        self._offset = target
        if was_paused:
            pygame.mixer.music.pause()

    def finished(self) -> bool:
        return self.state == Player.PLAYING and not pygame.mixer.music.get_busy()


# ---------------------------------------------------------------------------
# Serial link (background thread: connect, handshake, write queue, reconnect)
# ---------------------------------------------------------------------------
def find_esp32_port() -> Optional[str]:
    ports = list(list_ports.comports())
    for p in ports:
        if p.vid in KNOWN_USB_VIDS:
            return p.device
    keywords = ("cp210", "ch340", "ch910", "silicon labs", "usb serial", "uart")
    for p in ports:
        if any(k in (p.description or "").lower() for k in keywords):
            return p.device
    return ports[0].device if len(ports) == 1 else None


class SerialLink:
    def __init__(self, port: Optional[str], baud: int, enabled: bool = True):
        self._port_arg = port
        self._port = port or ""
        self._baud = baud
        self._enabled = enabled
        self._queue: "queue.Queue[Tuple[bytes, bool]]" = queue.Queue(maxsize=64)
        self._quit = threading.Event()
        self._resync = threading.Event()
        self._ser: Optional[serial.Serial] = None
        self._rx = bytearray()
        self._last_ack = 0.0
        self._last_keepalive = 0.0
        self.connected = False
        self.status = "disabled" if not enabled else "starting"
        self._thread = threading.Thread(target=self._run, name="serial-link", daemon=True)

    # ---- public API (called from the main thread) --------------------------
    def start(self) -> None:
        if self._enabled:
            self._thread.start()

    def send(self, frame: bytes, droppable: bool = False) -> bool:
        """Queue a frame. Returns False if not connected / dropped.
        Droppable frames (progress) are skipped when the queue is backing up;
        important frames evict the oldest entry instead of being lost."""
        if not self.connected:
            return False
        if droppable and self._queue.qsize() > 6:
            return False
        try:
            self._queue.put_nowait((frame, droppable))
        except queue.Full:
            try:
                self._queue.get_nowait()
                self._queue.put_nowait((frame, droppable))
            except (queue.Empty, queue.Full):
                return False
        return True

    def take_resync(self) -> bool:
        """True once after (re)connect or ESP32 reboot: caller must resend state."""
        if self._resync.is_set():
            self._resync.clear()
            return True
        return False

    def close(self, flush_timeout: float = 1.0) -> None:
        deadline = time.monotonic() + flush_timeout
        while self.connected and not self._queue.empty() and time.monotonic() < deadline:
            time.sleep(0.02)
        self._quit.set()
        if self._enabled and self._thread.is_alive():
            self._thread.join(1.5)
        self._drop("closed")

    # ---- worker thread -----------------------------------------------------
    def _run(self) -> None:
        last_attempt = 0.0
        while not self._quit.is_set():
            if self._ser is None:
                wait = RECONNECT_INTERVAL - (time.monotonic() - last_attempt)
                if last_attempt and wait > 0:
                    self._quit.wait(min(wait, 0.2))
                    continue
                last_attempt = time.monotonic()
                self._try_connect()
                continue
            try:
                self._pump()
            except (serial.SerialException, OSError) as exc:
                self._drop(f"lost connection ({exc.__class__.__name__}); retrying")

    def _try_connect(self) -> None:
        port = self._port_arg or find_esp32_port()
        if not port:
            self.status = "no serial port found (plug in the ESP32)"
            return
        self._port = port
        self.status = f"opening {port}"
        ser = serial.Serial()
        try:
            ser.port = port
            ser.baudrate = self._baud
            ser.timeout = 0
            ser.write_timeout = 1.0
            ser.dtr = False        # try to avoid auto-reset of the ESP32 on open
            ser.rts = False
            ser.open()
            self.status = f"{port}: waiting for ESP32"
            if not self._handshake(ser):
                ser.close()
                self.status = f"{port}: no answer from ESP32 (flash the firmware?)"
                return
        except (serial.SerialException, OSError, ValueError) as exc:
            try:
                ser.close()
            except Exception:
                pass
            self.status = f"cannot use {port}: {exc}"
            return
        self._ser = ser
        self._last_ack = time.monotonic()
        self._last_keepalive = time.monotonic()
        self.connected = True
        self.status = f"connected on {port}"
        self._resync.set()

    def _handshake(self, ser: "serial.Serial") -> bool:
        self._rx.clear()
        deadline = time.monotonic() + HANDSHAKE_TIMEOUT
        next_hello = 0.0
        while not self._quit.is_set() and time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_hello:
                ser.write(build_frame("H", PROTOCOL_VERSION))
                next_hello = now + 0.5
            if self._read_incoming(ser):
                return True
            self._quit.wait(0.02)
        return False

    def _pump(self) -> None:
        ser = self._ser
        assert ser is not None
        try:
            frame, droppable = self._queue.get(timeout=0.02)
        except queue.Empty:
            frame, droppable = None, False
        if frame is not None and not (droppable and ser.out_waiting > 256):
            ser.write(frame)
        self._read_incoming(ser)
        now = time.monotonic()
        if now - self._last_keepalive >= KEEPALIVE_INTERVAL:
            self._last_keepalive = now
            ser.write(build_frame("H", PROTOCOL_VERSION))
        if now - self._last_ack > 6.0:
            self.status = f"{self._port}: ESP32 not answering"
        else:
            self.status = f"connected on {self._port}"

    def _read_incoming(self, ser: "serial.Serial") -> bool:
        """Drain RX; returns True if an ACK / READY frame was seen."""
        waiting = ser.in_waiting
        if not waiting:
            return False
        self._rx.extend(ser.read(waiting))
        if len(self._rx) > 4096 and b"\n" not in self._rx:
            self._rx.clear()                     # junk without line breaks
        seen = False
        while True:
            end = self._rx.find(b"\n")
            if end < 0:
                break
            line = bytes(self._rx[:end])
            del self._rx[:end + 1]
            fields = parse_frame(line)
            if not fields:
                continue
            if fields[0] == "A":
                self._last_ack = time.monotonic()
                seen = True
            elif fields[0] == "R":               # ESP32 (re)booted
                self._last_ack = time.monotonic()
                self._resync.set()
                seen = True
        return seen

    def _drop(self, reason: str) -> None:
        ser, self._ser = self._ser, None
        self.connected = False
        self.status = reason
        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass
        while True:
            try:
                self._queue.get_nowait()
            except queue.Empty:
                break


# ---------------------------------------------------------------------------
# Keyboard + console
# ---------------------------------------------------------------------------
class KeyReader:
    """Non-blocking key reader. Windows: msvcrt. Other OS: line-based thread."""

    def __init__(self) -> None:
        self._windows = os.name == "nt"
        self._queue: "queue.Queue[str]" = queue.Queue()
        if not self._windows:
            threading.Thread(target=self._stdin_thread, daemon=True).start()

    def _stdin_thread(self) -> None:
        try:
            for line in sys.stdin:
                for ch in line.rstrip("\n"):
                    self._queue.put(ch)
                self._queue.put("ENTER")
        except Exception:
            pass

    def poll(self) -> List[str]:
        keys: List[str] = []
        if self._windows:
            import msvcrt
            while msvcrt.kbhit():
                ch = msvcrt.getwch()
                if ch in ("\x00", "\xe0"):
                    ch2 = msvcrt.getwch()
                    if ch2 == "K":
                        keys.append("LEFT")
                    elif ch2 == "M":
                        keys.append("RIGHT")
                elif ch == "\r":
                    keys.append("ENTER")
                elif ch == "\x1b":
                    keys.append("ESC")
                elif ch == "\x08":
                    keys.append("BACKSPACE")
                elif ch == "\x03":
                    keys.append("QUIT")
                else:
                    keys.append(ch)
        else:
            while True:
                try:
                    keys.append(self._queue.get_nowait())
                except queue.Empty:
                    break
        return keys


class Console:
    @staticmethod
    def _width() -> int:
        return max(40, shutil.get_terminal_size((100, 25)).columns)

    def log(self, message: str) -> None:
        width = self._width()
        sys.stdout.write("\r" + " " * (width - 1) + "\r" + message + "\n")
        sys.stdout.flush()

    def status(self, text: str) -> None:
        width = self._width() - 1
        sys.stdout.write("\r" + text[:width].ljust(width))
        sys.stdout.flush()


HELP_TEXT = (
    "Controls:  SPACE play/pause/resume | S stop | N next | B previous | , or <- seek -5s | "
    ". or -> seek +5s\n"
    "           number + ENTER = jump to song | L list songs | H help | Q quit"
)


# ---------------------------------------------------------------------------
# Application
# ---------------------------------------------------------------------------
class App:
    def __init__(self, args: argparse.Namespace, paths: List[Path]):
        self.ui = Console()
        self.keys = KeyReader()
        self.player = Player(paths, args.volume)
        self.link = SerialLink(args.port, args.baud, enabled=not args.no_serial)
        self.ascii_fold = not args.no_ascii_fold
        self.autoplay = not args.no_autoplay
        self.running = True
        self.select_buf = ""
        self.need_song_info = True
        self.last_lyric_key: Optional[Tuple[int, str]] = None
        self.beatmap: Optional[BeatMap] = None
        self.beat_cursor: Optional[int] = None
        self.next_energy = 0.0
        self.last_lyric_sent = 0.0
        self.next_progress = 0.0
        self.last_logged_idx: Optional[int] = None

    # ---- helpers -----------------------------------------------------------
    def _field(self, text: str, limit: int = MAX_FIELD_BYTES) -> str:
        return truncate_bytes(to_display_text(text, self.ascii_fold), limit)

    def list_songs(self) -> None:
        self.ui.log("Songs:")
        for i, path in enumerate(self.player.paths, 1):
            marker = ">" if i - 1 == self.player.index else " "
            lrc = "LRC" if find_lrc(path) else "no LRC"
            self.ui.log(f" {marker} {i:2d}. {path.name}  [{lrc}]")

    def play_index(self, index: int) -> None:
        if not 0 <= index < len(self.player.paths):
            self.ui.log(f"No song number {index + 1} (1-{len(self.player.paths)}).")
            return
        try:
            song = self.player.play(index)
        except Exception as exc:   # pygame.error, OSError, ...
            self.player.stop()
            self.ui.log(f"Cannot play {self.player.paths[index].name}: {exc}")
            return
        self.need_song_info = True
        self.last_logged_idx = None
        self.next_progress = 0.0
        self.beatmap, self.beat_cursor = None, None
        threading.Thread(target=self._analyze, args=(song,), daemon=True).start()
        lyric_info = (f"{len(song.lyrics)} lyric lines" if song.lyrics
                      else "NO LYRICS (missing/empty .lrc)")
        self.ui.log(f"Playing: {song.title}  ({fmt_time(song.duration)}, {lyric_info})")

    def _analyze(self, song: Song) -> None:
        bm = BeatMap.analyze(song.path)
        if self.player.song is song:
            self.beatmap, self.beat_cursor = bm, None
            if bm.ready:
                self.ui.log(f"Beat analysis ready: {len(bm.beats)} beats detected.")
            else:
                self.ui.log("Beat analysis unavailable (is numpy installed?) - using a steady pulse.")

    def toggle(self) -> None:
        state = self.player.state
        if state == Player.PLAYING:
            self.player.pause()
            self.ui.log("Paused.")
        elif state == Player.PAUSED:
            self.player.resume()
            self.ui.log("Resumed.")
        else:
            self.play_index(self.player.index if self.player.index >= 0 else 0)
        self.next_progress = 0.0

    def stop(self) -> None:
        self.player.stop()
        self.need_song_info = True
        self.last_logged_idx = None
        self.next_progress = 0.0
        self.ui.log("Stopped.")

    def handle_key(self, key: str) -> None:
        k = key.lower() if len(key) == 1 else key
        if k in ("q", "QUIT"):
            self.running = False
        elif k == " ":
            self.toggle()
        elif k == "s":
            self.stop()
        elif k == "n":
            self.play_index((self.player.index + 1) % len(self.player.paths))
        elif k == "b":
            self.play_index((self.player.index - 1) % len(self.player.paths))
        elif k in (",", "LEFT"):
            self.player.seek(-SEEK_STEP_S)
            self.next_progress = 0.0
            self.beat_cursor = None
        elif k in (".", "RIGHT"):
            self.player.seek(SEEK_STEP_S)
            self.next_progress = 0.0
            self.beat_cursor = None
        elif k == "l":
            self.list_songs()
        elif k in ("h", "?"):
            self.ui.log(HELP_TEXT)
        elif len(k) == 1 and k.isdigit():
            self.select_buf = (self.select_buf + k)[-3:]
        elif k in ("ESC", "BACKSPACE"):
            self.select_buf = ""
        elif k == "ENTER" and self.select_buf:
            number, self.select_buf = int(self.select_buf), ""
            self.play_index(number - 1)

    # ---- device update -----------------------------------------------------
    def update_device(self, now: float) -> str:
        """Compute lyric + beat info for the real playback position and push changes.
        Returns the current lyric text (for the console)."""
        player, song = self.player, self.player.song
        pos = player.position()
        if self.link.take_resync():
            self.need_song_info = True

        lookup = pos + (LYRIC_LEAD_MS / 1000.0 if player.state == Player.PLAYING else 0.0)
        idx, cur, line_ms, elapsed_ms = -2, "", 0, 0
        title, duration = "Music Player", 0.0
        if song is not None:
            title, duration = song.title, song.duration
            if song.lyrics:
                idx, _, cur, _ = song.lyrics.window(lookup)
                line_ms, elapsed_ms = song.lyrics.timing(idx, lookup)
        key = (idx, cur)

        if self.need_song_info:
            self.link.send(build_frame("T", self._field(title, MAX_TITLE_BYTES),
                                       int(duration * 1000)))
            self.last_lyric_key = None
            self.next_progress = 0.0
            self.need_song_info = False

        if key != self.last_lyric_key and now - self.last_lyric_sent >= MIN_LYRIC_INTERVAL:
            self.link.send(build_frame("L", self._field(cur), line_ms, elapsed_ms))
            self.last_lyric_key = key
            self.last_lyric_sent = now

        if now >= self.next_progress:
            self.link.send(build_frame("P", int(pos * 1000), int(duration * 1000),
                                       player.state), droppable=True)
            self.next_progress = now + PROGRESS_INTERVAL

        # audio energy + beat flag
        bm = self.beatmap
        if bm is not None and bm.ready and bm.levels and player.state == Player.PLAYING:
            level = bm.levels[min(len(bm.levels) - 1, int(lookup / BeatMap.HOP))]
            cursor = bisect.bisect_right(bm.beats, lookup)
            beat = 0
            if self.beat_cursor is not None and 0 < cursor - self.beat_cursor <= 2:
                beat = 1
            self.beat_cursor = cursor
            if beat:
                self.link.send(build_frame("E", level, 1))
                self.next_energy = now + ENERGY_INTERVAL
            elif now >= self.next_energy:
                self.link.send(build_frame("E", level, 0), droppable=True)
                self.next_energy = now + ENERGY_INTERVAL
        elif player.state != Player.PLAYING and now >= self.next_energy:
            self.link.send(build_frame("E", 0, 0), droppable=True)
            self.next_energy = now + 0.25

        # print lyrics to the console as they change (handy to check sync)
        if song is not None and player.state == Player.PLAYING and idx != self.last_logged_idx:
            self.last_logged_idx = idx
            if cur:
                self.ui.log(f"  [{fmt_time(pos)}] {cur}")
        return cur

    def status_line(self) -> str:
        player, song = self.player, self.player.song
        icon = {Player.STOPPED: "[]", Player.PLAYING: "> ", Player.PAUSED: "||"}[player.state]
        link = f"ESP32: {self.link.status}"
        if song is None:
            return f"{icon} no song | {link}"
        pos, dur = player.position(), song.duration
        filled = int(20 * min(1.0, pos / dur)) if dur > 0 else 0
        bar = "#" * filled + "-" * (20 - filled)
        text = (f"{icon} {player.index + 1}/{len(player.paths)} {song.title[:28]} [{bar}] "
                f"{fmt_time(pos)}/{fmt_time(dur)} | {link}")
        if self.select_buf:
            text += f" | go to: {self.select_buf}_"
        return text

    # ---- main loop ---------------------------------------------------------
    def run(self) -> None:
        self.link.start()
        self.ui.log(HELP_TEXT)
        self.list_songs()
        if self.autoplay:
            self.play_index(0)
        next_status = 0.0
        try:
            while self.running:
                now = time.monotonic()
                for key in self.keys.poll():
                    self.handle_key(key)
                if self.player.finished():
                    if self.player.index + 1 < len(self.player.paths):
                        self.play_index(self.player.index + 1)
                    else:
                        self.player.stop()
                        self.need_song_info = True
                        self.ui.log("Playlist finished.")
                self.update_device(now)
                if now >= next_status:
                    self.ui.status(self.status_line())
                    next_status = now + 0.2
                time.sleep(LOOP_INTERVAL)
        except KeyboardInterrupt:
            pass
        finally:
            self.shutdown()

    def shutdown(self) -> None:
        self.ui.log("\nShutting down...")
        try:
            self.player.stop()
            self.link.send(build_frame("T", "Goodbye", 0))
            self.link.send(build_frame("L", "", 0, 0))
            self.link.send(build_frame("P", 0, 0, 0))
        finally:
            self.link.close()
            pygame.mixer.quit()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="MP3 player that streams synced LRC lyrics to an ESP32 OLED.")
    p.add_argument("--port", help="serial port, e.g. COM5 (default: auto-detect)")
    p.add_argument("--baud", type=int, default=BAUD_RATE, help=f"baud rate (default {BAUD_RATE})")
    p.add_argument("--songs", type=Path, default=SONGS_DIR, help="folder with .mp3 and .lrc files")
    p.add_argument("--volume", type=float, default=0.8, help="0.0 - 1.0 (default 0.8)")
    p.add_argument("--no-serial", action="store_true", help="play music without an ESP32")
    p.add_argument("--no-autoplay", action="store_true", help="do not start the first song automatically")
    p.add_argument("--no-ascii-fold", action="store_true",
                   help="send raw UTF-8 (firmware folds Latin-1 only) instead of folding to ASCII")
    p.add_argument("--list-ports", action="store_true", help="list serial ports and exit")
    return p.parse_args()


def main() -> int:
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(encoding="utf-8", errors="replace")   # type: ignore[attr-defined]
        except Exception:
            pass
    args = parse_args()

    if args.list_ports:
        ports = list(list_ports.comports())
        if not ports:
            print("No serial ports found.")
        for port in ports:
            print(f"{port.device:10s} {port.description}  (VID:PID={port.vid}:{port.pid})")
        return 0

    paths = discover_songs(args.songs)
    if not paths:
        print(f"No .mp3 files found in: {args.songs}\n"
              "Copy your songs (and matching .lrc files with the same name) into that folder.")
        return 1
    try:
        app = App(args, paths)
    except pygame.error as exc:
        print(f"Audio device could not be opened: {exc}")
        return 1
    app.run()
    return 0


if __name__ == "__main__":
    sys.exit(main())
