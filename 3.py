#!/usr/bin/env python3
"""
4-DOF Arm Control Panel – v2.8  (17 Jul 2025)

CHANGES v2.7 → v2.8
• Fixed .startsWith → .startswith typo that crashed the RX thread
• Hardened SerialWorker: automatic reconnection, robust exception handling,
  thread-safe TX with throttle, clean shutdown
• Added Home-Record button, Play-Once / Play-Loop / Stop playback controls
• Detachable servo status LEDs and enriched console log
• Minor GUI polish and bug-fixes
---------------------------------------------------------------------------
Required pip packages:
    pip install pyserial ttkbootstrap
"""
from __future__ import annotations

import json
import queue
import sys
import threading
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Final, Callable

import serial                           # type: ignore
import serial.tools.list_ports          # type: ignore
import tkinter as tk
from tkinter import filedialog, messagebox

import ttkbootstrap as ttkbs            # type: ignore
from ttkbootstrap.dialogs import Messagebox


# ═════════════════════════════════ CONFIG ═════════════════════════════════ #

BAUD: Final[int]              = 115_200
PORT_DEFAULT: Final[str]      = "COM3" if sys.platform.startswith("win") else "/dev/ttyACM0"

STREAM_POLL_MS_GUI: Final[int] = 50        # GUI polling rate
RECONNECT_INTERVAL_S: Final[int] = 2       # Serial auto-reconnect cadence
CMD_THROTTLE_MS: Final[int]     = 30       # Min gap between two TX lines

HOME: Final[list[int]] = [90, 90, 180, 0, 0]

# (Name, min °, max °)
SERVOS_META: Final[list[tuple[str, int, int]]] = [
    ("base",     0, 180),
    ("shoulder", 10, 150),
    ("elbow",    10, 180),
    ("wrist",     0, 180),
    ("grip",      0,  85),
]

# ═════════════════════════════════ MODELS ═════════════════════════════════ #

@dataclass(slots=True)
class ArmState:
    base: int      = 90
    shoulder: int  = 90
    elbow: int     = 180
    wrist: int     = 0
    grip: int      = 0

    @classmethod
    def from_csv(cls, line: str) -> "ArmState":
        # Expected: ANGLES,base,shoulder,elbow,wrist,grip
        try:
            _, *nums = line.split(",", 5)
            nums = [int(x) for x in nums[:5]]
            return cls(*nums)
        except Exception:
            return cls()      # fallback neutral pose

    def to_dict(self) -> dict[str, int]:
        return asdict(self)


@dataclass(slots=True)
class SystemStatus:
    arduino_ok:   bool              = False
    joysticks_ok: bool              = False
    servo_attached: list[bool] | None = None

    def __post_init__(self) -> None:
        if self.servo_attached is None:
            self.servo_attached = [False] * len(SERVOS_META)

# ═════════════════════════════ SERIAL WORKER ══════════════════════════════ #

class SerialWorker(threading.Thread):
    """
    Separate thread handling all blocking serial I/O.
    – Auto-reconnects if cable is unplugged
    – Throttles outbound TX
    – Dispatches incoming ANGLES / STATUS lines via callbacks or queue
    """

    def __init__(
        self,
        port: str,
        baud: int,
        rx_q: queue.Queue[tuple[str, str]],
        log_cb: Callable[[str], None],
        err_cb: Callable[[str], None],
    ) -> None:
        super().__init__(daemon=True)
        self._port_target = port
        self._baud        = baud
        self._rx_q        = rx_q
        self._log         = log_cb
        self._err         = err_cb
        self._stop_evt    = threading.Event()
        self._tx_lock     = threading.Lock()
        self._last_tx_ms  = 0.0
        self._ser: serial.Serial | None = None

    # ───────────── PUBLIC ─────────────
    def stop(self) -> None:
        self._stop_evt.set()

    def send(self, text: str) -> None:
        """Thread-safe write with millisecond throttle."""
        if self._ser is None or not self._ser.is_open:
            self._err("TX dropped – port closed")
            return
        now_ms = time.perf_counter() * 1000
        if now_ms - self._last_tx_ms < CMD_THROTTLE_MS:
            return                       # debounced
        self._last_tx_ms = now_ms
        with self._tx_lock:
            try:
                self._ser.write(text.encode() + b"\n")
            except serial.SerialException as exc:
                self._err(f"TX error: {exc}")

    # ──────────── THREAD LOOP ─────────
    def run(self) -> None:
        while not self._stop_evt.is_set():
            if self._ensure_port():
                self._read_loop()
            else:
                time.sleep(RECONNECT_INTERVAL_S)
        self._close_port()

    # ──────────── INTERNALS ───────────
    def _ensure_port(self) -> bool:
        if self._ser and self._ser.is_open:
            return True
        try:
            self._ser = serial.Serial(
                self._port_target, self._baud, timeout=0.05
            )
            self._log(f"[Serial] Connected on {self._port_target}")
            time.sleep(2)                # allow board reset
            return True
        except serial.SerialException:
            self._err(f"No device at {self._port_target}")
            self._ser = None
            return False

    def _read_loop(self) -> None:
        assert self._ser is not None
        try:
            while not self._stop_evt.is_set() and self._ser.is_open:
                try:
                    line = self._ser.readline().decode(errors="ignore").strip()
                except serial.SerialException as exc:
                    self._err(f"RX error: {exc}")
                    break
                if not line:
                    continue
                if line.startswith("ANGLES"):
                    self._rx_q.put(("ANGLES", line))
                elif line.startswith("STATUS"):
                    self._rx_q.put(("STATUS", line))
                else:
                    self._rx_q.put(("DEBUG", line))
        finally:
            self._close_port()

    def _close_port(self) -> None:
        try:
            if self._ser:
                self._ser.close()
        except Exception:
            pass
        self._ser = None
        self._log("[Serial] Port closed")

# ════════════════════════════════ GUI APP ════════════════════════════════ #

class MainWin(tk.Tk):
    """Tk/ttkbootstrap GUI"""

    def __init__(self) -> None:
        super().__init__()
        ttkbs.Style("flatly")            # set theme early
        self.title("4-DOF Arm Control Panel v2.8")
        self.geometry("1050x700")
        self.minsize(920, 650)

        # ───── STATE ─────
        self.arm: ArmState         = ArmState()
        self.sys: SystemStatus     = SystemStatus()
        self._rx_q: queue.Queue[tuple[str, str]] = queue.Queue()
        self._worker: SerialWorker | None = None
        self._port_var = tk.StringVar(value=PORT_DEFAULT)

        # Sequence recording
        self._steps: list[dict[str, int]] = []
        self._sel_idx: int = -1

        # Playback control
        self._play_thread: threading.Thread | None = None
        self._stop_evt: threading.Event = threading.Event()
        self._looping: bool = False

        # ───── UI ─────
        self._build_ui()
        self.after(STREAM_POLL_MS_GUI, self._poll_serial_q)

    # ───────────────── UI BUILDERS ───────────────── #

    def _build_ui(self) -> None:
        nb = ttkbs.Notebook(self)
        nb.pack(fill="both", expand=True, padx=10, pady=10)

        self._tab_ctrl  = ttkbs.Frame(nb)
        self._tab_seq   = ttkbs.Frame(nb)
        nb.add(self._tab_ctrl, text="Control")
        nb.add(self._tab_seq,  text="Sequential Record")

        # Control tab
        self._build_connection_bar(self._tab_ctrl)
        self._build_command_bar(self._tab_ctrl)
        self._build_servo_grid(self._tab_ctrl)
        self._build_angles_bar(self._tab_ctrl)
        self._build_console(self._tab_ctrl)

        # Sequence tab
        self._build_sequence_tab(self._tab_seq)

    # ─────────── CONTROL TAB ─────────── #
    def _build_connection_bar(self, parent) -> None:
        frm = ttkbs.LabelFrame(parent, text="🔌 Connection", bootstyle="primary")
        frm.pack(fill="x", padx=10, pady=(10, 8))

        ttkbs.Label(frm, text="Port:", font=("Arial", 10, "bold")).pack(side="left", padx=(10,4))
        ttkbs.Entry(frm, width=15, textvariable=self._port_var,
                    font=("Consolas", 10)).pack(side="left")

        self._btn_conn = ttkbs.Button(
            frm, text="Connect", width=12,
            command=self._toggle_connection, bootstyle="success-outline")
        self._btn_conn.pack(side="left", padx=(10,20))

        # Status LEDs
        self._led_ardu  = ttkbs.Label(frm, text="●", font=("Arial", 16))
        self._led_js    = ttkbs.Label(frm, text="●", font=("Arial", 16))
        ttkbs.Label(frm, text="Arduino:").pack(side="left")
        self._led_ardu.pack(side="left", padx=3)
        ttkbs.Label(frm, text="Joystick:").pack(side="left", padx=(15,0))
        self._led_js.pack(side="left", padx=3)

    def _build_command_bar(self, parent) -> None:
        frm = ttkbs.LabelFrame(parent, text="🎮 Robot Commands", bootstyle="info")
        frm.pack(fill="x", padx=10, pady=(0, 8))

        def btn(txt, style, cb=None):
            cb = cb or (lambda t=txt: self._send(t))
            ttkbs.Button(frm, text=txt, width=12,
                         command=cb, bootstyle=style).pack(side="left", padx=4)

        btn("HOME",     "warning-outline")
        btn("ENABLE",   "success-outline")
        btn("DISABLE",  "danger-outline")
        btn("STATUS",   "info-outline")
        btn("?ANGLES",  "secondary-outline")

        ttkbs.Button(frm, text="📹 Record Step", width=15,
                     command=self._record_step,
                     bootstyle="success").pack(side="left", padx=(20,4))

    def _build_servo_grid(self, parent) -> None:
        frame = ttkbs.LabelFrame(parent, text="📊 Live Servo Angles (read-only)",
                                 bootstyle="secondary")
        frame.pack(fill="x", padx=10, pady=(0,8))
        grid = ttkbs.Frame(frame)
        grid.pack(fill="both", expand=True, padx=15, pady=10)
        grid.columnconfigure((0,1), weight=1)

        self._slider_vars: dict[str, tk.IntVar] = {}
        self._servo_leds: dict[str, ttkbs.Label] = {}

        for idx, (key, mn, mx) in enumerate(SERVOS_META):
            col, row = idx % 2, idx // 2
            cell = ttkbs.LabelFrame(grid, text=key.capitalize(), bootstyle="light")
            cell.grid(row=row, column=col, sticky="nsew", padx=8, pady=6)

            # LED
            led = ttkbs.Label(cell, text="●", font=("Arial",14))
            led.pack(anchor="e", padx=5, pady=2)
            self._servo_leds[key] = led

            # Slider
            var = tk.IntVar(value=getattr(self.arm, key))
            self._slider_vars[key] = var
            ttkbs.Scale(cell, from_=mn, to=mx,
                        variable=var, orient="horizontal",
                        state="disabled", length=280,
                        bootstyle="info").pack(fill="x", padx=10, pady=(0,10))

    def _build_angles_bar(self, parent) -> None:
        frame = ttkbs.LabelFrame(parent, text="🎯 Current Angles (°)",
                                 bootstyle="warning")
        frame.pack(fill="x", padx=10, pady=(0,8))
        inner = ttkbs.Frame(frame)
        inner.pack(fill="x", padx=15, pady=10)

        self._angle_vars: dict[str, tk.StringVar] = {}
        for key, _, _ in SERVOS_META:
            v = tk.StringVar(value="--")
            self._angle_vars[key] = v
            cell = ttkbs.Frame(inner)
            cell.pack(side="left", expand=True, padx=5)
            ttkbs.Label(cell, text=key.capitalize(),
                        font=("Arial",10,"bold")).pack()
            ttkbs.Label(cell, textvariable=v, width=6,
                        font=("Consolas",14,"bold"),
                        bootstyle="warning", relief="solid"
                        ).pack(pady=2)

    def _build_console(self, parent) -> None:
        frame = ttkbs.LabelFrame(parent, text="📋 Console Log", bootstyle="dark")
        frame.pack(fill="both", expand=True, padx=10, pady=(0,10))
        txt = tk.Text(frame, height=10, wrap="word", font=("Consolas",10),
                      bg="#2c3e50", fg="#ecf0f1", relief="flat")
        txt.pack(side="left", fill="both", expand=True, padx=(8,0), pady=8)
        scr = ttkbs.Scrollbar(frame, command=txt.yview)
        scr.pack(side="right", fill="y", pady=8)
        txt.configure(yscrollcommand=scr.set)
        self._console = txt

    # ───────────── SEQUENCE TAB ───────────── #
    def _build_sequence_tab(self, parent) -> None:
        hdr = ttkbs.LabelFrame(parent, text="📝 Sequence Management", bootstyle="primary")
        hdr.pack(fill="x", padx=10, pady=10)
        ttkbs.Label(hdr, text="Record servo positions from the Control tab, "
                    "then manage/play them here.", foreground="gray"
                    ).pack(padx=10, pady=(5,10))

        content = ttkbs.Frame(parent)
        content.pack(fill="both", expand=True, padx=10, pady=(0,10))

        # Listbox
        lst_frame = ttkbs.LabelFrame(content, text="🎬 Recorded Steps", bootstyle="info")
        lst_frame.pack(side="left", fill="both", expand=True, padx=(0,5))
        self._lst = tk.Listbox(lst_frame, font=("Consolas",10),
                               height=18, selectmode="single")
        self._lst.pack(side="left", fill="both", expand=True, padx=10, pady=10)
        ttkbs.Scrollbar(lst_frame, command=self._lst.yview
                        ).pack(side="right", fill="y", pady=10)
        self._lst.bind("<<ListboxSelect>>", self._on_select)

        # Controls
        ctl = ttkbs.LabelFrame(content, text="🎮 Step / Playback Controls",
                               bootstyle="warning")
        ctl.pack(side="right", fill="y", padx=(5,0))

        # ─ Step management
        grp = ttkbs.LabelFrame(ctl, text="Step Management")
        grp.pack(fill="x", padx=10, pady=10)
        self._btn_up    = ttkbs.Button(grp, text="⬆️ Move Up",
                                       command=self._move_up,
                                       bootstyle="info-outline")
        self._btn_down  = ttkbs.Button(grp, text="⬇️ Move Down",
                                       command=self._move_down,
                                       bootstyle="info-outline")
        self._btn_del   = ttkbs.Button(grp, text="🗑️ Delete",
                                       command=self._delete_step,
                                       bootstyle="danger-outline")
        for b in (self._btn_up, self._btn_down, self._btn_del):
            b.pack(fill="x", padx=5, pady=2)

        # ─ File ops
        grp2 = ttkbs.LabelFrame(ctl, text="Sequence File")
        grp2.pack(fill="x", padx=10, pady=10)
        ttkbs.Button(grp2, text="💾 Save",  command=self._save_seq,
                     bootstyle="success-outline").pack(fill="x", padx=5, pady=2)
        ttkbs.Button(grp2, text="📁 Load",  command=self._load_seq,
                     bootstyle="primary-outline").pack(fill="x", padx=5, pady=2)
        ttkbs.Button(grp2, text="🗑️ Clear", command=self._clear_seq,
                     bootstyle="warning-outline").pack(fill="x", padx=5, pady=2)

        # ─ Playback
        grp3 = ttkbs.LabelFrame(ctl, text="▶️ Playback")
        grp3.pack(fill="x", padx=10, pady=10)
        self._btn_home  = ttkbs.Button(grp3, text="🏠 Home Record",
                                       command=self._record_home,
                                       bootstyle="warning-outline")
        self._btn_once  = ttkbs.Button(grp3, text="▶️ Play Once",
                                       command=self._play_once,
                                       bootstyle="success-outline")
        self._btn_loop  = ttkbs.Button(grp3, text="🔁 Play Loop",
                                       command=self._play_loop,
                                       bootstyle="info-outline")
        self._btn_stop  = ttkbs.Button(grp3, text="⏹️ Stop",
                                       command=self._stop_play,
                                       bootstyle="danger-outline")
        for b in (self._btn_home, self._btn_once, self._btn_loop, self._btn_stop):
            b.pack(fill="x", padx=5, pady=2)

        # Info label
        grp4 = ttkbs.LabelFrame(ctl, text="Sequence Info")
        grp4.pack(fill="x", padx=10, pady=10)
        self._lbl_info = ttkbs.Label(grp4, text="No steps recorded",
                                     foreground="gray")
        self._lbl_info.pack(padx=5, pady=5)

        self._update_seq_controls()

    # ══════════════════════════ EVENT HANDLERS ══════════════════════════ #

    # ---- Connection / Serial ----
    def _toggle_connection(self) -> None:
        if self._worker is None:
            port = self._port_var.get().strip()
            if not port:
                self._error("Enter a COM/tty port.")
                return
            self._worker = SerialWorker(
                port=port, baud=BAUD, rx_q=self._rx_q,
                log_cb=self._log, err_cb=self._error)
            self._worker.start()
            self.sys.arduino_ok = True
            self._log(f"🔌 Opening {port} ...")
        else:
            self._worker.stop()
            self._worker = None
            self.sys = SystemStatus()
            self._log("🔌 Connection closed.")
        self._refresh_status()

    def _send(self, cmd: str) -> None:
        if self._worker is None:
            self._error("Not connected.")
            return
        self._worker.send(cmd)
        self._log(f"📤 {cmd}")

    # ---- Serial queue polling ----
    def _poll_serial_q(self) -> None:
        while not self._rx_q.empty():
            typ, line = self._rx_q.get_nowait()
            if typ == "ANGLES":
                self.arm = ArmState.from_csv(line)
            elif typ == "STATUS":
                self._parse_status(line)
            elif typ == "DEBUG":
                self._log(f"🤖 {line}")
        self._refresh_ui()
        self.after(STREAM_POLL_MS_GUI, self._poll_serial_q)

    def _parse_status(self, line: str) -> None:
        # STATUS,js_enabled,att0..4
        parts = line.split(",", 7)
        if len(parts) != 7:
            self._log(f"⚠️ Malformed STATUS: {line}")
            return
        self.sys.joysticks_ok = parts[1] == "1"
        self.sys.servo_attached = [p == "1" for p in parts[2:]]
        self._refresh_status()

    # ---- Console helpers ----
    def _log(self, msg: str) -> None:
        ts = time.strftime("%H:%M:%S")
        self._console.insert("end", f"[{ts}] {msg}\n")
        self._console.see("end")

    def _error(self, msg: str) -> None:
        self._log(f"❌ {msg}")
        Messagebox.show_error(title="4-DOF Arm Error", message=msg, parent=self,
                              alert=True)

    # ---- Servo LEDs / Status bar ----
    def _refresh_ui(self) -> None:
        for key, val in asdict(self.arm).items():
            self._angle_vars[key].set(f"{val}°")
            self._slider_vars[key].set(val)
        # Servo LEDs
        for idx, (key, _, _) in enumerate(SERVOS_META):
            attached = self.sys.servo_attached[idx] if idx < len(self.sys.servo_attached) else False
            self._servo_leds[key].configure(foreground="#2ecc71" if attached else "#e74c3c")
        self._refresh_status()

    def _refresh_status(self) -> None:
        ard = self.sys.arduino_ok and (self._worker is not None)
        self._led_ardu.configure(foreground="#2ecc71" if ard else "#e74c3c")
        self._btn_conn.configure(
            text="Disconnect" if ard else "Connect",
            bootstyle="danger-outline" if ard else "success-outline")
        self._led_js.configure(foreground="#2ecc71" if self.sys.joysticks_ok else "#e74c3c")
        self._update_seq_controls()

    # ---- Sequence list operations ----
    def _record_step(self) -> None:
        if not self.sys.arduino_ok:
            self._error("Arduino not connected.")
            return
        self._steps.append(self.arm.to_dict())
        self._update_lstbox()
        self._log(f"📹 Step {len(self._steps)} recorded")

    def _record_home(self) -> None:
        self._steps.append(dict(zip(["base","shoulder","elbow","wrist","grip"], HOME)))
        self._update_lstbox()
        self._log(f"🏠 Home step recorded")

    def _on_select(self, _evt) -> None:
        sel = self._lst.curselection()
        self._sel_idx = sel[0] if sel else -1
        self._update_seq_controls()

    def _move_up(self) -> None:
        if self._sel_idx <= 0:
            return
        self._steps[self._sel_idx-1], self._steps[self._sel_idx] = \
            self._steps[self._sel_idx], self._steps[self._sel_idx-1]
        self._sel_idx -= 1
        self._update_lstbox()
        self._lst.selection_set(self._sel_idx)

    def _move_down(self) -> None:
        if self._sel_idx < 0 or self._sel_idx >= len(self._steps)-1:
            return
        self._steps[self._sel_idx+1], self._steps[self._sel_idx] = \
            self._steps[self._sel_idx], self._steps[self._sel_idx+1]
        self._sel_idx += 1
        self._update_lstbox()
        self._lst.selection_set(self._sel_idx)

    def _delete_step(self) -> None:
        if self._sel_idx < 0:
            return
        del self._steps[self._sel_idx]
        self._sel_idx = -1
        self._update_lstbox()

    def _save_seq(self) -> None:
        if not self._steps:
            self._error("No steps to save.")
            return
        fp = filedialog.asksaveasfilename(
            defaultextension=".json", filetypes=[("JSON","*.json")],
            title="Save Sequence")
        if fp:
            try:
                with open(fp,"w") as f:
                    json.dump(self._steps,f,indent=2)
                self._log(f"💾 Saved {len(self._steps)} steps → {Path(fp).name}")
            except Exception as e:
                self._error(str(e))

    def _load_seq(self) -> None:
        fp = filedialog.askopenfilename(
            filetypes=[("JSON","*.json")], title="Load Sequence")
        if fp:
            try:
                with open(fp,"r") as f:
                    data = json.load(f)
                if not isinstance(data,list):
                    raise ValueError("File format")
                self._steps = data
                self._sel_idx = -1
                self._update_lstbox()
                self._log(f"📁 Loaded {len(self._steps)} steps")
            except Exception as e:
                self._error(str(e))

    def _clear_seq(self) -> None:
        if not self._steps:
            return
        if messagebox.askyesno("Clear Sequence",
                               f"Delete all {len(self._steps)} steps?",
                               icon="warning"):
            self._steps.clear()
            self._sel_idx = -1
            self._update_lstbox()

    def _update_lstbox(self) -> None:
        self._lst.delete(0,"end")
        for i, st in enumerate(self._steps,1):
            self._lst.insert("end",
                f"Step {i:02d}: B{st['base']:3d} S{st['shoulder']:3d} "
                f"E{st['elbow']:3d} W{st['wrist']:3d} G{st['grip']:2d}")
        self._update_seq_controls()

    def _update_seq_controls(self) -> None:
        n = len(self._steps)
        sel = self._sel_idx >= 0
        connected = self.sys.arduino_ok
        # Step buttons
        self._btn_up.configure(state="normal" if sel and self._sel_idx>0 else "disabled")
        self._btn_down.configure(state="normal" if sel and self._sel_idx<n-1 else "disabled")
        self._btn_del.configure(state="normal" if sel else "disabled")
        # Playback
        has = n>0
        state_pb = "normal" if has and connected else "disabled"
        self._btn_once.configure(state=state_pb)
        self._btn_loop.configure(state=state_pb)
        # Info
        txt = "No steps recorded" if n==0 else f"{n} step{'s'*(n!=1)}"
        if sel:
            txt += f" — selected #{self._sel_idx+1}"
        self._lbl_info.configure(text=txt)

    # ---- Playback threads ----
    def _play_once(self) -> None:
        self._start_play(loop=False)

    def _play_loop(self) -> None:
        self._start_play(loop=True)

    def _start_play(self, loop: bool) -> None:
        if not self._steps:
            self._error("No steps to play.")
            return
        if not self.sys.arduino_ok:
            self._error("Arduino not connected.")
            return
        self._stop_evt.clear()
        self._looping = loop
        self._play_thread = threading.Thread(target=self._playback, daemon=True)
        self._play_thread.start()
        self._log("▶️ Playback started" + (" (loop)" if loop else ""))

    def _stop_play(self) -> None:
        self._stop_evt.set()
        if self._play_thread and self._play_thread.is_alive():
            self._play_thread.join(timeout=0.5)
        self._log("⏹️ Playback stopped.")

    def _playback(self) -> None:
        loop_n = 0
        while not self._stop_evt.is_set():
            loop_n += 1
            if self._looping:
                self._log(f"🔁 Loop {loop_n}")
            for idx, st in enumerate(self._steps,1):
                if self._stop_evt.is_set():
                    break
                cmd = f"MOVE:{st['base']},{st['shoulder']},{st['elbow']}," \
                      f"{st['wrist']},{st['grip']}"
                if self._worker:
                    self._worker.send(cmd)
                self._log(f"📤 Step {idx} → {cmd}")
                # simple fixed delay 2 s
                for _ in range(20):
                    if self._stop_evt.is_set():
                        break
                    time.sleep(0.1)
            if not self._looping:
                break
        self._log("✅ Playback complete.")

    # ═════════════ WINDOW CLEANUP ═════════════
    def destroy(self) -> None:
        self._stop_play()
        if self._worker is not None:
            self._worker.stop()
            self._worker.join(timeout=2)
        super().destroy()
        sys.exit(0)

# ═════════════════════════════════ MAIN ═════════════════════════════════ #

def main() -> None:
    MainWin().mainloop()

if __name__ == "__main__":
    main()
