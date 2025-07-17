#!/usr/bin/env python3

"""
4-DOF Arm Control Panel – v2.6 (17 Jul 2025)

Changelog vs v2.5:
• Added tabbed interface with Sequential Record tab
• Enhanced Live Servo Positions panel with better styling
• Robust Current Angles panel with improved layout
• Record step functionality for sequence recording
• Move Up/Down, Delete operations for sequence management
• Save/Load/Clear sequence functionality
• Improved console logging for all operations

Requires:
• Python ≥3.9
• pyserial → pip install pyserial
• ttkbootstrap → pip install ttkbootstrap
"""

from __future__ import annotations

import json
import queue
import sys
import threading
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Final
from tkinter import filedialog, messagebox

import serial  # type: ignore
import ttkbootstrap as ttkbs  # type: ignore
from ttkbootstrap.dialogs import Messagebox
import tkinter as tk

# ─────────────────────────────── CONFIG ────────────────────────────── #

BAUD: Final[int] = 115_200
PORT_DEFAULT: Final[str] = (
    "COM3" if sys.platform.startswith("win") else "/dev/ttyACM0"
)

STREAM_POLL_MS_GUI: Final[int] = 50  # GUI poll loop
RECONNECT_INTERVAL_S: Final[int] = 2  # USB re-scan cadence
CMD_THROTTLE_MS: Final[int] = 30  # min gap between two TX

SERVOS_META: Final[list[tuple[str, int, int]]] = [
    ("base", 0, 180),
    ("shoulder", 10, 150),
    ("elbow", 10, 180),
    ("wrist", 0, 180),
    ("grip", 0, 85),
]

# ────────────────────────────── MODELS ─────────────────────────────── #

@dataclass(slots=True)
class ArmState:
    base: int = 90
    shoulder: int = 90
    elbow: int = 180
    wrist: int = 0
    grip: int = 0

    @classmethod
    def from_csv(cls, line: str) -> "ArmState":
        # Expected: ANGLES,base,shoulder,elbow,wrist,grip
        try:
            _, *nums = line.split(",", 5)
            nums = [int(x) for x in nums[:5]]
            return cls(*nums)
        except Exception:
            return cls()  # fallback to neutral pose

    def to_dict(self) -> dict[str, int]:
        return asdict(self)

@dataclass(slots=True)
class SystemStatus:
    arduino_ok: bool = False
    joysticks_ok: bool = False
    servo_attached: list[bool] | None = None

    def __post_init__(self) -> None:
        if self.servo_attached is None:
            self.servo_attached = [False] * len(SERVOS_META)

# ──────────────────────────── SERIAL WORKER ────────────────────────── #

class SerialWorker(threading.Thread):
    """
    Dedicated thread for blocking serial I/O.
    Self-heals the connection if unplugged.
    """

    def __init__(
        self,
        port: str,
        baud: int,
        rx_q: queue.Queue[tuple[str, str]],
        log_cb,
        err_cb,
    ) -> None:
        super().__init__(daemon=True)
        self._port_target = port
        self._baud = baud
        self._rx_q = rx_q
        self._log = log_cb
        self._err = err_cb
        self._tx_lock = threading.Lock()
        self._stop_evt = threading.Event()
        self._ser: serial.Serial | None = None
        self._last_tx_ts: float = 0.0

    # ───────── public API ───────── #

    def stop(self) -> None:
        self._stop_evt.set()

    def send(self, text: str) -> None:
        if self._ser is None or not self._ser.is_open:
            self._err("TX dropped – port closed")
            return

        now = time.perf_counter() * 1_000  # ms
        if now - self._last_tx_ts < CMD_THROTTLE_MS:
            return  # debounce

        self._last_tx_ts = now

        with self._tx_lock:
            try:
                self._ser.write(text.encode() + b"\n")
            except serial.SerialException as exc:
                self._err(f"TX error: {exc}")

    # ───────── thread loop ───────── #

    def run(self) -> None:
        while not self._stop_evt.is_set():
            if self._ensure_port():
                self._read_loop()  # blocking read loop
            else:
                time.sleep(RECONNECT_INTERVAL_S)
        self._close_port()

    # ───────── internals ───────── #

    def _ensure_port(self) -> bool:
        if self._ser and self._ser.is_open:
            return True

        try:
            self._ser = serial.Serial(
                self._port_target, self._baud, timeout=0.05
            )
            self._log(f"[Serial] Connected {self._port_target}")
            # Allow board reset
            time.sleep(2)
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

# ─────────────────────────────── GUI APP ────────────────────────────── #

class MainWin(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        ttkbs.Style("flatly")  # choose theme early
        self.title("4-DOF Arm Control Panel v2.6")
        self.geometry("1000x700")
        self.minsize(900, 650)

        # ―― state ――
        self.arm: ArmState = ArmState()
        self.sys: SystemStatus = SystemStatus()
        self._rx_q: queue.Queue[tuple[str, str]] = queue.Queue()
        self._worker: SerialWorker | None = None
        self._port_var = tk.StringVar(value=PORT_DEFAULT)
        
        # ―― sequence recording ――
        self._recorded_steps: list[dict[str, int]] = []
        self._selected_step_idx: int = -1

        # ―― UI ――
        self._build_ui()
        self.after(STREAM_POLL_MS_GUI, self._poll_serial_q)

    # ───────── UI builders ───────── #

    def _build_ui(self) -> None:
        # Create notebook for tabs
        self._notebook = ttkbs.Notebook(self)
        self._notebook.pack(fill="both", expand=True, padx=10, pady=10)

        # Create tabs
        self._control_tab = ttkbs.Frame(self._notebook)
        self._sequence_tab = ttkbs.Frame(self._notebook)

        self._notebook.add(self._control_tab, text="Control")
        self._notebook.add(self._sequence_tab, text="Sequential Record")

        # Build control tab
        self._build_control_tab()
        
        # Build sequence tab
        self._build_sequence_tab()

    def _build_control_tab(self) -> None:
        # Connection bar
        self._build_connection_bar(self._control_tab)
        
        # Control buttons
        self._build_control_buttons(self._control_tab)
        
        # Enhanced servo sliders
        self._build_servo_sliders(self._control_tab)
        
        # Enhanced angle readouts
        self._build_angle_readouts(self._control_tab)
        
        # Console
        self._build_console(self._control_tab)

    def _build_connection_bar(self, parent) -> None:
        frm = ttkbs.LabelFrame(parent, text="🔌 Connection Status", bootstyle="primary")
        frm.pack(fill="x", padx=10, pady=(10, 8))

        # Connection controls
        conn_frame = ttkbs.Frame(frm)
        conn_frame.pack(fill="x", padx=10, pady=8)

        ttkbs.Label(conn_frame, text="Port:", font=("Arial", 10, "bold")).pack(side="left", padx=(0, 5))
        
        port_entry = ttkbs.Entry(conn_frame, width=16, textvariable=self._port_var, font=("Consolas", 10))
        port_entry.pack(side="left", padx=(0, 15))

        self._btn_connect = ttkbs.Button(
            conn_frame, text="Connect", width=12, command=self._toggle_connection,
            bootstyle="success-outline"
        )
        self._btn_connect.pack(side="left", padx=(0, 20))

        # Status indicators
        status_frame = ttkbs.Frame(conn_frame)
        status_frame.pack(side="left", fill="x", expand=True)

        # Arduino status
        arduino_frame = ttkbs.Frame(status_frame)
        arduino_frame.pack(side="left", padx=(0, 25))
        ttkbs.Label(arduino_frame, text="Arduino:", font=("Arial", 10, "bold")).pack(side="left")
        self._led_arduino = ttkbs.Label(arduino_frame, text="●", font=("Arial", 16))
        self._led_arduino.pack(side="left", padx=(5, 0))

        # Joystick status
        js_frame = ttkbs.Frame(status_frame)
        js_frame.pack(side="left")
        ttkbs.Label(js_frame, text="Joystick:", font=("Arial", 10, "bold")).pack(side="left")
        self._led_js = ttkbs.Label(js_frame, text="●", font=("Arial", 16))
        self._led_js.pack(side="left", padx=(5, 0))

    def _build_control_buttons(self, parent) -> None:
        box = ttkbs.LabelFrame(parent, text="🎮 Robot Commands", bootstyle="info")
        box.pack(fill="x", padx=10, pady=(0, 8))

        # Main commands
        main_frame = ttkbs.Frame(box)
        main_frame.pack(padx=10, pady=8)

        def mk_btn(cmd: str, style: str = "primary-outline") -> ttkbs.Button:
            return ttkbs.Button(
                main_frame,
                text=cmd,
                width=12,
                command=lambda t=cmd: self._send(t),
                bootstyle=style
            )

        # Command buttons
        mk_btn("HOME", "warning-outline").pack(side="left", padx=4)
        mk_btn("ENABLE", "success-outline").pack(side="left", padx=4)
        mk_btn("DISABLE", "danger-outline").pack(side="left", padx=4)
        mk_btn("STATUS", "info-outline").pack(side="left", padx=4)
        mk_btn("?ANGLES", "secondary-outline").pack(side="left", padx=4)

        # Record button
        self._btn_record = ttkbs.Button(
            main_frame,
            text="📹 Record Step",
            width=15,
            command=self._record_current_step,
            bootstyle="success"
        )
        self._btn_record.pack(side="left", padx=(20, 4))

    def _build_servo_sliders(self, parent) -> None:
        self._slider_vars: dict[str, tk.IntVar] = {}
        self._servo_leds:   dict[str, ttkbs.Label] = {}

        grid = ttkbs.LabelFrame(parent,
            text="📊 Live Servo Positions (read-only)",
            bootstyle="secondary")
        grid.pack(fill="x", padx=10, pady=(0, 8))

        # Create responsive grid
        main_frame = ttkbs.Frame(grid)
        main_frame.pack(fill="both", expand=True, padx=15, pady=10)

        for idx, (key, mn, mx) in enumerate(SERVOS_META):
            col = idx % 2
            row = idx // 2

            cell = ttkbs.LabelFrame(main_frame,
                text=key.capitalize(), bootstyle="light")
            cell.grid(row=row, column=col,
                sticky="nsew", padx=8, pady=6)

            # ** FIXED HERE **
            main_frame.columnconfigure(col, weight=1)
            main_frame.rowconfigure(row,  weight=1)

            status_frame = ttkbs.Frame(cell)
            status_frame.pack(fill="x", padx=10, pady=(5, 0))
            ttkbs.Label(status_frame,
                text="Status:", font=("Arial", 9)).pack(side="left")
            led = ttkbs.Label(status_frame,
                text="●", font=("Arial", 14))
            led.pack(side="left", padx=(5, 0))
            self._servo_leds[key] = led

            angle_frame = ttkbs.Frame(cell)
            angle_frame.pack(fill="x", padx=10, pady=(5, 0))
            ttkbs.Label(angle_frame,
                text="Angle:", font=("Arial", 9)).pack(side="left")
            angle_label = ttkbs.Label(angle_frame,
                text="--°", font=("Arial", 10, "bold"))
            angle_label.pack(side="right")

            range_frame = ttkbs.Frame(cell)
            range_frame.pack(fill="x", padx=10, pady=(0, 5))
            ttkbs.Label(range_frame,
                text=f"Range: {mn}° - {mx}°",
                font=("Arial", 8),
                foreground="gray").pack()

            var = tk.IntVar(value=getattr(self.arm, key))
            self._slider_vars[key] = var

            slider = ttkbs.Scale(cell,
                from_=mn, to=mx,
                orient="horizontal",
                variable=var,
                length=300,
                state="disabled",
                bootstyle="info")
            slider.pack(fill="x", padx=10, pady=(0, 10))


    def _build_angle_readouts(self, parent) -> None:
        box = ttkbs.LabelFrame(parent, text="🎯 Current Angles (°)", bootstyle="warning")
        box.pack(fill="x", padx=10, pady=(0, 8))

        # Create responsive frame
        main_frame = ttkbs.Frame(box)
        main_frame.pack(fill="x", padx=15, pady=10)

        self._angle_vars: dict[str, tk.StringVar] = {}
        self._angle_labels: dict[str, ttkbs.Label] = {}

        for idx, (key, mn, mx) in enumerate(SERVOS_META):
            # Create angle display cell
            cell = ttkbs.Frame(main_frame)
            cell.pack(side="left", fill="x", expand=True, padx=5)

            # Servo name
            name_label = ttkbs.Label(cell, text=key.capitalize(), 
                                   font=("Arial", 10, "bold"))
            name_label.pack()

            # Angle value
            v = tk.StringVar(value="--")
            angle_label = ttkbs.Label(
                cell, 
                textvariable=v, 
                font=("Consolas", 14, "bold"),
                bootstyle="warning",
                relief="solid",
                padding=5
            )
            angle_label.pack(pady=(2, 0))
            
            # Range info
            range_label = ttkbs.Label(cell, text=f"({mn}-{mx})", 
                                    font=("Arial", 8), 
                                    foreground="gray")
            range_label.pack()

            self._angle_vars[key] = v
            self._angle_labels[key] = angle_label

    def _build_console(self, parent) -> None:
        box = ttkbs.LabelFrame(parent, text="📋 Console Log", bootstyle="dark")
        box.pack(fill="both", expand=True, padx=10, pady=(0, 10))

        console_frame = ttkbs.Frame(box)
        console_frame.pack(fill="both", expand=True, padx=8, pady=8)

        self._console = tk.Text(
            console_frame, 
            height=10, 
            wrap="word", 
            font=("Consolas", 10),
            bg="#2c3e50",
            fg="#ecf0f1",
            selectbackground="#3498db",
            relief="flat"
        )
        self._console.pack(side="left", fill="both", expand=True)

        scr = ttkbs.Scrollbar(console_frame, command=self._console.yview)
        scr.pack(side="right", fill="y")
        self._console.configure(yscrollcommand=scr.set)

    def _build_sequence_tab(self) -> None:
        # Sequence management header
        header_frame = ttkbs.LabelFrame(self._sequence_tab, text="📝 Sequence Management", bootstyle="primary")
        header_frame.pack(fill="x", padx=10, pady=10)

        # Instructions
        instructions = ttkbs.Label(
            header_frame,
            text="Record servo positions from the Control tab, then manage your sequence here.",
            font=("Arial", 10),
            foreground="gray"
        )
        instructions.pack(padx=10, pady=(5, 10))

        # Main content frame
        content_frame = ttkbs.Frame(self._sequence_tab)
        content_frame.pack(fill="both", expand=True, padx=10, pady=(0, 10))

        # Left side - sequence list
        list_frame = ttkbs.LabelFrame(content_frame, text="🎬 Recorded Steps", bootstyle="info")
        list_frame.pack(side="left", fill="both", expand=True, padx=(0, 5))

        # Sequence listbox with scrollbar
        list_container = ttkbs.Frame(list_frame)
        list_container.pack(fill="both", expand=True, padx=10, pady=10)

        self._sequence_listbox = tk.Listbox(
            list_container,
            font=("Consolas", 10),
            height=15,
            selectmode="single"
        )
        self._sequence_listbox.pack(side="left", fill="both", expand=True)
        
        list_scroll = ttkbs.Scrollbar(list_container, command=self._sequence_listbox.yview)
        list_scroll.pack(side="right", fill="y")
        self._sequence_listbox.configure(yscrollcommand=list_scroll.set)

        # Bind selection event
        self._sequence_listbox.bind('<<ListboxSelect>>', self._on_sequence_select)

        # Right side - controls
        controls_frame = ttkbs.LabelFrame(content_frame, text="🎮 Step Controls", bootstyle="warning")
        controls_frame.pack(side="right", fill="y", padx=(5, 0))

        # Step management buttons
        step_frame = ttkbs.LabelFrame(controls_frame, text="Step Management")
        step_frame.pack(fill="x", padx=10, pady=10)

        self._btn_move_up = ttkbs.Button(
            step_frame,
            text="⬆️ Move Up",
            width=15,
            command=self._move_step_up,
            bootstyle="info-outline"
        )
        self._btn_move_up.pack(fill="x", padx=5, pady=2)

        self._btn_move_down = ttkbs.Button(
            step_frame,
            text="⬇️ Move Down",
            width=15,
            command=self._move_step_down,
            bootstyle="info-outline"
        )
        self._btn_move_down.pack(fill="x", padx=5, pady=2)

        self._btn_delete_step = ttkbs.Button(
            step_frame,
            text="🗑️ Delete Step",
            width=15,
            command=self._delete_step,
            bootstyle="danger-outline"
        )
        self._btn_delete_step.pack(fill="x", padx=5, pady=2)

        # Sequence file operations
        file_frame = ttkbs.LabelFrame(controls_frame, text="Sequence File")
        file_frame.pack(fill="x", padx=10, pady=10)

        ttkbs.Button(
            file_frame,
            text="💾 Save Sequence",
            width=15,
            command=self._save_sequence,
            bootstyle="success-outline"
        ).pack(fill="x", padx=5, pady=2)

        ttkbs.Button(
            file_frame,
            text="📁 Load Sequence",
            width=15,
            command=self._load_sequence,
            bootstyle="primary-outline"
        ).pack(fill="x", padx=5, pady=2)

        ttkbs.Button(
            file_frame,
            text="🗑️ Clear Sequence",
            width=15,
            command=self._clear_sequence,
            bootstyle="warning-outline"
        ).pack(fill="x", padx=5, pady=2)

        # Sequence info
        info_frame = ttkbs.LabelFrame(controls_frame, text="Sequence Info")
        info_frame.pack(fill="x", padx=10, pady=10)

        self._sequence_info = ttkbs.Label(
            info_frame,
            text="No steps recorded",
            font=("Arial", 10),
            foreground="gray"
        )
        self._sequence_info.pack(padx=5, pady=5)

        # Update initial state
        self._update_sequence_controls()

    # ───────── Event handlers ───────── #

    def _toggle_connection(self) -> None:
        if self._worker is None:
            port = self._port_var.get().strip()
            if not port:
                self._error("Enter a port name.")
                return

            self._worker = SerialWorker(
                port=port,
                baud=BAUD,
                rx_q=self._rx_q,
                log_cb=self._log,
                err_cb=self._error,
            )
            self._worker.start()
            self.sys.arduino_ok = True
            self._log(f"🔌 Opening connection to {port}...")

        else:
            self._worker.stop()
            self._worker = None
            self.sys = SystemStatus()  # reset to defaults
            self._log("🔌 Connection closed.")

        self._update_status_widgets()

    def _send(self, cmd: str) -> None:
        if self._worker is None:
            self._error("Not connected to Arduino.")
            return

        self._worker.send(cmd)
        self._log(f"📤 >>> {cmd}")

    def _poll_serial_q(self) -> None:
        while not self._rx_q.empty():
            typ, line = self._rx_q.get_nowait()
            if typ == "ANGLES":
                self.arm = ArmState.from_csv(line)
            elif typ == "STATUS":
                self._parse_status(line)
            elif typ == "DEBUG":
                self._log(f"🤖 [Arduino] {line}")

        self._refresh_ui()
        self.after(STREAM_POLL_MS_GUI, self._poll_serial_q)

    # ───────── Sequence recording methods ───────── #

    def _record_current_step(self) -> None:
        if not self.sys.arduino_ok:
            self._error("❌ Cannot record step - Arduino not connected.")
            return

        # Record current arm state
        step_data = self.arm.to_dict()
        self._recorded_steps.append(step_data)
        
        # Update UI
        self._update_sequence_display()
        self._update_sequence_controls()
        
        step_num = len(self._recorded_steps)
        angles_str = f"B:{step_data['base']}° S:{step_data['shoulder']}° E:{step_data['elbow']}° W:{step_data['wrist']}° G:{step_data['grip']}°"
        self._log(f"📹 Step {step_num} recorded: {angles_str}")

    def _on_sequence_select(self, event) -> None:
        selection = self._sequence_listbox.curselection()
        if selection:
            self._selected_step_idx = selection[0]
        else:
            self._selected_step_idx = -1
        self._update_sequence_controls()

    def _move_step_up(self) -> None:
        if self._selected_step_idx <= 0:
            self._error("❌ Cannot move step up - select a step (not the first one).")
            return

        # Swap with previous step
        idx = self._selected_step_idx
        self._recorded_steps[idx], self._recorded_steps[idx-1] = \
            self._recorded_steps[idx-1], self._recorded_steps[idx]
        
        # Update UI and maintain selection
        self._update_sequence_display()
        self._sequence_listbox.selection_set(idx-1)
        self._selected_step_idx = idx-1
        self._update_sequence_controls()
        
        self._log(f"⬆️ Step {idx+1} moved up to position {idx}")

    def _move_step_down(self) -> None:
        if self._selected_step_idx < 0 or self._selected_step_idx >= len(self._recorded_steps) - 1:
            self._error("❌ Cannot move step down - select a step (not the last one).")
            return

        # Swap with next step
        idx = self._selected_step_idx
        self._recorded_steps[idx], self._recorded_steps[idx+1] = \
            self._recorded_steps[idx+1], self._recorded_steps[idx]
        
        # Update UI and maintain selection
        self._update_sequence_display()
        self._sequence_listbox.selection_set(idx+1)
        self._selected_step_idx = idx+1
        self._update_sequence_controls()
        
        self._log(f"⬇️ Step {idx+1} moved down to position {idx+2}")

    def _delete_step(self) -> None:
        if self._selected_step_idx < 0:
            self._error("❌ Cannot delete step - no step selected.")
            return

        idx = self._selected_step_idx
        step_data = self._recorded_steps[idx]
        
        # Remove step
        del self._recorded_steps[idx]
        
        # Update UI
        self._update_sequence_display()
        self._selected_step_idx = -1
        self._update_sequence_controls()
        
        angles_str = f"B:{step_data['base']}° S:{step_data['shoulder']}° E:{step_data['elbow']}° W:{step_data['wrist']}° G:{step_data['grip']}°"
        self._log(f"🗑️ Step {idx+1} deleted: {angles_str}")

    def _save_sequence(self) -> None:
        if not self._recorded_steps:
            self._error("❌ No sequence to save - record some steps first.")
            return

        file_path = filedialog.asksaveasfilename(
            defaultextension=".json",
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")],
            title="Save Sequence"
        )
        
        if file_path:
            try:
                with open(file_path, 'w') as f:
                    json.dump(self._recorded_steps, f, indent=2)
                self._log(f"💾 Sequence saved to {file_path} ({len(self._recorded_steps)} steps)")
            except Exception as e:
                self._error(f"❌ Failed to save sequence: {str(e)}")

    def _load_sequence(self) -> None:
        file_path = filedialog.askopenfilename(
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")],
            title="Load Sequence"
        )
        
        if file_path:
            try:
                with open(file_path, 'r') as f:
                    loaded_steps = json.load(f)
                
                # Validate loaded data
                if not isinstance(loaded_steps, list):
                    raise ValueError("Invalid file format")
                
                for step in loaded_steps:
                    if not isinstance(step, dict) or not all(key in step for key in ['base', 'shoulder', 'elbow', 'wrist', 'grip']):
                        raise ValueError("Invalid step format")
                
                self._recorded_steps = loaded_steps
                self._update_sequence_display()
                self._update_sequence_controls()
                self._selected_step_idx = -1
                
                self._log(f"📁 Sequence loaded from {file_path} ({len(self._recorded_steps)} steps)")
                
            except Exception as e:
                self._error(f"❌ Failed to load sequence: {str(e)}")

    def _clear_sequence(self) -> None:
        if not self._recorded_steps:
            self._log("ℹ️ Sequence already empty.")
            return

        result = messagebox.askyesno(
            "Clear Sequence",
            f"Are you sure you want to clear all {len(self._recorded_steps)} recorded steps?",
            icon="warning"
        )
        
        if result:
            step_count = len(self._recorded_steps)
            self._recorded_steps.clear()
            self._update_sequence_display()
            self._update_sequence_controls()
            self._selected_step_idx = -1
            self._log(f"🗑️ Sequence cleared ({step_count} steps removed)")

    # ───────── UI update methods ───────── #

    def _update_sequence_display(self) -> None:
        self._sequence_listbox.delete(0, tk.END)
        
        for i, step in enumerate(self._recorded_steps):
            step_text = f"Step {i+1:2d}: B:{step['base']:3d}° S:{step['shoulder']:3d}° E:{step['elbow']:3d}° W:{step['wrist']:3d}° G:{step['grip']:2d}°"
            self._sequence_listbox.insert(tk.END, step_text)

    def _update_sequence_controls(self) -> None:
        step_count = len(self._recorded_steps)
        selected = self._selected_step_idx >= 0
        
        # Update button states
        self._btn_move_up.configure(state="normal" if selected and self._selected_step_idx > 0 else "disabled")
        self._btn_move_down.configure(state="normal" if selected and self._selected_step_idx < step_count - 1 else "disabled")
        self._btn_delete_step.configure(state="normal" if selected else "disabled")
        
        # Update info label
        if step_count == 0:
            info_text = "No steps recorded"
        else:
            info_text = f"{step_count} step{'s' if step_count != 1 else ''} recorded"
            if selected:
                info_text += f"\nSelected: Step {self._selected_step_idx + 1}"
        
        self._sequence_info.configure(text=info_text)

    def _parse_status(self, line: str) -> None:
        # STATUS,js_enabled,servo0,servo1,servo2,servo3,servo4
        parts = line.split(",", 7)
        if len(parts) != 7:
            self._log(f"⚠️ Malformed STATUS: {line}")
            return

        self.sys.joysticks_ok = parts[1] == "1"
        self.sys.servo_attached = [p == "1" for p in parts[2:]]
        
        status_msg = "🎮 Joystick ENABLED" if self.sys.joysticks_ok else "🎮 Joystick DISABLED"
        self._log(status_msg)

    def _refresh_ui(self) -> None:
        # Update angle displays and sliders
        for key, val in asdict(self.arm).items():
            self._angle_vars[key].set(f"{val}°")
            self._slider_vars[key].set(val)

        # Update servo LEDs
        for idx, (key, _, _) in enumerate(SERVOS_META):
            if idx < len(self.sys.servo_attached):
                attached = self.sys.servo_attached[idx]
                color = "#2ecc71" if attached else "#e74c3c"  # green/red
                self._servo_leds[key].configure(foreground=color)

        self._update_status_widgets()

    def _update_status_widgets(self) -> None:
        # Arduino connection status
        arduino_connected = self.sys.arduino_ok and (self._worker is not None)
        arduino_color = "#2ecc71" if arduino_connected else "#e74c3c"
        self._led_arduino.configure(foreground=arduino_color)
        
        # Connection button
        btn_text = "Disconnect" if arduino_connected else "Connect"
        btn_style = "danger-outline" if arduino_connected else "success-outline"
        self._btn_connect.configure(text=btn_text, bootstyle=btn_style)
        
        # Joystick status
        js_color = "#2ecc71" if self.sys.joysticks_ok else "#e74c3c"
        self._led_js.configure(foreground=js_color)

    # ───────── Utility methods ───────── #

    def _log(self, msg: str) -> None:
        ts = time.strftime("%H:%M:%S")
        self._console.insert("end", f"[{ts}] {msg}\n")
        self._console.see("end")

    def _error(self, msg: str) -> None:
        self._log(f"❌ ERROR: {msg}")
        # Use ttkbootstrap's messagebox for consistent styling
        Messagebox.show_error(
            title="4-DOF Arm Control Error",
            message=msg,
            parent=self,
            alert=True
        )

    # ───────── Window cleanup ───────── #

    def destroy(self) -> None:
        if self._worker is not None:
            self._worker.stop()
            self._worker.join(timeout=2)
        super().destroy()
        sys.exit(0)

# ──────────────────────────────── MAIN ─────────────────────────────── #

def main() -> None:
    app = MainWin()
    app.mainloop()

if __name__ == "__main__":
    main()
