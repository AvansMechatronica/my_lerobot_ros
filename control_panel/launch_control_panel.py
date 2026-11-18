#!/usr/bin/env python3
"""ROS 2 Launch Control Panel - Overzicht van meerdere launches met start/stop knoppen."""

from __future__ import annotations

import os
import queue
import shlex
import signal
import subprocess
import threading
import tkinter as tk
from dataclasses import dataclass
from pathlib import Path
from tkinter import messagebox, ttk

import yaml


@dataclass
class LaunchConfig:
    """Configuration for a single launch."""

    name: str
    package: str
    launch_file: str
    arguments: str = ""
    rviz_argument: str = ""
    rviz_enabled: bool = False
    description: str = ""

    def command(self) -> list[str]:
        """Build the ros2 launch command."""
        command = ["ros2", "launch", self.package, self.launch_file]
        if self.arguments:
            command.extend(shlex.split(self.arguments))
        if self.rviz_argument:
            command.append(f"{self.rviz_argument}:={'true' if self.rviz_enabled else 'false'}")
        return command


class LaunchButton(ttk.Frame):
    """A single launch button with status indicator."""

    def __init__(self, parent, config: LaunchConfig, manager: LaunchControlPanel) -> None:
        super().__init__(parent, relief=tk.RAISED, borderwidth=1)
        self.config = config
        self.manager = manager
        self.process: subprocess.Popen[str] | None = None
        self.output_queue: queue.Queue[str] = queue.Queue()
        self.stopping = False

        self._build_ui()
        self._update_rviz_button()

    def _build_ui(self) -> None:
        """Build the UI."""
        # Header with status
        header = ttk.Frame(self)
        header.pack(fill=tk.X, padx=8, pady=(8, 4))

        # Status indicator
        self.status_label = tk.Label(header, text="●", fg="gray", font=("Arial", 16))
        self.status_label.pack(side=tk.LEFT, padx=(0, 8))

        # Title and description
        title_frame = ttk.Frame(header)
        title_frame.pack(side=tk.LEFT, fill=tk.X, expand=True)
        ttk.Label(title_frame, text=self.config.name, font=("Arial", 10, "bold")).pack(anchor=tk.W)
        if self.config.description:
            ttk.Label(title_frame, text=self.config.description, font=("Arial", 8), foreground="gray").pack(
                anchor=tk.W
            )

        # Buttons
        buttons = ttk.Frame(header)
        buttons.pack(side=tk.RIGHT)

        # RViz toggle button (only if launch supports it)
        if self.config.rviz_argument:
            self.rviz_btn = ttk.Button(buttons, text="📺 RViz: AAN", command=self._toggle_rviz, width=12)
            self.rviz_btn.pack(side=tk.LEFT, padx=(0, 4))

        self.start_btn = ttk.Button(buttons, text="▶ Start", command=self._start, width=8)
        self.start_btn.pack(side=tk.LEFT, padx=(4, 2))
        self.stop_btn = ttk.Button(buttons, text="⏹ Stop", command=self._stop, width=8, state=tk.DISABLED)
        self.stop_btn.pack(side=tk.LEFT, padx=(2, 0))

        # Output area
        output_frame = ttk.Frame(self)
        output_frame.pack(fill=tk.BOTH, expand=True, padx=8, pady=(0, 8))

        scrollbar = ttk.Scrollbar(output_frame)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

        self.output = tk.Text(
            output_frame,
            height=4,
            wrap=tk.WORD,
            state=tk.DISABLED,
            yscrollcommand=scrollbar.set,
            font=("Courier", 8),
        )
        self.output.pack(fill=tk.BOTH, expand=True)
        scrollbar.config(command=self.output.yview)

    def _status(self, text: str, color: str) -> None:
        """Update status indicator."""
        self.status_label.config(text="●", fg=color)
        self.status_label.after(0)

    def _update_rviz_button(self) -> None:
        """Update RViz button text based on current state."""
        if hasattr(self, 'rviz_btn'):
            state = "AAN" if self.config.rviz_enabled else "UIT"
            self.rviz_btn.config(text=f"📺 RViz: {state}")

    def _toggle_rviz(self) -> None:
        """Toggle RViz on/off and restart the launch."""
        if self.process is None or self.process.poll() is not None:
            messagebox.showinfo("Info", "Start eerst de launch voordat je RViz kan toggelen.", parent=self.manager)
            return

        # Stop the current process
        self._append("[RViz toggle: Stopt launch om RViz aan te passen…]")
        self._stop()

        # Schedule restart after a short delay
        def restart_with_rviz_toggle() -> None:
            # Wait for process to actually stop
            if self.process is not None:
                try:
                    self.process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    pass

            # Toggle RViz state
            self.config.rviz_enabled = not self.config.rviz_enabled
            self._update_rviz_button()

            # Restart
            self._append(f"[RViz toggle: Herstart met RViz={'AAN' if self.config.rviz_enabled else 'UIT'}…]")
            self._start()

        threading.Thread(target=restart_with_rviz_toggle, daemon=True).start()

    def _append(self, text: str) -> None:
        """Add text to output."""
        self.output.configure(state=tk.NORMAL)
        self.output.insert(tk.END, text + "\n")
        self.output.see(tk.END)
        self.output.configure(state=tk.DISABLED)

    def _start(self) -> None:
        """Start the launch process."""
        try:
            command = self.config.command()
            self.process = subprocess.Popen(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                encoding="utf-8",
                errors="replace",
                bufsize=1,
                start_new_session=True,
            )
        except (ValueError, OSError) as exc:
            messagebox.showerror(f"{self.config.name} starten mislukt", str(exc), parent=self.manager)
            return

        self.stopping = False
        self._status("Gestart", "green")
        self.start_btn.configure(state=tk.DISABLED)
        self.stop_btn.configure(state=tk.NORMAL)
        if hasattr(self, 'rviz_btn'):
            self.rviz_btn.configure(state=tk.NORMAL)
        self._append("$ " + shlex.join(command))
        self.manager.register_process(self)
        threading.Thread(target=self._read_output, daemon=True).start()

    def _read_output(self) -> None:
        """Read output from the process."""
        if self.process is None or self.process.stdout is None:
            return
        for line in self.process.stdout:
            self.output_queue.put(line)

    def _stop(self) -> None:
        """Stop the launch process."""
        if self.process is None or self.process.poll() is not None or self.stopping:
            return

        self.stopping = True
        self._status("Stopt…", "orange")
        self.stop_btn.configure(state=tk.DISABLED)
        if hasattr(self, 'rviz_btn'):
            self.rviz_btn.configure(state=tk.DISABLED)
        threading.Thread(target=self._stop_process, daemon=True).start()

    def _stop_process(self) -> None:
        """Stop the process gracefully."""
        if self.process is None:
            return
        try:
            os.killpg(self.process.pid, signal.SIGINT)
        except ProcessLookupError:
            return

        try:
            self.process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(self.process.pid, signal.SIGTERM)
                self.process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(self.process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                self.process.wait()
            except ProcessLookupError:
                pass

    def update_status(self) -> None:
        """Update process status and output."""
        while True:
            try:
                line = self.output_queue.get_nowait()
            except queue.Empty:
                break
            self._append(line.rstrip("\n"))

        if self.process is not None and self.process.poll() is not None:
            return_code = self.process.returncode
            self.process = None
            self.stopping = False
            self._status("Gestopt", "red")
            self.start_btn.configure(state=tk.NORMAL)
            self.stop_btn.configure(state=tk.DISABLED)
            if hasattr(self, 'rviz_btn'):
                self.rviz_btn.configure(state=tk.DISABLED)
            self._append(f"[Exitcode: {return_code}]")


class LaunchControlPanel(tk.Tk):
    """Main application window."""

    def __init__(self, config_file: Path | None = None) -> None:
        super().__init__()
        self.title("ROS 2 Launch Control Panel")
        self.geometry("900x700")

        self.config_file = config_file or Path(__file__).parent / "launches.yaml"
        self.buttons: list[LaunchButton] = []

        self._build_ui()
        self.protocol("WM_DELETE_WINDOW", self._close)
        self.after(100, self._update)

    def _build_ui(self) -> None:
        """Build the UI."""
        # Title
        ttk.Label(self, text="ROS 2 Launch Control Panel", font=("Arial", 14, "bold")).pack(
            fill=tk.X, padx=12, pady=(12, 4)
        )
        ttk.Label(self, text="Overzicht van launches - Klik op Start/Stop", foreground="gray").pack(
            fill=tk.X, padx=12, pady=(0, 12)
        )

        # Launches frame with scrollbar
        canvas_frame = ttk.Frame(self)
        canvas_frame.pack(fill=tk.BOTH, expand=True, padx=12, pady=(0, 12))

        canvas = tk.Canvas(canvas_frame, highlightthickness=0, bg="white")
        scrollbar = ttk.Scrollbar(canvas_frame, orient=tk.VERTICAL, command=canvas.yview)
        scrollable_frame = ttk.Frame(canvas)

        scrollable_frame.bind(
            "<Configure>", lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )

        canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)

        canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

        # Load and create launch buttons
        self._load_and_create_buttons(scrollable_frame)

    def _load_and_create_buttons(self, parent: tk.Widget) -> None:
        """Load launches from config and create buttons."""
        if not self.config_file.exists():
            messagebox.showerror(
                "Config niet gevonden",
                f"Launches.yaml niet gevonden op: {self.config_file}",
                parent=self,
            )
            return

        try:
            with open(self.config_file) as f:
                data = yaml.safe_load(f)
        except Exception as exc:
            messagebox.showerror("Config fout", f"Fout bij laden van launches.yaml:\n{exc}", parent=self)
            return

        launches = data.get("launches", [])
        for launch_data in launches:
            config = LaunchConfig(**launch_data)
            button = LaunchButton(parent, config, self)
            button.pack(fill=tk.X, pady=(0, 8))
            self.buttons.append(button)

    def register_process(self, button: LaunchButton) -> None:
        """Register a process to be managed."""
        # Stop other processes of the same type if needed
        pass

    def _update(self) -> None:
        """Update all buttons."""
        for button in self.buttons:
            button.update_status()
        self.after(100, self._update)

    def _close(self) -> None:
        """Close the application and stop all processes."""
        for button in self.buttons:
            if button.process is not None and button.process.poll() is None:
                button._stop()
        self.after(1000, self.destroy)


if __name__ == "__main__":
    LaunchControlPanel().mainloop()
