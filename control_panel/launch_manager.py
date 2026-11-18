#!/usr/bin/env python3
"""Small graphical controller for ROS 2 launch processes."""

from __future__ import annotations

import os
import queue
import shlex
import signal
import subprocess
import threading
import tkinter as tk
from tkinter import messagebox, ttk


class LaunchManager(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.title("ROS 2 Launch Manager")
        self.minsize(720, 480)

        self.package = tk.StringVar()
        self.launch_file = tk.StringVar()
        self.arguments = tk.StringVar()
        self.rviz_argument = tk.StringVar(value="rviz")
        self.rviz_enabled = tk.BooleanVar(value=True)
        self.status = tk.StringVar(value="Gestopt")

        self.process: subprocess.Popen[str] | None = None
        self.output_queue: queue.Queue[str] = queue.Queue()
        self.stopping = False

        self._build_ui()
        self.protocol("WM_DELETE_WINDOW", self._close)
        self.after(100, self._update)

    def _build_ui(self) -> None:
        form = ttk.Frame(self, padding=12)
        form.pack(fill=tk.X)
        form.columnconfigure(1, weight=1)

        ttk.Label(form, text="ROS 2-pakket").grid(row=0, column=0, sticky=tk.W, padx=(0, 8), pady=4)
        ttk.Entry(form, textvariable=self.package).grid(row=0, column=1, sticky=tk.EW, pady=4)

        ttk.Label(form, text="Launch-bestand").grid(row=1, column=0, sticky=tk.W, padx=(0, 8), pady=4)
        ttk.Entry(form, textvariable=self.launch_file).grid(row=1, column=1, sticky=tk.EW, pady=4)

        ttk.Label(form, text="Extra launch-argumenten").grid(
            row=2, column=0, sticky=tk.W, padx=(0, 8), pady=4
        )
        ttk.Entry(form, textvariable=self.arguments).grid(row=2, column=1, sticky=tk.EW, pady=4)

        rviz = ttk.Frame(form)
        rviz.grid(row=3, column=0, columnspan=2, sticky=tk.W, pady=(8, 4))
        ttk.Checkbutton(rviz, text="RViz inschakelen", variable=self.rviz_enabled).pack(side=tk.LEFT)
        ttk.Label(rviz, text="RViz-argument:").pack(side=tk.LEFT, padx=(16, 6))
        ttk.Entry(rviz, textvariable=self.rviz_argument, width=20).pack(side=tk.LEFT)

        actions = ttk.Frame(self, padding=(12, 0, 12, 8))
        actions.pack(fill=tk.X)
        self.start_button = ttk.Button(actions, text="Start", command=self._start)
        self.start_button.pack(side=tk.LEFT)
        self.stop_button = ttk.Button(actions, text="Stop", command=self._stop, state=tk.DISABLED)
        self.stop_button.pack(side=tk.LEFT, padx=(8, 16))
        ttk.Label(actions, textvariable=self.status).pack(side=tk.LEFT)

        ttk.Label(self, text="Uitvoer", padding=(12, 0, 12, 4)).pack(anchor=tk.W)
        self.output = tk.Text(self, height=16, wrap=tk.WORD, state=tk.DISABLED)
        self.output.pack(fill=tk.BOTH, expand=True, padx=12, pady=(0, 12))

    def _command(self) -> list[str]:
        package = self.package.get().strip()
        launch_file = self.launch_file.get().strip()
        if not package or not launch_file:
            raise ValueError("Vul zowel het ROS 2-pakket als het launch-bestand in.")

        command = ["ros2", "launch", package, launch_file]
        command.extend(shlex.split(self.arguments.get()))

        rviz_argument = self.rviz_argument.get().strip()
        if rviz_argument:
            command.append(f"{rviz_argument}:={'true' if self.rviz_enabled.get() else 'false'}")
        return command

    def _start(self) -> None:
        try:
            command = self._command()
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
            messagebox.showerror("ROS 2 starten mislukt", str(exc), parent=self)
            return

        self.stopping = False
        self.status.set("Gestart")
        self.start_button.configure(state=tk.DISABLED)
        self.stop_button.configure(state=tk.NORMAL)
        self._append("$ " + shlex.join(command))
        threading.Thread(target=self._read_output, args=(self.process,), daemon=True).start()

    def _read_output(self, process: subprocess.Popen[str]) -> None:
        assert process.stdout is not None
        for line in process.stdout:
            self.output_queue.put(line)

    def _stop(self) -> None:
        process = self.process
        if process is None or process.poll() is not None or self.stopping:
            return

        self.stopping = True
        self.status.set("Bezig met stoppen…")
        self.stop_button.configure(state=tk.DISABLED)
        threading.Thread(target=self._stop_process, args=(process,), daemon=True).start()

    @staticmethod
    def _stop_process(process: subprocess.Popen[str]) -> None:
        try:
            os.killpg(process.pid, signal.SIGINT)
        except ProcessLookupError:
            return

        try:
            process.wait(timeout=8)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                process.wait()
            except ProcessLookupError:
                pass

    def _update(self) -> None:
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
            self.status.set(f"Gestopt (exitcode {return_code})")
            self.start_button.configure(state=tk.NORMAL)
            self.stop_button.configure(state=tk.DISABLED)

        self.after(100, self._update)

    def _append(self, text: str) -> None:
        self.output.configure(state=tk.NORMAL)
        self.output.insert(tk.END, text + "\n")
        self.output.see(tk.END)
        self.output.configure(state=tk.DISABLED)

    def _close(self) -> None:
        self._stop()
        self.destroy()


if __name__ == "__main__":
    LaunchManager().mainloop()
