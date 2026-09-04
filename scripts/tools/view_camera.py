#!/usr/bin/env python3
"""Display a Gazebo camera through gzimgsrc and OpenCV."""

import argparse
from collections import deque
import os
from pathlib import Path
import re
import sys
import time


DEFAULT_TOPIC = "/X3/front_camera/image"
DEFAULT_PLUGIN_PATH = Path(__file__).resolve().parents[2] / "gst/gst_gzimgsrc/build"
RECORD_FPS = 30.0
HELP_TEXT = """OpenCV with GStreamer support (Ubuntu)

Install the distribution OpenCV package:
  sudo apt update
  sudo apt install python3-opencv

Verify GStreamer support:
  python3 -c 'import cv2; print(cv2.getBuildInformation())' | grep GStreamer

The result should contain: GStreamer: YES

Trim a recording quickly with FFmpeg:
  ffmpeg -ss 00:00:10 -i input.mp4 -t 00:00:20 -c copy clip.mp4

This starts at 10 seconds and keeps 20 seconds. Stream copying is fast, but
the cut can begin at the nearest keyframe. For a frame-accurate cut, re-encode:
  ffmpeg -ss 00:00:10 -i input.mp4 -t 00:00:20 -c:v libx264 clip.mp4
"""


def build_pipeline(topic: str) -> str:
    if not topic or any(character.isspace() for character in topic) or "!" in topic:
        raise ValueError("topic must be a non-empty GStreamer-safe topic name")
    return (
        f"gzimgsrc topic={topic} ! videoconvert ! video/x-raw,format=BGR "
        "! appsink max-buffers=1 drop=true sync=false"
    )


class RollingFps:
    def __init__(self) -> None:
        self._frames: deque[float] = deque()

    def update(self, timestamp: float) -> float:
        self._frames.append(timestamp)
        while timestamp - self._frames[0] > 1.0:
            self._frames.popleft()
        elapsed = timestamp - self._frames[0]
        return (len(self._frames) - 1) / elapsed if elapsed > 0.0 else 0.0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", default=DEFAULT_TOPIC)
    parser.add_argument("--plugin-path", type=Path, default=DEFAULT_PLUGIN_PATH)
    return parser.parse_args()


class ViewerWindow:
    def __init__(self, topic: str) -> None:
        import tkinter as tk
        from tkinter import filedialog, messagebox

        self._tk = tk
        self._filedialog = filedialog
        self._messagebox = messagebox
        self.root = tk.Tk()
        self.root.title(topic)
        self.closed = False
        self.recording = False
        self.path = tk.StringVar(value=str(Path.cwd() / "camera.mp4"))

        self.image = tk.Label(self.root)
        self.image.grid(row=0, column=0, columnspan=3)
        self.entry = tk.Entry(self.root, textvariable=self.path, width=55)
        self.entry.grid(row=1, column=0, padx=8, pady=8, sticky="ew")
        self.browse = tk.Button(self.root, text="Browse...", command=self._browse)
        self.browse.grid(row=1, column=1, padx=(0, 8), pady=8)
        self.help = tk.Button(
            self.root, text="Help", command=self._show_help
        )
        self.help.grid(row=1, column=2, padx=(0, 8), pady=8)
        self.toggle = tk.Button(self.root, command=self._toggle)
        self.toggle.grid(row=2, column=0, columnspan=3, padx=8, pady=(0, 8), sticky="ew")
        self.root.columnconfigure(0, weight=1)
        self.root.protocol("WM_DELETE_WINDOW", self._close)
        self.root.bind("<Escape>", lambda _event: self._close())
        self.root.bind("q", lambda _event: self._close() if self.root.focus_get() is not self.entry else None)
        self._refresh()

    def _show_help(self) -> None:
        window = self._tk.Toplevel(self.root)
        window.title("Help")
        window.transient(self.root)
        text = self._tk.Text(window, wrap="word", width=80, height=20, padx=8, pady=8)
        text.insert("1.0", HELP_TEXT)
        text.configure(state=self._tk.DISABLED)
        text.pack(fill="both", expand=True)
        self._tk.Button(window, text="Close", command=window.destroy).pack(pady=(0, 8))

    def _browse(self) -> None:
        current = Path(self.path.get()).expanduser()
        selected = self._filedialog.asksaveasfilename(
            defaultextension=".mp4",
            filetypes=(("MP4 video", "*.mp4"), ("All files", "*.*")),
            initialdir=current.parent if current.parent.is_dir() else Path.cwd(),
            initialfile=current.name,
        )
        if selected:
            self.path.set(selected)

    def _toggle(self) -> None:
        if not self.recording:
            path = Path(self.path.get()).expanduser()
            if not self.path.get().strip() or not path.parent.is_dir():
                self._messagebox.showerror("Recording", "Choose a file in an existing directory.")
                return
        self.recording = not self.recording
        self._refresh()

    def _refresh(self) -> None:
        state = self._tk.DISABLED if self.recording else self._tk.NORMAL
        self.entry.configure(state=state)
        self.browse.configure(state=state)
        self.toggle.configure(text="Stop Recording" if self.recording else "Start Recording")

    def _close(self) -> None:
        self.closed = True
        self.root.destroy()

    def update(self) -> bool:
        if self.closed:
            return False
        try:
            self.root.update_idletasks()
            self.root.update()
        except self._tk.TclError:
            self.closed = True
            return False
        return True

    def show(self, frame, cv2) -> None:
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        height, width = rgb.shape[:2]
        ppm = f"P6\n{width} {height}\n255\n".encode() + rgb.tobytes()
        self._photo = self._tk.PhotoImage(data=ppm, format="PPM")
        self.image.configure(image=self._photo)

    def recording_path(self) -> Path:
        return Path(self.path.get()).expanduser()


def run(args: argparse.Namespace) -> int:
    plugin_path = args.plugin_path.expanduser().resolve()
    if not plugin_path.is_dir():
        raise RuntimeError(f"GStreamer plugin directory does not exist: {plugin_path}")
    os.environ["GST_PLUGIN_PATH"] = os.pathsep.join(
        filter(None, (str(plugin_path), os.environ.get("GST_PLUGIN_PATH")))
    )

    try:
        import cv2
    except ImportError as exc:
        raise RuntimeError("OpenCV is not installed in this Python environment") from exc
    if not re.search(r"GStreamer\s*:\s*YES", cv2.getBuildInformation()):
        raise RuntimeError("OpenCV was built without GStreamer support")

    capture = cv2.VideoCapture(build_pipeline(args.topic), cv2.CAP_GSTREAMER)
    if not capture.isOpened():
        raise RuntimeError("failed to open the gzimgsrc GStreamer pipeline")

    writer = None
    fps = RollingFps()
    window = ViewerWindow(args.topic)
    try:
        while True:
            ok, frame = capture.read()
            if not ok:
                raise RuntimeError("camera stream ended or produced no frame")

            if window.recording:
                if writer is None:
                    record_path = window.recording_path()
                    writer = cv2.VideoWriter(
                        str(record_path),
                        cv2.VideoWriter_fourcc(*"mp4v"),
                        RECORD_FPS,
                        (frame.shape[1], frame.shape[0]),
                    )
                    if not writer.isOpened():
                        raise RuntimeError(f"failed to open recording: {record_path}")
                writer.write(frame)
            elif writer is not None:
                writer.release()
                writer = None

            display = frame.copy()
            current_fps = fps.update(time.monotonic())
            cv2.putText(display, f"FPS: {current_fps:.1f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                        0.8, (0, 255, 0), 2, cv2.LINE_AA)
            window.show(display, cv2)
            if not window.update():
                return 0
    finally:
        capture.release()
        if writer is not None:
            writer.release()
        if not window.closed:
            window.root.destroy()


def main() -> int:
    try:
        return run(parse_args())
    except (RuntimeError, ValueError) as exc:
        print(f"Camera viewer failed: {exc}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
