"""Lightweight system resource monitor using /proc.

No external dependencies (psutil not required).
Reads /proc/meminfo and /proc/stat for memory and CPU usage.
"""

import threading
import time
from collections import deque
from typing import Optional


class SystemSnapshot:
    __slots__ = (
        "timestamp",
        "mem_total_mb",
        "mem_used_mb",
        "mem_available_mb",
        "mem_percent",
        "cpu_percent",
    )

    def __init__(self):
        self.timestamp = 0.0
        self.mem_total_mb = 0.0
        self.mem_used_mb = 0.0
        self.mem_available_mb = 0.0
        self.mem_percent = 0.0
        self.cpu_percent = 0.0

    def __repr__(self):
        return (
            f"Memory: {self.mem_used_mb:.0f}/{self.mem_total_mb:.0f} MB "
            f"({self.mem_percent:.1f}%) | CPU: {self.cpu_percent:.1f}%"
        )


def _read_meminfo() -> dict:
    info = {}
    with open("/proc/meminfo", "r") as f:
        for line in f:
            parts = line.split(":")
            if len(parts) == 2:
                key = parts[0].strip()
                val = parts[1].strip().split()[0]
                try:
                    info[key] = int(val)  # kB
                except ValueError:
                    pass
    return info


def _read_cpu_times() -> tuple:
    """Return (idle, total) jiffies from /proc/stat."""
    with open("/proc/stat", "r") as f:
        line = f.readline()
    parts = line.split()
    # cpu  user nice system idle iowait irq softirq steal guest guest_nice
    vals = [int(x) for x in parts[1:]]
    idle = vals[3] + (vals[4] if len(vals) > 4 else 0)
    total = sum(vals)
    return idle, total


class SystemMonitor:
    """Background thread that samples CPU and memory at a fixed interval."""

    def __init__(self, interval_sec: float = 1.0, history_size: int = 300):
        self.interval = interval_sec
        self.history: deque = deque(maxlen=history_size)
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._prev_idle = 0
        self._prev_total = 0
        self._peak_mem_mb = 0.0
        self._baseline: Optional[SystemSnapshot] = None

    def start(self):
        """Start background monitoring thread."""
        # initial CPU reading for delta calculation
        self._prev_idle, self._prev_total = _read_cpu_times()
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop_event.set()
        if self._thread:
            self._thread.join(timeout=3.0)

    def snapshot(self) -> SystemSnapshot:
        """Take a single snapshot (can also be called externally)."""
        s = SystemSnapshot()
        s.timestamp = time.time()

        mem = _read_meminfo()
        total = mem.get("MemTotal", 0)
        available = mem.get("MemAvailable", 0)
        s.mem_total_mb = total / 1024.0
        s.mem_available_mb = available / 1024.0
        s.mem_used_mb = s.mem_total_mb - s.mem_available_mb
        s.mem_percent = (s.mem_used_mb / s.mem_total_mb * 100.0) if s.mem_total_mb > 0 else 0.0

        idle, total_j = _read_cpu_times()
        d_idle = idle - self._prev_idle
        d_total = total_j - self._prev_total
        if d_total > 0:
            s.cpu_percent = (1.0 - d_idle / d_total) * 100.0
        self._prev_idle = idle
        self._prev_total = total_j

        if s.mem_used_mb > self._peak_mem_mb:
            self._peak_mem_mb = s.mem_used_mb

        return s

    def _run(self):
        while not self._stop_event.is_set():
            s = self.snapshot()
            self.history.append(s)
            self._stop_event.wait(self.interval)

    def get_latest(self) -> Optional[SystemSnapshot]:
        if self.history:
            return self.history[-1]
        return None

    def get_peak_mem_mb(self) -> float:
        return self._peak_mem_mb

    def set_baseline(self):
        self._baseline = self.snapshot()

    def get_baseline(self) -> Optional[SystemSnapshot]:
        return self._baseline

    def get_summary(self) -> dict:
        if not self.history:
            return {}
        cpus = [s.cpu_percent for s in self.history]
        mems = [s.mem_used_mb for s in self.history]
        return {
            "samples": len(self.history),
            "cpu_mean": sum(cpus) / len(cpus),
            "cpu_max": max(cpus),
            "mem_mean_mb": sum(mems) / len(mems),
            "mem_peak_mb": max(mems),
            "mem_total_mb": self.history[-1].mem_total_mb,
        }
