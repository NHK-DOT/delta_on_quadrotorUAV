from __future__ import annotations

import queue
import threading
import tkinter as tk
from pathlib import Path
from tkinter import messagebox, ttk
import shutil

from .config import (
    ServoProfile,
    build_updated_config,
    ensure_initial_snapshot,
    initial_snapshot_path,
    load_config,
    save_config,
)
from .mapping import ServoMapping
from .service import LX225Service, ServoSnapshot


BG = "#060c17"
PANEL = "#0d1524"
PANEL_ALT = "#101b2f"
BORDER = "#16314f"
ACCENT = "#38bdf8"
ACCENT_ALT = "#22d3ee"
TEXT = "#e2f3ff"
MUTED = "#7aa6c7"
OK = "#34d399"
WARN = "#f59e0b"
ERROR = "#fb7185"
LOG_BG = "#040914"


class LX225ToolGUI:
    def __init__(self, root: tk.Tk, config_path: Path) -> None:
        self.root = root
        self.config_path = Path(config_path).resolve()
        self.initial_config_path = ensure_initial_snapshot(self.config_path)
        self.cfg = load_config(self.config_path)
        self.log_queue: queue.Queue[tuple[str, str]] = queue.Queue()
        self._busy = False
        self.last_snapshot: ServoSnapshot | None = None
        self.last_scan: dict[int, int] = {}
        self.multi_realtime_enabled = False
        self.multi_realtime_running = False
        self.multi_realtime_job: str | None = None
        self.multi_realtime_service: LX225Service | None = None
        self.multi_realtime_interval_ms = 200
        self.multi_realtime_timeout_s = 0.15
        self._mapping_text_cache = ""
        self._snapshot_text_cache = ""
        self._last_multi_live_signature: tuple[tuple[tuple[int, int | None], ...], tuple[str, ...]] | None = None
        self._last_multi_live_focus: tuple[int, int] | None = None

        default_profile = self._default_profile()
        self.active_servo_id = tk.StringVar(value=str(default_profile.id))
        self.profile_name = tk.StringVar(value=default_profile.name)

        self.current_raw_value = tk.StringVar(value="--")
        self.current_coord_existing_value = tk.StringVar(value="--")
        self.current_coord_draft_value = tk.StringVar(value="--")
        self.detected_ids_value = tk.StringVar(value="--")
        self.driver_mode_value = tk.StringVar(value="simple 0x15 read")
        self.multi_realtime_button_text = tk.StringVar(value="开启多舵机实时读数")
        self.multi_realtime_status_text = tk.StringVar(value="实时读取：关闭")

        self.edit_raw_min = tk.StringVar()
        self.edit_raw_max = tk.StringVar()
        self.edit_coord_at_raw_min = tk.StringVar()
        self.edit_coord_at_raw_max = tk.StringVar()
        self.edit_position_step = tk.StringVar()

        self.preview_raw_value = tk.StringVar(value="")
        self.anchor_coord_value = tk.StringVar(value="1000")
        self.anchor_span_value = tk.StringVar(value="2000")
        self.coord_value = tk.StringVar(value="")
        self.raw_value = tk.StringVar(value="")

        self._setup_window()
        self._setup_style()
        self._build_layout()
        self._load_profile_into_editor()
        self._refresh_mapping_preview()
        self._poll_log_queue()
        self._log("info", "GUI 已启动。主流程只围绕 raw 读取和本地映射，不发送任何运动命令。")
        self._log("info", f"初始配置快照：{self.initial_config_path}")

    def _default_profile(self) -> ServoProfile:
        return next(iter(self.cfg.servos.values()))

    def _setup_window(self) -> None:
        self.root.title("LX225 Setup Console v2026-04-26 blueblack")
        self.root.geometry("1420x960")
        self.root.minsize(1240, 820)
        self.root.configure(bg=BG)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    def _setup_style(self) -> None:
        style = ttk.Style(self.root)
        style.theme_use("clam")
        style.configure(".", background=BG, foreground=TEXT, fieldbackground=PANEL)
        style.configure("Page.TFrame", background=BG)
        style.configure("Panel.TFrame", background=PANEL)
        style.configure("PanelAlt.TFrame", background=PANEL_ALT)
        style.configure("TLabelframe", background=PANEL, foreground=TEXT, bordercolor=BORDER, relief="solid")
        style.configure("TLabelframe.Label", background=PANEL, foreground=TEXT, font=("Microsoft YaHei UI", 11, "bold"))
        style.configure("Title.TLabel", background=BG, foreground=TEXT, font=("Consolas", 22, "bold"))
        style.configure("Hint.TLabel", background=BG, foreground=MUTED, font=("Microsoft YaHei UI", 10))
        style.configure("PanelHint.TLabel", background=PANEL, foreground=MUTED, font=("Microsoft YaHei UI", 10))
        style.configure("InlineHint.TLabel", background=PANEL, foreground=MUTED, font=("Microsoft YaHei UI", 9))
        style.configure("InlineAltHint.TLabel", background=PANEL_ALT, foreground=MUTED, font=("Microsoft YaHei UI", 9))
        style.configure("Key.TLabel", background=PANEL, foreground=MUTED, font=("Consolas", 10))
        style.configure("Value.TLabel", background=PANEL, foreground=TEXT, font=("Consolas", 11))
        style.configure("HeroKey.TLabel", background=PANEL_ALT, foreground=MUTED, font=("Consolas", 11))
        style.configure("HeroValue.TLabel", background=PANEL_ALT, foreground=ACCENT, font=("Consolas", 28, "bold"))
        style.configure("StatusValue.TLabel", background=PANEL_ALT, foreground=TEXT, font=("Consolas", 18, "bold"))
        style.configure("TEntry", padding=7)
        style.configure("TCombobox", padding=5)
        style.configure(
            "Action.TButton",
            background=ACCENT,
            foreground="#02111d",
            bordercolor=ACCENT,
            padding=(12, 8),
            focusthickness=0,
            font=("Microsoft YaHei UI", 10, "bold"),
        )
        style.map("Action.TButton", background=[("active", ACCENT_ALT), ("disabled", "#34556a")])
        style.configure(
            "Soft.TButton",
            background="#12233b",
            foreground=TEXT,
            bordercolor=BORDER,
            padding=(10, 7),
            focusthickness=0,
        )
        style.map("Soft.TButton", background=[("active", "#17304d")])

    def _build_layout(self) -> None:
        outer = ttk.Frame(self.root, style="Page.TFrame")
        outer.pack(fill=tk.BOTH, expand=True)
        outer.columnconfigure(0, weight=1)
        outer.rowconfigure(0, weight=1)

        self.canvas = tk.Canvas(outer, bg=BG, highlightthickness=0, borderwidth=0)
        self.canvas.grid(row=0, column=0, sticky="nsew")
        scrollbar = ttk.Scrollbar(outer, orient="vertical", command=self.canvas.yview)
        scrollbar.grid(row=0, column=1, sticky="ns")
        self.canvas.configure(yscrollcommand=scrollbar.set)

        self.scroll_content = ttk.Frame(self.canvas, style="Page.TFrame", padding=18)
        self.scroll_content.columnconfigure(0, weight=1)
        self._canvas_window = self.canvas.create_window((0, 0), window=self.scroll_content, anchor="nw")
        self.canvas.bind("<Configure>", self._on_canvas_configure)
        self.scroll_content.bind("<Configure>", self._on_frame_configure)
        self._bind_mousewheel()

        header = ttk.Frame(self.scroll_content, style="Page.TFrame")
        header.grid(row=0, column=0, sticky="ew")
        header.columnconfigure(0, weight=1)
        ttk.Label(header, text="LX225 BLUEBLACK RAW CONSOLE", style="Title.TLabel").grid(row=0, column=0, sticky="w")
        ttk.Label(
            header,
            text="收口后的主流程：扫描单舵机 raw -> 固定目标 ID -> 读取当前 raw -> 用当前 raw 生成你自己的新坐标映射。",
            style="Hint.TLabel",
        ).grid(row=1, column=0, sticky="w", pady=(4, 0))

        top = ttk.Frame(self.scroll_content, style="Page.TFrame")
        top.grid(row=1, column=0, sticky="ew", pady=(16, 12))
        top.columnconfigure(0, weight=5)
        top.columnconfigure(1, weight=4)
        self._build_hardware_card(top).grid(row=0, column=0, sticky="nsew", padx=(0, 8))
        self._build_live_card(top).grid(row=0, column=1, sticky="nsew", padx=(8, 0))

        middle = ttk.Frame(self.scroll_content, style="Page.TFrame")
        middle.grid(row=2, column=0, sticky="ew")
        middle.columnconfigure(0, weight=1)
        self._build_mapping_card(middle).grid(row=0, column=0, sticky="nsew")

        bottom = ttk.Frame(self.scroll_content, style="Page.TFrame")
        bottom.grid(row=3, column=0, sticky="ew", pady=(12, 0))
        bottom.columnconfigure(0, weight=1)
        bottom.columnconfigure(1, weight=1)
        self._build_snapshot_card(bottom).grid(row=0, column=0, sticky="nsew", padx=(0, 8))
        self._build_status_note_card(bottom).grid(row=0, column=1, sticky="nsew", padx=(8, 0))

        self._build_log_card(self.scroll_content).grid(row=4, column=0, sticky="nsew", pady=(12, 0))

    def _build_card(self, parent: ttk.Frame, title: str, padding: int = 14) -> ttk.Frame:
        return ttk.LabelFrame(parent, text=title, padding=padding)

    def _build_hardware_card(self, parent: ttk.Frame) -> ttk.Frame:
        frame = self._build_card(parent, "硬件读链路")
        frame.columnconfigure(1, weight=1)

        serial_text = (
            f"port={self.cfg.serial.port}    baudrate={self.cfg.serial.baudrate}    "
            f"timeout={self.cfg.serial.timeout:.2f}s    connect_delay={self.cfg.serial.connect_delay:.2f}s"
        )
        ttk.Label(frame, text="串口参数", style="Key.TLabel").grid(row=0, column=0, sticky="w")
        ttk.Label(frame, text=serial_text, style="Value.TLabel").grid(row=0, column=1, columnspan=4, sticky="w", padx=(10, 0))

        ttk.Label(frame, text="当前舵机 ID", style="Key.TLabel").grid(row=1, column=0, sticky="w", pady=(12, 0))
        ttk.Entry(frame, textvariable=self.active_servo_id, width=16).grid(row=1, column=1, sticky="w", pady=(12, 0))
        ttk.Button(
            frame,
            text="扫描单舵机 raw",
            style="Action.TButton",
            command=lambda: self._safe_ui_action("扫描单舵机 raw", self._scan_single_servo_raw),
        ).grid(
            row=1, column=2, sticky="w", padx=(12, 0), pady=(12, 0)
        )
        ttk.Button(
            frame,
            text="读取当前 raw",
            style="Soft.TButton",
            command=lambda: self._safe_ui_action("读取当前 raw", self._read_snapshot),
        ).grid(
            row=1, column=3, sticky="w", padx=(12, 0), pady=(12, 0)
        )
        ttk.Button(
            frame,
            text="读取配置内多舵机 raw",
            style="Soft.TButton",
            command=lambda: self._safe_ui_action("读取配置内多舵机 raw", self._read_configured_multi_raw),
        ).grid(
            row=1, column=4, sticky="w", padx=(12, 0), pady=(12, 0)
        )

        ttk.Label(frame, text="当前映射舵机", style="Key.TLabel").grid(row=2, column=0, sticky="w", pady=(12, 0))
        self.profile_combo = ttk.Combobox(
            frame,
            textvariable=self.profile_name,
            state="readonly",
            values=list(self.cfg.servos.keys()),
        )
        self.profile_combo.grid(row=2, column=1, sticky="ew", pady=(12, 0))
        self.profile_combo.bind(
            "<<ComboboxSelected>>",
            lambda _event: self._safe_ui_action("切换映射模板", self._load_profile_into_editor),
        )
        ttk.Button(
            frame,
            text="重新加载配置",
            style="Soft.TButton",
            command=lambda: self._safe_ui_action("重新加载配置", self._reload_config),
        ).grid(
            row=2, column=2, sticky="w", padx=(12, 0), pady=(12, 0)
        )


        ttk.Label(frame, text="舵机快捷读取", style="Key.TLabel").grid(row=3, column=0, sticky="nw", pady=(12, 0))
        self.shortcut_frame = ttk.Frame(frame, style="Panel.TFrame")
        self.shortcut_frame.grid(row=3, column=1, columnspan=4, sticky="ew", pady=(12, 0))
        self._rebuild_servo_shortcuts()

        ttk.Label(
            frame,
            text="“当前舵机 ID”是单舵机目标；“当前映射舵机”表示你正在编辑哪只舵机的映射。先选舵机，再读当前 raw，再设锚点。",
            style="PanelHint.TLabel",
        ).grid(row=4, column=0, columnspan=5, sticky="w", pady=(12, 0))
        return frame

    def _build_live_card(self, parent: ttk.Frame) -> ttk.Frame:
        frame = self._build_card(parent, "实时读数")
        frame.columnconfigure(0, weight=1)

        hero = ttk.Frame(frame, style="PanelAlt.TFrame", padding=16)
        hero.grid(row=0, column=0, sticky="ew")
        hero.columnconfigure(0, weight=1)
        hero.columnconfigure(1, weight=1)

        ttk.Label(hero, text="CURRENT RAW", style="HeroKey.TLabel").grid(row=0, column=0, sticky="w")
        ttk.Label(hero, textvariable=self.current_raw_value, style="HeroValue.TLabel").grid(row=1, column=0, sticky="w", pady=(2, 8))
        ttk.Label(hero, text="DETECTED IDS", style="HeroKey.TLabel").grid(row=0, column=1, sticky="w")
        ttk.Label(hero, textvariable=self.detected_ids_value, style="StatusValue.TLabel").grid(row=1, column=1, sticky="w", pady=(2, 8))

        ttk.Label(hero, text="模板坐标值", style="HeroKey.TLabel").grid(row=2, column=0, sticky="w")
        ttk.Label(hero, textvariable=self.current_coord_existing_value, style="StatusValue.TLabel").grid(row=3, column=0, sticky="w")
        ttk.Label(hero, text="草稿坐标值", style="HeroKey.TLabel").grid(row=2, column=1, sticky="w")
        ttk.Label(hero, textvariable=self.current_coord_draft_value, style="StatusValue.TLabel").grid(row=3, column=1, sticky="w")

        details = ttk.Frame(frame, style="Panel.TFrame")
        details.grid(row=1, column=0, sticky="ew", pady=(14, 0))
        ttk.Label(details, text="当前驱动模式", style="Key.TLabel").pack(anchor="w")
        ttk.Label(details, textvariable=self.driver_mode_value, style="Value.TLabel").pack(anchor="w", pady=(2, 0))
        return frame

    def _build_mapping_card(self, parent: ttk.Frame) -> ttk.Frame:
        frame = self._build_card(parent, "自定义坐标映射")
        for idx in range(4):
            frame.columnconfigure(idx, weight=1)

        ttk.Label(
            frame,
            text="coord = 你自己的坐标值，不是官方 0..240°。它可以是实际角度，也可以是你自定义的 0/1000/2000 这类控制坐标。",
            style="PanelHint.TLabel",
        ).grid(row=0, column=0, columnspan=4, sticky="w")

        ttk.Label(frame, text="raw_min", style="Key.TLabel").grid(row=1, column=0, sticky="w", pady=(12, 0))
        ttk.Entry(frame, textvariable=self.edit_raw_min, width=16).grid(row=1, column=1, sticky="w", pady=(12, 0))
        ttk.Label(frame, text="raw_max", style="Key.TLabel").grid(row=1, column=2, sticky="w", pady=(12, 0))
        ttk.Entry(frame, textvariable=self.edit_raw_max, width=16).grid(row=1, column=3, sticky="w", pady=(12, 0))
        ttk.Label(frame, text="这一端的原始 raw 值", style="InlineHint.TLabel").grid(row=2, column=0, columnspan=2, sticky="w")
        ttk.Label(frame, text="另一端的原始 raw 值", style="InlineHint.TLabel").grid(row=2, column=2, columnspan=2, sticky="w")

        ttk.Label(frame, text="coord@raw_min", style="Key.TLabel").grid(row=3, column=0, sticky="w", pady=(10, 0))
        ttk.Entry(frame, textvariable=self.edit_coord_at_raw_min, width=16).grid(row=3, column=1, sticky="w", pady=(10, 0))
        ttk.Label(frame, text="coord@raw_max", style="Key.TLabel").grid(row=3, column=2, sticky="w", pady=(10, 0))
        ttk.Entry(frame, textvariable=self.edit_coord_at_raw_max, width=16).grid(row=3, column=3, sticky="w", pady=(10, 0))
        ttk.Label(frame, text="raw_min 对应的你的坐标值", style="InlineHint.TLabel").grid(row=4, column=0, columnspan=2, sticky="w")
        ttk.Label(frame, text="raw_max 对应的你的坐标值", style="InlineHint.TLabel").grid(row=4, column=2, columnspan=2, sticky="w")

        ttk.Label(frame, text="position_step", style="Key.TLabel").grid(row=5, column=0, sticky="w", pady=(10, 0))
        ttk.Entry(frame, textvariable=self.edit_position_step, width=16).grid(row=5, column=1, sticky="w", pady=(10, 0))
        ttk.Label(frame, text="raw 量化步长。5 表示每 5 个 raw 算一档。", style="InlineHint.TLabel").grid(
            row=5, column=2, columnspan=2, sticky="w", pady=(10, 0)
        )

        anchor = ttk.Frame(frame, style="PanelAlt.TFrame", padding=12)
        anchor.grid(row=6, column=0, columnspan=4, sticky="ew", pady=(14, 0))
        for idx in range(4):
            anchor.columnconfigure(idx, weight=1)

        ttk.Label(anchor, text="锚点 raw", style="Key.TLabel").grid(row=0, column=0, sticky="w")
        ttk.Entry(anchor, textvariable=self.preview_raw_value, width=16).grid(row=0, column=1, sticky="w")
        ttk.Button(
            anchor,
            text="把当前读取值设为锚点",
            style="Soft.TButton",
            command=lambda: self._safe_ui_action("用当前 raw", self._use_current_raw_as_preview),
        ).grid(
            row=0, column=2, sticky="w", padx=(12, 0)
        )
        ttk.Label(anchor, text="把刚读到的当前位置 raw 复制到这里", style="InlineAltHint.TLabel").grid(
            row=0, column=3, sticky="w"
        )

        ttk.Label(anchor, text="锚点坐标值", style="Key.TLabel").grid(row=1, column=0, sticky="w", pady=(10, 0))
        ttk.Entry(anchor, textvariable=self.anchor_coord_value, width=16).grid(row=1, column=1, sticky="w", pady=(10, 0))
        ttk.Label(anchor, text="总坐标跨度", style="Key.TLabel").grid(row=1, column=2, sticky="w", pady=(10, 0))
        ttk.Entry(anchor, textvariable=self.anchor_span_value, width=16).grid(row=1, column=3, sticky="w", pady=(10, 0))
        ttk.Label(anchor, text="你想让当前位置对应成多少", style="InlineAltHint.TLabel").grid(row=2, column=0, columnspan=2, sticky="w")
        ttk.Label(anchor, text="新映射从一端到另一端总共跨多少", style="InlineAltHint.TLabel").grid(row=2, column=2, columnspan=2, sticky="w")

        anchor_buttons = ttk.Frame(anchor, style="PanelAlt.TFrame")
        anchor_buttons.grid(row=3, column=0, columnspan=4, sticky="ew", pady=(12, 0))
        ttk.Button(
            anchor_buttons,
            text="锚点设为新最小端",
            style="Soft.TButton",
            command=lambda: self._safe_ui_action("锚点设为新最小端", self._apply_anchor_as_min),
        ).pack(side=tk.LEFT)
        ttk.Button(
            anchor_buttons,
            text="锚点设为新中点",
            style="Soft.TButton",
            command=lambda: self._safe_ui_action("锚点设为新中点", self._apply_anchor_as_center),
        ).pack(side=tk.LEFT, padx=(10, 0))
        ttk.Button(
            anchor_buttons,
            text="锚点设为新最大端",
            style="Soft.TButton",
            command=lambda: self._safe_ui_action("锚点设为新最大端", self._apply_anchor_as_max),
        ).pack(side=tk.LEFT, padx=(10, 0))

        convert = ttk.Frame(frame, style="Panel.TFrame")
        convert.grid(row=7, column=0, columnspan=4, sticky="ew", pady=(14, 0))
        for idx in range(4):
            convert.columnconfigure(idx, weight=1)
        ttk.Label(convert, text="输入坐标值", style="Key.TLabel").grid(row=0, column=0, sticky="w")
        ttk.Entry(convert, textvariable=self.coord_value, width=16).grid(row=0, column=1, sticky="w")
        ttk.Label(convert, text="输入 raw", style="Key.TLabel").grid(row=0, column=2, sticky="w")
        ttk.Entry(convert, textvariable=self.raw_value, width=16).grid(row=0, column=3, sticky="w")
        ttk.Label(convert, text="这里填你的坐标值", style="InlineHint.TLabel").grid(row=1, column=0, columnspan=2, sticky="w")
        ttk.Label(convert, text="这里填底层 raw 值", style="InlineHint.TLabel").grid(row=1, column=2, columnspan=2, sticky="w")

        convert_buttons = ttk.Frame(frame, style="Panel.TFrame")
        convert_buttons.grid(row=8, column=0, columnspan=4, sticky="ew", pady=(10, 0))
        ttk.Button(
            convert_buttons,
            text="刷新预览",
            style="Action.TButton",
            command=lambda: self._safe_ui_action("刷新预览", self._refresh_mapping_preview),
        ).pack(side=tk.LEFT)
        ttk.Button(
            convert_buttons,
            text="坐标 -> raw",
            style="Soft.TButton",
            command=lambda: self._safe_ui_action("坐标 -> raw", self._coord_to_raw),
        ).pack(side=tk.LEFT, padx=(10, 0))
        ttk.Button(
            convert_buttons,
            text="raw -> 坐标",
            style="Soft.TButton",
            command=lambda: self._safe_ui_action("raw -> 坐标", self._raw_to_coord),
        ).pack(side=tk.LEFT, padx=(10, 0))
        ttk.Button(
            convert_buttons,
            text="恢复当前舵机原始模板",
            style="Soft.TButton",
            command=lambda: self._safe_ui_action("从模板重载", self._load_profile_into_editor),
        ).pack(side=tk.LEFT, padx=(10, 0))
        ttk.Button(
            convert_buttons,
            text="保存当前舵机映射到配置文件",
            style="Action.TButton",
            command=lambda: self._safe_ui_action("保存当前舵机映射到配置文件", self._save_current_profile_mapping),
        ).pack(side=tk.LEFT, padx=(10, 0))
        ttk.Button(
            convert_buttons,
            text="从初始配置恢复整个配置",
            style="Soft.TButton",
            command=lambda: self._safe_ui_action("从初始配置恢复整个配置", self._restore_full_config_from_initial),
        ).pack(side=tk.LEFT, padx=(10, 0))

        mapping_preview = ttk.Frame(frame, style="Panel.TFrame")
        mapping_preview.grid(row=9, column=0, columnspan=4, sticky="nsew", pady=(14, 0))
        mapping_preview.columnconfigure(0, weight=1)
        mapping_preview.rowconfigure(0, weight=1)
        self.mapping_text = tk.Text(
            mapping_preview,
            height=16,
            bg="#07101d",
            fg=TEXT,
            insertbackground=TEXT,
            relief="flat",
            font=("Consolas", 10),
            padx=10,
            pady=10,
        )
        self.mapping_text.grid(row=0, column=0, sticky="nsew")
        mapping_scrollbar = ttk.Scrollbar(mapping_preview, orient="vertical", command=self.mapping_text.yview)
        mapping_scrollbar.grid(row=0, column=1, sticky="ns")
        self.mapping_text.configure(yscrollcommand=mapping_scrollbar.set)
        self._bind_text_scroll(self.mapping_text)
        self.mapping_text.config(state="disabled")
        return frame

    def _build_snapshot_card(self, parent: ttk.Frame) -> ttk.Frame:
        frame = self._build_card(parent, "当前读数明细")
        frame.columnconfigure(0, weight=1)
        toolbar = ttk.Frame(frame, style="Panel.TFrame")
        toolbar.grid(row=0, column=0, sticky="ew", pady=(0, 10))
        ttk.Button(
            toolbar,
            textvariable=self.multi_realtime_button_text,
            style="Action.TButton",
            command=lambda: self._safe_ui_action("切换多舵机实时读数", self._toggle_multi_realtime),
        ).pack(side=tk.LEFT)
        ttk.Label(toolbar, textvariable=self.multi_realtime_status_text, style="Value.TLabel").pack(side=tk.LEFT, padx=(12, 0))

        snapshot_view = ttk.Frame(frame, style="Panel.TFrame")
        snapshot_view.grid(row=1, column=0, sticky="nsew")
        snapshot_view.columnconfigure(0, weight=1)
        snapshot_view.rowconfigure(0, weight=1)
        self.snapshot_text = tk.Text(
            snapshot_view,
            height=12,
            bg="#07101d",
            fg=TEXT,
            insertbackground=TEXT,
            relief="flat",
            font=("Consolas", 10),
            padx=10,
            pady=10,
        )
        self.snapshot_text.grid(row=0, column=0, sticky="nsew")
        snapshot_scrollbar = ttk.Scrollbar(snapshot_view, orient="vertical", command=self.snapshot_text.yview)
        snapshot_scrollbar.grid(row=0, column=1, sticky="ns")
        self.snapshot_text.configure(yscrollcommand=snapshot_scrollbar.set)
        self._bind_text_scroll(self.snapshot_text)
        self._set_snapshot_text(
            "推荐顺序：\n"
            "1. 点击“扫描单舵机 raw”定位当前单舵机。\n"
            "2. 点击“读取当前 raw”刷新当前位置与回退信息。\n"
            "3. 如果要观察多舵机一起对齐，开启“多舵机实时读数”。\n"
            "4. 去“自定义坐标映射”里点“用当前 raw”，再定义新最小端/中点/最大端。"
        )
        return frame

    def _build_status_note_card(self, parent: ttk.Frame) -> ttk.Frame:
        frame = self._build_card(parent, "使用顺序")
        note = "\n".join(
            [
                "1. 先扫单舵机 raw，确认硬件 ID。",
                "2. 再读当前 raw，确认当前位置。",
                "3. 映射模板只负责换算，不等于硬件目标。",
                "4. 在中间区域把当前位置定义成你的新坐标值。",
                "5. 用“坐标 -> raw / raw -> 坐标”反复核对，不发运动命令。",
            ]
        )
        label = tk.Label(
            frame,
            text=note,
            bg=PANEL,
            fg=TEXT,
            justify="left",
            anchor="w",
            font=("Microsoft YaHei UI", 10),
        )
        label.pack(fill=tk.BOTH, expand=True)
        return frame

    def _build_log_card(self, parent: ttk.Frame) -> ttk.Frame:
        frame = self._build_card(parent, "日志")
        frame.columnconfigure(0, weight=1)
        frame.rowconfigure(0, weight=1)

        log_view = ttk.Frame(frame, style="Panel.TFrame")
        log_view.grid(row=0, column=0, sticky="nsew")
        log_view.columnconfigure(0, weight=1)
        log_view.rowconfigure(0, weight=1)
        self.log_text = tk.Text(
            log_view,
            height=14,
            bg=LOG_BG,
            fg=TEXT,
            insertbackground=TEXT,
            relief="flat",
            font=("Consolas", 10),
            padx=10,
            pady=10,
        )
        self.log_text.grid(row=0, column=0, sticky="nsew")
        log_scrollbar = ttk.Scrollbar(log_view, orient="vertical", command=self.log_text.yview)
        log_scrollbar.grid(row=0, column=1, sticky="ns")
        self.log_text.configure(yscrollcommand=log_scrollbar.set)
        self._bind_text_scroll(self.log_text)
        self.log_text.config(state="disabled")
        ttk.Button(frame, text="清空日志", style="Soft.TButton", command=self._clear_log).grid(row=1, column=0, sticky="w", pady=(10, 0))
        return frame

    def _bind_mousewheel(self) -> None:
        self.canvas.bind_all("<MouseWheel>", self._on_mousewheel)
        self.canvas.bind_all("<Button-4>", self._on_mousewheel)
        self.canvas.bind_all("<Button-5>", self._on_mousewheel)

    def _bind_text_scroll(self, widget: tk.Text) -> None:
        widget.bind("<MouseWheel>", lambda event, widget=widget: self._on_text_mousewheel(event, widget))
        widget.bind("<Button-4>", lambda event, widget=widget: self._on_text_mousewheel(event, widget))
        widget.bind("<Button-5>", lambda event, widget=widget: self._on_text_mousewheel(event, widget))

    def _wheel_units_from_event(self, event: tk.Event) -> int | None:
        if hasattr(event, "delta") and event.delta:
            magnitude = max(1, abs(int(event.delta)) // 120)
            return -magnitude if event.delta > 0 else magnitude
        if getattr(event, "num", None) == 4:
            return -1
        if getattr(event, "num", None) == 5:
            return 1
        return None

    def _on_mousewheel(self, event: tk.Event) -> None:
        step = self._wheel_units_from_event(event)
        if step is None:
            return
        self.canvas.yview_scroll(step, "units")

    def _on_text_mousewheel(self, event: tk.Event, widget: tk.Text) -> str:
        step = self._wheel_units_from_event(event)
        if step is None:
            return "break"
        widget.yview_scroll(step, "units")
        return "break"

    def _on_canvas_configure(self, event: tk.Event) -> None:
        self.canvas.itemconfigure(self._canvas_window, width=event.width)

    def _on_frame_configure(self, _event: tk.Event) -> None:
        self.canvas.configure(scrollregion=self.canvas.bbox("all"))

    def _poll_log_queue(self) -> None:
        try:
            while True:
                level, message = self.log_queue.get_nowait()
                self._append_log(level, message)
        except queue.Empty:
            pass
        self.root.after(120, self._poll_log_queue)

    def _append_log(self, level: str, message: str) -> None:
        color = {"info": TEXT, "ok": OK, "warn": WARN, "error": ERROR}.get(level, TEXT)
        self.log_text.config(state="normal")
        tag = f"tag_{level}"
        self.log_text.tag_configure(tag, foreground=color)
        self.log_text.insert("end", message + "\n", tag)
        self.log_text.see("end")
        self.log_text.config(state="disabled")

    def _clear_log(self) -> None:
        self.log_text.config(state="normal")
        self.log_text.delete("1.0", "end")
        self.log_text.config(state="disabled")

    def _log(self, level: str, message: str) -> None:
        self.log_queue.put((level, message))

    def _safe_ui_action(self, title: str, fn) -> None:
        try:
            fn()
        except Exception as exc:
            self._log("error", f"[{title}] 失败: {exc}")
            messagebox.showerror("操作失败", f"{title}\n\n{exc}")

    def _set_snapshot_text(self, text: str) -> None:
        if text == self._snapshot_text_cache:
            return
        self.snapshot_text.config(state="normal")
        self.snapshot_text.delete("1.0", "end")
        self.snapshot_text.insert("1.0", text)
        self.snapshot_text.config(state="disabled")
        self._snapshot_text_cache = text

    def _set_mapping_text(self, text: str) -> None:
        if text == self._mapping_text_cache:
            return
        self.mapping_text.config(state="normal")
        self.mapping_text.delete("1.0", "end")
        self.mapping_text.insert("1.0", text)
        self.mapping_text.config(state="disabled")
        self._mapping_text_cache = text

    def _set_busy(self, busy: bool) -> None:
        self._busy = busy
        self.root.configure(cursor="watch" if busy else "")

    def _set_multi_realtime_enabled(self, enabled: bool) -> None:
        self.multi_realtime_enabled = enabled
        if enabled:
            self.multi_realtime_button_text.set("停止多舵机实时读数")
            self.multi_realtime_status_text.set("实时读取：运行中")
        else:
            self.multi_realtime_button_text.set("开启多舵机实时读数")
            suffix = "本轮读取结束" if self.multi_realtime_running else "关闭"
            self.multi_realtime_status_text.set(f"实时读取：{suffix}")

    def _run_task(self, title: str, fn) -> None:
        serial_titles = {"扫描单舵机 raw", "读取当前 raw", "读取配置内多舵机 raw"}
        if self.multi_realtime_enabled and title in serial_titles:
            self._log("warn", "多舵机实时读数运行中。先停止实时读取，再执行其他串口读取。")
            return
        if self._busy:
            self._log("warn", "上一条操作还没完成，请先等它结束。")
            return

        self._set_busy(True)
        self._log("info", f"[{title}] 开始")

        def worker() -> None:
            try:
                result = fn()
            except Exception as exc:
                self.root.after(0, lambda exc=exc, title=title: self._task_failed(title, exc))
                return
            self.root.after(0, lambda result=result, title=title: self._task_done(title, result))

        threading.Thread(target=worker, daemon=True).start()

    def _task_done(self, title: str, result) -> None:
        self._set_busy(False)
        try:
            if callable(result):
                result()
        except Exception as exc:
            self._log("error", f"[{title}] UI 更新失败: {exc}")
            messagebox.showerror("界面更新失败", f"{title}\n\n{exc}")
            return
        self._log("ok", f"[{title}] 完成")

    def _task_failed(self, title: str, exc: Exception) -> None:
        self._set_busy(False)
        self._log("error", f"[{title}] 失败: {exc}")
        messagebox.showerror("操作失败", f"{title}\n\n{exc}")

    def _reload_config(self) -> None:
        def task():
            cfg = load_config(self.config_path)

            def update_ui() -> None:
                current_profile = self.profile_name.get()
                self.cfg = cfg
                values = list(self.cfg.servos.keys())
                self.profile_combo.configure(values=values)
                if current_profile not in values:
                    current_profile = values[0]
                self.profile_name.set(current_profile)
                self._rebuild_servo_shortcuts()
                self._load_profile_into_editor()
                self._log("ok", f"配置已重新加载：{self.config_path}")

            return update_ui

        self._run_task("重新加载配置", task)

    def _reload_config_in_place(self) -> None:
        current_profile = self.profile_name.get()
        self.cfg = load_config(self.config_path)
        values = list(self.cfg.servos.keys())
        self.profile_combo.configure(values=values)
        if current_profile not in values:
            current_profile = values[0]
        self.profile_name.set(current_profile)
        self._rebuild_servo_shortcuts()
        self._load_profile_into_editor()

    def _save_current_profile_mapping(self) -> None:
        if self.multi_realtime_enabled:
            raise RuntimeError("请先停止多舵机实时读数，再保存映射配置。")

        profile = self._current_profile()
        draft = self._draft_mapping()
        updated_profile = ServoProfile(name=profile.name, id=profile.id, mapping=draft)
        updated_config = build_updated_config(self.cfg, updated_profile)
        save_config(updated_config, self.config_path)
        self._reload_config_in_place()
        self._log("ok", f"已保存 {profile.name} 的映射到配置文件：{self.config_path}")

    def _restore_full_config_from_initial(self) -> None:
        if self.multi_realtime_enabled:
            raise RuntimeError("请先停止多舵机实时读数，再恢复配置。")
        confirmed = messagebox.askyesno(
            "恢复初始配置",
            f"这会用初始快照覆盖当前配置文件：\n\n{self.initial_config_path}\n->\n{self.config_path}\n\n是否继续？",
        )
        if not confirmed:
            self._log("info", "已取消恢复初始配置。")
            return
        shutil.copyfile(self.initial_config_path, self.config_path)
        self._reload_config_in_place()
        self._log("ok", f"已从初始快照恢复整个配置：{self.initial_config_path}")

    def _current_servo_id(self) -> int:
        raw = self.active_servo_id.get().strip()
        if not raw:
            raise ValueError("当前舵机 ID 为空")
        return int(raw)

    def _current_profile(self) -> ServoProfile:
        return self.cfg.resolve_servo(self.profile_name.get())

    def _find_profile_by_servo_id(self, servo_id: int) -> ServoProfile | None:
        for profile in self.cfg.servos.values():
            if profile.id == servo_id:
                return profile
        return None

    def _rebuild_servo_shortcuts(self) -> None:
        for child in self.shortcut_frame.winfo_children():
            child.destroy()

        for col, profile in enumerate(sorted(self.cfg.servos.values(), key=lambda item: item.id)):
            self.shortcut_frame.columnconfigure(col, weight=1)
            button_text = f"{profile.name} / ID {profile.id}\n读取当前值并切换"
            ttk.Button(
                self.shortcut_frame,
                text=button_text,
                style="Soft.TButton",
                command=lambda name=profile.name: self._safe_ui_action(
                    f"读取 {name} 当前 raw",
                    lambda name=name: self._select_profile_and_read(name),
                ),
            ).grid(row=0, column=col, sticky="ew", padx=(0 if col == 0 else 8, 0))

    def _configured_profiles_sorted(self) -> list[ServoProfile]:
        profiles = sorted(self.cfg.servos.values(), key=lambda item: item.id)
        if not profiles:
            raise ValueError("当前配置里没有任何舵机条目。")
        return profiles

    def _load_profile_into_editor(self) -> None:
        profile = self._current_profile()
        mapping = profile.mapping
        self.edit_raw_min.set(str(mapping.raw_min))
        self.edit_raw_max.set(str(mapping.raw_max))
        self.edit_coord_at_raw_min.set(str(mapping.mapped_angle_at_raw_min))
        self.edit_coord_at_raw_max.set(str(mapping.mapped_angle_at_raw_max))
        self.edit_position_step.set(str(mapping.position_step))
        self._refresh_mapping_preview()

    def _draft_mapping(self) -> ServoMapping:
        return ServoMapping(
            raw_min=int(self.edit_raw_min.get().strip()),
            raw_max=int(self.edit_raw_max.get().strip()),
            mapped_angle_at_raw_min=float(self.edit_coord_at_raw_min.get().strip()),
            mapped_angle_at_raw_max=float(self.edit_coord_at_raw_max.get().strip()),
            position_step=int(self.edit_position_step.get().strip()),
        )

    def _select_profile_and_read(self, profile_name: str) -> None:
        profile = self.cfg.resolve_servo(profile_name)
        self.profile_name.set(profile.name)
        self.active_servo_id.set(str(profile.id))
        self._load_profile_into_editor()
        self._read_snapshot()

    def _effective_preview_raw(self) -> int | None:
        if self.last_snapshot is not None and self.last_snapshot.position_raw is not None:
            return self.last_snapshot.position_raw
        raw_text = self.preview_raw_value.get().strip()
        if not raw_text:
            return None
        return int(raw_text)

    def _use_current_raw_as_preview(self) -> None:
        raw_value = self._effective_preview_raw()
        if raw_value is None:
            raise ValueError("当前没有可用 raw。先扫描或读取一次。")
        self.preview_raw_value.set(str(raw_value))
        self.raw_value.set(str(raw_value))
        self._refresh_mapping_preview()
        self._log("info", f"已把 raw={raw_value} 复制到锚点。")

    def _anchor_inputs(self) -> tuple[int, float, float]:
        raw_text = self.preview_raw_value.get().strip()
        if not raw_text:
            raise ValueError("请先给锚点 raw 一个值。")
        anchor_raw = int(raw_text)
        anchor_coord = float(self.anchor_coord_value.get().strip())
        total_span = float(self.anchor_span_value.get().strip())
        if total_span == 0:
            raise ValueError("总坐标跨度不能为 0")
        return anchor_raw, anchor_coord, total_span

    def _apply_anchor_as_min(self) -> None:
        anchor_raw, anchor_coord, total_span = self._anchor_inputs()
        self.edit_raw_min.set(str(anchor_raw))
        self.edit_coord_at_raw_min.set(str(anchor_coord))
        self.edit_coord_at_raw_max.set(str(anchor_coord + total_span))
        self.raw_value.set(str(anchor_raw))
        self.coord_value.set(str(anchor_coord))
        self._refresh_mapping_preview()
        self._log("info", f"已把 raw={anchor_raw} 设为新最小端，坐标值={anchor_coord}。")

    def _apply_anchor_as_max(self) -> None:
        anchor_raw, anchor_coord, total_span = self._anchor_inputs()
        self.edit_raw_max.set(str(anchor_raw))
        self.edit_coord_at_raw_max.set(str(anchor_coord))
        self.edit_coord_at_raw_min.set(str(anchor_coord - total_span))
        self.raw_value.set(str(anchor_raw))
        self.coord_value.set(str(anchor_coord))
        self._refresh_mapping_preview()
        self._log("info", f"已把 raw={anchor_raw} 设为新最大端，坐标值={anchor_coord}。")

    def _apply_anchor_as_center(self) -> None:
        anchor_raw, anchor_coord, total_span = self._anchor_inputs()
        draft = self._draft_mapping()
        raw_span = abs(draft.raw_max - draft.raw_min)
        if raw_span == 0:
            raise ValueError("当前 raw 范围为 0，无法把锚点放到中点。")
        new_min = anchor_raw - raw_span // 2
        new_max = new_min + raw_span
        self.edit_raw_min.set(str(new_min))
        self.edit_raw_max.set(str(new_max))
        self.edit_coord_at_raw_min.set(str(anchor_coord - total_span / 2.0))
        self.edit_coord_at_raw_max.set(str(anchor_coord + total_span / 2.0))
        self.raw_value.set(str(anchor_raw))
        self.coord_value.set(str(anchor_coord))
        self._refresh_mapping_preview()
        self._log("info", f"已把 raw={anchor_raw} 设为新中点，中心坐标值={anchor_coord}。")

    def _refresh_mapping_preview(self) -> None:
        try:
            profile = self._current_profile()
            base = profile.mapping
            draft = self._draft_mapping()
        except Exception as exc:
            self._set_mapping_text(f"映射草稿无效：{exc}")
            self.current_coord_existing_value.set("--")
            self.current_coord_draft_value.set("--")
            return

        lines = [
            f"template_name             : {profile.name}",
            f"template_servo_id         : {profile.id}",
            "",
            f"existing raw_min          : {base.raw_min}",
            f"existing raw_max          : {base.raw_max}",
            f"existing coord@raw_min    : {base.mapped_angle_at_raw_min:.6f}",
            f"existing coord@raw_max    : {base.mapped_angle_at_raw_max:.6f}",
            f"existing coord/raw        : {base.angle_per_raw:.6f}",
            "",
            f"draft raw_min             : {draft.raw_min}",
            f"draft raw_max             : {draft.raw_max}",
            f"draft coord@raw_min       : {draft.mapped_angle_at_raw_min:.6f}",
            f"draft coord@raw_max       : {draft.mapped_angle_at_raw_max:.6f}",
            f"draft coord/raw           : {draft.angle_per_raw:.6f}",
            f"draft position_step       : {draft.position_step}",
        ]

        raw_value = self._effective_preview_raw()
        if raw_value is not None:
            current_existing = base.raw_to_angle(raw_value)
            current_draft = draft.raw_to_angle(raw_value)
            self.current_raw_value.set(str(raw_value))
            self.current_coord_existing_value.set(f"{current_existing:.3f}")
            self.current_coord_draft_value.set(f"{current_draft:.3f}")
            lines.extend(
                [
                    "",
                    f"preview_raw              : {raw_value}",
                    f"coord(existing)          : {current_existing:.6f}",
                    f"coord(draft)             : {current_draft:.6f}",
                ]
            )
        else:
            self.current_raw_value.set("--")
            self.current_coord_existing_value.set("--")
            self.current_coord_draft_value.set("--")
            lines.extend(["", "preview_raw              : <先扫描或读取>"])

        lines.extend(
            [
                "",
                "说明：这里改的是本地换算，不写串口，不会让舵机突然运动。",
                "只有点击“保存当前舵机映射到配置文件”才会真正落盘。",
                f"初始快照保存在：{self.initial_config_path.name}",
                f"当前活动配置文件：{self.config_path.name}",
                "",
                "suggested_config_snippet:",
                f"[servos.{profile.name}]",
                f"id = {profile.id}",
                f"position_step = {draft.position_step}",
                f"raw_min = {draft.raw_min}",
                f"raw_max = {draft.raw_max}",
                f"mapped_angle_at_raw_min = {draft.mapped_angle_at_raw_min}",
                f"mapped_angle_at_raw_max = {draft.mapped_angle_at_raw_max}",
            ]
        )

        self._set_mapping_text("\n".join(lines))

    def _scan_single_servo_raw(self) -> None:
        def task():
            with LX225Service(self.cfg) as service:
                found = service.driver.scan_simple_positions(servo_id_min=1, servo_id_max=8, timeout=0.6)
            if not found:
                raise RuntimeError("没有扫到任何舵机 ID。")

            def update_ui() -> None:
                self.last_scan = dict(sorted(found.items()))
                joined = ", ".join(f"{servo_id}:{raw}" for servo_id, raw in self.last_scan.items())
                self.detected_ids_value.set(joined)
                self.driver_mode_value.set("simple 0x15 read (controller-safe)")
                if len(self.last_scan) == 1:
                    servo_id, raw_value = next(iter(self.last_scan.items()))
                    matched_profile = self._find_profile_by_servo_id(servo_id)
                    self.last_snapshot = None
                    self.active_servo_id.set(str(servo_id))
                    self.preview_raw_value.set(str(raw_value))
                    self.raw_value.set(str(raw_value))
                    self.current_raw_value.set(str(raw_value))
                    lines = [
                        "scan_mode        : simple 0x15",
                        f"detected_servo_id: {servo_id}",
                        f"position_raw     : {raw_value}",
                    ]
                    if matched_profile is not None:
                        lines.append(f"matched_template : {matched_profile.name} (id={matched_profile.id})")
                    lines.extend(
                        [
                            "",
                            "next_step:",
                            "1. 左侧“当前舵机 ID”已经自动回填。",
                            "2. 检查“映射模板”是否与你想用的模板一致。",
                            "3. 点击“读取当前 raw”刷新明细。",
                            "4. 去中间点“用当前 raw”开始定义新映射。",
                        ]
                    )
                    self._set_snapshot_text("\n".join(lines))
                    self._log("info", f"单舵机扫描命中：id={servo_id}, raw={raw_value}")
                    if matched_profile is not None and matched_profile.name != self.profile_name.get():
                        self._log(
                            "warn",
                            f"当前硬件 ID={servo_id}，当前模板仍是 {self.profile_name.get()}。如需匹配模板，可切换到 {matched_profile.name}。",
                        )
                else:
                    self._log("warn", f"扫描到多个 ID：{joined}")
                    self._set_snapshot_text(
                        "\n".join(
                            [
                                f"scan_mode        : simple 0x15",
                                f"detected_ids     : {joined}",
                                "",
                                "当前扫到多个 ID，请确认总线上是否真的只接了一个舵机。",
                            ]
                        )
                    )
                self._refresh_mapping_preview()

            return update_ui

        self._run_task("扫描单舵机 raw", task)

    def _read_snapshot(self) -> None:
        servo_id = self._current_servo_id()
        profile = self._current_profile()

        def task():
            with LX225Service(self.cfg) as service:
                position_raw = service.driver.read_position_simple(servo_id, timeout=service._simple_timeout())
            if position_raw is None:
                raise RuntimeError(f"simple 0x15 读取未返回 id={servo_id} 的 raw")

            snapshot = ServoSnapshot(
                servo_name=profile.name,
                servo_id=servo_id,
                position_raw=position_raw,
                mapped_angle=profile.mapping.raw_to_angle(position_raw),
                offset=None,
                limit_min_raw=None,
                limit_max_raw=None,
            )

            def update_ui() -> None:
                self.last_snapshot = snapshot
                self.driver_mode_value.set("simple 0x15 read (controller-safe)")
                self.preview_raw_value.set(str(snapshot.position_raw))
                self.raw_value.set(str(snapshot.position_raw))
                self.detected_ids_value.set(f"{snapshot.servo_id}:{snapshot.position_raw}")
                coord_text = f"{profile.mapping.raw_to_angle(snapshot.position_raw):.3f}"

                lines = [
                    f"mapping_template: {profile.name} (id={profile.id})",
                    f"servo_id        : {snapshot.servo_id}",
                    f"position_raw    : {snapshot.position_raw}",
                    f"coord(template) : {coord_text}",
                    "read_mode       : simple 0x15 only",
                    "offset          : <已跳过，避免控制板误解标准寄存器读>",
                    "limit_min       : <已跳过，避免控制板误解标准寄存器读>",
                    "limit_max       : <已跳过，避免控制板误解标准寄存器读>",
                ]
                if profile.id != snapshot.servo_id:
                    lines.extend(
                        [
                            "",
                            f"template_note   : 当前硬件 ID={snapshot.servo_id}，但你正在用模板 {profile.name}(id={profile.id}) 做换算。",
                        ]
                    )
                lines.extend(
                    [
                        "",
                        "safety_note     : 当前 GUI 单舵机读取已禁用非 0x15 标准读，避免在控制板口误触发运动。",
                    ]
                )

                self._set_snapshot_text("\n".join(lines))
                self._refresh_mapping_preview()
                self._log("info", f"安全读取成功：id={snapshot.servo_id}, raw={snapshot.position_raw}")

            return update_ui

        self._run_task("读取当前 raw", task)

    def _read_configured_multi_raw(self) -> None:
        profiles = self._configured_profiles_sorted()

        def task():
            rows, errors = self._collect_configured_multi_raw_rows(profiles)

            def update_ui() -> None:
                joined = self._apply_multi_raw_rows(rows, errors, live=False)
                if errors:
                    self._log("warn", f"多舵机读取完成，但有 {len(errors)} 项未读到。")
                else:
                    self._log("info", f"多舵机 raw：{joined}")

            return update_ui

        self._run_task("读取配置内多舵机 raw", task)

    def _collect_configured_multi_raw_rows(
        self,
        profiles: list[ServoProfile],
        *,
        timeout_override: float | None = None,
        service: LX225Service | None = None,
    ) -> tuple[list[tuple[ServoProfile, int | None]], list[str]]:
        rows: list[tuple[ServoProfile, int | None]] = []
        errors: list[str] = []

        def collect(open_service: LX225Service) -> tuple[list[tuple[ServoProfile, int | None]], list[str]]:
            timeout = open_service._simple_timeout() if timeout_override is None else float(timeout_override)
            ids = [profile.id for profile in profiles]
            found = open_service.driver.read_positions_simple(ids, timeout=timeout)
            for profile in profiles:
                rows.append((profile, found.get(profile.id)))
            missing_ids = [profile.id for profile in profiles if profile.id not in found]
            for servo_id in missing_ids:
                profile = next(item for item in profiles if item.id == servo_id)
                errors.append(f"{profile.name}(id={profile.id}): no response in batch read")
            return rows, errors

        if service is not None:
            return collect(service)

        with LX225Service(self.cfg) as one_shot_service:
            return collect(one_shot_service)

    def _apply_multi_raw_rows(self, rows: list[tuple[ServoProfile, int | None]], errors: list[str], *, live: bool) -> str:
        found = {profile.id: raw for profile, raw in rows if raw is not None}
        self.last_scan = dict(sorted(found.items()))
        joined = ", ".join(f"{servo_id}:{raw}" for servo_id, raw in self.last_scan.items())

        lines = [
            "multi_read_mode  : configured servos, simple 0x15 sequential read",
            f"configured_count : {len(rows)}",
            f"refresh_mode     : {'live' if live else 'one-shot'}",
            "",
            "servo_name        servo_id    raw         coord(template)",
            "--------------------------------------------------------",
        ]
        for profile, raw_value in rows:
            if raw_value is None:
                coord_text = "<未读到>"
                raw_text = "<未读到>"
            else:
                coord_text = f"{profile.mapping.raw_to_angle(raw_value):.3f}"
                raw_text = str(raw_value)
            lines.append(f"{profile.name:<16} {profile.id:<11} {raw_text:<11} {coord_text}")

        servo_count = len(rows)

        if errors:
            lines.extend(["", "notes:"])
            lines.extend(f"- {item}" for item in errors)
        else:
            lines.extend(
                [
                    "",
                    f"用途：把配置里的 {servo_count} 只舵机都手摆到你认定的同一参考姿态，再读取 raw，比较它们是否对齐。",
                    f"如果这 {servo_count} 只舵机的 raw 不同，就用各自模板映射把这个参考姿态归一到同一个上层坐标值。",
                ]
            )

        selected_id: int | None
        try:
            selected_id = self._current_servo_id()
        except Exception:
            selected_id = None

        chosen_profile: ServoProfile | None = None
        chosen_raw: int | None = None
        if selected_id is not None:
            for profile, raw_value in rows:
                if profile.id == selected_id and raw_value is not None:
                    chosen_profile, chosen_raw = profile, raw_value
                    break
        if chosen_profile is None:
            for profile, raw_value in rows:
                if raw_value is not None:
                    chosen_profile, chosen_raw = profile, raw_value
                    break

        signature = (
            tuple((profile.id, raw_value) for profile, raw_value in rows),
            tuple(errors),
        )
        focus_key = None if chosen_profile is None or chosen_raw is None else (chosen_profile.id, chosen_raw)
        if live and signature == self._last_multi_live_signature and focus_key == self._last_multi_live_focus:
            return joined

        self.detected_ids_value.set(joined if joined else "--")
        self.driver_mode_value.set("simple 0x15 sequential read")
        self._set_snapshot_text("\n".join(lines))

        if chosen_profile is not None and chosen_raw is not None:
            preview_changed = focus_key != self._last_multi_live_focus or not live
            self.last_snapshot = ServoSnapshot(
                servo_name=chosen_profile.name,
                servo_id=chosen_profile.id,
                position_raw=chosen_raw,
                mapped_angle=chosen_profile.mapping.raw_to_angle(chosen_raw),
                offset=None,
                limit_min_raw=None,
                limit_max_raw=None,
            )
            self.active_servo_id.set(str(chosen_profile.id))
            self.preview_raw_value.set(str(chosen_raw))
            self.raw_value.set(str(chosen_raw))
            self.current_raw_value.set(str(chosen_raw))
            if preview_changed:
                self._refresh_mapping_preview()
        else:
            self.last_snapshot = None
            self.current_raw_value.set("--")
            if not live:
                self._log("warn", "配置内多舵机读取完成，但没有拿到任何 raw。")

        self._last_multi_live_signature = signature
        self._last_multi_live_focus = focus_key
        return joined

    def _toggle_multi_realtime(self) -> None:
        if self.multi_realtime_enabled:
            self._stop_multi_realtime()
        else:
            self._start_multi_realtime()

    def _start_multi_realtime(self) -> None:
        self._configured_profiles_sorted()
        if self.multi_realtime_service is None:
            service = LX225Service(self.cfg)
            try:
                service.connect()
            except Exception:
                service.close()
                raise
            self.multi_realtime_service = service
        self._set_multi_realtime_enabled(True)
        self._log("info", "多舵机实时读数已开启。串口保持常连，并批量读取多个 ID 的 raw。")
        self._schedule_multi_realtime_tick(immediate=True)

    def _stop_multi_realtime(self) -> None:
        if self.multi_realtime_job is not None:
            self.root.after_cancel(self.multi_realtime_job)
            self.multi_realtime_job = None
        self._set_multi_realtime_enabled(False)
        self._last_multi_live_signature = None
        self._last_multi_live_focus = None
        if not self.multi_realtime_running:
            self._close_multi_realtime_service()
        self._log("info", "多舵机实时读数已停止。")

    def _schedule_multi_realtime_tick(self, *, immediate: bool = False) -> None:
        if not self.multi_realtime_enabled:
            return
        delay = 0 if immediate else self.multi_realtime_interval_ms
        self.multi_realtime_job = self.root.after(delay, self._run_multi_realtime_tick)

    def _run_multi_realtime_tick(self) -> None:
        self.multi_realtime_job = None
        if not self.multi_realtime_enabled:
            return
        if self.multi_realtime_running or self._busy:
            self._schedule_multi_realtime_tick()
            return

        profiles = self._configured_profiles_sorted()
        if self.multi_realtime_service is None:
            self._log("warn", "实时读数服务未连接，正在重建串口连接。")
            self._start_multi_realtime()
            return
        self.multi_realtime_running = True
        self.multi_realtime_status_text.set("实时读取：采集中")

        def worker() -> None:
            try:
                rows, errors = self._collect_configured_multi_raw_rows(
                    profiles,
                    timeout_override=self.multi_realtime_timeout_s,
                    service=self.multi_realtime_service,
                )
            except Exception as exc:
                self.root.after(0, lambda exc=exc: self._finish_multi_realtime_tick_with_error(exc))
                return
            self.root.after(0, lambda rows=rows, errors=errors: self._finish_multi_realtime_tick(rows, errors))

        threading.Thread(target=worker, daemon=True).start()

    def _finish_multi_realtime_tick(self, rows: list[tuple[ServoProfile, int | None]], errors: list[str]) -> None:
        self.multi_realtime_running = False
        if self.multi_realtime_enabled:
            self.multi_realtime_status_text.set("实时读取：运行中")
            self._apply_multi_raw_rows(rows, errors, live=True)
            self._schedule_multi_realtime_tick()
        else:
            self._close_multi_realtime_service()
            self.multi_realtime_status_text.set("实时读取：关闭")

    def _finish_multi_realtime_tick_with_error(self, exc: Exception) -> None:
        self.multi_realtime_running = False
        if self.multi_realtime_enabled:
            self.multi_realtime_status_text.set("实时读取：运行中")
            self._log("warn", f"多舵机实时读数本轮失败：{exc}")
            self._schedule_multi_realtime_tick()
        else:
            self._close_multi_realtime_service()

    def _close_multi_realtime_service(self) -> None:
        if self.multi_realtime_service is not None:
            try:
                self.multi_realtime_service.close()
            finally:
                self.multi_realtime_service = None

    def _on_close(self) -> None:
        if self.multi_realtime_job is not None:
            self.root.after_cancel(self.multi_realtime_job)
            self.multi_realtime_job = None
        self.multi_realtime_enabled = False
        self._close_multi_realtime_service()
        self.root.destroy()

    def _coord_to_raw(self) -> None:
        draft = self._draft_mapping()
        coord = float(self.coord_value.get().strip())
        raw_value = draft.angle_to_raw(coord, quantize=True)
        self.raw_value.set(str(raw_value))
        self.current_coord_draft_value.set(f"{coord:.3f}")
        self._log("info", f"草稿映射换算：coord {coord} -> raw {raw_value}")

    def _raw_to_coord(self) -> None:
        draft = self._draft_mapping()
        raw_value = int(self.raw_value.get().strip())
        coord = draft.raw_to_angle(raw_value)
        self.coord_value.set(f"{coord:.3f}")
        self._log("info", f"草稿映射换算：raw {raw_value} -> coord {coord:.3f}")


def launch_gui(config_path: Path) -> None:
    root = tk.Tk()
    LX225ToolGUI(root, config_path=config_path)
    root.mainloop()

